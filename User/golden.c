#include "stm32f10x.h"
#include "golden.h"
#include "W25Q64.h"
#include "boot_flag.h"          /* ★ 回退标志用的备份寄存器（★ 和 Boot 那份一致 ✓）*/
#include "Serial.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"             /* ★★★ 审计 G1：访问 W25Q64 要【互斥】✓ */
#include "Crc16.h"              /* ★ 复用【全工程同一套】CRC16 ✗ 不另发明 ✓ */

/*==============================================================================
 *  ★★★★★ 金副本实现（★ 2026-10-14 建 ✓）完整设计理由见 golden.h
 *
 *  ★★★★ 本版 = 首版 + 【独立审计报告】的修复（★ 审计日期 2026-10-14 ✓）：
 *    · ★★ **G1** ✗ 加 W25Q64 互斥量 + `SaveCurrent` 后重置 `s_filling/s_page`
 *      （★ 原版两个写者同为优先级 1 ✗ 时间片 1ms < 一页忙等 1.4ms ✗
 *        【写一页的中途【必定】被切换 ✗ 【备份静默损坏】】✓✓）
 *    · ★★★ **G2** ✗ `B` 不再无条件擦头
 *      （★ 原版会把【"上一版好固件"】当成【当前坏固件】清掉 ✗
 *        【升级之后槽里装的是【旧的好版本】✗ 一把擦掉 = 【再也回不去】】✓✓）
 *    · ★★★ **G7** ✗ 写页循环里喂狗 + `taskYIELD()`
 *      （★ 原版一口气写 224 页 ✗ 同优先级的 `task_SD` 被饿 1~3 秒 ✓）
 *    · ★★ **G8** ✗ 镜像退到 `0x001000`（★ 头独占一扇区 ✗ 见 golden.h ✓）
 *    · ★ 未做（★ 见 §9.3 与审计报告 §11 的分批计划 ✓）✗
 *      G9 换 CRC32 ✗ G10 黑名单扩槽 ✗ G11 回退标记 ✓
 *============================================================================*/

/* ---- 头（★ 64 字节 ✗ 独占 0x000000 那个扇区 ✓）---------------------------- */
typedef struct {
	uint32_t magic;             /* ★★ 【最后才写】✗ 见 golden.h ✓ */
	uint32_t size;              /* ★ 镜像字节数 ✓ */
	uint16_t crc16;             /* ★ 整段 CRC-16/KERMIT ✓ */
	uint16_t fw_tag;            /* ★ 版本标识（★ v1 直接放 CRC ✗ 留作以后用 ✓）*/
	uint32_t uptime_s;          /* ★ 写入时已运行秒数（★ 诊断用 ✓）*/
	uint8_t  reserved[48];
} GoldenHdr_t;

/* ---- 元数据（★ 4KB 扇区里只用了不到 64B ✓）-------------------------------- */
typedef struct {
	uint32_t magic;             /* ★ 最后写 ✓ */
	uint16_t blacklist_crc;     /* ★★ 被"作废"的那一版的 CRC ✓ */
	uint16_t backup_crc;        /* ★ 冗余记录 ✓ */
	uint32_t seq;               /* ★ 写入次数（★ 诊断 ✓）*/
	uint8_t  reserved[48];
} GoldenMeta_t;

#define GOLDEN_PAGES    (GOLDEN_APP_SIZE / W25_PAGE_SIZE)     /* ★ 224 页 ✓ */
#define GOLDEN_FAIL_MAX 3u                                    /* ★ 连续失败几次就放弃 ✓ */

static uint8_t  s_on      = 0u;     /* ★ W25Q64 在位？不在位就【整个功能静默关闭】✓ */
static uint16_t s_my_crc  = 0u;     /* ★ 当前运行镜像的 CRC ✓ */
static uint16_t s_black   = GOLDEN_CRC_NONE;
static uint8_t  s_filling = 0u;     /* ★ 1 = 正在后台填 ✓ */
static uint16_t s_page    = 0u;     /* ★ 已经传到第几页 ✓ */
static uint8_t  s_fail    = 0u;     /* ★★ 连续失败次数（★ 超过上限就放弃 ✓）*/

/*==============================================================================
 *  ★★★★★ W25Q64 互斥量（★ 审计 G1 的修复 ✓）
 *
 *  ★★★★ 为什么【必须有它**✗✓：
 *    · ★ 【两个写者】✗
 *      · `task_GOLDEN` ✗ `Golden_Task()`（★ 后台慢慢填 ✓）
 *      · `task_DISP`   ✗ `Golden_SaveCurrent()` / `Golden_ClearBackup()` ✓
 *    · ★★ 【两者【同为优先级 1】】✗ 而 FreeRTOS 同优先级是【时间片轮转】
 *      （★ tick = 1ms ✓）✓
 *    · ★★★ 而【软件 SPI 写一页 256B ≈ 1.4ms 忙等】✗
 *      → 【1.4ms > 1ms ✗ 【写一页的中途【必定】发生一次任务切换】】✓✓
 *      → 【另一个任务接着驱动【同一组 GPIO】✗ 字节序列被打断 ✗ CS 被意外拉高】✓
 *    · ★★★★ 结果 ✗：【备份数据【静默损坏】✗ 【直到你要用它救命的时候才发现】】✓✓
 *
 *  ⚠️ ★★ 注意 ✗：★ FreeRTOS 的【普通互斥量【不可递归】**✗✓
 *    → 【所以【只在【公共入口】和【每段 GPIO 操作】外面加锁】✗
 *      【内部函数【一律假定"锁已经在我手里"】】✓✓
 *============================================================================*/
static SemaphoreHandle_t s_w25_lock = NULL;


/*==============================================================================
 *  ★★★ 辅助：把一段内部 Flash 算成 CRC16
 *    ★ 为什么【用整段 56KB ✗ 而不是"实际代码长度"】**✗✓：
 *      · ★ App 【不知道自己的长度】✗（★ 没有任何地方记着 ✓）
 *      · ★★ 而【未用部分【全是 0xFF ✗ 内容是确定的】】✓
 *      → ★★★ 【所以整段算 ✗ 双方（App / Boot）【得到的结果稳定一致】】✓✓
 *    ★★ 代价 ✗：★ 56KB × 8 位 = 45 万次循环 ≈ **40~60ms** ✓
 *      （★ 只在【开机一次】和【写备份前】算 ✗ 可接受 ✓）
 *============================================================================*/
uint16_t Golden_ImageCrc(void)
{
	return Crc16_Kermit((const uint8_t *)GOLDEN_APP_BASE, (uint32_t)GOLDEN_APP_SIZE);
}

/* ★★ 失败收尾（★ 审计 G7 相关 ✗ 加"重试上限"✗ 免得坏了就永远转 ✓）---------
 *  ★ 为什么【要 `vTaskDelay`**✗：★ 失败路径【原来直接 return ✗ 下一拍又冲上来】✗
 *    【变成【没有延时的空转】】✓✓ */
static void Golden_Fail(const char *what)
{
	s_fail++;
	Serial_Printf("GOLD: %s FAILED (%u/%u)\r\n", what, (unsigned)s_fail, (unsigned)GOLDEN_FAIL_MAX);

	if (s_fail >= GOLDEN_FAIL_MAX)
	{
		Serial_Printf("GOLD: giving up -> check wiring / chip\r\n");
		s_filling = 0u;
	}
	else
	{
		s_page = 0u;                                /* ★ 从第 0 页【重来】✓ */
	}

	vTaskDelay(pdMS_TO_TICKS(2000u));               /* ★ 喘口气 ✗ 别空转 ✓ */
}

/* ★★★ 读头（★ 返回 1 = 【头有效】✓）✗ crc 通过 out 参数带出来 ✓
 *  ⚠️ ★ 本函数【不加锁】✗【调用者负责】✓ */
static uint8_t Golden_ReadHdr(uint16_t *crc_out)
{
	GoldenHdr_t h;

	if (W25Q64_ReadData(GOLDEN_ADDR_HDR, (uint8_t *)&h, sizeof(h)) == 0u)
	{
		return 0u;
	}
	if ((h.magic != GOLDEN_HDR_MAGIC) || (h.size != (uint32_t)GOLDEN_APP_SIZE))
	{
		return 0u;                                  /* ★★ 头没写完 / 没写过 ✓ */
	}
	if (crc_out != 0)
	{
		*crc_out = h.crc16;
	}
	return 1u;
}

/* ★★ 读元数据（★ 拿黑名单 ✓）✗ 读不到就用"没有黑名单" ✓
 *  ⚠️ ★ 只在 `Golden_Init` 里调（★ 那时【还没多任务】✗ 不用锁 ✓）*/
static void Golden_ReadMeta(void)
{
	GoldenMeta_t m;

	s_black = GOLDEN_CRC_NONE;

	if (W25Q64_ReadData(GOLDEN_ADDR_META, (uint8_t *)&m, sizeof(m)) == 0u)
	{
		return;
	}
	if (m.magic == GOLDEN_META_MAGIC)
	{
		s_black = m.blacklist_crc;
	}
}

/* ★★★★ 写头（★ **magic 最后写**✗ 见 golden.h ✓）
 *  ⚠️ ★ 本函数【不加锁】✗【调用者负责】✓ */
static uint8_t Golden_WriteHdr(uint16_t crc)
{
	GoldenHdr_t h;
	uint32_t    m;
	uint8_t     i;

	h.magic    = 0xFFFFFFFFu;                       /* ★ 先留空 ✗ 最后才写 ✓ */
	h.size     = (uint32_t)GOLDEN_APP_SIZE;
	h.crc16    = crc;
	h.fw_tag   = crc;
	h.uptime_s = (uint32_t)(xTaskGetTickCount() / (TickType_t)configTICK_RATE_HZ);

	for (i = 0u; i < (uint8_t)sizeof(h.reserved); i++)
	{
		h.reserved[i] = 0xFFu;                      /* ★ 保持"擦除态"✗ 别写垃圾 ✓ */
	}

	if (W25Q64_WriteData(GOLDEN_ADDR_HDR, (const uint8_t *)&h, (uint32_t)sizeof(h)) == 0u)
	{
		return 0u;
	}

	/* ★★★★ 到这里【数据全在】✗ 才写上 magic ✓
	 *   ★ 此刻之前【掉电 ✗ 头是无效的 ✗ 这份备份【自动作废】】✓✓ */
	m = GOLDEN_HDR_MAGIC;
	return W25Q64_WriteData(GOLDEN_ADDR_HDR, (const uint8_t *)&m, 4u);
}

/*==============================================================================
 *  ★★★★★ 擦掉【整个金副本镜像区】（★ 2026-10-14 加 ✗ 为 128KB 芯片铺路 ✓）
 *
 *  ★★★★ 为什么【不能只擦【一个】64KB 块**✗✓：
 *    · ★ 现在 App = 56KB ✗ 镜像 + 64B 头【一个块装得下】✓
 *    · ★★ 而【换 `STM32F103CB`（128KB）之后 ✗ App 是 **120KB**】✗
 *      → 【跨【两个】64KB 块】✓
 *    · ★★★ 那时若【只擦第一块**✗：
 *      → 【后一半【还是旧的（没擦）】✗
 *      → 【写进去变成【旧值 AND 新值】✗ 【数据【静默写坏】】】✓✓
 *      → ★★★★ 而且【头【照样会被写上】✗ 声明"这是有效的"】✓
 *        → 【备份【表面有效 ✗ 实际校验不过】**✗
 *          【直到你要用它救命的时候才发现】✓✓
 *
 *  ★★★ 所以【按 `GOLDEN_APP_SIZE` 算出要擦几个块 ✗ 循环擦】✓✓
 *    · ★ 现在 `ceil((0x1000 + 56KB) / 64KB)` = **1 块** ✓（★ 和以前行为一样 ✓）
 *    · ★★ 换 CBT6 后 = **2 块** ✓✓
 *      → ★★★ 【这行代码【自动跟着变 ✗ 不用再改】】✓✓
 *
 *  ⚠️ ★★ 调用者【必须已经持有 `s_w25_lock`】✓
 *    · ★★ 擦两块最坏约 **4 秒** ✗ 而 `Golden_SaveCurrent` 的
 *      【等待窗口是 10 秒】✗ 【够 ✓】（★ 这就是当初从 3 秒调上去的原因 ✓）
 *============================================================================*/
static uint8_t Golden_EraseImageArea(void)
{
	uint32_t span = GOLDEN_IMG_OFFSET + (uint32_t)GOLDEN_APP_SIZE;
	uint32_t n    = (span + GOLDEN_BLOCK_SIZE - 1u) / GOLDEN_BLOCK_SIZE;
	uint32_t off;

	for (off = 0u; off < (n * GOLDEN_BLOCK_SIZE); off += GOLDEN_BLOCK_SIZE)
	{
		if (W25Q64_EraseBlock64K(GOLDEN_ADDR_HDR + off) == 0u)
		{
			return 0u;
		}
	}
	return 1u;
}

/* ★★★ 把整份镜像传一遍（★ 擦块 → 逐页 → 写头 ✓）
 *  ★ `slow` = 1 时每页让出 CPU（★ 后台用 ✓）✗ = 0 时一口气干完（★ 升级前用 ✓）
 *  ⚠️ ★★ 【调用者【必须已经拿到 `s_w25_lock`】】✓✓ */
static uint8_t Golden_WriteAll(uint8_t slow)
{
	uint16_t p;

	if (Golden_EraseImageArea() == 0u)
	{
		Serial_Printf("GOLD: erase block FAILED\r\n");
		return 0u;
	}

	for (p = 0u; p < (uint16_t)GOLDEN_PAGES; p++)
	{
		/* ★★★ 源地址【直接指内部 Flash】✗ 【不需要 RAM 缓冲】✓✓ */
		const uint8_t *src = (const uint8_t *)(GOLDEN_APP_BASE + ((uint32_t)p * W25_PAGE_SIZE));

		if (W25Q64_WriteData(GOLDEN_ADDR_IMG + ((uint32_t)p * W25_PAGE_SIZE), src,
		                     W25_PAGE_SIZE) == 0u)
		{
			Serial_Printf("GOLD: page %u write FAILED\r\n", (unsigned)p);
			return 0u;
		}

		/* ★★★★ 审计 G7 的修复 ✗ 每 16 页（= 4KB ✗ 正好一个扇区）做两件事 ✓：
		 *   · ★ `IWDG_ReloadCounter()` ✗
		 *     【因为【平时喂狗的是【空闲钩子】✗ 而它优先级 0】】✓
		 *     → 【本任务忙等的这段时间 ✗ 【钩子【根本跑不到】】】✓✓
		 *     （★ 账 ✗ 整趟约 2 秒 ≪ 6.55 秒 ✗ 其实够 ✓
		 *       ★★ 但【不该把安全余量【寄托在"应该够"上】】✓）
		 *   · ★★ `taskYIELD()` ✗
		 *     【让同优先级的 `task_SD`（都是 1）【插进来落盘】】✓✓
		 *     （★ 否则 SD 被饿 1~3 秒 ✗ 乒乓缓冲填满 ✗ PROC 丢数据 ✓）✓ */
		if ((p & 0x0Fu) == 0x0Fu)
		{
			IWDG_ReloadCounter();
			taskYIELD();
		}

		if (slow != 0u)
		{
			vTaskDelay(pdMS_TO_TICKS(GOLDEN_PAGE_INTERVAL_MS));
		}
	}

	return Golden_WriteHdr(s_my_crc);
}


/*==============================================================================
 *  ★★★ 初始化（★ 建任务之前调一次 ✓）
 *    ★ 现在【调度器还没启动】✗ 【单线程】✗ 所以这里【不用加锁】✓
 *============================================================================*/
uint8_t Golden_Init(void)
{
	uint16_t hdr_crc = 0u;

	/* ★★★ 先建互斥量 ✗ 审计 G1 ✓ */
	s_w25_lock = xSemaphoreCreateMutex();
	if (s_w25_lock == NULL)
	{
		Serial_Printf("GOLD: !! mutex create FAILED -> backup disabled\r\n");
		s_on = 0u;
		return 0u;
	}

	s_on = W25Q64_IsPresent();
	if (s_on == 0u)
	{
		Serial_Printf("GOLD: W25Q64 NOT PRESENT -> backup disabled\r\n");
		return 0u;
	}

	Golden_ReadMeta();
	s_my_crc = Golden_ImageCrc();

	Serial_Printf("GOLD: my image crc=0x%04X  blacklist=0x%04X\r\n",
	              (unsigned)s_my_crc, (unsigned)s_black);

	/* ---- 判断要不要开始填（★ 规则见 golden.h 的四条 ✓）---- */
	if (Golden_ReadHdr(&hdr_crc) != 0u)
	{
		/* ★ 槽里已有【有效备份】✓ */
		if (hdr_crc == s_my_crc)
		{
			Serial_Printf("GOLD: backup already matches this image -> nothing to do\r\n");
		}
		else
		{
			/* ★★ 【绝不动它】✗✓ —— 它代表【上一个能跑的版本】✓
			 *   （★ "从旧换新"只能由【升级流程】显式做 ✗ 见 golden.h ✓）*/
			Serial_Printf("GOLD: backup holds ANOTHER version (crc=0x%04X) -> keep it\r\n",
			              (unsigned)hdr_crc);
		}
		return 1u;
	}

	/* ★ 槽是空的（或头无效 ✓）→ 看看这一版是不是被作废过 ✓ */
	if (s_my_crc == s_black)
	{
		Serial_Printf("GOLD: this image is BLACKLISTED -> auto backup disabled\r\n");
		return 1u;
	}

	Serial_Printf("GOLD: no backup -> will fill it in background (%u pages)\r\n",
	              (unsigned)GOLDEN_PAGES);
	s_filling = 1u;
	s_page    = 0u;
	return 1u;
}


/*==============================================================================
 *  ★★★★ 后台填充任务（★ 由 task_GOLDEN 反复调用 ✗ 每次推进一步 ✓）
 *    ★ 为什么【一次只写一页**✗✓：
 *      · ★ 软件 SPI 是【忙等】✗ 一页 256B ≈ 1.4ms ✓
 *      · ★★ 一口气写 224 页 = **300ms 连续占用 CPU** ✗
 *      → ★★★ 【一页一跳 ✗ 采集任务【随时能抢占】】✓✓
 *
 *  ★★★ 锁的粒度（★ 审计 G1 ✓）✗【一次 GPIO 操作 = 一个临界区】✓
 *    · ★ 【不整趟拿锁】✗ 因为【块擦要约 1 秒】✗ 拿那么久会拖住 `SaveCurrent` ✓
 *    · ★★ 【只把"擦块"和"写一页"各自包起来】✗ 短 ✗ 而且【足够防交错】✓✓
 *============================================================================*/
void Golden_Task(void)
{
	if ((s_on == 0u) || (s_filling == 0u))
	{
		vTaskDelay(pdMS_TO_TICKS(5000));            /* ★ 没事干 ✗ 长睡 ✓ */
		return;
	}

	if (s_page == 0u)
	{
		uint8_t ok = 0u;

		Serial_Printf("GOLD: start background backup (%u pages, %ums/page)\r\n",
		              (unsigned)GOLDEN_PAGES, (unsigned)GOLDEN_PAGE_INTERVAL_MS);

		if (xSemaphoreTake(s_w25_lock, portMAX_DELAY) == pdTRUE)
		{
			ok = Golden_EraseImageArea();
			xSemaphoreGive(s_w25_lock);
		}

		if (ok == 0u)
		{
			Golden_Fail("erase block");
			return;
		}
	}

	{
		const uint8_t *src = (const uint8_t *)(GOLDEN_APP_BASE + ((uint32_t)s_page * W25_PAGE_SIZE));
		uint8_t ok = 0u;

		if (xSemaphoreTake(s_w25_lock, portMAX_DELAY) == pdTRUE)
		{
			/* ★★★★★ 【拿到锁之后【再确认一次**✗✓（★ 二次复核后加 ✓）
			 *   · ★ 场景 ✗ `SaveCurrent`（`DISP` 任务）在我【拿到锁之前】
			 *     就已经把【整份备份写完 ✗ 连头都写了】✓ 并且把 `s_filling` 清了 ✓
			 *   · ★★ 那时我再写一页 ✗
			 *     【就是往一份【刚写好的完整备份】上【补一刀】】✓
			 *     （★ 数据相同 ✗ 其实无害 ✗
			 *       ★★★ 但【不该靠"碰巧两个写者写的内容一样"来保证正确】✓✓）
			 *   · ★★★★ 【双重检查 ✗ 拿锁后再看一次标志 ✗ 是关掉这个窗口的最小改法】✓✓
			 *     （★ 和 §13.5 建议的"整趟持锁"【效果等价】✗
			 *       ★★ 但【后台路径【做不到整趟持锁】✗ 500ms/页 × 224 页 = 110 秒 ✓）✓ */
			if (s_filling == 0u)
			{
				xSemaphoreGive(s_w25_lock);
				return;                                 /* ★ 别人已经写完了 ✗ 收工 ✓ */
			}

			ok = W25Q64_WriteData(GOLDEN_ADDR_IMG + ((uint32_t)s_page * W25_PAGE_SIZE), src,
			                      W25_PAGE_SIZE);
			xSemaphoreGive(s_w25_lock);
		}

		if (ok == 0u)
		{
			Golden_Fail("page write");
			return;
		}
	}

	s_page++;
	s_fail = 0u;

	if (s_page >= (uint16_t)GOLDEN_PAGES)
	{
		uint8_t ok = 0u;

		if (xSemaphoreTake(s_w25_lock, portMAX_DELAY) == pdTRUE)
		{
			ok = Golden_WriteHdr(s_my_crc);
			xSemaphoreGive(s_w25_lock);
		}

		if (ok != 0u)
		{
			Serial_Printf("GOLD: backup DONE  size=%u  crc=0x%04X\r\n",
			              (unsigned)GOLDEN_APP_SIZE, (unsigned)s_my_crc);
			s_filling = 0u;
		}
		else
		{
			Serial_Printf("GOLD: header write FAILED (backup invalid)\r\n");
			Golden_Fail("header write");
		}
		return;
	}
}


/*==============================================================================
 *  ★★★★ 【升级前】同步写一份（★ 约 2~3 秒 ✗ 用户主动升级 ✗ 慢一点没关系 ✓）
 *
 *  ★★ 这一步【才是"备份"真正的来源**✗✓：
 *    · ★ 【此刻正在跑的版本 ✗ 是我们【确定它好】的版本】✓
 *    · ★★ 所以【升级之前【先把它存下来 ✗ 再动内部 Flash】】✓✓
 *
 *  ⚠️ ★★★★ 【失败必须如实上报**✗✓：
 *    · ★ 调用者要【把"这次升级没有备份"告诉人】✓
 *    · ★★ 【不能【静默地不带备份就升级】✗ 那是【拿唯一的命去赌】】✓✓
 *      （★ 审计 G6 ✗ 调用方【默认要中止】✗ 除非用户显式强制 ✓）
 *============================================================================*/
uint8_t Golden_SaveCurrent(void)
{
	uint8_t ok;

	if (s_on == 0u)
	{
		Serial_Printf("GOLD: no W25Q64 -> upgrade WITHOUT backup\r\n");
		return 0u;
	}

	/* ★★★★★ 等待窗口【10 秒】（★ 2026-10-14 二次复核后从 3 秒上调 ✓）
	 *
	 *  ★★★ 为什么原来 3 秒【太紧**✗✓：
	 *    · ★ 本函数【自己】拿到锁之后 ✗ 会【一直持有到把整份写完 + 写头】✓
	 *      （★ 见 `Golden_WriteAll` 的约定 ✗ 它【假定调用者持锁】✓）
	 *    · ★★ 而【另一个持锁人可能是 `Golden_Task` 的【块擦】】✗
	 *      【64KB 块擦最坏 2 秒 ✗ 软件 SPI 搬运还要加】✓
	 *    → ★★★ 【3 秒的等待窗口【只比最坏擦除多一点点】✗
	 *      【一旦超时 ✗ `U` 就会因"没有备份"被【中止】（★ G6 ✓）】✓✓
	 *    · ★★★★ 而【多等几秒的代价 ≈ 0**✗
	 *      【升级本来就是人主动发的 ✗ 慢两秒完全无所谓】✓✓
	 *  → ★★★★★ 【宁可等久 ✗ 不要误判"忙"】✓ */
	if (xSemaphoreTake(s_w25_lock, pdMS_TO_TICKS(10000)) != pdTRUE)
	{
		Serial_Printf("GOLD: !! W25Q64 busy -> backup skipped\r\n");
		return 0u;
	}

	Serial_Printf("GOLD: saving current image as backup ...\r\n");
	s_my_crc = Golden_ImageCrc();

	ok = Golden_WriteAll(0u);

	/* ★★★★ 状态收尾（★ 审计 G1 附带发现的那个 bug ✓）✗
	 *   · ★ 成功 → 【后台【不用再填了】】✓
	 *   · ★★ 失败 → 【让后台【从第 0 页重来】】✓✓
	 *     （★ 绝不能"从中间接着写" ✗ 那是【往一份半成品上叠】✓）✓ */
	if (ok != 0u)
	{
		s_filling = 0u;
		s_page    = 0u;
		s_fail    = 0u;
		Serial_Printf("GOLD: backup saved  size=%u  crc=0x%04X\r\n",
		              (unsigned)GOLDEN_APP_SIZE, (unsigned)s_my_crc);
	}
	else
	{
		s_page    = 0u;
		s_filling = 1u;
		Serial_Printf("GOLD: !! backup FAILED\r\n");
	}

	xSemaphoreGive(s_w25_lock);
	return ok;
}


/*==============================================================================
 *  ★★★ 收到 `B` ✗ 清备份 + 把当前版本记成【黑名单】
 *
 *  ★★★★ 审计 G2 的修复 ✗ 【不再无条件擦头】✗✓
 *
 *  ★★★★ 原版错在哪 ✗（★ 灾难场景 ✓）✗：
 *    ```
 *    ① v1 良好运行 ✗ 槽里填的是 v1
 *    ② 发 `U` 升级 ✗ 槽被覆盖为 v1（★ 当前跑的 ✗ 也就是好的 ✓）✗ 升到 v2
 *    ③ v2 有 bug ✗ 用户发 `B` ✗ 【本意是"作废 v2"】
 *    ④ 原版行为 ✗【擦头 ✗ → 槽里的 v1（★ 唯一能救命的 ✓）被毁】+ 黑名单记 v2
 *    ⑤ 结局 ✗【好备份没了 ✗ v2 还在跑 ✗ 【再也回不去】】✓✓
 *    ```
 *  ★★★★★ 修正后的规则 ✗【看"槽里装的是不是【当前这一版】"再决定】✓✓：
 *    · ★ 备份 == 当前版本 → 【它确实就是我们要作废的那个】→ 【擦头】✓
 *    · ★★ 备份 != 当前版本 → 【它是【上一版好固件】✗ 【一个字节都不动】】
 *      → 【只记黑名单】✓✓
 *  ★★★ 无论哪条路 ✗【黑名单都要记】✗
 *    （★ 保证【当前这一版【不会被后台自动填进去】】✓）✓
 *============================================================================*/
void Golden_ClearBackup(void)
{
	GoldenMeta_t m;
	uint16_t     hdr_crc  = 0u;
	uint8_t      have_hdr;
	uint8_t      erased   = 0u;
	uint8_t      i;

	if (s_on == 0u)
	{
		Serial_Printf("GOLD: no W25Q64 -> nothing to clear\r\n");
		return;
	}

	if (xSemaphoreTake(s_w25_lock, pdMS_TO_TICKS(10000)) != pdTRUE)
	{
		Serial_Printf("GOLD: !! W25Q64 busy -> clear skipped\r\n");
		return;
	}

	s_my_crc = Golden_ImageCrc();
	have_hdr = Golden_ReadHdr(&hdr_crc);

	/* ★★★★ 先停掉后台填充 ✗ 【免得它把这一版填进去】✓ */
	s_filling = 0u;

	if ((have_hdr != 0u) && (hdr_crc == s_my_crc))
	{
		/* ★ 槽里【就是当前这一版】✗ 才敢擦头 ✓ */
		if (W25Q64_EraseSector(GOLDEN_ADDR_HDR) != 0u)
		{
			erased = 1u;
		}
		else
		{
			Serial_Printf("GOLD: !! erase header FAILED\r\n");
		}
	}
	else if (have_hdr != 0u)
	{
		/* ★★★★ 【关键分支】✗ 槽里是【别的版本】（★ 大概率是上一版好固件 ✓）
		 *   → 【绝不动它】✓ 用户还能用 `R` 回退 ✓✓ */
		Serial_Printf("GOLD: backup holds ANOTHER version (crc=0x%04X) -> KEEPING it\r\n",
		              (unsigned)hdr_crc);
		Serial_Printf("GOLD: (only blacklisting the running image 0x%04X)\r\n",
		              (unsigned)s_my_crc);
	}
	else
	{
		Serial_Printf("GOLD: no valid backup in slot -> nothing to erase\r\n");
	}

	/* ★★ 黑名单【无论哪条路都要记】✓ */
	m.magic         = 0xFFFFFFFFu;
	m.blacklist_crc = s_my_crc;
	m.backup_crc    = have_hdr;
	m.seq           = 0u;
	for (i = 0u; i < (uint8_t)sizeof(m.reserved); i++)
	{
		m.reserved[i] = 0xFFu;
	}

	if (W25Q64_EraseSector(GOLDEN_ADDR_META) == 0u)
	{
		Serial_Printf("GOLD: !! erase meta FAILED\r\n");
		xSemaphoreGive(s_w25_lock);
		return;
	}
	if (W25Q64_WriteData(GOLDEN_ADDR_META, (const uint8_t *)&m, (uint32_t)sizeof(m)) == 0u)
	{
		Serial_Printf("GOLD: !! write meta FAILED\r\n");
		xSemaphoreGive(s_w25_lock);
		return;
	}
	{
		uint32_t mg = GOLDEN_META_MAGIC;
		(void)W25Q64_WriteData(GOLDEN_ADDR_META, (const uint8_t *)&mg, 4u);
	}

	s_black = s_my_crc;
	xSemaphoreGive(s_w25_lock);

	if (erased != 0u)
	{
		Serial_Printf("GOLD: backup CLEARED, crc=0x%04X blacklisted\r\n", (unsigned)s_my_crc);
	}
	else
	{
		Serial_Printf("GOLD: blacklisted crc=0x%04X (backup left intact)\r\n", (unsigned)s_my_crc);
	}
	Serial_Printf("GOLD: (flash the good version -> it will back itself up)\r\n");
}


/*==============================================================================
 *  ★★★★ 收到 `R` ✗ 请求回退（★ 真正的恢复由 Boot 做 ✓）
 *    ★ 这里【只置标志 + 软复位】✗ 【一个字节都不写内部 Flash】✓
 *    ★★ 因为【回退 = 用备份覆盖 App 区 ✗
 *      【那件事【必须在 Boot 里做】✗ App 自己【做不了（正被覆盖）】】✓✓
 *
 *  ★★★ 审计 G4 的修复 ✗【复位前先确认备份真的存在】✓✓
 *    · ★ 原版【不问青红皂白就复位】✗
 *      → 【空槽时按 `R` = 【白复位一次 ✗ 而且会让人以为"回退过了"】】✓
 *    · ★★ 现在【先看 Dump 状态】✗ 没有有效备份就【就地拒绝 + 说清原因】✓✓
 *      （★ 比"复位后 Boot 再发现没得恢复"【清楚得多】✓）
 *============================================================================*/
void Golden_RequestRollback(void)
{
	uint16_t hdr_crc = 0u;
	uint8_t  have;

	if (s_on == 0u)
	{
		Serial_Printf("\r\nGOLD: ROLLBACK refused -> no W25Q64\r\n");
		return;
	}

	if (xSemaphoreTake(s_w25_lock, pdMS_TO_TICKS(1000)) == pdTRUE)
	{
		have = Golden_ReadHdr(&hdr_crc);
		xSemaphoreGive(s_w25_lock);
	}
	else
	{
		Serial_Printf("\r\nGOLD: ROLLBACK refused -> W25Q64 busy\r\n");
		return;
	}

	if (have == 0u)
	{
		Serial_Printf("\r\nGOLD: ROLLBACK refused -> no valid backup in slot\r\n");
		return;
	}

	Serial_Printf("\r\nGOLD: ROLLBACK requested -> resetting into Bootloader\r\n");
	Serial_Printf("GOLD: will restore backup crc=0x%04X (running 0x%04X)\r\n",
	              (unsigned)hdr_crc, (unsigned)s_my_crc);

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) { }

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	PWR_BackupAccessCmd(ENABLE);
	BKP_WriteBackupRegister(BOOT_ROLLBACK_DR, BOOT_ROLLBACK_MAGIC);

	NVIC_SystemReset();

	for (;;) { }                                     /* ★ 不会到这儿 ✓ */
}


/*==============================================================================
 *  ★★ 备份状态（★ 给人看的 ✓）
 *    0 = 无备份 ✗ 1 = 有备份且和当前版本一致 ✗ 2 = 有备份但是别的版本 ✓
 *============================================================================*/
uint8_t Golden_BackupState(void)
{
	uint16_t hdr_crc = 0u;
	uint8_t  have;

	if (s_on == 0u) { return 0u; }

	if (xSemaphoreTake(s_w25_lock, pdMS_TO_TICKS(1000)) != pdTRUE) { return 0u; }
	have = Golden_ReadHdr(&hdr_crc);
	xSemaphoreGive(s_w25_lock);

	if (have == 0u) { return 0u; }
	return (hdr_crc == s_my_crc) ? 1u : 2u;
}
