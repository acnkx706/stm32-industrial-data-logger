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
 *      （★ 原版一口气写 **448 页** ✗ 同优先级的 `task_SD` 被饿 1~3 秒 ✓）
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

#define GOLDEN_PAGES    (GOLDEN_APP_SIZE / W25_PAGE_SIZE)     /* ★ = 448 页 ✓
                                                               *   （112KB / 256B ✗
                                                               *    原注释写 224 ✗ 是旧 56KB 值 ✓）*/
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
/* ★★ 2026-10-19 加：互斥量的【静态控制块】（★ RAM 优化 R2 ✗ 不再从堆里分配 ✓）*/
static StaticSemaphore_t s_w25_mutex_cb;


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
 *    · ★ **现在就是 CBT6 / 112KB** ✗
 *      `ceil((0x1000 + 112KB) / 64KB)` = **2 块** ✓✓
 *      （★ 原注释分两句写"现在 1 块 / 换 CBT6 后 2 块"✗
 *        ★★ 而板上【早就是 CBT6】了 ⇒ 【两句都过期】✓）
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
		 *
		 *  ★★★★★ 2026-10-18 补记（⑤ 用户拍板 A）：**喂狗的语义要说清** ✗✓
		 *
		 *  ★ 事实 ✗：全工程喂狗点【有 3 个】✓
		 *    ① 启动 IWDG 前（`freertos_demo.c`）
		 *    ② 空闲钩子（常规）
		 *    ③ **就是这里**（备份时每 16 页）
		 *
		 *  ★★★ 后果（必须承认）✗：
		 *    · 只要【备份任务还在转圈】✗
		 *      【哪怕系统其它部分全死了，狗照样被喂】✓✓
		 *    · ⇒ 「喂狗 = 系统还活着」这个判据【在这里被削弱】✓
		 *    · ★★ 而 Boot 的自动回退判据⑤（IWDG 复位）**正是靠它**发现 App 卡死 ✓
		 *
		 *  ★★★★ 为什么不干脆去掉这一口 ✗（★ 不要"看起来更纯"就删 ✓）：
		 *    · 备份是【0.6~4 秒忙等】✗ 期间空闲钩子【根本跑不到】✓
		 *    · ★★ 去掉 ⇒ 【备份时必然被狗咬】⇒ 复位循环 ⇒ 比污染判据糟糕得多 ✓✓
		 *
		 *  ⇒ ★ 所以正确的说法是（★ 请按这个理解，别按字面）✗：
		 *    **喂狗 = "关键路径还活着"，不是"整机每处都正常"。**
		 *    · 判据⑤真正能抓到的：**连关键路径都停摆了**（死循环/硬件挂死）✓
		 *    · 判据⑤抓不到的：**备份期间某一处逻辑卡住** ✓
		 *      → 那种情况由【任务自身的超时/失败计数】负责 ✓（本项目已有 ✓）
		 *  ★★ 这也解释了为什么"3 个喂狗点"是【权衡后保留】✗ 而不是疏忽 ✓
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
 *  ★★★★★ 2026-10-18 加：**掉卡缓存**（★ 对外说明见 golden.h ✓）
 *
 *  ★ 一句话 ✗：SD 写不进去的时候，把【数据批】和【事件行】先存进 W25，
 *    等卡好了再按顺序搬回卡里 ✓✓
 *
 *  ★★ 为什么能成立 ✗：W25 走【软件 SPI（PA8/PA11/PA12/PB13）】✓
 *    SD 走【硬件 SPI1（PA5/PA6/PA7/PA4）】⇒ 【两条独立的线】✓✓
 *
 *  ★★★ v1 的取舍（★ 简单优先 ✓）：
 *    · **满了就停**（不覆盖旧项）⇒ 绝不丢掉已缓存的数据 ✓
 *    · 全部回写完成后【指针归零】⇒ 下次断卡从第一个扇区重新擦写 ✓
 *    · 指针页【每次写前先擦】✗ 代价 45 ms 一次 ✗ 换来"掉电也认得出"✓✓
 *      （★ 不这么做的话：断电 ⇒ 指针失效 ⇒ 缓存数据变成"没人认领"✓）
 *
 *  ★ 调用者约定 ✗：**先 Peek → 写卡成功 → 再 Pop** ✓
 *    （★ 反过来会出现"指针前进了但数据没写进去"⇒ 丢 ✓）
 *============================================================================*/
/*============================================================================
 *  ★★★★★ 2026-10-19 改：**扩容 + 环形覆盖**（★ 整夜实测推动 ✓）
 *
 *  ★ 为什么必须改 ✗（★ 一整夜的实测数据 ✓）：
 *    · 2026-10-19 整夜：卡从开机 30 秒就坏 ✗ 而缓存【开机 21 分钟就满了】✓
 *    · ★★ 满了之后到早上，抽了 40,128 次队列【全部失败】✓
 *      ⇒ **整夜约 8 小时、约 288,000 条数据全丢** ✗✗
 *    · ★★★ 根因两条 ✗：
 *      ① 容量太小（2048 × 1024 B = 2 MB ✗ 而 W25 有 8 MB ✓）
 *      ② 满了就【停】✗ ⇒ 后面的数据一点都保不住 ✓✓
 *
 *  ★ W25 完整内存图（★ 改地址前必须对着它看 ✓）：
 *      0x000000   4 KB   金副本头（GOLDEN_ADDR_HDR）
 *      0x001000 112 KB   金副本镜像（GOLDEN_ADDR_IMG ✗ 到 0x1D000 止 ✓）
 *      0x020000   4 KB   金副本元数据（GOLDEN_ADDR_META）
 *      0x021000   4 KB   ★ 缓存：数据指针日志 A（DC_DATA_HDR_A ✓）
 *      0x022000   4 KB   ★ 缓存：数据指针日志 B（DC_DATA_HDR_B ✓）
 *      0x030000   4 KB   （★ 旧数据指针页 ✗ 2026-10-20 G5 起【不再读写】✓）
 *      0x031000 7.43 MB  ★ 缓存：数据槽（★ 7,616 × 1024B ✗ 到 0x7A1000 ✓）
 *      0x7A1000   4 KB   （★ 旧事件指针页 ✗ 2026-10-20 G5 起【不再读写】✓）
 *      0x7A2000 240 KB   ★ 缓存：事件槽（★ 960 × 256B ✓）
 *      0x7DE000   4 KB   ★ 缓存：事件指针日志 A（DC_EVT_HDR_A ✓）
 *      0x7DF000   4 KB   ★ 缓存：事件指针日志 B（DC_EVT_HDR_B ✓）
 *      0x7E0000 124 KB   余量
 *      0x7FF000   4 KB   自检扇区（★ W25_SELFTEST_ADDR ✗ 绝不许碰 ✓）
 *
 *  ★★ 为什么事件区要挪到高位 ✗：数据区要吃掉中间那 5.5 MB 空闲 ✓
 *    · 旧事件区（0x240000）正好落在【新数据区】里 ✗
 *    · ★ 所以把它整体挪到 0x7A1000 ✓ 数据区一口气吃到 0x7A1000 ✓✓
 *
 *  ★★★ 容量换算（★ 实测批大小 ✓）：
 *    · 实测【6 条/批】✗ 7616 批 × 6 = **45,696 条** ✓
 *    · 记录率 10 条/秒 ⇒ **约 76 分钟**（旧版只有 20 分钟 ✓）
 *  ★★★★ 环形覆盖 ✗：满了【丢最旧的】✗ 保住"最近的" ✓
 *    · 取舍说清楚 ✗：宁可丢 2 小时前的 ✗ 也要保住刚才的 ✓✓
 *      （★ 断卡场景下"最近的数据"才是要救的 ✓）
 *    · ★ 溢出次数会累计在头里（`rsv` 字段 ✓）并打到串口 ✓
 *
 *  ★ 兼容性 ✗（★ 重要 ✓）：
 *    · 数据区 base/item 都没变 ✗ 只是变长
 *      ⇒ **旧缓存里还没回写的数据【照样能读出来回写】** ✓✓
 *    · ★ 但事件区换了地址 ⇒ 旧事件区里没回写的诊断行会【被新数据覆盖】✗
 *      （★ 只影响 `# LOST` 这类诊断行 ✗ 不影响记录数据 ✓）
 *============================================================================*/
/*==============================================================================
 *  ★★★★★ 2026-10-20 G5：**指针页 → 指针日志（A/B 乒乓）**
 *
 *  ★★ 原来的做法（已废弃 ✗ 但必须记下为什么 ✓）：
 *     `Dc_SaveHdr()` = `W25Q64_EraseSector(0x030000)` + 写回 16 B ✓
 *     · 每存一批（`Dc_Put`）擦一次 ✗ 每取走一批（`Dc_Pop`）再擦一次 ✓
 *     · = **为了 16 字节，擦掉 4,096 字节**（写放大 256×）✓
 *
 *  ★★★ 两笔账（★ 这才是要改的理由 ✓）：
 *    ① **寿命** ✗ 断卡时 1.67 批/秒 ⇒ 数据路径 **3.3 次擦/秒**
 *       ✗ 再叠加事件路径（`Loss_Flush` 每轮一条 `# LOST` ⇒ 再擦一次）✓
 *       W25Q64 每扇区 **10 万次**擦除 ⇒ 断卡 **1~3 小时**就把 0x030000 写穿 ✓✓
 *    ② **掉电窗口** ✗ `先擦后写` 之间要 **45~400 ms**（扇区擦除 typ 45 ms ✓）
 *       ⇒ 这 45 ms 里掉电 = 【唯一一份指针没了】⇒ 开机指针归零
 *       ⇒ **7,616 批（≈76 分钟）缓存被静默作废** ✓✓✓（★ 比寿命更要命 ✓）
 *
 *  ★★★★ 现在的做法（★ FlashDB 的思路 ✓）：
 *    · 指针改成【追加写】✗ 一条记录 16 B ✗ 一个 4 KB 扇区装 **256 条** ✓
 *    · 两个扇区 **A/B 乒乓** ✗ A 写满 256 条 ⇒ 擦 B ⇒ 从 B 第 0 条继续 ✓
 *    · 记录里带 **`seq`（单调递增）** ⇒ 开机扫 A/B，取 **seq 最大且 CRC 对**的 ✓
 *    · ⇒ 擦除率 = **1/256**（断卡 10 万次擦除 ≈ 20 天连轴 ✓）
 *      ✗ 掉电最多丢 **1 批**（上一条记录还在扇区里 ✓ 读得回来 ✓）
 *    · ⇒ 擦除延时也从"每批 45 ms 占着 W25 锁"变成"每 256 批一次"✓
 *
 *  ★ 为什么不学 FlashDB TSDB 的"不存指针、开机扫槽头重建"：
 *    · 那要在 7.43 MB 里按扇区读 **1,904 个扇区头** ✗ 开机多花约 0.2 s ✓
 *    · ★ 而本工程的指针只有 `wr`/`rd` 两个 16 位数 ✗ 日志方案已经够省 ✓
 *      ⇒ 留作"真零堆"那一轮再评估 ✓
 *
 *  ★★ 升级代价（★ 必须说清楚 ✓）：
 *    · `DC_MAGIC_D` 升 `DCD4` → **`DCD5`** ✗ 旧指针页【直接当不存在】✓
 *      ⇒ 升级那一刻缓存里还没回写的批【会丢】✗ 所以**升级前要确认 `pending=0`** ✓
 *    · ★ 旧的两个指针页（0x030000 / 0x7A1000）从此【不再读写】✗ 也不要挪作他用 ✓
 *============================================================================*/
#define DC_DATA_HDR_A  0x021000u        /* ★ 数据指针日志 A（独占一个 4KB 扇区）✓ */
#define DC_DATA_HDR_B  0x022000u        /* ★ 数据指针日志 B（★ A 满了擦它 ✓）*/
#define DC_DATA_BASE   0x031000u        /* ★ 扇区对齐 ✗ 数据槽起点 ✓ */
/* ★★ 槽必须 ≥ 512+4 ✗ 而且要【扇区整除】✓
 *   · 一批最多 512 B ⇒ 加 4 B 长度前缀 = 516 ✗ 512 槽装不下 ✓
 *   · ★ 选 1024（= 4 页 = 1/4 扇区）⇒ 【4 项一个扇区】✓ 擦除判断简单 ✓
 *   · ★★★ 2026-10-19：2048 → **7616**（3.72×）
 *     · 7616 × 1024 = 7,798,784 B ✓ 正好铺到 0x7A1000 ✓
 *     · 7616 % 4 == 0 ⇒ 【扇区整除】✓ 擦除判断不用改 ✓✓ */
/* ★★★★★ 2026-10-19：**G2 + G10 已修并验证完** ⇒ 环【改回正式值 7616**✓✓
 *  ★ 复现/验证的经过（★ 留着当教材 ✓）：
 *    · 临时把本值改成 **32u** + `DC_MAGIC_D` 升到 `DCD3` ⇒ 出 `ver 28t`
 *    · ★★ 拔卡 318 秒 ⇒ 环满 ⇒ 卡回来【回写一声不响地什么都没做】
 *      （`pending` 冻在 32 ✗ `replay done` 永不出现 ✓）= **G2+G10 复现** ✓
 *    · 修完再跑同一实验 ✗ `ovr` 改成"每次 +4"✗ `BAD CACHE ITEM` **0 次**
 *      ✗ `replay done 30 batches / 168 recs` ✗ `pending` 归 0 ✓✓
 *  ★★ 所以这里恢复 7616u ✗ 并把 `DC_MAGIC_D` 再升一版（`DCD4`）✓
 *     （★ 取模基数变了 ✗ 旧指针必读错位 ✓ 这是 ver 17 的血泪 ✓）*/
#define DC_DATA_SLOTS  7616u            /* ★ 正式值（★ 测试时曾临时用 32 ✓）*/
#define DC_DATA_ITEM   1024u
#define DC_EVT_HDR_A   0x7DE000u        /* ★ 事件指针日志 A（★ 原 0x7A1000 已弃用 ✓）*/
#define DC_EVT_HDR_B   0x7DF000u        /* ★ 事件指针日志 B ✓ */
#define DC_EVT_BASE    0x7A2000u
#define DC_EVT_SLOTS   960u             /* 960 × 256B = 240 KB ✓ */
#define DC_EVT_ITEM    256u

/* ★ 指针日志的几何（★ 数据区和事件区共用同一套 ✓）：
 *   · 一条记录 = `sizeof(DcHdr_t)` = 16 B（★ 下面有编译期断言 ✓）
 *   · 一个扇区 = 4,096 / 16 = **256 条** ✓
 *   ★ 别乱改这两个数 ✗ 改了等于换了日志格式 ⇒ **必须同时升 magic** ✓
 *     （★ 否则旧记录会被当成"CRC 对的新记录"读过来 ✓）*/
#define DC_LOG_SEC_SZ  4096u
#define DC_LOG_REC_SZ  16u
/* ★★★ 2026-10-20 测试开关：**注入一条坏记录**（★ 正式版必须是 0 ✓）
 *   · 打开时：若开机扫到的写游标在 [1, N) ⇒ 往下一格灌 16 个 0x00
 *     ⇒ 之后那条记录永远 CRC 不对（★ 模拟"写这 16 B 时掉电" ✓）
 *   · 用途：验证 `Dc_RecBlank()` 那条分支（★ 认出坏记录 ⇒ 换扇区 ✓）
 *   · ★★ 为什么不做成"跟 N==8 绑"✗：N=8 时一个扇区 **7 秒就写满**
 *     ⇒ 还没来得及复位就已经翻面、坏记录被盖过去 ✓
 *     ⇒ 必须用**正式容量 256**（★ 窗口 ≈ 4 分钟 ✓）*/
#define DC_LOG_INJECT_TEST 0u
#define DC_LOG_N       256u            /* ★ = DC_LOG_SEC_SZ / DC_LOG_REC_SZ ✓ */

#define DC_MAGIC_D     0x44434435u      /* ★★★ 'DCD6' **测试版专用**（★ 正式版是 DCD5 ✓）
                                          *
                                          *  ★ 为什么每次改槽数都要升一版 ✗：
                                          *    · 槽号是【取模算的】✗ 基数一变 ⇒ 旧指针读出来【全错位】✓
                                          *    · ★★ 换 magic ⇒ `Dc_LogLoad` 扫不到旧记录
                                          *      ⇒ 【指针自动归零 ⇒ 缓存清空重来】✓✓
                                          *    · ★★★ 2026-10-19：DCD3（32 槽测试）→ **DCD4（正式 7616）**
                                          *    · ★★★ 2026-10-20：**DCD4 → DCD5**（G5：指针页 → 指针日志 ✓）
                                          *
                                          *  ★★★★ DCD1 → DCD2 那次的血泪（★ 留着当教材 ✓）：
                                          *    · ver 17 把【槽数从 2048 改成 7616】✗
                                          *        旧数据：`idx = wr % 2048`
                                          *        新代码：`idx = rd % 7616`
                                          *      ⇒ **映射完全变了 ⇒ 读出来的位置全错** ✓✓
                                          *    · ★★★ 实测后果 ✗：
                                          *        `pending=2488` 但 `PeekBatch` **一直返回 0**
                                          *        ⇒ `rd` 永不前进 ⇒ **回写永久卡死** ✓✓✓
                                          *      （★ 长度前缀是垃圾 ⇒ `n > 512` ⇒ 判坏项 ✓）
                                          *  ★ 代价 ✗：旧缓存里那 2488 批【作废】
                                          *    （★ 反正它们本来就【读不出来】✗ 作废 = 止损 ✓）*/
#define DC_MAGIC_E     0x45434532u      /* ★ 'ECE2'（★ 2026-10-20 G5：事件指针也改日志 ✓）*/

/* ★★★ 指针记录（★ 一条 = 一个日志槽 ✓）
 *   ★ 为什么加 `seq` ✗：A/B 两个扇区里【都有记录】时
 *     · 只能靠"谁更新"判 ⇒ 必须有一个【单调递增】的数 ✓
 *     · ★ 不能靠"记录在扇区里的位置"✗ 两个扇区的 pos 各自独立 ✓
 *   ★ 为什么 seq 放第 2 个字段（★ 紧接 magic ✓）：
 *     · 4 字节自然对齐 ⇒ 整个结构 = 4+4+2+2+2+2 = **16 B** ✗ 正好一条日志 ✓
 *     · ★ 若按"magic, wr, rd, rsv, seq, crc"排 ⇒ 中间要插 2 B 填充 = 18 B
 *       ⇒ 4 KB 装不下整数条 ⇒ 白浪费 ✓✓ */
typedef struct {
	uint32_t magic;
	uint32_t seq;                   /* ★ 单调递增 ✗ 开机取最大 ✓ */
	uint16_t wr;
	uint16_t rd;
	uint16_t rsv;
	uint16_t crc;                   /* ★ 覆盖上面 14 B（★ 算时先把自己清零 ✓）*/
} DcHdr_t;

/* ★★ 编译期断言（★ 比运行期断言好 ✗ 一字不花 ✓）：
 *   结构一旦不再是 16 B ⇒ **直接编不过** ✗ 免得悄悄改了日志格式 ✓ */
typedef char Dc_LogRecSizeOk[(sizeof(DcHdr_t) == DC_LOG_REC_SZ) ? 1 : -1];

/* ★ 日志写游标（★ 每区一个 ✓）
 *   · `pos` = 下一条写到第几条 ✗ 可以等于 256（★ 表示"该换扇区了"✓）
 *   · `sec` = 0 用 A / 1 用 B ✓ */
typedef struct {
	uint16_t pos;
	uint8_t  sec;
	uint8_t  pad;
} DcLog_t;

static DcHdr_t s_dc_d;                  /* 数据批区 */
static DcHdr_t s_dc_e;                  /* 事件行区 */
static DcLog_t s_log_d;                 /* 数据区日志游标 */
static DcLog_t s_log_e;                 /* 事件区日志游标 */
static uint8_t s_dc_on = 0u;

/* ★ 诊断用（★ 只在上电后累计 ✗ 掉电清零 ✓）：
 *   · 它是"这次开机擦了几次指针扇区"✗ 直接证明 G5 改对了 ✓
 *   · ★ 原来每存一批就 +2（Put 一次 + Pop 一次）✗ 现在每 256 批才 +1 ✓ */
static uint16_t s_dc_hdr_erases = 0u;
static uint16_t s_dc_evt_erases = 0u;

static uint16_t Dc_HdrCrc(const DcHdr_t *h)
{
	DcHdr_t t = *h;
	t.crc = 0u;
	return Crc16_Kermit((const uint8_t *)&t, (uint32_t)sizeof(t));
}

/* ★★★★★ 2026-10-19 加（★ 修审计 G2 用 ✓）：一个扇区（4 个槽）里【还有没有未回写的项】✓
 *
 *  ★ 为什么需要它 ✗：
 *    · W25 是 NOR ✗ 【写之前目标必须已经是 0xFF】✓
 *      ⇒ 进新扇区【只能整扇区擦】（4KB ✗ 没有按槽擦这种操作 ✓）
 *    · ★★ 而一擦就是 4 个槽 ✗ 若里面【还有 `rd` 没消费的项】
 *      ⇒ 那几项【被静默抹掉】✓✓
 *    · ★★★ 实测（32 槽测试版 ✗ 2026-10-19）✗：
 *      满环后卡回来 ⇒ `PeekBatch` 读到 0xFF（长度前缀 0xFFFF）
 *      ⇒ 返回 0 ✗ `rd` 永不前进 ⇒ **32 批缓存被无声遗弃** ✓✓
 *
 *  ★★ 判据 ✗：某槽有效 ⇔ 它的【绝对号】落在 `[rd, wr)` 这个窗口里 ✓
 *    · 窗口长度 ≤ slots（★ 由"满了就覆盖最旧"保证 ✓）
 *    · ★ 所以用【相对 rd 的取模偏移】判 ✗ 不用绝对号 ✓
 *
 *  ★ 返回 ✗：1 = 该扇区里【还有有效数据】（★ 擦之前必须先处置 ✓）
 *            0 = 4 个槽都空（★ 可以安全擦 ✓）*/
static uint8_t Dc_SectorHasData(const DcHdr_t *h, uint16_t slots, uint16_t s0)
{
	uint16_t used = (uint16_t)(h->wr - h->rd);      /* ★ 有效项数（≤ slots ✓）*/
	uint16_t base = (uint16_t)(h->rd % slots);      /* ★ rd 落在哪个槽 ✓ */
	uint8_t  i;

	if (used == 0u) { return 0u; }

	for (i = 0u; i < 4u; i++)
	{
		uint16_t s   = (uint16_t)((s0 + i) % slots);
		uint16_t off = (uint16_t)((s + slots - base) % slots);
		if (off < used) { return 1u; }              /* ★ 这一槽还有效 ✗ 不能擦 ✓ */
	}
	return 0u;
}

static void Dc_HdrDefault(DcHdr_t *h, uint32_t magic)
{
	h->magic = magic; h->seq = 0u; h->wr = 0u; h->rd = 0u; h->rsv = 0u;
	h->crc   = Dc_HdrCrc(h);
}

/* ★★★★★ 读一条指针记录（★ 三道关：读得回 / magic 对 / CRC 对 / 数据合理 ✓）
 *   ★ 为什么 CRC 要盖住 `seq` ✗：seq 是"谁更新"的唯一依据
 *     · 若 seq 不受 CRC 保护 ⇒ 一个坏字节就能让**旧记录被当成新的**
 *       ⇒ `rd`/`wr` 倒退 ⇒ **重复回写**（轻）或**跳过待回写数据**（重）✓✓
 *   ★★ 2026-10-20 改：**无论有效与否都把读到的 16 B 交出去** ✓
 *     · 因为调用方还要判"这条是**空**（0xFF）还是**坏**（写过但 CRC 不对）"
 *     · ★★★ 这两种情况的处置**完全不同**（见 `Dc_LogLoad` 的注释 ✓）
 *   ★ 返回 1 = 这条有效 ✓ / 0 = 无效 ✓ */
static uint8_t Dc_LogRead(uint32_t addr, DcHdr_t *out, uint32_t magic, uint16_t slots)
{
	if (W25Q64_ReadData(addr, (uint8_t *)out, DC_LOG_REC_SZ) == 0u)
	{
		/* ★ 读失败 ⇒ 填成"非空"（0x00）✗ 让调用方当成【坏记录】处理 ✓
		 *   （★ 宁可少用半扇区 ✗ 也不要在读不到的地方原地写 ✓）*/
		uint16_t i;
		for (i = 0u; i < DC_LOG_REC_SZ; i++) { ((uint8_t *)out)[i] = 0x00u; }
		return 0u;
	}
	if ((out->magic != magic) || (out->crc != Dc_HdrCrc(out))) { return 0u; }
	if ((uint16_t)(out->wr - out->rd) > slots)                { return 0u; }
	return 1u;
}

/* ★ 一条记录是不是"空的"（★ 全 0xFF = 擦除态 ✓）
 *   ★ 为什么必须区分"空"和"坏"✗ —— 这就是这一轮的**第二号坑** ✓：
 *     · **空** ⇒ 只是写到这儿为止 ✗ 接着往后写就行 ✓
 *     · **坏** ⇒ 那 16 B 里已经有"0 位"了 ✗★★ NOR 只能把 1 写成 0，
 *        **不能把 0 写回 1** ✓ ⇒ 在同一位置重写，新数据被旧 0 位"与"掉
 *        ⇒ 这条记录**永远 CRC 不对** ⇒ 每次开机都从这一格重来
 *        ⇒ **缓存从此永久失效**（★ 而且一条错误打印都没有 ✓✓✓）
 *   ★ 什么时候会产生"坏"记录 ✗：掉电正好落在**写那 16 B 的 0.7 ms 里** ✓
 *     （★ W25 页编程 typ 0.7 ms / max 3 ms ✓）
 *   ⇒ ★★ 处置：**换扇区**（擦另一个，从它的第 0 条继续）✓
 *      · 代价 = 这一扇区剩下的格子不要了（≤ 255 条 ✗ 一辈子可能只遇一次 ✓）
 *      · ★ 收益 = 缓存**不会**静默死掉 ✓✓ */
static uint8_t Dc_RecBlank(const DcHdr_t *t)
{
	const uint8_t *p = (const uint8_t *)t;
	uint16_t i;

	for (i = 0u; i < DC_LOG_REC_SZ; i++)
	{
		if (p[i] != 0xFFu) { return 0u; }
	}
	return 1u;
}

/* ★★★★★ 开机：扫 A/B 两个扇区，重建指针 + 写游标（★ 取代旧的 `Dc_LoadHdr` ✓）
 *
 *  ★★ 为什么"扫到第一条无效的就停"✗：
 *    · 记录是**从第 0 条连续往后追加**的 ✗ 中间不会有空洞 ✓
 *      （★ 换扇区时是"擦干净→从 0 写"✗ 从不回填 ✓）
 *    · ⇒ 一条无效 = 这个扇区到此为止 ✓（★ 最多读 512 次 ✗ 通常 2~3 次 ✓）
 *
 *  ★★★★ 停下来的那一条要分两种（★ 2026-10-20 加 ✗ 这是关键的一步 ✓）：
 *    · **空（0xFF）** ⇒ 正常到底 ✗ 下一条就写在这儿 ✓
 *    · **坏（写过但 CRC 不对）** ⇒ 【绝不能原地重写】（★ 见 `Dc_RecBlank` 注释）
 *      ⇒ 记下"这个扇区已被写坏"✗ 游标直接推到 256 ⇒ 下一次保存**换扇区** ✓✓
 *
 *  ★★ 找不到任何有效记录怎么办 ✗：
 *    · 当"缓存为空"✗ 并**把 A 擦干净**⇒ 从第一条开始追加 ✓
 *    · ★ 这一擦【只为满足"写前必是 0xFF"】✗ 不是每次都擦 ✓
 *    · ★★ 什么时候会走到这里：① 全新片子 ② 刚升 magic（★ 本次 DCD4→DCD5 ✓）
 *      ③ 两个扇区都被写坏（★ 极不可能 ✓）
 *
 *  ★★★ 完整性的边界（★ 说清楚 ✗ 别夸大 ✓）：
 *    · 掉电最多丢【最后一条】记录 ⇒ 最多重搬 1 批（Pop）或漏 1 批（Put）✓
 *    · ★ 再也不会出现"擦扇区中途掉电 ⇒ 76 分钟缓存全废"✓✓ */
static void Dc_LogLoad(DcHdr_t *h, uint32_t sec_a, uint32_t sec_b,
                       uint32_t magic, uint16_t slots, DcLog_t *log)
{
	DcHdr_t  t;
	uint8_t  found    = 0u;
	uint8_t  best_sec = 0u;
	uint16_t best_pos = 0u;
	uint8_t  bad[2];                    /* ★ 每个扇区"停下来的那一条是坏的吗"✓ */
	uint8_t  s;
	uint16_t p;

	log->sec = 0u;
	log->pos = 0u;
	bad[0]   = 0u;
	bad[1]   = 0u;

	for (s = 0u; s < 2u; s++)
	{
		uint32_t sec = (s == 0u) ? sec_a : sec_b;

		for (p = 0u; p < DC_LOG_N; p++)
		{
			if (Dc_LogRead(sec + ((uint32_t)p * DC_LOG_REC_SZ), &t, magic, slots) == 0u)
			{
				/* ★ 不是空 ⇒ 这条被写坏过 ⇒ 这个扇区不能再原地往后写 ✓ */
				if (Dc_RecBlank(&t) == 0u) { bad[s] = 1u; }
				break;
			}
			if ((found == 0u) || (t.seq >= h->seq))
			{
				*h        = t;          /* ★ seq 更大 ⇒ 这条更新 ✓ */
				found     = 1u;
				best_sec  = s;
				best_pos  = (uint16_t)(p + 1u);   /* ★ 下一条写在它后面 ✓ */
			}
		}
	}

	if (found == 0u)
	{
		Dc_HdrDefault(h, magic);
		if (W25Q64_EraseSector(sec_a) != 0u)
		{
			if (magic == DC_MAGIC_D) { s_dc_hdr_erases++; }
			else                     { s_dc_evt_erases++; }
		}
		log->sec = 0u;
		log->pos = 0u;
		return;
	}

	log->sec = best_sec;

	/* ★★★ 停在"坏记录"上 ⇒ 游标推到 256（★ 下一次保存就换扇区 ✓）
	 *   ★ 为什么不是"擦本扇区"✗：本扇区里躺着**最新的那份指针**
	 *     ⇒ 一擦就把 `wr`/`rd` 丢了 ✓✓ */
	if (bad[best_sec] != 0u)
	{
		log->pos = (uint16_t)DC_LOG_N;
	}
	else
	{
		log->pos = best_pos;
	}
}

/* ★★★★★ 存一条指针记录（★ 追加写 ✗ 满了才擦另一个扇区 ✓）
 *
 *  ★★ 顺序【故意】是"先擦另一个、再往当前写"✗：
 *    · 当前扇区里躺着**最新的那份指针** ✗ 全程不动它 ✓
 *    · 擦的是**另一个**（它的记录全都更旧）⇒ 擦掉不影响正确性 ✓
 *    · ★★★ 于是任何时刻**至少有一个扇区持有一份完整可用的指针** ✓✓
 *      （★ 这就是"45 ms 掉电窗口"被消灭的原因 ✓）
 *
 *  ★★ 失败语义（★ 必须和 `Dc_Put` / `Dc_Pop` 的"回滚"配合 ✓）：
 *    · 没写住 ⇒ 返回 0 ✗ `seq` 回退（**写进去的才算数** ✓）
 *    · `Dc_Put` 见 0 ⇒ `h->wr--` ✓（★ 那批数据成了孤儿槽 ✗ 会被下次覆盖 ✓）
 *    · `Dc_Pop` 见 0 ⇒ 不理 ✓（★ 指针没前进 ⇒ 最多重搬一次 ✓ 不丢 ✓）*/
static uint8_t Dc_LogSave(DcHdr_t *h, uint32_t sec_a, uint32_t sec_b, DcLog_t *log)
{
	uint32_t sec;
	uint32_t other;

	if (log->pos >= (uint16_t)DC_LOG_N)         /* ★ 本扇区 256 条写满 ⇒ 换边 ✓ */
	{
		other = (log->sec == 0u) ? sec_b : sec_a;
		if (W25Q64_EraseSector(other) == 0u) { return 0u; }
		if (h->magic == DC_MAGIC_D) { s_dc_hdr_erases++; }
		else                        { s_dc_evt_erases++; }
		log->sec = (uint8_t)(log->sec ^ 1u);
		log->pos = 0u;
	}

	sec = (log->sec == 0u) ? sec_a : sec_b;

	h->seq = (uint32_t)(h->seq + 1u);
	h->crc = Dc_HdrCrc(h);

	if (W25Q64_WriteData(sec + ((uint32_t)log->pos * DC_LOG_REC_SZ),
	                     (const uint8_t *)h, DC_LOG_REC_SZ) == 0u)
	{
		h->seq = (uint32_t)(h->seq - 1u);       /* ★ 没写住 ⇒ 序号回退 ✓ */
		return 0u;
	}

	log->pos = (uint16_t)(log->pos + 1u);
	return 1u;
}

/* 写一项 ✗ 返回 1=成功 / 0=写失败 ✓
 *  ★★★ 2026-10-19 改：**不再返回 2（满）** ✗
 *    · 满了改成【环形覆盖最旧的】✓ 见下面注释 ✓
 *    · ★ 所以调用者只判 `!= 1u` 就是"真失败"✓ 语义更干净 ✓
 *  ★ `len_prefix` = 1 ⇒ 先写 4 字节【实际长度】再写数据 ✓
 *    ★★ 为什么必须带长度 ✗：槽是固定 1024 B ✗ 而一批可能只有 400 B ✓
 *      不留长度的话，回写时【把槽里残留的旧字节一起写进 CSV】✓✓ */
static uint8_t Dc_Put(uint32_t sec_a, uint32_t sec_b, DcLog_t *log,
                      uint32_t base, uint16_t slots, uint16_t item,
                      DcHdr_t *h, const uint8_t *src, uint16_t len, uint8_t len_prefix)
{
	uint16_t idx;
	uint16_t i;
	uint32_t addr;

	if ((len == 0u) || (len > item)) { return 0u; }

	/* ★★★★★ 2026-10-19 改：**满了不再停 ✗ 改成环形覆盖** ✓
	 *
	 *  ★ 原来是 `return 2u`（满 ⇒ 停）✗
	 *    · 后果（★ 整夜实测 ✓）：开机 21 分钟满 ⇒ 之后 8 小时全丢 ✓✓
	 *  ★ 现在 ✗：满了就【把最旧的一项作废】✗ 给新数据让位 ✓
	 *    · 取舍：宁可丢 2 小时前的 ✗ 也要保住刚才的 ✓✓
	 *      （★ 断卡场景要救的正是"最近的"数据 ✓）
	 *  ★★ 为什么这样做是安全的 ✗：
	 *    · 覆盖的槽 = `wr % slots` ✗ 满时它正好 == `rd % slots` ✓
	 *      ⇒ 覆盖的【就是最旧的那一项】✓✓
	 *    · ★ 而它所在的扇区（4 项）全都 ≤ rd ⇒ 【擦除不会碰到待回写数据】✓✓
	 *  ★★★ 溢出计数 ✗：借头里的 `rsv` 字段累加 ✓
	 *    （★ 16 位 ✗ 溢出会回绕 ✓ 只当"有没有发生过"看 ✓）
	 *  ★ 注意 ✗：这里【只改内存里的 rd】✗
	 *    真正的落盘在下面 `Dc_LogSave` 一次完成 ✓（★ 只追加 16 B ✗ 不擦扇区 ✓）*/
	if ((uint16_t)(h->wr - h->rd) >= slots)
	{
		h->rd++;
		h->rsv = (uint16_t)(h->rsv + 1u);       /* ★ 溢出计数 ✓ */
	}

	idx  = (uint16_t)(h->wr % slots);
	addr = base + ((uint32_t)idx * (uint32_t)item);

	/* ★ 进入新扇区 ⇒ 先擦（每 4096 B 一个扇区）✓
	 *
	 *  ★★★★★ 2026-10-19 修（★ 审计 G2 ✗ 真丢数据 ✓）：
	 *    ★ 原来 ✗：只看"进没进新扇区"就直接擦 ✓
	 *      · ★★ 满环时扇区 4 项里【必有 3 项】是 `rd`/`rd+1`/`rd+2`
	 *        （★ 最旧的、【还没回写】✓）
	 *      · ⇒ 【一次 4KB 擦 = 静默抹掉 3 条待回写数据】
	 *        而 `rsv`（串口 `ovr=`）只记了 1 条 ✗✓
	 *      · ★★★ 后果（★ 2026-10-19 实测复现 ✓）✗
	 *        被抹成 0xFF 的槽【长度前缀 = 0xFFFF】
	 *        ⇒ `PeekBatch` 判坏项返回 0 ⇒ **回写永久卡死** ✓✓
	 *    ★★★★ 修法 ✗：擦之前【先把该扇区里还有效的项处置掉】✓
	 *      · 它们本来就是【最旧的】✗ 与"环形覆盖最旧"的策略【一致】✓
	 *      · ★★ 但必须【如实记进 `rsv`】✗ 不许静默 ✓✓
	 *      · ★★★ 丢到【整个扇区 4 个槽都空】为止 ✗ 再擦 ✓
	 *        （★ 只丢到 "idx 空" 不够 ✗ idx+1..idx+3 会被一起擦掉 ✓）*/
	if ((((uint32_t)idx * (uint32_t)item) % 4096u) == 0u)
	{
		uint16_t guard = 0u;

		while (Dc_SectorHasData(h, slots, idx) != 0u)
		{
			if (h->rd == h->wr) { break; }              /* ★ 空环 ⇒ 没得丢 ✓ */
			h->rd  = (uint16_t)(h->rd + 1u);
			h->rsv = (uint16_t)(h->rsv + 1u);           /* ★ 如实记账 ✓ */
			guard++;
			if (guard > slots) { break; }               /* ★ 死循环保护 ✓ */
		}

		if (Dc_SectorHasData(h, slots, idx) != 0u)
		{
			return 0u;                                  /* ★ 丢不干净 ⇒ 宁可不写 ✓ */
		}

		if (W25Q64_EraseSector(addr) == 0u) { return 0u; }
	}

	/* ★ 长度前缀 ✓ */
	if (len_prefix != 0u)
	{
		uint8_t lp[4];
		lp[0] = (uint8_t)(len & 0xFFu);
		lp[1] = (uint8_t)((len >> 8) & 0xFFu);
		lp[2] = 0u;
		lp[3] = 0u;
		if (W25Q64_WriteData(addr, lp, 4u) == 0u) { return 0u; }
		addr += 4u;
	}

	/* ★ 按 256 B 页写 ✗ 不跨页 ✓（W25 页写不能跨页）*/
	for (i = 0u; i < len; i = (uint16_t)(i + 256u))
	{
		uint16_t n = (uint16_t)(((uint16_t)(len - i) > 256u) ? 256u : (uint16_t)(len - i));
		if (W25Q64_WriteData(addr + (uint32_t)i, src + i, (uint32_t)n) == 0u) { return 0u; }
	}

	h->wr++;
	if (Dc_LogSave(h, sec_a, sec_b, log) == 0u) { h->wr--; return 0u; }   /* ★ 指针没存住 ⇒ 回退 ✓ */
	return 1u;
}

static uint8_t Dc_Peek(uint32_t base, uint16_t slots, uint16_t item,
                       const DcHdr_t *h, uint8_t *dst)
{
	uint16_t idx;
	if (h->wr == h->rd) { return 0u; }
	idx = (uint16_t)(h->rd % slots);
	return W25Q64_ReadData(base + ((uint32_t)idx * (uint32_t)item), dst, (uint32_t)item);
}

static void Dc_Pop(uint32_t sec_a, uint32_t sec_b, DcLog_t *log, DcHdr_t *h)
{
	if (h->wr == h->rd) { return; }
	h->rd++;
	if (h->rd == h->wr)                 /* ★ 全搬完了 ⇒ 归零 ✓ 下轮从扇区首重新擦 ✓ */
	{
		h->wr = 0u;
		h->rd = 0u;
	}
	(void)Dc_LogSave(h, sec_a, sec_b, log);  /* ★ 指针没存住 ⇒ 最多重搬一次 ✓ 不会丢 ✓ */
}

/* ★ 运行时入口都要拿锁 ✗ 但【调度器没起来之前不能拿】✓ */
static uint8_t Dc_Lock(void)
{
	if ((s_dc_on == 0u) || (s_w25_lock == NULL)) { return 0u; }
	return (xSemaphoreTake(s_w25_lock, pdMS_TO_TICKS(2000)) == pdTRUE) ? 1u : 0u;
}

uint8_t DropCache_Init(void)
{
	/* ★ 本函数在 `Golden_Init()` 里调 ✗ 那时【调度器还没起来】✓
	 *   ⇒ 和 `Golden_Init` 同样的前提：单线程，不拿锁 ✓ */
	if (W25Q64_IsPresent() == 0u)
	{
		s_dc_on = 0u;
		return 0u;
	}
	/* ★★ 2026-10-20 G5：指针改成【日志 + A/B 乒乓】✗ 开机扫两条日志取最新 ✓ */
	Dc_LogLoad(&s_dc_d, DC_DATA_HDR_A, DC_DATA_HDR_B, DC_MAGIC_D,
	           (uint16_t)DC_DATA_SLOTS, &s_log_d);
	Dc_LogLoad(&s_dc_e, DC_EVT_HDR_A, DC_EVT_HDR_B, DC_MAGIC_E,
	           (uint16_t)DC_EVT_SLOTS, &s_log_e);
	s_dc_on = 1u;

	/* ★★★★★ 2026-10-20 **测试版专用**（★ 正式版 `DC_LOG_N == 256` ⇒ 这段整段不参与编译 ✓）
	 *   ★ 目的：人为造一条"坏记录"✗ 验证 `Dc_RecBlank()` 那条分支 ✓
	 *   · 触发条件：**游标在 [1, N)** —— ★ 必须 ≥1 ✗ 因为 pos==0 时前面
	 *     【没有任何有效记录】✗ 灌坏它等于把整个指针清空 ✓
	 *   · 动作：往"下一条要写的位置"灌 16 个 0x00
	 *     ⇒ 下一次保存写在那儿 ⇒ 数据被 0x00【与】掉 ⇒ **CRC 必然不对** ✓
	 *   · ★★ 期望：再复位一次 ⇒ `DROP: log d=256/<sec>`（★ 检测到坏记录 ✓）
	 *     再一次保存 ⇒ 擦另一个扇区、翻面、`hdrer` +1 ✓✓ */
#if (DC_LOG_INJECT_TEST == 1u)
	if ((s_log_d.pos >= 1u) && (s_log_d.pos < (uint16_t)DC_LOG_N))
	{
		uint8_t  junk[DC_LOG_REC_SZ];
		uint16_t i;

		for (i = 0u; i < DC_LOG_REC_SZ; i++) { junk[i] = 0x00u; }
		(void)W25Q64_WriteData(((s_log_d.sec == 0u) ? DC_DATA_HDR_A : DC_DATA_HDR_B) +
		                       ((uint32_t)s_log_d.pos * DC_LOG_REC_SZ),
		                       junk, DC_LOG_REC_SZ);
		Serial_Printf("DROP: TEST injected a CORRUPT record at %c[%u]\r\n",
		              (s_log_d.sec == 0u) ? 'A' : 'B', (unsigned)s_log_d.pos);
	}
#endif

	/* ★★★★★ 2026-10-20 加：**开机把日志游标打出来**（★ G5 的验收观测点 ✓）
	 *   ★ 为什么必须打 ✗：日志方案的"连续性"是**看不见的** ✓
	 *     · `d(pos,sec)` = 数据区下一条写在哪 ✗ 掉电重启后应该【接着原来的位置】✓
	 *     · `pos` 从 0 一路涨到 256 ⇒ 换扇区 ⇒ **回到 0 且 sec 翻面** ✓✓
	 *   ★★ 判据 ✗：
	 *     · 每次重启 `pos` 都在涨（不是 0）⇒ 【指针真的从日志里读回来了】✓
	 *     · 断电前 `pos=17` 上电后 `pos=17` ⇒ 【没丢也没重】✓✓
	 *     · ★ 若每次都是 `pos=0 sec=0` ⇒ 说明**扫描没认出来**（magic/CRC 不对）✓
	 *       ⇒ 那缓存就是"每次开机清空" ✗ 必须回头看 ✓ */
	Serial_Printf("DROP: log d=%u/%u e=%u/%u  pending=%u/%u\r\n",
	              (unsigned)s_log_d.pos, (unsigned)s_log_d.sec,
	              (unsigned)s_log_e.pos, (unsigned)s_log_e.sec,
	              (unsigned)(uint16_t)(s_dc_d.wr - s_dc_d.rd),
	              (unsigned)(uint16_t)(s_dc_e.wr - s_dc_e.rd));
	return 1u;
}

uint8_t DropCache_PutBatch(const uint8_t *p, uint16_t len)
{
	uint8_t r;
	if (Dc_Lock() == 0u) { return 0u; }
	r = Dc_Put(DC_DATA_HDR_A, DC_DATA_HDR_B, &s_log_d, DC_DATA_BASE,
	           (uint16_t)DC_DATA_SLOTS, (uint16_t)DC_DATA_ITEM, &s_dc_d, p, len, 1u);
	xSemaphoreGive(s_w25_lock);
	return r;
}

uint8_t DropCache_PeekBatch(uint8_t *dst, uint16_t *len)
{
	uint8_t  lp[4];
	uint16_t n;
	uint32_t addr;
	uint8_t  r;

	if (Dc_Lock() == 0u) { return 0u; }
	if (s_dc_d.wr == s_dc_d.rd) { xSemaphoreGive(s_w25_lock); return 0u; }

	addr = DC_DATA_BASE + ((uint32_t)((uint16_t)(s_dc_d.rd % (uint16_t)DC_DATA_SLOTS))
	                       * (uint32_t)DC_DATA_ITEM);

	/* ★ 先读 4 字节长度 ✓ 再按长度读数据 ✓ 绝不整槽搬运 ✓ */
	r = W25Q64_ReadData(addr, lp, 4u);
	if (r != 0u)
	{
		n = (uint16_t)((uint16_t)lp[0] | ((uint16_t)lp[1] << 8));
		if ((n == 0u) || (n > 512u))
		{
			r = 0u;                      /* ★ 长度不合理 ⇒ 当坏项 ✓ 由调用者决定跳过 */
		}
		else
		{
			r = W25Q64_ReadData(addr + 4u, dst, (uint32_t)n);
			if ((r != 0u) && (len != NULL)) { *len = n; }
		}
	}
	xSemaphoreGive(s_w25_lock);
	return r;
}

void DropCache_PopBatch(void)
{
	if (Dc_Lock() == 0u) { return; }
	Dc_Pop(DC_DATA_HDR_A, DC_DATA_HDR_B, &s_log_d, &s_dc_d);
	xSemaphoreGive(s_w25_lock);
}

uint16_t DropCache_PendingBatches(void)
{
	return (s_dc_on != 0u) ? (uint16_t)(s_dc_d.wr - s_dc_d.rd) : 0u;
}

/* ★★★★ 2026-10-19 加：**环形溢出次数**（★ 满了以后丢了多少项 ✓）
 *   ★ 为什么需要看它 ✗：
 *     · 环形覆盖是"丢最旧的保最新的"✗ 是有意的取舍 ✓
 *     · ★★ 但必须【让人看得见】✗ 否则"丢过东西"这件事会静默 ✓✓
 *   ★ 用法 ✗：串口 `drained ... ovr=N` ✓
 *     · `ovr` 一直不动 ⇒ 缓存没满 ✓ 一切正常
 *     · ★★ `ovr` 在涨 ⇒ 【缓存满了 ✗ 正在丢最旧的数据】✓✓
 *       ⇒ 该处理的是"卡为什么这么久回不来"✓ 而不是缓存 ✓
 *   ★ 存在头里的 `rsv` 字段 ✗ 掉电也保留 ✓（★ 16 位会回绕 ✓ 看趋势够用 ✓）*/
uint16_t DropCache_OverflowCount(void)
{
	return (s_dc_on != 0u) ? s_dc_d.rsv : 0u;
}

/* ★★★★★ 2026-10-20 加（★ G5 的"证据"✓）：指针日志扇区【擦了几次】✓
 *   · 改前 ✗ 每存一批擦一次 + 每取一批再擦一次（断卡 ≈ **3.3 次/秒**）
 *   · 改后 ✗ 每 **256 条**记录才擦一次
 *   ⇒ 掉卡跑几分钟，`hdrer` 应该只涨 **0~1** ✓
 *     （★ 若它按秒在涨 ⇒ 说明没走到日志路径 ✗ 立刻回头看 ✓✓）
 *   ★ 只在上电后累计 ✗ 重启清零 ✓（★ 当"本次开机擦了几次"读 ✓）*/
uint16_t DropCache_HdrErases(void)
{
	return s_dc_hdr_erases;
}

uint16_t DropCache_EvtHdrErases(void)
{
	return s_dc_evt_erases;
}

uint8_t DropCache_PutEvent(const char *line)
{
	uint8_t  r;
	uint16_t i;
	uint8_t  buf[DC_EVT_ITEM];

	if (line == NULL) { return 0u; }
	for (i = 0u; i < (uint16_t)(DC_EVT_ITEM - 1u); i++)
	{
		buf[i] = (uint8_t)line[i];
		if (line[i] == '\0') { break; }
	}
	buf[DC_EVT_ITEM - 1u] = 0u;
	for (i = (uint16_t)(i + 1u); i < (uint16_t)DC_EVT_ITEM; i++) { buf[i] = 0u; }

	if (Dc_Lock() == 0u) { return 0u; }
	r = Dc_Put(DC_EVT_HDR_A, DC_EVT_HDR_B, &s_log_e, DC_EVT_BASE,
	           (uint16_t)DC_EVT_SLOTS, (uint16_t)DC_EVT_ITEM, &s_dc_e, buf,
	           (uint16_t)DC_EVT_ITEM, 0u);
	xSemaphoreGive(s_w25_lock);
	return r;
}

uint8_t DropCache_PeekEvent(char *dst, uint16_t sz)
{
	uint8_t  raw[DC_EVT_ITEM];
	uint16_t i;
	uint8_t  r;

	if (Dc_Lock() == 0u) { return 0u; }
	r = Dc_Peek(DC_EVT_BASE, (uint16_t)DC_EVT_SLOTS, (uint16_t)DC_EVT_ITEM, &s_dc_e, raw);
	xSemaphoreGive(s_w25_lock);

	if (r == 0u) { return 0u; }
	for (i = 0u; (i < (uint16_t)(sz - 1u)) && (i < (uint16_t)DC_EVT_ITEM); i++)
	{
		dst[i] = (char)raw[i];
		if (raw[i] == 0u) { break; }
	}
	dst[sz - 1u] = '\0';
	return 1u;
}

void DropCache_PopEvent(void)
{
	if (Dc_Lock() == 0u) { return; }
	Dc_Pop(DC_EVT_HDR_A, DC_EVT_HDR_B, &s_log_e, &s_dc_e);
	xSemaphoreGive(s_w25_lock);
}

uint16_t DropCache_PendingEvents(void)
{
	return (s_dc_on != 0u) ? (uint16_t)(s_dc_e.wr - s_dc_e.rd) : 0u;
}


/*==============================================================================
 *  ★★★ 初始化（★ 建任务之前调一次 ✓）
 *    ★ 现在【调度器还没启动】✗ 【单线程】✗ 所以这里【不用加锁】✓
 *============================================================================*/
uint8_t Golden_Init(void)
{
	uint16_t hdr_crc = 0u;

	/* ★★★ 先建互斥量 ✗ 审计 G1 ✓
	 *   ★★ 2026-10-19：改【静态创建】✗ 控制块放 .bss 里 ✓（★ RAM 优化 R2 ✓）*/
	s_w25_lock = xSemaphoreCreateMutexStatic(&s_w25_mutex_cb);
	if (s_w25_lock == NULL)
	{
		Serial_Printf("GOLD: !! mutex create FAILED -> backup disabled\r\n");
		s_on = 0u;
		return 0u;
	}

	s_on = W25Q64_IsPresent();

	/* ★★ 2026-10-18：顺手把【掉卡缓存的两个指针】也读出来 ✓
	 *   ★ 同一个前提（调度器没起来、单线程）⇒ 不拿锁 ✓
	 *   ★★ 不在这里读的话：断电重启后【缓存里的数据就没人认领了】✓ */
	(void)DropCache_Init();
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
 *      · ★★ 一口气写 **448 页** = 约 **630ms** 连续占用 CPU ✗
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
			/* ★★★★★ 2026-10-19 修（★ 审计 G1 ✗ 擦除路径缺二次确认 ✓）：
			 *   ★ 为什么必须有 ✗（★ 和下面写页那处【同一个道理】✓）：
			 *     · 本函数进来时判过 `s_filling != 0` ✗
			 *       但那之后【还没拿到锁】✗ 中间隔着几十毫秒的串口打印 ✓
			 *     · ★★ 这期间 `Golden_SaveCurrent()`（★ `DISP` 任务 ✗
			 *       同优先级 1 ✗ 1ms 时间片就能抢占 ✓）可能【整趟跑完】✗
			 *       把整份备份连头一起写好 ✗ 并顺手把 `s_filling` 清 0 ✓✓
			 *     · ★★★ 那时我拿到锁再擦 ✗
			 *       = 【把刚写好的那两份 64KB 备份【整块擦掉】】✓✓
			 *     · ★★★★ 更糟的是后面 ✗ 我接着往下走到写页那处 ✗
			 *       那里的二次确认（`s_filling == 0`）会让我直接 return ✓
			 *       ⇒ 【头再也不会写 ✗ 备份永久缺失 ✗ 而且没有任何报错】✓✓
			 *         （★ 现象 ✗：`R` 只能回到"没有备份"的状态 ✓）
			 *   ★ 修法 ✗：拿到锁之后【再看一眼标志】✗
			 *     这是关掉这个窗口的【最小改法】✓（★ 与"整趟持锁"效果等价 ✓）*/
			if (s_filling == 0u)
			{
				xSemaphoreGive(s_w25_lock);
				return;                             /* ★ 别人已经写完了 ✗ 收工 ✓ */
			}

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
			 *       ★★ 但【后台路径【做不到整趟持锁】✗ 50ms/页 × 448 页 = 22 秒 ✓）✓ */
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

	/* ★★★★ 2026-10-18 加（④ 用户拍板 B）：每写一页【让出 50ms】✗✓
	 *
	 *  ★ 加之前是什么行为 ✗：
	 *    · `Golden_Task()` 【自己没有任何延时】✓
	 *    · `task_GOLDEN` 的 `for(;;)` 紧接调用 ⇒ 【一页接一页连写】✓
	 *    · 448 页 ≈ 0.6 秒（含两次 64KB 块擦最坏 4 秒）✓✓
	 *
	 *  ★★ 为什么必须让 ✗：
	 *    · 本任务优先级 1 ✗ `task_SD` 也是 1 ⇒ 【同级轮转】✓
	 *    · 连写期间 SD 拿不到 CPU ⇒ 落盘被推迟 0.6~4 秒 ✓
	 *    · ★★★ 而乒乓缓冲只有 2 块（≈1.8 秒数据）
	 *      ⇒ 【超过就开始 no_buf 丢数据】✓✓
	 *
	 *  ★ 让 50ms 之后 ✗：448 × 50 ≈ 22 秒填完
	 *    · 每页之间都放 SD 一马 ✓
	 *    · CPU 占用 = 1.4ms / 50ms ≈ **2.8%** ✓ 仍是静默后台 ✓
	 *  ★★ 注意 ✗：这个延时【也会在最后一页之后执行一次】✗ 无所谓 ✓ */
	vTaskDelay(pdMS_TO_TICKS(GOLDEN_PAGE_INTERVAL_MS));

	if (s_page >= (uint16_t)GOLDEN_PAGES)
	{
		uint8_t ok = 0u;

		if (xSemaphoreTake(s_w25_lock, portMAX_DELAY) == pdTRUE)
		{
			/* ★★★★★ 2026-10-19 修（★ 审计 G8 ✗ 写头分支同缺二次确认 ✓）：
			 *   ★ 窗口 ✗：`ClearBackup()`（`B` 命令）会清 `s_filling` ✓
			 *     若它正好落在我【写完最后一页、还没写头】的那 50ms 延时里 ✓
			 *     · 它清完 `s_filling = 0` ✗ 我这边照样把头写上 ✓
			 *   · ★★ 后果 ✗：`B` 的本意是"作废这一版 ✗ 别再自动备份"✓
			 *     而我【偏偏把它写成了一份有效备份】✗
			 *     ⇒ 【`R` 还能回退进这一版】✗ 与 `B` 的语义直接相反 ✓✓
			 *   ★ 修法同擦除路径 ✗ 拿到锁后【再看一眼标志】✓ */
			if (s_filling == 0u)
			{
				xSemaphoreGive(s_w25_lock);
				return;                             /* ★ 这版已被作废 ✗ 别写头 ✓ */
			}

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
