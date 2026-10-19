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
 *      0x001000  ~60 KB  金副本镜像（GOLDEN_ADDR_IMG ✗ 64KB 块 ✓）
 *      0x020000   4 KB   金副本元数据（GOLDEN_ADDR_META）
 *      0x030000   4 KB   ★ 缓存：数据指针页
 *      0x031000  ───     ★ 缓存：数据槽（★ 本次扩到 0x7A1000 ✓）
 *      0x7A1000   4 KB   ★ 缓存：事件指针页（★ 从 0x240000 挪来 ✓）
 *      0x7A2000 240 KB   ★ 缓存：事件槽
 *      0x7DE000 128 KB   余量
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
#define DC_DATA_HDR    0x030000u        /* 指针页（独占一个 4KB 扇区）*/
#define DC_DATA_BASE   0x031000u        /* ★ 扇区对齐 ✗ 免得和指针页互相擦 ✓ */
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
#define DC_EVT_HDR     0x7A1000u        /* ★ 从 0x240000 挪到高位 ✓ */
#define DC_EVT_BASE    0x7A2000u
#define DC_EVT_SLOTS   960u             /* 960 × 256B = 240 KB ✓ */
#define DC_EVT_ITEM    256u

#define DC_MAGIC_D     0x44434434u      /* ★★★ 'DCD4'（★ 2026-10-19 定版：环已改回 7616 ✓）
                                          *
                                          *  ★ 为什么每次改槽数都要升一版 ✗：
                                          *    · 槽号是【取模算的】✗ 基数一变 ⇒ 旧指针读出来【全错位】✓
                                          *    · ★★ 换 magic ⇒ `Dc_LoadHdr` 判不符
                                          *      ⇒ 【指针自动归零 ⇒ 缓存清空重来】✓✓
                                          *    · ★★★ 本轮经历 ✗ DCD3（32 槽测试）→ **DCD4（正式 7616）** ✓
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
#define DC_MAGIC_E     0x45434531u      /* 'ECE1' */

typedef struct {
	uint32_t magic;
	uint16_t wr;
	uint16_t rd;
	uint16_t rsv;
	uint16_t crc;
} DcHdr_t;

static DcHdr_t s_dc_d;                  /* 数据批区 */
static DcHdr_t s_dc_e;                  /* 事件行区 */
static uint8_t s_dc_on = 0u;

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
	h->magic = magic; h->wr = 0u; h->rd = 0u; h->rsv = 0u;
	h->crc   = Dc_HdrCrc(h);
}

/* 读头：magic / CRC / 合理性（待回写数 ≤ 槽数）任一不对 ⇒ 【重置指针】✓
 *   ★ 重置 = 当作"缓存为空"✗ 不会去读垃圾 ✓（宁可少救，不要读出乱码 ✓）*/
static void Dc_LoadHdr(DcHdr_t *h, uint32_t addr, uint32_t magic, uint16_t slots)
{
	if (W25Q64_ReadData(addr, (uint8_t *)h, (uint32_t)sizeof(DcHdr_t)) == 0u)
	{
		Dc_HdrDefault(h, magic);
		return;
	}
	if ((h->magic != magic) || (h->crc != Dc_HdrCrc(h)) ||
	    ((uint16_t)(h->wr - h->rd) > slots))
	{
		Dc_HdrDefault(h, magic);
	}
}

static uint8_t Dc_SaveHdr(DcHdr_t *h, uint32_t hdr_addr)
{
	h->crc = Dc_HdrCrc(h);
	/* ★ 先擦再写 ✗ NOR 不能"原地改"✓ 一块 4KB 扇区只放这 16 字节 ✓ */
	if (W25Q64_EraseSector(hdr_addr) == 0u) { return 0u; }
	return W25Q64_WriteData(hdr_addr, (const uint8_t *)h, (uint32_t)sizeof(DcHdr_t));
}

/* 写一项 ✗ 返回 1=成功 / 0=写失败 ✓
 *  ★★★ 2026-10-19 改：**不再返回 2（满）** ✗
 *    · 满了改成【环形覆盖最旧的】✓ 见下面注释 ✓
 *    · ★ 所以调用者只判 `!= 1u` 就是"真失败"✓ 语义更干净 ✓
 *  ★ `len_prefix` = 1 ⇒ 先写 4 字节【实际长度】再写数据 ✓
 *    ★★ 为什么必须带长度 ✗：槽是固定 1024 B ✗ 而一批可能只有 400 B ✓
 *      不留长度的话，回写时【把槽里残留的旧字节一起写进 CSV】✓✓ */
static uint8_t Dc_Put(uint32_t hdr_addr, uint32_t base, uint16_t slots, uint16_t item,
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
	 *    真正的落盘在下面 `Dc_SaveHdr` 一次完成 ✓（★ 减少一次擦写 ✓）*/
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
	if (Dc_SaveHdr(h, hdr_addr) == 0u) { h->wr--; return 0u; }   /* ★ 指针没存住 ⇒ 回退 ✓ */
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

static void Dc_Pop(uint32_t hdr_addr, DcHdr_t *h)
{
	if (h->wr == h->rd) { return; }
	h->rd++;
	if (h->rd == h->wr)                 /* ★ 全搬完了 ⇒ 归零 ✓ 下轮从扇区首重新擦 ✓ */
	{
		h->wr = 0u;
		h->rd = 0u;
	}
	(void)Dc_SaveHdr(h, hdr_addr);      /* ★ 指针没存住 ⇒ 最多重搬一次 ✓ 不会丢 ✓ */
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
	Dc_LoadHdr(&s_dc_d, DC_DATA_HDR, DC_MAGIC_D, (uint16_t)DC_DATA_SLOTS);
	Dc_LoadHdr(&s_dc_e, DC_EVT_HDR,  DC_MAGIC_E, (uint16_t)DC_EVT_SLOTS);
	s_dc_on = 1u;
	return 1u;
}

uint8_t DropCache_PutBatch(const uint8_t *p, uint16_t len)
{
	uint8_t r;
	if (Dc_Lock() == 0u) { return 0u; }
	r = Dc_Put(DC_DATA_HDR, DC_DATA_BASE, (uint16_t)DC_DATA_SLOTS,
	           (uint16_t)DC_DATA_ITEM, &s_dc_d, p, len, 1u);
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
	Dc_Pop(DC_DATA_HDR, &s_dc_d);
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
	r = Dc_Put(DC_EVT_HDR, DC_EVT_BASE, (uint16_t)DC_EVT_SLOTS,
	           (uint16_t)DC_EVT_ITEM, &s_dc_e, buf, (uint16_t)DC_EVT_ITEM, 0u);
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
	Dc_Pop(DC_EVT_HDR, &s_dc_e);
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
