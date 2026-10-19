/**
  ******************************************************************************
  * @file    task_sd.c
  * @brief   task_SD —— SD 卡落盘任务【骨架】（执行表 T2.3/T2.5）
  *
  *  骨架法约定（执行表 §九）：驱动全给（SD_SPI.c / diskio_spi.c / FatFs 已就绪），
  *  业务流程与注释写全，关键决策点留 TODO(你) 由你拍板填空。
  *
  *  任务状态机（两个状态）：
  *
  *    【未挂载态】 ←── 连续 3 批写失败 / 开机首次
  *        │  每 3 秒重试：f_mount→(disk_initialize=完整 SD_Init)→开/建 YYMMDDNN.CSV
  *        ↓ 成功
  *    【挂载态】收乒乓缓冲 → f_write → f_sync → 归还缓冲
  *        │  写失败：连续 3 批 → 置 g_sd_error、强制卸载 → 回未挂载态
  *        │  （不足 3 次的失败：本批数据已丢，缓冲照还，继续收下一批）
  *        └─ 拔卡插回后：重试挂载成功 → g_sd_error 清 0 → 自动续写（f_lseek 到尾）
  *
  *  为什么错误处理长这样（对应执行表 T2.6 验收③）：
  *    - diskio 层失败时会把自己标成 STA_NOINIT，所以"重试挂载"必然重跑
  *      完整 SD 上电序列（281kHz 起步），插回的卡不需要任何特殊照顾。
  *    - 3 次重试的门槛放业务层而不是驱动层：驱动层的 3 次重试救瞬时错误，
  *      业务层的 3 批门槛判"卡离场"——两层数字是各管各的，别混为一谈。
  *
  *  f_write / f_sync 策略（T2.5）：每批写完就 f_sync（SD_SYNC_EVERY_BATCH=1），
  *  掉电最多丢最后一批；不逐条 sync（写放大）也不攒很多批（丢数据多）。
  *  空闲超时也兜底 sync 一次，保证"每秒至少落盘一次"的验收口径。
  ******************************************************************************
  */
#include "stm32f10x.h"
#include <stdio.h>                      /* sprintf：拼 "0:26101001.CSV" 文件名（YYMMDDNN）*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "ff.h"                         /* FatFs（FatFs\ 目录，裁剪版 R0.12c） */
#include "Serial.h"                     /* Serial_Printf：报挂载/失败信息 */
#include "task_sd.h"
#include "MyRTC.h"
/* ★★ 2026-10-18 加：掉卡缓存 API（★ 实现在 golden.c ✓ W25 归它所有 ✓）
 *   ★ 不加这一行 ⇒ 编译器【隐式声明】✗ 返回类型全按 int 处理
 *     → 参数/返回值都不校验 ⇒ 出错时【查不出来】✓✓ */
#include "golden.h"
/* ★★ 2026-10-19 加：验"文件尾行 CRC"要用它（★ 与 PC 侧同一套参数 ✓）*/
#include "Crc16.h"

/*============================================================================
 *  配置常量（拍板后改这里，别散在流程里）
 *==========================================================================*/
#define SD_SYNC_EVERY_BATCH     1   /* 每写完几批 f_sync 一次（T2.5：每批，掉电最多丢一批） */
#define SD_REMOUNT_PERIOD_MS    3000/* 未挂载态的重试周期（无卡时别狂转） */
#define SD_FAIL_STREAK_LIMIT    3   /* 连续写失败多少批 → 判卡离场（执行表：3 次） */
/* ★ SD_FILE_IDX_MAX 已删（2026-10-11 独立审查 F15）✗
 *   它只服务于旧的 "LOG%04u 逐个探测空位" 机制 ✗ 那套已在 2026-10-10 废弃 ✓
 *   现在文件数上限由 SD_FILE_NN_MAX（同一天 ≤99 个）管着 ✓ 它没有任何引用 ✓ */
#define SD_FILE_SIZE_LIMIT      (200u * 1024u * 1024u)
                                    /* ★ 单文件大小兜底 = 200 MB（2026-10-10 加）
                                     *   正常一天约 78 MB ✓ → 兜底不会误触发 ✓
                                     *   真触发了（异常情况 ✓）→ 序号 +1 继续记 ✓ 不丢数据 ✓✓ */
#define SD_FILE_NN_MAX          99  /* ★ 同一天最多 99 个文件（YYMMDDNN，NN 两位 ✓）*/

/*============================================================================
 *  ★★ 滚动周期的"键"抽象（2026-10-11 加）
 *
 *  为什么要抽象成"键"✗：跨天检查只需要【判断"周期变没变"】✗
 *    → 不必逐字段比较年月日 ✓ 一个 uint32 就够 ✓✓
 *
 *  ★★★ SD_DEBUG_ROLL_MINUTE —— 把"天"换成"分钟"✗（★ 专为验证而设 ✓）
 *    为什么需要它：★★ 跨天滚动【没法在一天内验证】✗
 *      打开 → 每分钟换一个文件 ✓ → 跑 5 分钟就能验证"滚动逻辑对不对" ✓✓
 *      验证完改回 0 ✓（★ 正式版必须为 0 ✗）
 *    ★ 这是"难验证功能"的标准做法：把时间尺度压小 ✓
 *    ★★ 注意：调试模式下文件名仍是 YYMMDDNN ✗
 *       → 每分钟换文件 = 同一天的 NN 递增（26101101 → 26101102 …）✓
 *         因为挂载时会从 nn=1 起扫、找第一个未超兜底的文件 ✓
 *         （同一天内换文件本来就靠 NN 递增 ✓ 机制天然复用 ✓✓）
 * ============================================================================*/
#define SD_DEBUG_ROLL_MINUTE    0   /* 0 = 按天滚动（正式 ✓）  1 = 按分钟滚动（仅验证用 ✓）*/

#if SD_DEBUG_ROLL_MINUTE
#define SD_ROLL_KEY(dt)         ((uint32_t)(dt).minute)
#else
/* 正式模式：把年月日打包成一个键 ✓ 高位在前便于阅读 ✓ */
#define SD_ROLL_KEY(dt)         ( ((uint32_t)(dt).year  << 16) | \
                                  ((uint32_t)(dt).month <<  8) | \
                                   (uint32_t)(dt).day )
#endif

/*============================================================================
 *  ★★★ SD_DEBUG_MOUNT_TRACE —— 挂载过程的阶段打印（2026-10-11 加 ✓）
 *
 *  为什么要它（★ 这是用几小时换来的 ✓）：
 *    2026-10-11 那个"f_open 忘了开 FA_READ"的 bug ✗
 *    排查时【整条路径上没有任何可观测点】✗ ——
 *      `f_mount` 之后到 `f_open` 之前【零打印】✓
 *      → "卡死"和"静默失败"完全分不出来 ✗
 *      → 而我靠推理连错三次 ✗ 白折腾了几个小时 ✓
 *    ★★ 加了这 8 个阶段打印之后：【第三次读串口就定案了】✓✓
 *
 *  ★ 所以默认【开着】✗ = 1 ✓（它的价值远大于那几行串口 ✓）
 *    真要干净输出可以改 0 ✓ —— ★★ 但代价是"下次再出问题又变盲区"✗
 *    （★ 挂载不频繁 ✓ 每次也就 8 行 ✓ 不构成刷屏 ✓）
 *
 *  ★ 打印点（按执行顺序 ✓）：
 *    [1] f_mount 成功 / 失败      [2] 取到的日期
 *    [3] 正在试哪个文件名          [4] f_open 成功 + 文件大小
 *    [5] f_lseek 到末尾            [6] f_lseek 到 sz-1
 *    [7] 读末尾 1 字节             [8] ★ 挂载函数出口
 *       —— [8] 是关键 ✓：看到它【就证明"函数返回了"】✗ 而不是"卡在里面"✓✓
 *
 *  ★★ 【不受这个开关控制】的打印（错误 / 事件类 ✗ 永远保留 ✓）：
 *    "SD: log = ..."              —— 挂载成功的唯一正向信号 ✓
 *    "SD: skip ..."               —— 文件级错误换名（只打前 3 个 ✓）
 *    "SD: all N names failed"     —— 最多 **98** 个全试完 ✓
 *                                    （★ NN=90 是【事件文件】✗ 跳过 ✓
 *                                      ★★ 原注释写"99 个"✓ 已修正 ✓）
 *    "SD: seal partial line"      —— 发现残行（很少发生 ✓ 发生了就该知道 ✓）
 *    —— ★ 这些只在【有事情发生】时打 ✓ 不吵 ✓ 而且丢了就没法查 ✓
 * ============================================================================*/
#define SD_DEBUG_MOUNT_TRACE    1

#if SD_DEBUG_MOUNT_TRACE
#define SD_TRACE(...)           Serial_Printf(__VA_ARGS__)
#else
#define SD_TRACE(...)           ((void)0)
#endif

/*============================================================================
 *  FatFs 对象（★ 必须全局 static 放置，不能进任务栈：
 *  TINY=1 后 FATFS≈550B + FIL≈40B，放栈上 1KB 直接爆）
 *==========================================================================*/
static FATFS s_fs;                      /* 卷对象（f_mount 登记） */
static FIL   s_log_file;                /* 日志文件（挂载态唯一打开的写文件） */

/*============================================================================
 *  ★ 当前日志文件"属于哪个周期"（2026-10-10 加 · 2026-10-11 改成"键"✓）
 *
 *  为什么要记住它：★ 按天滚动【不能只在挂载时判断】✗
 *    板子连续跑过午夜时不会重新挂载 ✗ → 必须【在写入过程中也检查】✓
 *    → task_SD 每批写入成功后比对"当前周期键"和这个值 ✓ 不同就换文件 ✓✓
 *
 *  ★ 日期从【文件名】就能读出来 ✗ —— 文件名就是 YYMMDDNN.CSV ✓
 *    （所以续写判断不需要读文件内容 ✗ 不需要 f_stat ✗ 不需要 f_readdir ✓✓
 *      —— 这是"文件名带日期"的设计红利 ✓）
 *
 *  ★★ 键由 SD_ROLL_KEY() 生成 ✗ —— 正式是"年月日"✓、调试是"分钟"✓
 *    只有【一个变量】→ 不存在"某个字段设了没用"的警告 ✓✓
 *==========================================================================*/
static uint32_t s_cur_key = 0u;

/*============================================================================
 *  乒乓交接（PROC ↔ task_SD）
 *==========================================================================*/
/* 双缓冲本体：extern 出去给 PROC 填（见 task_sd.h 的交接说明） */
uint8_t g_sd_buf[2][SD_BUF_SIZE];

volatile uint8_t g_sd_error = 0;        /* 1=SD 故障（DISP/PROC 只读展示） */

/* ★ Y2 修复（第 6 课）：最近一次失败的原因码（见 task_sd.h 的完整说明）。
 *   有了它 OLED 才能从"坏了"（g_sd_error=1）升级成"为什么坏了"：
 *   E:03=FR_NOT_READY（没插卡/接触不良）、E:01=FR_DISK_ERR（写中拔卡）、
 *   E:07=FR_DENIED（根目录满/卷满，R2 修复路径的报警）…… */
volatile uint8_t g_sd_last_err = 0;

/* ★★★★ 安全关机握手（★ 2026-10-13 深夜加 ✓ 完整说明见 task_sd.h）✗
 *   · ★ `g_shutdown_req`  ✗ 【task_KEY 写】✗ task_SD 读 ✓
 *   · ★★ `g_shutdown_done` ✗ 【task_SD 写】✗ task_DISP 读 ✓
 *   · ★★★ 都是 volatile ✗ 【跨任务可见性靠它】✓ */
volatile uint8_t g_shutdown_req  = 0;
volatile uint8_t g_shutdown_done = 0;

/* ★★★★ 2026-10-18 加：这次关机【到底存住了没有】✗✓
 *
 *  ★ 为什么需要它 ✗：
 *    · 原来只有 `g_shutdown_done` 一个标志 ✗ 它只说"我处理完了"✓
 *    · ★★ 而屏上【一律】显示 `DATA SAVED` ✗
 *      → 写失败也显示 ✓
 *      → ★★★ 【根本没插卡也显示】✓✓（`mounted==0` 时整个落盘被跳过）
 *
 *  ★ 含义 ✗：1 = 文件【真的 f_close 成功】✓
 *             0 = 没存住 / 从没挂载 ✓
 *  ★ 谁写谁读：`task_SD` 写、`task_DISP` 读 ⇒ 必须 volatile ✓ */
volatile uint8_t g_shutdown_saved = 0;

/* 交接消息：指针 + 有效长度（len 单独传，CSV 尾批不满 512B 也能写对） */
typedef struct {
    uint8_t  *buf;
    uint16_t len;
    uint16_t nrec;      /* ★ 2026-10-18 加：这批里【有几条记录】✗✓
                         *   ★ 为什么要它 ✗：写卡失败时【按条记账】✓
                         *     原来只知道"一批"✗ 一批约 40 条 ⇒ 记成 1 ✓
                         *     → 丢掉 4000 条只报"100 次"✓✓ */
} SdMsg_t;

static QueueHandle_t      s_xQueueSdPtr  = NULL;    /* 工作队列：PROC → task_SD，传 {指针,长度} */
static QueueHandle_t      s_xQueueFree   = NULL;    /* 空闲队列：缓冲指针在两条队列间循环流转 */

/*============================================================================
 *  task_sd_Init：建交接用的内核对象
 *  ★ 必须在【调度器启动前】调（放 freertos_demo() 里，见 README 集成步骤）：
 *    task_SD 优先级 1 比 PROC 的 2 低，调度器一开 PROC 可能抢先运行，
 *    若队列还没建好，第一次 SD_BufAcquire/SD_LogEnqueue 就是空指针解引用
 *==========================================================================*/
void task_sd_Init(void)
{
	/* ★ R4 修复（2026-10-05 审查）：两条队列【都】建好才算初始化完成。
	 * 原来只判 s_xQueueSdPtr —— 若第一条建成、第二条失败，第二次调用直接返回，
	 * 半初始化状态永久卡死；而 task_SD 又会在 xQueueReceive(NULL) 上崩溃。 */
	if ((s_xQueueSdPtr != NULL) && (s_xQueueFree != NULL))
	{
		return;
	}

	/* 两条队列都长 2 = 缓冲块数。所有权流转规则（全程唯一，无拷贝）：
	 *   空闲队列 --SD_BufAcquire--> PROC 填写 --SD_LogEnqueue--> 工作队列
	 *   --> task_SD 写卡 --写完归还--> 空闲队列
	 * 一块缓冲任一时刻只在一个人手里 —— 这就是乒乓交接的全部安全性来源 */
	s_xQueueSdPtr = xQueueCreate(2, sizeof(SdMsg_t));
	s_xQueueFree  = xQueueCreate(2, sizeof(uint8_t *));

	if ((s_xQueueSdPtr == NULL) || (s_xQueueFree == NULL))
	{
		/* 12KB heap 后到这里失败基本不可能，但失败必须喊出来（老规矩：静默失败掩盖 RAM 不足） */
		Serial_Printf("!!! task_sd kernel objects FAILED (out of heap) !!!\r\n");
		/* ★ R4 修复：把已建成的删掉并置 NULL，避免"半初始化"状态残留 */
		if (s_xQueueSdPtr != NULL) { vQueueDelete(s_xQueueSdPtr); s_xQueueSdPtr = NULL; }
		if (s_xQueueFree  != NULL) { vQueueDelete(s_xQueueFree);  s_xQueueFree  = NULL; }
		return;
	}

	/* 两块缓冲的"自由身"初始都挂在空闲队列上 */
	{
		uint8_t *p;
		p = &g_sd_buf[0][0];
		(void)xQueueSend(s_xQueueFree, &p, 0);
		p = &g_sd_buf[1][0];
		(void)xQueueSend(s_xQueueFree, &p, 0);
	}
}

/*============================================================================
 *  SD_BufAcquire：PROC 申请一块空缓冲（交接第①步）
 *  为什么用"指针队列"而不是计数信号量：信号量只会计数，认不出"哪块"缓冲
 *  空闲 —— PROC 连续申请两次会拿到同一个指针，覆盖正在写卡的那块（真雷）。
 *  指针队列把所有权精确到块，绝无歧义。
 *  超时 0：PROC 绝不许为写卡阻塞（它卡住会顶住采集队列，违反架构前提）。
 *==========================================================================*/
BaseType_t SD_BufAcquire(uint8_t **pbuf)
{
	uint8_t *p = NULL;

	if ((s_xQueueFree == NULL) || (pbuf == NULL))
	{
		return pdFAIL;
	}
	if (xQueueReceive(s_xQueueFree, &p, 0) != pdPASS)
	{
		return pdFAIL;                  /* 两块都在 task_SD 手里：调用方丢弃本条 */
	}
	*pbuf = p;
	return pdPASS;
}

/*============================================================================
 *  SD_LogEnqueue：PROC 交缓冲入口（交接第②步，task_sd.h 有完整时序）
 *==========================================================================*/
BaseType_t SD_LogEnqueue(uint8_t *buf, uint16_t len, uint16_t nrec)
{
	SdMsg_t msg;

	/* ★ R5 修复（2026-10-05 审查）：参数/归属校验，且失败时【绝不】把非法指针塞回空闲池。
	 * 原来四种失败情况共用一个"还回空闲队列"的动作，后果很严重：
	 *   s_xQueueFree==NULL -> xQueueSend(NULL,...) 崩溃；
	 *   buf==NULL          -> 把 NULL 当空闲缓冲入池，之后 PROC 拿到 NULL 去写 -> 空指针；
	 *   buf 不属于池中两块 -> 池子被污染，真正的 A/B 永久丢失 -> 整条链静默停摆；
	 *   还池用 (void) 吞返回值 -> 队列满时缓冲永久泄漏且无人知晓。 */
	if ((s_xQueueSdPtr == NULL) || (s_xQueueFree == NULL))
	{
		return pdFAIL;                  /* 内核对象没就绪：调用方自己保留缓冲 */
	}
	if ((buf == NULL) || (len == 0) || (len > SD_BUF_SIZE))
	{
		return pdFAIL;                  /* 非法参数：不入池、原样拒收 */
	}
	if ((buf != &g_sd_buf[0][0]) && (buf != &g_sd_buf[1][0]))
	{
		return pdFAIL;                  /* 不是池中两块之一：拒绝，防止污染池子 */
	}

	msg.buf  = buf;
	msg.len  = len;
	msg.nrec = (nrec != 0u) ? nrec : 1u;   /* ★ 兜底：至少 1 条 ✓ */

	/* 超时 0：PROC 绝不许为写卡阻塞（它卡住会顶住采集队列，违反架构前提）。
	 * 正常情况下队列不可能满（空闲队列已保证在途缓冲 ≤2），失败=时序被破坏 */
	if (xQueueSend(s_xQueueSdPtr, &msg, 0) != pdPASS)
	{
		/* 交接失败：缓冲必须还回去，否则永久泄漏。
		 * 还池竟然也失败 = 所有权已被破坏 -> 必须喊出来，不能静默 */
		if (xQueueSend(s_xQueueFree, &buf, 0) != pdPASS)
		{
			g_sd_error = 1;
		}
		return pdFAIL;
	}
	return pdPASS;
}

/*============================================================================
 *  内部：挂载 + 打开（或续写）日志文件
 *  返回 FR_OK = 可以开始写了；其他 = 未挂载态继续等
 *==========================================================================*/
/*============================================================================
 *  ★★★★ F2：读文件【最后一条记录的 rtc】（2026-10-12 加 ✓）
 *
 *  ★ 为什么需要它 ✗：★ 没装 CR2032 ✗ → 每次上电用【编译时刻】初始化 RTC ✓
 *     → 如果【烧回旧固件】✗ → 时间倒退 ✓
 *     → ★★ 那次挂载会去【续写"过去那天"的文件】✗ → 时间线不单调 ✓✓
 *
 *  ★★ 做法：★ 从文件末尾【倒回一段】✗ → 找最后一个 '\n' ✓
 *    → 解析那一行的第 4 个字段（rtc ✓）
 *    ★ 一行约 90 字节 ✗ → 【读 256 字节一定够】✓✓
 *    ★★ 而且【调用点已经在它之前封过残行】✗✓
 *      → 所以最后一行是【完整的】✓ 不用担心半行 ✓✓
 *
 *  ★ 返回 0 = 读不到 / 文件太短 / 解析失败 ✓（调用者据此跳过检查 ✓）
 *  ★★ 注意：本函数会【改变文件指针】✗ —— 调用者之后必须重新 lseek 到末尾 ✓✓
 *
 *  ★★★ 2026-10-12 改版 ✗：★ 增加 line/linesz 输出【那一行的原文】✗✓
 *    · ★ 原因：★ 用户的设计要求【把"偏差那一行"拎出来留证据】✗✓
 *    · ★★ 所以【不能只返回 rtc ✗ 还要把原文带出去】✓✓
 *============================================================================*/
static uint32_t SD_ReadLastLine(FIL *fp, char *line, uint16_t linesz)
{
	uint8_t  buf[256];
	UINT     br   = 0u;
	FSIZE_t  sz;
	FSIZE_t  from;
	FRESULT  fr;
	uint32_t v = 0u;
	int      i, j, field;

	if (fp == NULL) { return 0u; }
	if (line != NULL) { line[0] = '\0'; }

	sz = f_size(fp);
	if (sz < 8u) { return 0u; }                 /* ★ 太短 → 没有数据行 ✓ */

	from = (sz > (FSIZE_t)sizeof(buf)) ? (sz - (FSIZE_t)sizeof(buf)) : 0u;

	fr = f_lseek(fp, from);
	if (fr != FR_OK) { return 0u; }
	fr = f_read(fp, buf, (UINT)(sz - from), &br);
	if ((fr != FR_OK) || (br == 0u)) { return 0u; }

	/* ★★★ 从末尾往回找 '\n' ✗ —— ★★ 注意【末尾本来就有 '\n'】✗✓!!
	 *
	 *  ⚠️ 这里原来是【只找一次】✗ → 【2026-10-12 实测踩到的 bug】✓：
	 *    · ★ 文件每行都以 "\r\n" 结尾 ✗ → buf 最后一个是 '\n' ✓
	 *    · ★★ `i = br-1` 正好指向它 → while 条件不成立 → 一次都不进 ✓
	 *    · ★★★ `i++` 之后 i == br ✗ → 后面的解析循环【一次都不执行】✓
	 *    · ★★★★ v 保持 0 → return 0 ✗ → 调用者【跳过检查 ✗ 静默通过】✓✓
	 *    → 现象：用户把最后一行 rtc 改成未来值 ✗ 试两次【完全没反应】✓
	 *
	 *  ★ 正确做法：★ 【先跳过末尾的 \r\n ✗ 再找上一行的结尾 '\n'】✓✓ */
	i = (int)br - 1;
	if ((i >= 0) && (buf[i] == (uint8_t)'\n')) { i--; }     /* ★ 跳过末尾 '\n' ✓ */
	if ((i >= 0) && (buf[i] == (uint8_t)'\r')) { i--; }     /* ★ 跳过 '\r' ✓ */
	while ((i >= 0) && (buf[i] != (uint8_t)'\n')) { i--; }  /* ★ 再找上一个 '\n' ✓ */
	i++;                                                     /* ★ 指向最后一行开头 ✓ */

	/* ★★★ 把这一行的【原文】拷出去 ✗✓（★ 供"留证据"用 ✓）
	 *   · ★ 遇到 '\r' / '\n' 就停 ✓（★ 不带行尾 ✓）
	 *   · ★★ 长度受 linesz 限制 ✗ → 【超长就截断 ✓ 但 '\0' 一定补上】✓✓ */
	if (line != NULL)
	{
		int k = 0;
		for (j = i; (j < (int)br) && (k < ((int)linesz - 1)); j++)
		{
			if ((buf[j] == (uint8_t)'\r') || (buf[j] == (uint8_t)'\n')) { break; }
			line[k] = (char)buf[j];
			k++;
		}
		line[k] = '\0';
	}

	/* ★★ 数逗号 ✗ → 第 4 个字段（field == 3 ✓）就是 rtc ✓ */
	field = 0;
	for (j = i; j < (int)br; j++)
	{
		if (buf[j] == (uint8_t)',')
		{
			if (field >= 3) { break; }      /* ★ rtc 字段已结束 ✓ */
			field++;
			continue;
		}
		if (field == 3)
		{
			if ((buf[j] >= (uint8_t)'0') && (buf[j] <= (uint8_t)'9'))
			{
				v = v * 10u + (uint32_t)(buf[j] - (uint8_t)'0');
			}
			else
			{
				break;                      /* ★ 非数字 → 结束 ✓ */
			}
		}
	}
	return v;
}

/*============================================================================
 *  ★★★★ F2：把"偏差那一行"留证据（★ 用户设计 ✓ 2026-10-12）
 *
 *  ★ 做什么 ✗：★ 往 `2610MMDD90.CSV` 追加【两行】✓
 *    · ★ 第 1 行：★ 说明（'#' 开头 ✗ 一眼看出是注释 ✓）
 *      —— 含【来源文件名 / file_rtc / now / 偏差秒数】✓
 *    · ★★ 第 2 行：★ 那一行的【原文】✗✓（★ 原始现场 ✓）
 *  ★★ 文件不存在 ✗ → 【先写标准表头】✓（★ 用户明确要求 ✓）
 *  ★★★ 用完就关 ✗ —— 【不占着句柄 ✗ 主文件完全不受影响】✓✓
 *  ★★★★ 输出 evname ✗ → 【调用者要"切换"时，用它重新打开】✓✓
 *============================================================================*/
static FRESULT SD_AppendEvidence(const char *src,
                                 uint32_t file_rtc, uint32_t now_ts,
                                 const char *rawline,
                                 char *evname, uint16_t evnamesz)
{
	FIL     ef;
	FRESULT fr;
	char    txt[224];
	UINT    bw  = 0u;
	uint16_t len;
	RTC_DateTime_t d;
	uint16_t yy;
	uint8_t  mm, dd;

	(void)evnamesz;                         /* ★ 名字固定 14 字符 ✗ 不会溢出 ✓ */

	MyRTC_GetDateTime(&d);
	yy = (uint16_t)(d.year % 100u);
	mm = d.month;
	dd = d.day;

	sprintf(evname, "0:%02u%02u%02u%02u.CSV",
	        (unsigned)yy, (unsigned)mm, (unsigned)dd, 90u);

	fr = f_open(&ef, evname, FA_WRITE | FA_READ | FA_OPEN_ALWAYS);
	if (fr != FR_OK) { return fr; }

	if (f_size(&ef) == 0u)
	{
		/* ★ 新文件 → 先写标准表头 ✓（★ 用户要求 ✓）
		 *   ★★ 必须和主文件那份【一字不差】✗（★ 契约 ✓ 30 号 §15.2 ✓）*/
		static const char hdr[] =
		    "seq,tick,press,rtc,ax,ay,az,gx,gy,gz,temp_ds,temp_chip,humi,light,vbat,crc\r\n";
		fr = f_write(&ef, hdr, (UINT)(sizeof(hdr) - 1u), &bw);
		if (fr != FR_OK) { (void)f_close(&ef); return fr; }
	}
	else
	{
		fr = f_lseek(&ef, f_size(&ef));
		if (fr != FR_OK) { (void)f_close(&ef); return fr; }
	}

	/* ★ 第 1 行：说明（'#' 开头 ✗ 不会被当成数据行 ✓）*/
	len = (uint16_t)sprintf(txt,
	        "# TIME BACKWARD from %s : file_rtc=%lu now=%lu diff=%lds\r\n",
	        (src != NULL) ? src : "?",
	        (unsigned long)file_rtc, (unsigned long)now_ts,
	        (long)((int32_t)file_rtc - (int32_t)now_ts));
	fr = f_write(&ef, txt, len, &bw);
	if (fr != FR_OK) { (void)f_close(&ef); return fr; }

	/* ★★ 第 2 行：那一行的原文（★ 原始现场 ✓）*/
	len = (uint16_t)sprintf(txt, "# RAW: %s\r\n",
	                        (rawline != NULL) ? rawline : "(none)");
	fr = f_write(&ef, txt, len, &bw);
	if (fr != FR_OK) { (void)f_close(&ef); return fr; }

	fr = f_sync(&ef);
	(void)f_close(&ef);
	return fr;
}

/*============================================================================
 *  ★★★★★ 2026-10-18 加：丢失记账（★ 复盘 §7 ★★★ 第 1 条 ✓）
 *==========================================================================*/

/* 待落盘的增量（★ 只加不减 ✗ 冲掉的由 flusher 扣 ✓）*/
static volatile uint16_t s_loss_pending[LOSS_N];

/* 理由名（★ 写进事件文件用 ✗ 纯 ASCII ✓）*/
static const char *const s_loss_name[LOSS_N] = {
	"q_full", "no_buf", "enq_fail", "row_big", "sd_write"
};

/* ★★★ 任何任务都能调 ✗ 但【中断里不许调】✓（用了临界区）*/
void Loss_Add(uint8_t reason, uint16_t n)
{
	if ((reason >= LOSS_N) || (n == 0u)) { return; }

	taskENTER_CRITICAL();
	s_loss_pending[reason] = (uint16_t)(s_loss_pending[reason] + n);
	taskEXIT_CRITICAL();
}

/* ★★★★ 往事件文件追加【一条通用事件行】✗✓
 *
 *  ★ 为什么要一个比 `SD_AppendEvidence` 更通用的 ✗：
 *    · `SD_AppendEvidence` 是【专门给"时间倒退"写的】✗ 固定写两行 ✓
 *    · ★★ 而"丢了多少"需要的格式不同 ✓
 *    · ★ 复用一个通用入口 ⇒ 命名/表头/关闭策略【只有一份】✓✓
 *
 *  ★★ 约定 ✗：
 *    · `#` 开头 ⇒ 所有 PC 解析器都当注释跳过 ✓
 *    · 写完【立刻 f_sync】✗ 这种行丢了就等于没记 ✓
 *  ★ 只能在 task_SD 里调（唯一允许碰 FatFs 的任务 ✓）*/
static FRESULT SD_AppendEventLine(const char *line)
{
	FIL        ef;
	FRESULT    fr;
	char       evname[16];
	char       txt[160];
	UINT       bw = 0u;
	uint16_t   len = 0u;
	RTC_DateTime_t d;

	MyRTC_GetDateTime(&d);
	(void)sprintf(evname, "0:%02u%02u%02u%02u.CSV",
	              (unsigned)(d.year % 100u), (unsigned)d.month, (unsigned)d.day, 90u);

	fr = f_open(&ef, evname, FA_WRITE | FA_READ | FA_OPEN_ALWAYS);
	if (fr != FR_OK) { return fr; }

	if (f_size(&ef) == 0u)
	{
		static const char hdr[] =
		    "seq,tick,press,rtc,ax,ay,az,gx,gy,gz,temp_ds,temp_chip,humi,light,vbat,crc\r\n";
		fr = f_write(&ef, hdr, (UINT)(sizeof(hdr) - 1u), &bw);
		if (fr != FR_OK) { (void)f_close(&ef); return fr; }
	}
	else
	{
		fr = f_lseek(&ef, f_size(&ef));
		if (fr != FR_OK) { (void)f_close(&ef); return fr; }
	}

	/* ★ 拷贝 + CRLF（★ 不用 strlen ✗ 少一个头文件依赖 ✓）*/
	while ((line[len] != '\0') && (len < (uint16_t)(sizeof(txt) - 3u)))
	{
		txt[len] = line[len];
		len++;
	}
	txt[len++] = '\r';
	txt[len++] = '\n';

	fr = f_write(&ef, txt, len, &bw);
	if (fr == FR_OK) { fr = f_sync(&ef); }
	(void)f_close(&ef);
	return fr;
}

/* ★★★★ 把 RAM 里攒的增量冲进事件文件 ✗✓
 *
 *  ★★ 关键策略 ✗：**【先清零、写成功才算数】**
 *    · 写失败 ⇒ 【把数补回去】✗ 等下一轮再试 ✓
 *      → ★ 否则就成了"记账自己也丢"✓✓
 *  ★ 全部为 0 时【不做任何 FatFs 操作】✓（每轮调一次也不费）*/
static void Loss_Flush(void)
{
	char    line[120];
	uint8_t i;

	for (i = 0u; i < LOSS_N; i++)
	{
		uint16_t n;

		taskENTER_CRITICAL();
		n = s_loss_pending[i];
		s_loss_pending[i] = 0u;
		taskEXIT_CRITICAL();

		if (n == 0u) { continue; }

		(void)sprintf(line, "# LOST,%lu,%u,%s",
		              (unsigned long)MyRTC_GetUnix(), (unsigned)n, s_loss_name[i]);

		if (SD_AppendEventLine(line) != FR_OK)
		{
			/* ★★★ 2026-10-18 加：写不进卡 ⇒ 【先存进 W25 缓存】✗✓
			 *   ★ 为什么这条最值钱 ✗：事件行是"证据"✓
			 *     · 原来只能【攒在 RAM】✗ 等卡好了补记 ✓
			 *     · ★★★ 而这期间一断电（现场最常见的"以为死机→拔电源"）
			 *       ⇒ 【证据全丢】✓✓
			 *   · ★ W25 走独立的软件 SPI ⇒ 【SD 坏着它也写得进】✓
			 *   ⇒ 存进缓存 = 【断电也不丢证据】✓✓ */
			if (DropCache_PutEvent(line) != 0u)
			{
				continue;                    /* ★ 已保住 ⇒ 不用补回 RAM ✓ */
			}

			/* ★ 没写进去 ⇒ 补回去 ✗ 别让它凭空消失 ✓ */
			taskENTER_CRITICAL();
			s_loss_pending[i] = (uint16_t)(s_loss_pending[i] + n);
			taskEXIT_CRITICAL();

			Serial_Printf("SD: LOST record FAILED to write (pending kept)\r\n");
			return;                      /* ★ 后面几个也别试了 ✗ 卡坏了 ✓ */
		}
	}
}

/*============================================================================
 *  ★★★★ 【已删除】F2 原来在这里有个 SD_OpenIsolated()（2026-10-12 删 ✓）
 *
 *  ★ 它原来做什么 ✗：
 *    · 时间倒退时【改用 NN = 90~99 的隔离文件】重新打开 ✓
 *    · 设计意图：★ 把"异常期的数据"和正常序列分开 ✓
 *
 *  ★★★★ 为什么删掉 ✗（★ 用户拍板 ✓）：
 *    · ★ 一旦触发就【永远触发】✗✓ ——
 *      · RTC 落后（没 VBAT + 断过电）→ 每次挂载都判定倒退 ✓
 *      · → ★★ 【记录【全部】跑进 90 文件】✗
 *      · → ★★★ 而正常的 26101201.CSV 【停在那里不再增长】✓✓
 *    · ★★ 一次偶发问题 ✗ → 【导致永久跑偏】✗✓ —— 这是【设计缺陷】✓
 *
 *  ★ 现在的做法 ✗：★【只报警 ✗ 继续写同一个正常文件】✓✓
 *    · ★ 数据【全在一个文件里 ✗ 一行不丢】✓
 *    · ★★ 也不去改/删"出问题的那一行"✗（★ 它是有效数据 ✓）
 *    · ★★★ 代价：★ 文件里 rtc 会【先大后小】✗
 *      → ★★★★ PC 侧按【seq】排序即可 ✓（★ seq 永远单调 ✓✓）
 *
 *  ★★ 根本解决 ✗：★【装 VBAT 电池】✗✓ ——
 *    有电池后【断电 RTC 继续走 ✗ 不再倒退】✓✓
 *    （★ 详见 30 号 §二十四 ✓）
 *============================================================================*/

/*============================================================================
 *  内部：挂载 + 打开（或续写）日志文件
 *  返回 FR_OK = 可以开始写了；其他 = 未挂载态继续等
 *==========================================================================*/
/*============================================================================
 *  ★★★★ 2026-10-19 加：验一行 CSV 的 CRC（★ 用于"挂载时验文件尾"✓）
 *
 *  ★ 契约 ✗：一行 = 16 列 ✗ 前 15 列文本算 CRC16/KERMIT ⇒ 第 16 列 ✓
 *    （★ 和 `check_csv_crc.py` 完全同一套 ✓）
 *  ★★ 为什么需要它 ✗（★ 实测证据 ✓）：
 *    · 卡上 `26101801.CSV` 里 **38,440 行是二进制垃圾**（16% ✗）
 *    · 而 FatFs【完全不知道】内容坏了 ✓
 *    · ★★★ 成因：反复"写卡中途拔卡/断电"
 *      ⇒ 廉价卡把 FAT 认为有数据的块搞坏/重映射 ✓✓
 *  ★ 返回 ✗：1 = 这一行 CRC 对得上 ✓ 0 = 对不上 / 不是数据行 ✓
 *============================================================================*/
static uint8_t Sd_LineCrcOk(const char *line)
{
	uint32_t stored = 0u;
	uint16_t i;
	uint16_t n        = 0u;
	uint16_t lastcomma = 0u;

	if (line == NULL) { return 0u; }

	/* ① 找【最后一个逗号】⇒ 它前面就是被 CRC 覆盖的 15 列 ✓ */
	for (i = 0u; (line[i] != '\0') && (i < 300u); i++)
	{
		if (line[i] == ',') { lastcomma = i; }
	}
	if (lastcomma == 0u) { return 0u; }         /* ★ 连逗号都没有 ⇒ 不是数据行 ✓ */
	n = lastcomma;

	/* ② 解析逗号后面的数字（第 16 列 = CRC 十进制 ✓）
	 *   ★★ 2026-10-19 ✗：CRC-32 方案【已废弃】⇒ 这里也退回单一 CRC-16 ✓
	 *     （★ 曾经的"数位数分流两代"已删 ✗ 不留死代码 ✓）*/
	for (i = (uint16_t)(lastcomma + 1u);
	     (line[i] >= '0') && (line[i] <= '9');
	     i++)
	{
		stored = stored * 10u + (uint32_t)(line[i] - '0');
	}
	if (i == (uint16_t)(lastcomma + 1u)) { return 0u; }   /* ★ 后面没数字 ✓ */

	/* ③ 算前 15 列 ✓ 比对 ✓ */
	return (Crc16_Kermit((const uint8_t *)line, (uint32_t)n) == (uint16_t)stored) ? 1u : 0u;
}


/*============================================================================
 *  ★★★★ 2026-10-19 加：**多点抽样校验**（★ 专门抓"中间坏了、尾巴还好"✓）
 *
 *  ★ 为什么尾行校验不够 ✗（★ 实测证据 ✓）：
 *    · 卡上 `26101801.CSV` 有 38,440 行二进制垃圾（9~16% ✗）
 *    · 但【垃圾在中间】⇒ **尾行是好的** ⇒ 尾行校验【放行】✓
 *    · ★★★ 于是它【继续往坏文件里追加】✓✓（实测：+133 KB ✓）
 *
 *  ★ 做法 ✗：等距取 4 个窗口（各 512 B）✗ 每窗按行验 CRC ✓
 *    · 任一窗口【坏行 ≥ 60%】⇒ 判这个文件不可用 ✓
 *  ★★ 为什么不用全文件扫描 ✗：21 MB @硬件 SPI1 9MHz ≈ 19 秒 ✗ 不可接受 ✓
 *    · 4 × 512 B = 2 KB ⇒ 约 2 ms ✓（挂载时一次 ✓）
 *  ★ 为什么用 static 缓冲 ✗：task_SD 栈只有 1.5 KB ✓（★ 不吃栈 ✓）
 *  ★ 返回 ✗：1 = 看着是坏的 ✓  0 = 看着正常 / 小文件（不抽样 ⇒ 不误杀 ✓）
 *============================================================================*/
static uint8_t Sd_SampleLooksBad(FIL *fp)
{
	static const uint16_t pct[4] = { 12u, 37u, 62u, 87u };
	static uint8_t  s_smp[512];                 /* ★ static：不吃栈 ✓ */
	static char     s_line[200];
	FSIZE_t   sz = f_size(fp);
	uint8_t   k;

	if (sz < 8192u) { return 0u; }              /* ★ 文件小 ⇒ 不抽样 ⇒ 绝不误杀 ✓ */

	for (k = 0u; k < 4u; k++)
	{
		FSIZE_t  off = (sz / 100u) * (FSIZE_t)pct[k];
		UINT     br  = 0u;
		uint16_t good = 0u;
		uint16_t bad  = 0u;
		uint16_t i;

		if (f_lseek(fp, off) != FR_OK) { continue; }
		if (f_read(fp, s_smp, (UINT)sizeof(s_smp), &br) != FR_OK) { continue; }
		if (br < 128u) { continue; }

		/* ★ 跳到【第一个换行之后】✗ 免得把半行当坏行 ✓ */
		i = 0u;
		while ((i < (uint16_t)br) && (s_smp[i] != (uint8_t)'\n')) { i++; }
		i++;

		while (i < (uint16_t)br)
		{
			uint16_t start = i;
			uint16_t len;
			uint16_t n;
			uint16_t j;

			while ((i < (uint16_t)br) && (s_smp[i] != (uint8_t)'\n')) { i++; }
			len = (uint16_t)(i - start);

			/* ★ 只统计【末尾有换行的完整行】✓（最后一行可能是半行 ⇒ 不算 ✓）*/
			if ((i < (uint16_t)br) && (len > 20u))
			{
				n = (len > (uint16_t)(sizeof(s_line) - 1u)) ? (uint16_t)(sizeof(s_line) - 1u) : len;
				for (j = 0u; j < n; j++) { s_line[j] = (char)s_smp[start + j]; }
				while ((n > 0u) && ((s_line[n - 1u] == '\r') || (s_line[n - 1u] == '\n'))) { n--; }
				s_line[n] = '\0';

				if (Sd_LineCrcOk(s_line) != 0u) { good++; } else { bad++; }
			}
			if (i < (uint16_t)br) { i++; }
		}

		/* ★ 至少 3 行才有统计意义 ✓ 坏行 ≥ 60% ⇒ 这一片基本是垃圾 ✓ */
		if ((uint16_t)(good + bad) >= 3u)
		{
			if (((uint32_t)bad * 10u) >= ((uint32_t)(good + bad) * 6u))
			{
				return 1u;
			}
		}
	}

	return 0u;
}


/*============================================================================
 *  ★★★★ 2026-10-19 加：**错误站点探针**（★ 定位 E:09 = FR_INVALID_OBJECT ✓）
 *
 *  ★ 为什么需要 ✗（★ 用户报"时不时 E:09 一下又自己好了"✓）：
 *    · E:09 = `FR_INVALID_OBJECT` ✗ 含义是【用了一个已失效的文件对象/卷】✓
 *      不是卡坏、也不是写不进 ✓
 *    · ★★ 但 `g_sd_last_err` 被【六处】赋值 ✗ 光看 E:09 不知道是谁报的 ✓✓
 *    · 读代码能排除一部分（关文件后都紧跟 continue/return ✓
 *      `sd_note_failure` 三个调用者都清了 mounted ✓
 *      `SD_AppendEventLine` 用自己的 FIL ✓）✗ 但剩下的靠读不能断定 ✓
 *  ★ 做法 ✗：每次设 `g_sd_last_err` 时【同时记下站点码 + 打一行串口】✓
 *  ★★ 站点码表 ✗（★ 一次复现就能对上 ✓）：
 *      1 = 批次写失败 f_write（挂载态主路径）
 *      2 = 写失败后 f_sync/f_close 的结果
 *      3 = 挂载失败（sd_mount_and_open 返回）
 *      4 = ★ 缓存回写失败（DropCache Peek 后的 f_write/f_sync）
 *      5 = 连击记账：写批失败（sd_note_failure 第 1 个调用者）
 *      6 = 连击记账：f_sync 失败（第 2 个）
 *      7 = 连击记账：跨天 f_sync 失败（第 3 个）
 *      8 = 跨天滚动的 f_sync 结果
 *      9 = 关机落盘路径
 *  ★ 怎么读 ✗：串口出现 `SD: ERR fr=9 site=N` ⇒ 【N 就是元凶】✓✓
 *  ★ 定位完可以删掉本探针（★ 或者留着 ✗ 成本只有几字节 ✓）
 *============================================================================*/
volatile uint8_t g_sd_err_site = 0u;

static void sd_set_err(uint8_t err, uint8_t site)
{
	g_sd_last_err  = err;
	g_sd_err_site  = site;
	Serial_Printf("SD: ERR fr=%u site=%u\r\n", (unsigned)err, (unsigned)site);
}


static FRESULT sd_mount_and_open(uint32_t *pidx)
{
	FRESULT fr;
	char name[24];                      /* "0:26101001.CSV" = 14 字符 + '\0' ✓ 留足余量 */

	/* f_mount(opt=1)：强制挂载 —— 会触发 disk_initialize，即完整 SD_Init
	 * （74 时钟/CMD0/ACMD41/切 9MHz 全在里头）。没插卡在这里就会失败 */
	fr = f_mount(&s_fs, "0:", 1);
	if (fr != FR_OK)
	{
		/* ★★ 阶段打印 [1]（2026-10-11 加 · 临时诊断 ✗）：
		 *   为什么需要它：★ f_mount 之后到 f_open 之前【一句打印都没有】✗✓
		 *     → 实测里"卡住"和"静默失败"【分不出来】✓（Task list 照打 ✓ 但 SD 没下文 ✓）
		 *   ★ 定位完可以撤掉 ✓ 或者留着当"挂载失败原因"的现场记录 ✓✓ */
		SD_TRACE("SD: [1] f_mount FAILED fr=%u\r\n", (unsigned)fr);
		return fr;                      /* FR_NOT_READY/FR_DISK_ERR = 卡不在或坏 */
	}
	SD_TRACE("SD: [1] f_mount OK\r\n");

	/*============================================================================
	 *  ★★ 文件名 = YYMMDDNN.CSV（2026-10-10 定版）
	 *    26 | 10 | 10 | 01  =  2026-10-10 当天第 01 个 ✓
	 *    ★ 8 字符主名 + 3 字符扩展名 = 正好 8.3 ✓（_USE_LFN=0 的硬约束 ✓）
	 *    ★★ 好处：① 字典序 = 时间序 ✓ ② 看名字就知道是哪天 ✓
	 *              ③ ★ 续写判断【不用读文件内容】✗ —— 名字里就有日期 ✓✓
	 *                 （不用 f_stat ✓ 不用 f_readdir ✓ 不用解析末行 ✓）
	 *
	 *  ★ 滚动规则：
	 *    · 同一天 → 【续写】当天那个文件 ✓（它已存在 ✓）
	 *    · 跨天   → 用新日期的新名字 ✓ 自然就是新文件 ✓
	 *    · ★ 单文件超 SD_FILE_SIZE_LIMIT ✗ → NN+1 ✓（继续记，不丢数据 ✓）
	 *    · ★ NN 到 99 还没完 ✗ → 明确失败 ✓（上层报警 ✓ 不静默覆盖 ✓✓）
	 *
	 *  ★ 旧机制（LOG%04u 逐个探测空位 ✗）已废弃：
	 *    它每次挂载都新开一个文件 ✗ → 文件数无限增长 ✓
	 *    （但保留它的安全教训：★ 任何扫描都必须【带上限】✗ —— 见 SD_FILE_NN_MAX ✓）
	 *==========================================================================*/
	{
		RTC_DateTime_t now;
		uint16_t yy;
		uint8_t  mm, dd, nn;
		FRESULT  last_fr = FR_OK;           /* ★ 记住最近一次【文件级】错误 ✓ 兜底上报用 */

		MyRTC_GetDateTime(&now);
		yy = (uint16_t)(now.year % 100u);       /* ★ 两位年份 ✓（26 = 2026 ✓）*/
		mm = now.month;
		dd = now.day;
		/* ★★ 阶段打印 [2]（临时诊断 ✗）：验证 RTC 读取没卡住 ✓
		 *   （★ `MyRTC_GetDateTime` 里是 do-while 等两次读数一致 ✗
		 *     —— 正常不会卡 ✓ 但要证明 ✓）*/
		SD_TRACE("SD: [2] date = 20%02u-%02u-%02u\r\n",
		              (unsigned)yy, (unsigned)mm, (unsigned)dd);

		for (nn = 1u; nn <= (uint8_t)SD_FILE_NN_MAX; nn++)
		{
			/* ★★★★ 2026-10-18 加：**跳过 90** ✗✓（★ 用户拍板 A ✓）
			 *
			 *  ★ 为什么必须跳 ✗：
			 *    · `NN = 90` 是【事件文件】的固定编号 ✓
			 *      （`# LOST` / `# SD_RECOVER` / `# TIME BACKWARD` 都写它 ✓）
			 *    · ★★ 而数据文件是从 01 往后【一路试】✗ 原来【没有跳过 90】✓
			 *      → 一旦 1~89 全不可用（目录满 / 长期不换卡 / 写保护）
			 *      ⇒ ★★★ 数据就写进 `??90.CSV` ✗
			 *        【数据行和事件行混一个文件】✓✓
			 *    · ★ 后果很阴 ✗：所有 PC 工具都把 90 当"事件文件"跳过
			 *      ⇒ 【那批数据看起来凭空消失】✗ 而且【不报任何错】✓✓
			 *
			 *  ★ 代价（说清楚）✗：极端情况下"能写的只剩 90"时
			 *    ⇒ 【写不进去 ⇒ 报 SD 错误】✓ 但那本来就该报警 ✓✓ */
			if (nn == 90u) { continue; }
			sprintf(name, "0:%02u%02u%02u%02u.CSV",
			        (unsigned)yy, (unsigned)mm, (unsigned)dd, (unsigned)nn);

			/* ★★ 阶段打印 [3]（临时诊断 ✗）：证明【走进了循环】✓
			 *   ★ 只打前 3 个 ✗（和下面的 skip 打印同理 ✓ 免得刷屏 ✓）*/
			if (nn <= 3u)
			{
				SD_TRACE("SD: [3] nn=%u trying %s\r\n", (unsigned)nn, name);
			}

			/* ★★★★ 2026-10-11 修复（dsh 自己引入的 bug ✗）：
			 *   必须带 FA_READ ✗✓ —— 因为下面【封残行】要 f_read 1 字节 ✓
			 *
			 *   FatFs 源码 ff.c:3518：
			 *     if (!(fp->flag & FA_READ)) LEAVE_FF(fs, FR_DENIED);
			 *   ★★ 也就是说：文件对象没开读权限时，f_read 立刻返回 FR_DENIED(7) ✗
			 *
			 *   ★ 原来的代码只写 FA_WRITE 是对的 ✗ —— 那时【不读文件】✓
			 *     （不续写、不封残行 ✓）
			 *   ★★ 我后来加了"封残行"✗ 却没同步开读权限 ✗ →
			 *     · 表头照写 ✓（写发生在 f_read 之前 ✓）→ 文件有正确的 76 字节表头 ✓
			 *     · 然后封残行 f_read → 7 → return ✗ → 【数据批一条都没开始】✓
			 *     · 现场表现：卡上文件只有表头、没有数据 ✓
			 *       sd_last=7 恒定 ✓ drop 每秒 +10 ✓
			 *     · ★★★ 而且【和卡无关】✗ → 所以【格式化怎么弄都没用】✓✓
			 *   ★ 教训：加一个"读文件"的动作时，必须回头检查文件对象的打开模式 ✓*/
			fr = f_open(&s_log_file, name, FA_WRITE | FA_READ | FA_OPEN_ALWAYS);
			if (fr == FR_OK)
			{
				if (f_size(&s_log_file) == 0u)
				{
					break;                  /* ★ 新文件：下面写表头，就用它 ✓ */
				}
				if (f_size(&s_log_file) < (FSIZE_t)SD_FILE_SIZE_LIMIT)
				{
					/* ★★★★★ 2026-10-19 加：续写之前【先验尾行 CRC】✗✓
					 *
					 *  ★ 为什么必须验 ✗（★ 实测证据 ✓）：
					 *    · 卡上 `26101801.CSV` 里【38,440 行是二进制垃圾】
					 *      （占 16% ✗ 而 FatFs【完全不知道】✓）
					 *    · ★★ 成因：反复"写卡中途拔卡/断电"
					 *      ⇒ 廉价卡把 FAT 认为有数据的块搞坏/重映射 ✓✓
					 *    · ★★★ 【在同一根坏链上继续追加】= 继续污染 ✓
					 *
					 *  ★ 判据 ✗：读最后一行 ⇒ 前 15 列 + CRC16 对得上 ✓
					 *    · 对不上 ⇒ 【关掉它 ✗ 试下一个 NN】✓
					 *      （★ 坏文件【留着当证据】✗ 绝不删 ✓）
					 *
					 *  ★ 位置为什么【必须在这里】✗：
					 *    · 这是【决定"要不要续写这个文件"】的那个判断点 ✓
					 *    · ★★ 而且它在 `for (nn...)` 循环【里面】
					 *      ⇒ `continue` 才能正确地"试下一个 NN" ✓✓
					 *
					 *  ★ 成本 ✗：一次读行 + 一次 CRC16 ≈ 几十微秒（只在挂载时 ✓）
					 *  ★★ 覆盖面（说清楚）✗：
					 *    · 能抓【尾部已坏】✓ ⇒ 不会继续追加上去 ✓
					 *    · 抓不住【中间坏了但尾巴还好】✗
					 *      ⇒ 那种只能靠 PC 侧校验发现 ✓（`check_csv_crc.py` ✓）*/
					char tail[200];

					(void)SD_ReadLastLine(&s_log_file, tail, (uint16_t)sizeof(tail));
					if (Sd_LineCrcOk(tail) != 0u)
					{
						/* ★★★ 2026-10-19 加：尾行过了【还不够】✗✓
						 *   ★ 实测：`26101801.CSV` 的垃圾在【中间】✗
						 *     尾行是好的 ⇒ 只验尾行【会放行】✓✓
						 *   ⇒ ★ 再【多点抽样】一轮 ✓（抓"中间坏"）*/
						if (Sd_SampleLooksBad(&s_log_file) == 0u)
						{
							break;          /* ★ 抽样也正常 ⇒ 续写它 ✓ */
						}
						Serial_Printf("SD: sample CRC BAD -> skip %s (mid-file garbage)\r\n", name);
					}
					else
					{
						Serial_Printf("SD: tail CRC BAD -> skip %s (try next NN)\r\n", name);
					}

					(void)f_close(&s_log_file);
					fr = FR_OK;
					continue;               /* ★ 换下一个 NN ✓ */
				}
				/* ★★★★ 2026-10-14 改：不能再 `(void)` 丢掉返回值 ✗
				 *   ★ 为什么 ✗：`fr` 里还留着【上一次的 FR_OK】✓
				 *     → 丢掉 f_close 的失败 ⇒ 【这一处"关不上"永远查不出来】✓✓
				 *   ★★ f_close 会做最后的目录/目录项落盘 ✗
				 *     它失败 = 这个文件的元数据可能不完整 ✓
				 *   ★★★ 所以要【如实上报】✗ 让屏上的 E:xx 亮起来 ✓✓ */
				fr = f_close(&s_log_file);
				if (fr != FR_OK)
				{
					return fr;              /* ★ 上报 ✗ 由 task_SD 走错误流程 ✓ */
				}
				continue;
			}

			/* ★★ f_open 失败的处理（2026-10-11 改 · 独立审查的建议 + 实测推动）✗
			 *
			 *   ★★★ 核心认知：★【"这一个文件打不开"✗ ≠ "所有文件都打不开"】✗✓
			 *     原来是一失败就 `return fr` ✗ →
			 *       task_SD 判"卡不行"✗ → 每 3 秒重试【同一个文件】✗ →
			 *       ★ 永远失败 → 【全链路丢数据】✗✓
			 *     （2026-10-11 实测就是这个现象：sd_last=7 恒定 ✓ drop 每秒 +10 ✓）
			 *
			 *   ★ 所以按错误类型分流：
			 *     · 文件级（换一个 NN 就可能好 ✓）→ 【continue 试下一个】✓✓
			 *       FR_DENIED(7)       —— 访问被拒 / ★ 目录满 / 目录项异常 ✓
			 *       FR_NO_FILE(4)      —— 找不到 ✓
			 *       FR_INVALID_NAME(6) —— 名字不合法 ✓
			 *       FR_EXIST(8)        —— 已存在（FA_CREATE_NEW 才可能 ✓ 防御性保留）
			 *     · 卡级（换名字也没用 ✗）→ 【立即上浮】✓ 保持快速失败 ✓
			 *       FR_NOT_READY(3) / FR_DISK_ERR(1) / FR_INVALID_OBJECT(9) /
			 *       FR_WRITE_PROTECTED(10) / FR_NO_FILESYSTEM(13) 等 ✓
			 *
			 *   ★ 代价：最坏情况要试 99 次 f_open ✓
			 *     每次约 1~2ms ✓ → 最坏约 0.2 秒 ✓（挂载时一次 ✓ 可接受 ✓）*/
			if ((fr == FR_DENIED) || (fr == FR_NO_FILE) ||
			    (fr == FR_INVALID_NAME) || (fr == FR_EXIST))
			{
				if (nn <= 3u)               /* ★ 只打前 3 个 ✗ 否则最坏刷 99 行 ✓ */
				{
					Serial_Printf("SD: skip %s (fr=%u) -> try next NN\r\n",
					              name, (unsigned)fr);
				}
				last_fr = fr;
				continue;                   /* ★ 试下一个 NN ✓✓ */
			}

			return fr;                      /* ★ 卡级错误：上浮 ✓ 重试整个挂载没意义 ✗ */
		}
		if (nn > (uint8_t)SD_FILE_NN_MAX)
		{
			/* ★ 99 个都不行 ✗ → 上报【最后一个真原因】✗
			 *   （不要硬编 FR_DENIED ✓ 否则 OLED 上的 E:xx 会丢信息 ✓）*/
			Serial_Printf("SD: all %u names failed (last fr=%u)\r\n",
			              (unsigned)SD_FILE_NN_MAX, (unsigned)last_fr);
			return (last_fr != FR_OK) ? last_fr : FR_DENIED;
		}

		s_cur_key = SD_ROLL_KEY(now);       /* ★ 记住归属周期 → 运行中滚动检查要用 ✓
		                                     *   正式模式 = 年月日打包 ✓
		                                     *   调试模式 = 分钟 ✓（见 SD_ROLL_KEY）*/
		/* ★★ 阶段打印 [4]（临时诊断 ✗）：证明【循环结束、选定了文件】✓
		 *   ★ 走到这里说明 f_open 成功 ✓ 而且【没卡在 f_open 里面】✓ */
		SD_TRACE("SD: [4] opened %s  size=%lu\r\n",
		              name, (unsigned long)f_size(&s_log_file));
	}

	/* ★ R6 语义定版（2026-10-05 审查后拍板）：**滚动新文件**，不做续写。
	 *   ★★ 2026-10-10 改版：现在【按天续写】✗ —— 同一天写同一个文件 ✓
	 *      因为文件名带了日期 ✗ → 一天一个文件 ✓ 归档直观 ✓
	 *      而"会话边界"这个原本的好处由【文件名日期】接管了 ✓✓ */
	fr = f_lseek(&s_log_file, f_size(&s_log_file));     /* ★ 定位到文件尾（续写 ✓）*/
	SD_TRACE("SD: [5] lseek(end) fr=%u\r\n", (unsigned)fr);   /* ★ 临时诊断 ✓ */
	if (fr == FR_OK)
	{
		if (f_size(&s_log_file) == 0)       /* 新文件：先写表头 */
		{
			/* ★ R6 落地：表头必须与 task_PROC 拼的 CSV 行列序【一字不差】。
			 * 列名与 30 号 §15.2 的字段表一一对应（16 列 / 40 字节记录）。
			 * ★ PROC 落地时若改列序或列名，必须同步改这一行，否则 PC 端解析全错。 */
			static const char csv_hdr[] =
			    "seq,tick,press,rtc,ax,ay,az,gx,gy,gz,temp_ds,temp_chip,humi,light,vbat,crc\r\n";
			UINT hbw = 0;

			fr = f_write(&s_log_file, csv_hdr, (UINT)(sizeof(csv_hdr) - 1u), &hbw);
			if ((fr != FR_OK) || (hbw != (UINT)(sizeof(csv_hdr) - 1u)))
			{
				return (fr == FR_OK) ? FR_DISK_ERR : fr;   /* 首行就失败=卡有问题，早暴露 */
			}

			/* ★★ 立刻 f_sync（2026-10-11 加）✗
			 *   为什么必须有它：FatFs 把【文件大小写在目录项里】✗
			 *     而目录项只在 f_sync / f_close 时才更新 ✓
			 *     → 只写表头不 sync ✗ → 目录项大小【仍然是 0】✗✓
			 *     → 拔卡/断电后缓存丢掉 → ★ 文件停在 0 字节 ✓
			 *     → ★★ 让人误判成"什么都没写"✗（2026-10-11 真的发生了这个误判 ✓：
			 *        卡上 26101101.CSV 是 0 字节 ✓ 而板子其实【一直在正常记录】✓）
			 *   加上它之后：
			 *     · ★ "文件已建立"变成【持久】的事实 ✓ 一眼看出跑到哪一步 ✓✓
			 *     · 下次挂载 f_size != 0 ✗ → 不会重复写表头 ✓
			 *   ★ 代价：挂载时多一次 sync（最坏几百 ms ✓ 且【只在建新文件时】发生 ✓）*/
			fr = f_sync(&s_log_file);
			if (fr != FR_OK)
			{
				return fr;                  /* ★ 表头都落不了盘 = 卡有问题 ✓ 早暴露 ✓ */
			}
		}
	}

	/*============================================================================
	 *  ★★ 封闭"残行"（2026-10-10 加）—— 续写已有文件时【必须有】✗✓
	 *  为什么：热拔卡/掉电会让文件尾部留下【半行】✗（写了一半的 CSV ✓）
	 *    直接往上追加 → 新行会和残行粘成一行 ✗ → 那一行彻底坏掉 ✓
	 *  做法：读末尾 1 字节 ✗ → 不是 '\n' → 补一个 "\r\n" ✓ 把残行封闭掉 ✓✓
	 *  ★ 那个残行会变成【列数不足的坏行】✗
	 *    → PC 端按"列数 ≠ 16"或"CRC 不过"就能识别并丢掉它 ✓✓
	 *    （★ 不用 f_truncate ✗ —— _FS_MINIMIZE=2 已把它裁掉 ✓ 而且这样更简单 ✓）
	 *==========================================================================*/
	if (fr == FR_OK)
	{
		FSIZE_t sz = f_size(&s_log_file);

		if (sz > 0u)
		{
			uint8_t last = 0u;
			UINT    br   = 0u;

			fr = f_lseek(&s_log_file, sz - 1u);
			SD_TRACE("SD: [6] lseek(sz-1) fr=%u\r\n", (unsigned)fr);   /* ★ 临时诊断 ✓ */
			if (fr == FR_OK) { fr = f_read(&s_log_file, &last, 1u, &br); }
			SD_TRACE("SD: [7] read1 fr=%u br=%u last=0x%02X\r\n",
			              (unsigned)fr, (unsigned)br, (unsigned)last);         /* ★ 临时诊断 ✓ */
			if ((fr == FR_OK) && (br == 1u) && (last != (uint8_t)'\n'))
			{
				UINT bw = 0u;
				Serial_Printf("SD: seal partial line -> append CRLF\r\n");
				fr = f_write(&s_log_file, "\r\n", 2u, &bw);

				/* ★★ 同样立刻 f_sync（2026-10-11 加）✗
				 *   理由和上面"表头后 sync"一样 ✓：
				 *     不 sync ✗ → 这个 "\r\n" 只进了缓存 ✓ → 拔卡就丢 ✗
				 *     → 残行仍是"半行"✗ → 下次挂载又要再封一次 ✓（不致命 ✓ 但白做 ✓）
				 *   ★ 封完立刻 sync ✗ → "文件尾部是干净的整行"这件事【持久】✓✓
				 *     也是对"下次续写"的保证 ✓（否则新行又可能和半行粘连 ✗）*/
				if (fr == FR_OK) { fr = f_sync(&s_log_file); }
			}
			if (fr == FR_OK) { fr = f_lseek(&s_log_file, f_size(&s_log_file)); }
		}
	}

	/*============================================================================
	 *  ★★★★ F2：时间回拨检测（2026-10-12 加 ✓ · 同日改版 ✗）
	 *
	 *  ★ 背景 ✗：★ 没装 VBAT 电池 ✗ →
	 *    · ★ 备份域【靠 VDD 撑着】✓（★ RTC + BKP_DR1 魔数都在备份域 ✓）
	 *    · ★★ 一旦【断电】✗ → 【计数器清零 ✗ 魔数也清零】✓
	 *    · ★★★ 下次上电 ✗ → MyRTC_Init 看到魔数不在 ✗
	 *      → 【用编译时刻（__DATE__/__TIME__）重设 RTC】✓✓
	 *    → ★★★★ 所以"倒退"的本相是：★【RTC 被【重设】成编译那一刻】✗
	 *      而【不是它自己在走】✓（★ 按复位键【不会】倒退 ✓ 只有断电才会 ✓）
	 *
	 *  ★★ 判据 ✗：★ 文件里【最后一条记录的 rtc】> 当前时间 ✗
	 *     → 说明那些数据来自"未来" ✗ → ★【现在的时间落后了】✓✓
	 *
	 *  ★★★★ 处理（★ 2026-10-12 用户拍板设计 ✓ 分四档 ✗）：
	 *
	 *    ★ ① 【< 1 分钟】✗ → 【静默 ✗ 什么都不做】
	 *       · 理由：★ 断电重启【本来就有几十秒偏差】✗
	 *         → ★★ 【为它报警会变成【每次都报的噪声】】✓✓
	 *
	 *    ★★ ② 【1 分钟 ~ 1 天】✗ → ★【报警 + 留证据 ✗ 但【继续写原文件】】✓✓
	 *       · ★ 证据 = 【往 2610MMDD90.CSV 追加两行】✗
	 *         （★ 说明行 + 【那一行的原文】✓ 保留原始现场 ✓）
	 *       · ★★ 理由：★【这是最典型的场景】✗（断电几小时 ✓）
	 *         → ★★★ 时钟错了 ✗ 但【数据是准的 ✗ 不该拆散】✓✓
	 *
	 *    ★★★ ③ 【> 1 天】✗ → ★【报警 + 留证据 + 【切换到 90 继续记录】】✓✓
	 *       · ★ 理由：★ 这么大的偏差 ✗ 说明【问题严重】✓
	 *         → ★★ 【隔离 ✗ 避免继续污染正常序列】✓✓
	 *       · ★★★ 而【护栏就在这里】✗：★ 1 天以内【绝不切换】✗
	 *         → ★★★★ 【所以不会重现"永远跑偏"那个缺陷】✓✓
	 *
	 *    ★★★★ ④ 【极端值（> 10 年）】✗ → 【只报警 ✗ 不动任何文件】✓
	 *       · ★ 理由：★ 这不像"断电"✗ 更像【卡被换 / 时间被乱改】✓
	 *         → ★★ 【保守处理 ✗ 不信任它】✓✓
	 *
	 *  ★ 调用时机【很关键】✗：★ 必须在【封残行之后】✓
	 *    —— 因为要读最后一行 ✗ 而残行会让解析失败 ✓✓
	 *  ⚠️ 本函数会改变文件指针 ✗ → 所以【之后要恢复到末尾】✓
	 *
	 *  ★★ 根本解决 ✗：★【装 VBAT 电池】✗✓ —— 详见 30 号 §二十四 ✓
	 *==========================================================================*/
	if (fr == FR_OK)
	{
		char     lastline[200];
		char     evname[24];
		uint32_t last_rtc = SD_ReadLastLine(&s_log_file, lastline, (uint16_t)sizeof(lastline));
		uint32_t now_ts   = MyRTC_GetUnix();
		int32_t  diff     = (int32_t)((int32_t)last_rtc - (int32_t)now_ts);   /* ★ >0 = 倒退 ✓ */

		/* ★★ 诊断打印（★ 无论有没有倒退都打 ✓）
		 *   —— "没有输出"曾经让我误判两次 ✗ 所以这里【必须有输出】✓✓ */
		Serial_Printf("SD: last_rtc=%lu  now=%lu  diff=%lds\r\n",
		              (unsigned long)last_rtc, (unsigned long)now_ts, (long)diff);

		/* ★ 无论走哪条路 ✗ 都要把文件指针恢复到末尾 ✓ */
		fr = f_lseek(&s_log_file, f_size(&s_log_file));

		if ((fr == FR_OK) && (last_rtc != 0u) && (diff > 0))
		{
			if (diff < 60)
			{
				/* ★ ① < 1 分钟 ✗ → 【静默】✓
				 *   （★ 断电重启的正常抖动 ✗ 不值得报警 ✓）*/
			}
			else if (diff < 86400)
			{
				/* ★★ ② 1 分钟 ~ 1 天 ✗ → 【报警 + 留证据 ✗ 继续原文件】✓✓ */
				Serial_Printf("SD: TIME BACKWARD %lds (<1d) - keep SAME file\r\n", (long)diff);
				(void)SD_AppendEvidence(name, last_rtc, now_ts, lastline,
				                        evname, (uint16_t)sizeof(evname));
			}
			else if (diff < (86400L * 3650L))
			{
				/* ★★★★ 2026-10-18 改：③ **不再切 90** ✗✓（★ 用户拍板 A ✓）
				 *
				 *  ★ 原来的行为 ✗：报警 + 留证据 + 【关主文件、改开 90】✓
				 *  ★★★ 为什么必须改 ✗：
				 *    · 本板【没有 VBAT】⇒ 每次断电后 RTC 从【编译时刻】起算 ✓
				 *      → ★★ "倒退 > 1 天"【会反复发生】✗ 不是罕见事件 ✓✓
				 *    · ★★★ 而一旦切到 90 ⇒ 【正常文件停止增长】✓
				 *      历史上"一旦触发就永远触发"那个缺陷就是这么来的 ✓
				 *      （★ 本文件上方"已删除 `SD_OpenIsolated`"那段注释
				 *        写的就是"只报警、继续写同一个文件"✗ 代码现在终于一致 ✓）
				 *
				 *  ★ 代价 ✗：文件里 `rtc` 会【先大后小】✓
				 *    ⇒ ★★ PC 侧按 **seq** 排序即可（已有约定 ✓ 见 30 号 §二十四 ✓） */
				Serial_Printf("SD: TIME BACKWARD %lds (>1d) - keep SAME file (no switch)\r\n",
				              (long)diff);
				(void)SD_AppendEvidence(name, last_rtc, now_ts, lastline,
				                        evname, (uint16_t)sizeof(evname));
			}
			else
			{
				/* ★★★★ ④ 极端值 ✗ → 【只报警 ✗ 不动文件】✓ */
				Serial_Printf("SD: TIME BACKWARD %lds (ABSURD) - ignored (no file change)\r\n",
				              (long)diff);
			}
		}
	}

	if (fr == FR_OK)
	{
		Serial_Printf("SD: log = %s  size=%lu\r\n",
		              name, (unsigned long)f_size(&s_log_file));
#if SD_DEBUG_ROLL_MINUTE
		/* ★★ 醒目警告（2026-10-11）：调试开关没关 ✗
		 *   它的效果是"每分钟换一个文件"✗ —— 正式使用会导致文件数爆炸 ✓
		 *   ★ 看到这行就说明【当前固件不是正式版】✗✓ */
		Serial_Printf("!!! WARNING: SD_DEBUG_ROLL_MINUTE=1 -> rolling by MINUTE !!!\r\n");
		Serial_Printf("!!!          This build is for VERIFICATION ONLY. Set it back to 0. !!!\r\n");
#endif
	}
	*pidx = 1u;                             /* ★ 旧序号变量已无实际作用，保留签名不动 ✓ */
	/* ★★ 阶段打印 [8]（临时诊断 ✗ 关键 ✓）：
	 *   ★ 这是挂载函数的【唯一出口】✗ —— 只要看到它 ✓ 就证明
	 *     【函数正常返回了】✓ 而不是"卡在里面"✗✓
	 *   （★ 这一步把"挂死"和"失败返回"彻底分开 ✓✓）*/
	SD_TRACE("SD: [8] mount exit fr=%u\r\n", (unsigned)fr);
	return fr;
}

/*============================================================================
 *  内部：强制卸载（判卡离场时调）
 *  为什么不 f_close：文件对象随卡一起"临终"，f_close 内部还会尝试
 *  f_sync 再碰卡 —— 对着已经拔掉的卡收尾只会再吃一串超时。
 *  直接 f_mount(NULL) 作废卷登记；数据完整性由"每批 f_sync"兜底
 *  （已 sync 的批都安全，未 sync 的最多丢最后一批，T2.5 设计如此）
 *==========================================================================*/
static void sd_unmount_force(void)
{
	(void)f_mount(NULL, "0:", 0);
}

/*============================================================================
 *  内部：把一次"真失败"记入连击；够数就判卡离场
 *  ★ R3 修复（2026-10-05 审查）：f_sync 的返回值也必须喂进来！
 *    _FS_TINY=1 时"不满一扇区的尾巴"只在 fs->win 里 —— f_write 返回 FR_OK 且 bw==len
 *    并不代表已落盘，真正进卡只有 f_sync 知道（它才触发 disk_write）。
 *    原来两处 f_sync 都用 (void) 丢掉：若"暂停采集时卡被拔走"，没有新数据到达，
 *    fail_streak 永远停在 1、mounted 永远 1、g_sd_error 永远 0 ->
 *    OLED 不报警，"拔卡插回 3 秒恢复"这条验收路径永远不触发。
 *  ★ Y2 修复（第 6 课）：新增 err 参数 —— 每次真失败把 FRESULT 原码记入
 *    g_sd_last_err，OLED 从"坏了"升级成"为什么"。注意连击 1~2 次（还不到
 *    判离场门槛）时 g_sd_error 仍是 0、但 last_err 已记录 —— 屏上提前留痕。
 *  返回：1 = 本次失败已判定卡离场（调用者需把 mounted 清 0）
 *==========================================================================*/
static uint8_t sd_note_failure(uint8_t *streak, uint8_t err, uint8_t site)
{
	(*streak)++;
	sd_set_err(err, site);                  /* ★ Y2 + 2026-10-19：失败原因【和站点】一起上浮 */
	if (*streak >= SD_FAIL_STREAK_LIMIT)
	{
		g_sd_error = 1;
		sd_unmount_force();
		return 1;
	}
	return 0;
}

/*============================================================================
 *  任务本体
 *==========================================================================*/
/* ★★★ 2026-10-19 加：从一批 CSV 文本里取【首行 / 末行】的 seq 和 rtc ✗✓
 *   ★ 为什么连 rtc 一起取 ✗（★ 这是实测抓到的缺陷 ✓）：
 *     · 一开始我只记 `seq=A-B` ✗
 *     · ★★ 但 **`seq` 每次重启都从 1 重数** ⇒ 同一个 seq 范围
 *       在一个文件里【出现十几次】✓
 *     · ★ 实测：`seq=258~267` 在文件里出现了 **10 次**（10 个会话）✓✓
 *     ⇒ ★★ 光有 seq 范围【根本定位不到是哪几行】✓
 *     ⇒ 加上【那一行的 rtc】⇒ `(rtc, seq)` 组合才唯一 ✓✓
 *   ★ 不引 `atoi` ✗ 省 flash ✓ */
static void Sd_ParseRowRange(const uint8_t *p, uint16_t len,
                             uint32_t *seq1, uint32_t *rtc1,
                             uint32_t *seq2, uint32_t *rtc2)
{
	uint16_t i;
	uint16_t k;
	uint16_t field;

	if (seq1) { *seq1 = 0u; }
	if (rtc1) { *rtc1 = 0u; }
	if (seq2) { *seq2 = 0u; }
	if (rtc2) { *rtc2 = 0u; }
	if ((p == NULL) || (len == 0u)) { return; }

	/* ---- 首行：从第 0 字节开始 ✓ ---- */
	i = 0u;
	if (seq1)
	{
		while ((i < len) && (p[i] >= (uint8_t)'0') && (p[i] <= (uint8_t)'9'))
		{
			*seq1 = (*seq1) * 10u + (uint32_t)(p[i] - (uint8_t)'0');
			i++;
		}
	}
	if (rtc1)
	{
		field = 0u;
		while ((i < len) && (p[i] != (uint8_t)'\n'))
		{
			if (p[i] == (uint8_t)',') { field++; }
			else if ((field == 3u) && (p[i] >= (uint8_t)'0') && (p[i] <= (uint8_t)'9'))
			{
				*rtc1 = (*rtc1) * 10u + (uint32_t)(p[i] - (uint8_t)'0');
			}
			i++;
		}
	}

	/* ---- 末行：退掉行尾 \r\n ✗ 再退回最后一个 \n 之后 ✓ ---- */
	i = len;
	while ((i > 0u) && ((p[i - 1u] == (uint8_t)'\n') || (p[i - 1u] == (uint8_t)'\r'))) { i--; }
	k = i;
	while ((k > 0u) && (p[k - 1u] != (uint8_t)'\n')) { k--; }

	if (seq2)
	{
		uint16_t j = k;
		while ((j < len) && (p[j] >= (uint8_t)'0') && (p[j] <= (uint8_t)'9'))
		{
			*seq2 = (*seq2) * 10u + (uint32_t)(p[j] - (uint8_t)'0');
			j++;
		}
	}
	if (rtc2)
	{
		uint16_t j = k;
		field = 0u;
		while ((j < i) && (p[j] != (uint8_t)'\n'))
		{
			if (p[j] == (uint8_t)',') { field++; }
			else if ((field == 3u) && (p[j] >= (uint8_t)'0') && (p[j] <= (uint8_t)'9'))
			{
				*rtc2 = (*rtc2) * 10u + (uint32_t)(p[j] - (uint8_t)'0');
			}
			j++;
		}
	}
}


/*============================================================================
 *  ★★★★ 2026-10-19 加：**把工作队列抽干 ⇒ 批进缓存 + 缓冲还池** ✗✓
 *
 *  ★ 为什么抽成一个函数 ✗：要在【两处】调 ✓
 *    · 未挂载态每轮开头一次 ✓
 *    · ★★ 以及失败重试的 3 秒里【每 100ms 一次】✓✓
 *
 *  ★★ 为什么要"勤抽" ✗（★ 实测数据推动 ✓）：
 *    · 工作队列深度只有 **2** ✗ ⇒ 一轮最多捞 12 条 ✓
 *    · 原来 **3 秒才抽一次** ⇒ 实测拔卡 23 秒只救回 **78 条（约 34%）**✓
 *    · ★ 计算方法：7 轮 × 12 条 ≈ 84 ✓ 和实测 78 吻合 ✓✓
 *    · ★★ 而生产速度只有 0.55 批/秒 ✗ 抽 10 次/秒 ⇒ 【理论上全救】✓✓
 *
 *  ★ 为什么打印 ✗：抽队列这条路【以前一声不响】✗
 *    ⇒ 日志里只看到"写失败"那 1 次 `cached` ✗ 看不出真存了多少 ✓✓
 *  ★ 返回 ✗：本轮捞到几批 ✓
 *============================================================================*/
/* ★ 2026-10-19：降采样 + 攒批函数的前向声明（★ 定义在 task_SD 之前 ✗ 但被更早的
 *   `Sd_DrainToCache` 调用 ✓ 所以必须先声明 ✓ 否则 AC5 只当它是"外部函数"✗
 *   会报 L6218E 未定义符号 ✓✓）
 * ★★ 注意 ✗：降采样用的【静态变量也必须在前面】✗
 *    因为 `Sd_DrainToCache`（更早）要用它们做"超时刷"✓✓ */
static uint8_t  s_ds_buf[SD_BUF_SIZE];
static uint16_t s_ds_len = 0u;
/* ★★★★★ 2026-10-19 加：攒批缓冲里【现在压着几条记录】✗✓
 *   ★ 为什么必须单独记 ✗：
 *     · `s_ds_len` 是【字节数】✗ 而 `Loss_Add()` 记账的单位是【条】✓
 *     · ★★ 缓存真写失败时 ✗ 要能把"这几条"如实记成 `# LOST`
 *       而不是【一声不吭地丢】✓✓ */
static uint16_t s_ds_lines = 0u;
/* ★★★ 2026-10-19 加：攒批的【起点时刻】✗ 用来做"超时刷" ✓
 *   ★ 为什么需要 ✗（★ 用户提出的窗口问题 ✓）：
 *     · 攒批缓冲在 RAM 里 ✗ 若一直没攒满、卡又没回来 ⇒
 *       这几条数据【既不在 W25 也不在 SD】✓
 *     · ★★ 万一此时断电 ⇒ 那几条丢了 ✓
 *   ★ 修法 ✗：攒批持续超过 `SD_DS_FLUSH_MS` ⇒ **不等攒满也刷** ✓
 *
 *   ★★★ 阈值取 5 秒（★ 2026-10-19 用户拍板 ✓）：
 *     · 用户原话 ✗：「剩下的数据断电没了也没什么所谓，
 *       大部分保在 W25 就行」✓✓
 *     · ★★ 取舍算清楚 ✗：
 *         2 秒 ⇒ 一槽约 2 条 ⇒ 容量 ≈ 4.2 小时 ✗ 窗口 2 秒 ✓
 *         **5 秒 ⇒ 一槽约 5 条 ⇒ 容量 ≈ 10.6 小时** ✓ 窗口 5 秒 ✓
 *     · ★★★ 即"**用最多 5 秒的窗口，换 2.5 倍的缓存时长**"✓
 *       （★ 断卡是大故障 ✗ 保住 10 小时的数据远比"最后 5 秒"重要 ✓）
 *   ★ 注意 ✗：攒满 512 B（约 5 条）时【仍然立刻刷】✗ 不等 5 秒 ✓
 *     ⇒ 实际每槽接近 5 条 ⇒ 容量真的能到 ~10.6 小时 ✓✓ */
static uint32_t s_ds_t0  = 0u;
#define SD_DS_FLUSH_MS  5000u

static uint16_t Sd_DsPush(const uint8_t *src, uint16_t len);
static uint16_t Sd_DsFlush(void);           /* ★ 返回【真存进缓存几批】✓ */

static uint16_t Sd_DrainToCache(void)
{
	SdMsg_t  msg;
	uint16_t n      = 0u;               /* ★ 从队列里【抽走】几批 ✓ */
	uint16_t stored = 0u;               /* ★★ 其中【真存进缓存】几批（★ 2026-10-19 加 ✓）*/

	while (xQueueReceive(s_xQueueSdPtr, &msg, 0) == pdPASS)
	{
		/* ★★★ 2026-10-19 加：**降采样 + 攒批**（★ 只留每秒第一条 ✓）
		 *   · 省 10 倍空间 ⇒ 容量 76 分钟 → 约 **10.6 小时** ✓✓
		 *     （★ 这个数字【2026-10-19 实测校过】✗ 见下面 `Sd_DsPush` 那段 ✓）
		 *   · ★ 返回"本次存进缓存的批数"✗ 0 = 还在攒（正常 ✓）*/
		stored = (uint16_t)(stored + Sd_DsPush(msg.buf, msg.len));  /* ★ 只累计"真存进去"的 ✓ */
		(void)xQueueSend(s_xQueueFree, &msg.buf, 0); /* ★ 还池 ⇒ PROC 不再 no_buf ✓ */
		n++;
	}

	/* ★★★★ 2026-10-19 加：**超时刷**（★ 用户提出的窗口问题 ✓）
	 *   ★ 真空窗口 ✗：攒批缓冲在 RAM 里 ✗
	 *     若一直没攒满、卡又没回来 ⇒ 这几条【既不在 W25 也不在 SD】✓
	 *     此时断电 ⇒ 最多丢 5 条 ✓
	 *   ★ 本函数【每 100ms 被调一次】（★ 3 秒节流里切的 ✓）
	 *     ⇒ 攒批超过 `SD_DS_FLUSH_MS`（★ 现在 = **5000ms** ✗ 用户拍板 ✓）就刷 ✓
	 *       ⇒ **窗口 ≤ 5 秒** ✓✓
	 *       （★ 原来这里写"超过 2 秒就刷 / 窗口 ≤ 2 秒"✗ 与宏不符 ✓
	 *         ★★ 2026-10-19 修 ✓ 免得以后有人按"2 秒"去算窗口 ✓）*/
	if ((s_ds_len != 0u) && (s_ds_t0 != 0u) &&
	    ((uint32_t)((uint32_t)xTaskGetTickCount() - s_ds_t0) >= SD_DS_FLUSH_MS))
	{
		stored = (uint16_t)(stored + Sd_DsFlush());  /* ★ 这才是"真存进去"的大头 ✓ */
	}

	if (n != 0u)
	{
		/* ★★★ 2026-10-19 改：打印里加 `ovr=`（环形溢出次数 ✓）
		 *   · `ovr` 不动 ⇒ 缓存没满 ✓ 一切正常
		 *   · ★★ `ovr` 在涨 ⇒ 缓存满 ✗ 正在丢【最旧】的数据 ✓✓
		 *     ⇒ 那说明"卡太久没回来" ✗ 该查卡 ✓ 不是缓存的问题 ✓
		 *
		 * ★★★★ 2026-10-19 再改（★ 交接文档 P1 第 7 条 ✗ 这个数字一直虚高 ✓）：
		 *   · 原来打的是 `n` = 【从队列里抽走几批】✗
		 *   · ★★ 而降采样 + 攒批之后 ✗ 抽走【不等于】存进 W25 ✓
		 *     （★ 很可能还在 RAM 里攒着 ✓）
		 *     ⇒ 实测（掉卡 41 秒那次）✗ 这一行打了 **69 次**
		 *       而真存进缓存的只有 **8 批** ✓✓ 差 8 倍 ✗ 判据会被带偏 ✓
		 *   · ★★★ 现在 ✗ `stored` 累计 `Sd_DsPush()` 的返回值
		 *     = 【真存进缓存的批数】✓
		 *     并把 RAM 里还攒着几条也报出来（`ram=`）✓✓ */
		Serial_Printf("SD: drained %u batch (stored %u, ram=%u recs) pending=%u ovr=%u\r\n",
		              (unsigned)n,
		              (unsigned)stored,
		              (unsigned)s_ds_lines,
		              (unsigned)DropCache_PendingBatches(),
		              (unsigned)DropCache_OverflowCount());
	}
	return n;
}


/*============================================================================
 *  ★★★★★ 2026-10-19 加：**降采样 —— 只留"每秒第一条"**（★ 用户提议 ✓）
 *
 *  ★ 为什么 ✗（★ 实测需求 ✓）：
 *    · 卡坏掉时数据只能进 W25 ✗ 而缓存容量有限 ✓
 *    · 10 条/秒 ⇒ 7616 批 × 6 条 ≈ **76 分钟**就满 ✓
 *    · ★★ 一满就环形覆盖 ⇒ 丢掉【最早的】✗
 *      （★ 整夜实测：开机 21 分钟满 ⇒ 之后 8 小时全丢 ✓✓）
 *
 *  ★ 做法 ✗：进缓存前【只保留每个 RTC 秒的第一条】✓
 *    · 10 条/秒 ⇒ 每秒留 1 条 ⇒ **省 10 倍空间** ✓✓
 *    · 容量：7616 批 × 6 条(每批 6 秒) ≈ **12 小时**（★ 76 分钟 → 12 小时 ✓）
 *
 *  ★★ 为什么用【记录自己的 rtc】当判据 ✗：
 *    · 若用"每 10 条留 1 条"✗ 丢批/重启会让【相位漂掉】✓
 *    · 用 rtc ⇒ 【自对齐 ✗ 不用维护相位】✓✓
 *
 *  ★★★ 代价（必须说清）✗：故障期间分辨率 10 条/秒 → **1 条/秒** ✓
 *    · 但保住的行【都是真数据】（rtc / CRC 都对 ✓）
 *    · ★ 卡正常时【照旧全分辨率】✗ 只有进缓存才降 ✓✓
 *
 *  ★ 原地压缩 ✗：输出长度 ≤ 输入 ⇒ 安全 ✓
 *  ★ 状态 ✗：`s_ds_last_rtc` 跨批保留 ⇒ 同一秒跨批也不会重复留 ✓
 *  ★ 边界 ✗：最后一行可能没有 '\n' ⇒ 全程 `w < len` 保护 ✓✓
 *============================================================================*/
static uint32_t s_ds_last_rtc = 0u;

/* ★★ 攒批缓冲的【定义已挪到文件前面】✗ 因为 `Sd_DrainToCache` 要用它做超时刷 ✓
 *   （★ 见文件上方 `s_ds_buf / s_ds_len / s_ds_t0 / SD_DS_FLUSH_MS` ✓）*/

/*============================================================================
 *  ★★★★ 降采样 + 攒批：把一批原始记录"过滤 + 攒起来" ✗✓
 *  ★ 输入 ✗：`src`（一批 6 条原始 CSV ✗ 512 B 上限 ✓）
 *  ★ 输出 ✗：`s_ds_buf`（只含"每秒第一条"✗ 攒满就自动存进缓存 ✓）
 *  ★ 返回 ✗：本次【成功存进缓存的批数】✓
 *    ★★★★ 2026-10-19 改 ✗：【调用者【不能】拿它判"有没有丢"】✓✓
 *      · 0 = 还在攒（★ 数据在 RAM ✗ 没丢 ✓）✗ 与"存失败"【无法区分】✓
 *      · ★★ 所以"丢没丢"的记账 ✗ 【在本函数内部完成】✓
 *        （★ 失败时 `Loss_Add(LOSS_SD_WRITE, s_ds_lines)` ✓
 *          `Sd_DsFlush()` 里同款 ✓）
 *      · ★ 返回值的用处只剩【统计 / 调试】✓
 *  ★★ 为什么攒 ✗：一条降采样记录约 100 B ✗ 而槽是 1024 B ✓
 *    直接一条一槽 ⇒ 浪费 90% ⇒ 时长只有 2.1 小时 ✓✓
 *    攒成 5 条一槽 ⇒ **10.6 小时** ✓✓
 *
 *  ★★★★★ 2026-10-19：**这两个数【实测校过】了** ✗✓
 *    · 两次环满实测（32 槽测试版）✗ 回写汇总行给的是
 *      **`replay done 30 batches / 168 recs`** 与 **`12 batches / 70 recs`** ✓
 *      ⇒ 平均 **5.6 ~ 5.8 条 / 批**（★ 不是写死的 5 条 ✓）
 *    · ★★ 降采样后进缓存的速率实测约 **1.03 ~ 1.14 条/秒** ✓
 *    · ⇒ 容量 = 7616 槽 × 5.7 条 ÷ 1.1 条/秒 ≈ **39,000 秒 ≈ 10.8 小时** ✓✓
 *      （★ 与上面 10.6 小时吻合 ✗ 所以这个注释【现在算实测值】✓）
 *    · ★★★ 注意 ✗ 它取决于【平均行宽】（★ 数值位数一变就变 ✓）
 *      想更准就按 `replay done` 那行反算 ✗ 别信估的 ✓✓
 *      （★ 行宽 = 512 B ÷ 5.7 条 ≈ **90 B/条** ✓）
 *============================================================================*/
static uint16_t Sd_DsPush(const uint8_t *src, uint16_t len)
{
	uint16_t i     = 0u;
	uint16_t saved = 0u;

	while (i < len)
	{
		uint16_t end = i;
		uint16_t k;
		uint16_t n;
		uint16_t need;
		uint32_t rtc   = 0u;
		uint8_t  field = 0u;

		while ((end < len) && (src[end] != (uint8_t)'\n')) { end++; }
		n = (uint16_t)(end - i);
		if (n == 0u)
		{
			if (end >= len) { break; }
			i = (uint16_t)(end + 1u);
			continue;
		}

		/* ★ 取第 4 个字段 = rtc ✗ 逗号分隔 ✓ */
		for (k = i; k < end; k++)
		{
			if (src[k] == (uint8_t)',') { field++; }
			else if ((field == 3u) && (src[k] >= (uint8_t)'0') && (src[k] <= (uint8_t)'9'))
			{
				rtc = rtc * 10u + (uint32_t)(src[k] - (uint8_t)'0');
			}
		}

		/* ★ 只留"和上一条不同秒"的那一行 ✓ */
		if ((rtc != 0u) && (rtc != s_ds_last_rtc))
		{
			s_ds_last_rtc = rtc;
			need = (uint16_t)(n + 1u);              /* ★ 含结尾 '\n' ✓ */

			if ((uint16_t)(s_ds_len + need) > SD_BUF_SIZE)
			{
				/* ★ 装不下 ⇒ 先把攒的存掉（★ 这就是"攒批"✓）
				 *   ★★★★ 2026-10-19 修（★ 回归：静默丢 ✗✓）：
				 *     · 原来【不管存成没存成】都 `s_ds_len = 0` ✗
				 *       ⇒ 缓存写失败时这几条【直接没了 ✗ 也不记账】✓✓
				 *     · ★★ 现在 ✗ 失败就【按条】记进 `# LOST` ✓
				 *       （★ 单位是"条"✗ 所以要用 `s_ds_lines` ✗ 不是 `s_ds_len` ✓）*/
				if (s_ds_len != 0u)
				{
					if (DropCache_PutBatch(s_ds_buf, s_ds_len) == 1u)
					{
						saved++;
					}
					else
					{
						Loss_Add(LOSS_SD_WRITE, s_ds_lines);
						Serial_Printf("SD: cache put FAILED -> lost %u recs\r\n",
						              (unsigned)s_ds_lines);
					}
				}
				s_ds_len   = 0u;
				s_ds_lines = 0u;
			}

			if ((uint16_t)(s_ds_len + need) <= SD_BUF_SIZE)
			{
				for (k = 0u; k < n; k++) { s_ds_buf[s_ds_len + k] = src[i + k]; }
				s_ds_len = (uint16_t)(s_ds_len + n);
				s_ds_buf[s_ds_len] = (uint8_t)'\n';
				s_ds_len++;
				s_ds_lines++;                       /* ★ 又压进来一条 ✓ */

				/* ★★ 2026-10-19 加：记下"攒批起点"✗ 供超时刷用 ✓ */
				if (s_ds_t0 == 0u) { s_ds_t0 = (uint32_t)xTaskGetTickCount(); }
			}
			else
			{
				/* ★★★★ 这条【当前到不了】✗✓
				 *   · 上面刚把缓冲清空 ✗ 而单行 `need ≤ 约 125` ⇒ 必装得下 ✓
				 *   · ★★ 但"到不了"【不该靠人脑记住】✗✓
				 *     ⇒ 【留一行兜底 ✗ 真丢了也记账】✓✓ */
				Loss_Add(LOSS_SD_WRITE, 1u);
			}
		}

		if (end >= len) { break; }
		i = (uint16_t)(end + 1u);
	}
	return saved;
}

/* ★ 把攒着的那点也存掉 ✗✓（★ 卡回来了 ⇒ 必须先刷 ✓ 否则卡在 RAM 里没人搬 ✓）
 *   ★★ 2026-10-19 加：同时复位"攒批起点"✗ 让下次重新计时 ✓ */
static uint16_t Sd_DsFlush(void)
{
	uint16_t r = 0u;

	s_ds_t0 = 0u;
	if (s_ds_len == 0u) { return 0u; }

	/* ★★★★ 2026-10-19 修（★ 回归：静默丢 ✗✓）：
	 *   · 原来【不看返回值】就 `s_ds_len = 0` ✗
	 *     ⇒ 缓存写失败时攒着的这几条【直接没了 ✗ 也不记账】✓✓
	 *   · ★★ 这里【必须记账】✗ 因为攒批缓冲是"卡不在时的落脚点"✗
	 *     它写不进缓存 ⇒ 【这几条真的没了 ✗ 只有记账能留下证据】✓✓
	 *
	 * ★★★★★ 2026-10-19 再修（★ `stored` 打印曾经恒为 0 ✓）：
	 *   · ★ 实测发现 ✗ 攒批【真正落进缓存的大头就是这里】
	 *     （★ 5 秒超时刷 ✓ 比"攒满 512B"更常触发 ✓）
	 *   · ★★ 而调用方 `Sd_DrainToCache()` 里的 `stored`
	 *     只累计了 `Sd_DsPush()` 的返回 ⇒ **恒为 0** ✗✓
	 *   · ★★★ 所以本函数【必须把战果返给调用者】✓ */
	if (DropCache_PutBatch(s_ds_buf, s_ds_len) != 1u)
	{
		Loss_Add(LOSS_SD_WRITE, s_ds_lines);
		Serial_Printf("SD: cache flush FAILED -> lost %u recs\r\n", (unsigned)s_ds_lines);
	}
	else
	{
		r = 1u;                                     /* ★ 真存进 1 批 ✓ */
	}

	s_ds_len   = 0u;
	s_ds_lines = 0u;
	return r;
}


void task_SD(void *pvParameters)
{
	SdMsg_t  msg;
	FRESULT  fr;
	UINT     bw;                                /* f_write 的"实际写入字节数" */
	uint8_t  mounted      = 0;                  /* 0=未挂载态 1=挂载态 */
	uint8_t  fail_streak  = 0;                  /* 连续写失败批数 */

	/* ★★★★★ 2026-10-18 加：本次关机请求【是否已经处理过】✗✓
	 *
	 *  ★ 为什么【必须有】✗（★ 这是我自己改出来的 bug，实测抓到的 ✓）：
	 *    · 关机分支在循环顶部 ✗ `g_shutdown_req` 不撤就一直轮到 ✓
	 *    · ★★ 而第一趟处理完会把 `mounted = 0`（已卸卷 ✓）
	 *    · ★★★ 于是第二趟就进了 `else` 分支
	 *      → 【重复打印 NOTHING saved】✗ 每 200ms 一条 = 刷屏 ✓
	 *      → 更糟：【把 `g_shutdown_saved` 从 1 改回 0】✗
	 *        ⇒ 屏上本该 `DATA SAVED`，结果变成 `NOT SAVED !!` ✓✓
	 *
	 *  ★ 语义 ✗：1 = 这一轮关机【已经落盘 + 关文件 + 判定完毕】✓
	 *    撤消退出的那条路会把它清 0 ✓（下次长按还能重新走一遍 ✓）*/
	uint8_t  shutdown_handled = 0;

	/* ★★ 2026-10-19 加：回写统计（★ 用来写 `# SD_REPLAY` 那一行 ✓）
	 *   · 一批批累计 ✗ 等【缓存彻底排空】的那一刻写一行总结 ✓
	 *   · ★ 为什么不在每批后写 ✗：会有几十行噪声 ✗ 而且没信息量 ✓ */
	static uint32_t s_rp_batches = 0u;   /* 本次连续回写了几批 */
	static uint32_t s_rp_recs    = 0u;   /* 本次共回写多少条 */
	static uint32_t s_rp_first   = 0u;   /* 第一行的 seq */
	static uint32_t s_rp_last    = 0u;   /* 最后一行的 seq */
	static uint32_t s_rp_rtc1    = 0u;   /* ★ 第一行的 rtc（★ seq 会重数 ⇒ 必须配 rtc ✓）*/
	static uint32_t s_rp_rtc2    = 0u;   /* ★ 最后一行的 rtc */

	/* ★★★ 2026-10-18 加：本次故障【从什么时候开始】✗✓
	 *   · 0 = 当前没在故障 ✓
	 *   · 非 0 = 故障起始的 Unix 秒 ⇒ 恢复时能算出"断了多久" ✓ ✓
	 *   ★ 为什么需要它 ✗：复盘里那条"恢复即抹掉故障史"✓
	 *     → 光清标志的话【曾经断记多久】就彻底没了 ✓✓ */
	uint32_t s_err_since = 0;

	/* ★★ 2026-10-18 加：故障期间【最后一次的错误码】✗
	 *   ★ 为什么不直接用 `g_sd_last_err` ✗：
	 *     · 关机时我们会把 `g_sd_last_err` 清 0（让屏回到 E:00 ✓）
	 *     · ★★ 但恢复时那行 `# SD_RECOVER` 还要报"当时是多少"✓
	 *   ⇒ 单独存一份 ✓ */
	uint8_t  s_err_code = 0;
	uint32_t sync_counter = 0;                  /* 距上次 f_sync 的批数 */
	uint32_t file_idx     = 1;                  /* ★ 已废弃（2026-10-10）✗：
	                                             *   文件名改成 YYMMDDNN.CSV 后不再需要序号 ✓
	                                             *   保留变量只为不改进 sd_mount_and_open 的签名 ✓
	                                             *   （审查 F9 已确认全项目无其它读者 ✓）*/

	/* ★ R4 修复（2026-10-05 审查）：内核对象没建好就绝不往下走 ——
	 * 否则 xQueueReceive(NULL) 会直接 configASSERT（vAssertCalled 死循环）或 HardFault。
	 * 用阻塞等待而不是 return：任务函数返回会触发 FreeRTOS 的"任务返回"错误钩子。 */
	while ((s_xQueueSdPtr == NULL) || (s_xQueueFree == NULL))
	{
		g_sd_error = 1;
		vTaskDelay(pdMS_TO_TICKS(1000));
	}

	for (;;)
	{
		/*------------------------------------------------------------
		 *  ★★★★ 安全关机（★ 2026-10-13 深夜加 ✓）
		 *
		 *  ★ 顺序【一步都不能换】✗✓：
		 *    ① 【先把队列里剩下的数据写完】✗ 再关 ✓
		 *       （★ 否则【最后那一批【明明还在内存里 ✗ 却被丢掉】】✓）
		 *    ② `f_sync`  ✗ 把 FS 缓冲真正写进卡 ✓
		 *    ③ `f_close` ✗ 【写回目录项 + 释放文件对象】✓
		 *       （★★ 这一条才是"安全关机"和"直接拔电"的【本质区别】✓）
		 *    ④ `f_mount(NULL)` ✗ 注销卷 ✗ 回到"干净未挂载"状态 ✓
		 *    ⑤ 【最后】才置 done ✗ 告诉 DISP "可以显示了"✓✓
		 *
		 *  ★★ 为什么【必须 continue ✗ 不许再往下走到收队列】✗✓：
		 *    · ★ 关了文件【还去 f_write ✗ 那个文件对象已经无效了】✓
		 *    · ★★ 写它会返回 **`FR_INVALID_OBJECT(9)`**✗
		 *      → 【OLED 上冒出一个 E:09 ✗ 让人以为卡坏了】✓✓
		 *    → ★★★ 【所以【这一轮直接结束】✓✓
		 *
		 *  ★★★ 关机态里【一直在 delay】✗：
		 *    · ★ 不清 done ✗ 让屏幕【保持"可以断电"】✓
		 *    · ★★ 200ms 一跳 ✗ 【不空转烧 CPU】✓✓
		 *----------------------------------------------------------*/
		if (g_shutdown_req != 0u)
		{
			/* ★★★★ 2026-10-18 加：整段【只走一趟】✗✓
			 *   · 不加这一层 ⇒ 200ms 后 `mounted` 已是 0
			 *     ⇒ 进 else 分支 ⇒ 【刷屏 + 覆盖 saved 标志】✓✓
			 *   ★ 先置位再干活 ✗：万一中途 return/continue 也不会漏 ✓ */
			if (shutdown_handled == 0u)
			{
			shutdown_handled = 1u;

			if (mounted)
			{
				/* ★★★★ 2026-10-18 加：先等【尾批】落进队列 ✗✓
				 *
				 *  ★ 为什么必须等 ✗：
				 *    · PROC 的尾批规则是"500ms 没新数据才交"✓
				 *    · ★★ 关机时最后那批【还在 PROC 手里】✗
				 *    · ★★★ 不等 ⇒ 我们这边把队列排空、f_close 掉
				 *      → 那批【之后才入队】⇒ 【永远写不进去】✓✓
				 *      → 而屏上已经写着 `DATA SAVED` ✓（说谎）
				 *
				 *  ★ 为什么是 700ms ✗：PROC 的超时是 500ms
				 *    ⇒ 留 200ms（一拍 DISP）余量 ✓
				 *  ★★ 代价：关机多花 0.7 秒 ✗ 换来"最后一批不丢"✓ */
				vTaskDelay(pdMS_TO_TICKS(700));

				/* ★ ① 把还在队列里的最后一批写完 ✓
				 *   （★ 缓冲【必须还池**✗ 否则 PROC 以后拿不到空缓冲 ✓）*/
				while (xQueueReceive(s_xQueueSdPtr, &msg, 0) == pdPASS)
				{
					/* ★★★ 2026-10-18 改：`f_write` 的返回值【不许丢】✗✓
					 *   · 原来 `(void)f_write(...)` ✗ 写失败【完全静默】✓
					 *   · ★★ 而下面只看 `f_sync` ⇒ 【写没写进去根本不知道】✓
					 *     → 屏上照样 `DATA SAVED` ✓✓
					 *   ⇒ 接住返回值，失败就把错误码留给 OLED/STAT ✓ */
					fr = f_write(&s_log_file, msg.buf, (UINT)msg.len, &bw);
					if (fr != FR_OK)
					{
						sd_set_err((uint8_t)fr, 9u);
						g_sd_error    = 1u;
					}
					(void)xQueueSend(s_xQueueFree, &msg.buf, 0);
				}

				/* ★ ②③④ 落盘 → 关文件 → 卸卷 ✓ */
				fr = f_sync(&s_log_file);
				if (fr == FR_OK)
				{
					fr = f_close(&s_log_file);
				}
				(void)f_mount(NULL, "0:", 0);   /* ★ 无论成败【都要作废卷登记】✓ */
				mounted = 0;

				/* ★★★ 2026-10-18 加：只有【真的关成功】才算"已保存"✓
				 *   ★ 判据 = `f_close` 返回 FR_OK ✓
				 *     （☆ 不信"我写了" ✗ 要看"它真的进去了" ✓ 和 `U` 命令同一个原则）*/
				g_shutdown_saved = (fr == FR_OK) ? 1u : 0u;
				if (fr != FR_OK)
				{
					sd_set_err((uint8_t)fr, 9u);
					g_sd_error    = 1u;
				}

				/* ★ 关键路径【不许静默**✗ 关得成不成功都要说出来 ✓ */
				Serial_Printf("SD: SAFE SHUTDOWN fr=%u saved=%u\r\n",
				              (unsigned)fr, (unsigned)g_shutdown_saved);
			}
			else
			{
				/* ★★★ 2026-10-18 加：从没挂载过 = 【一个字节都没写进卡】✗
				 *   ★ 这种时候屏上【绝不能】显示 `DATA SAVED` ✓
				 *   ★★ 这是复盘里最让人难受的一条 ✗：
				 *     没插卡、按电源键、屏上写着"数据已保存，可以断电"✓✓ */
				g_shutdown_saved = 0u;
				Serial_Printf("SD: shutdown while NOT MOUNTED -> NOTHING saved\r\n");
			}

			/* ★★★★★ 2026-10-18 加：关机完成 ⇒ 【运行期故障标志作废】✗✓
			 *
			 *  ★ 实测痛点（用户 2026-10-18 报）✗：
			 *    · 关机态下蜂鸣器【一直哔哔叫】✗
			 *    · ★★ 而且【把卡插回去也不停】✗ 必须重新开机 ✓✓
			 *
			 *  ★ 为什么 ✗：
			 *    · 关机分支结尾是 `continue` ✗
			 *      ⇒ 【永远不会走到"未挂载态重试挂载"那段】✓
			 *    · ★★ 于是 `g_sd_error` 一旦为 1
			 *      【再也没有任何代码会把它清 0】✓
			 *    · ★★★ 而蜂鸣器正是在 task_DISP 里看 `g_sd_error` 的 ✓
			 *      → 【一直响到重启为止】✓✓
			 *
			 *  ★ 为什么清掉是【对的】而不是掩盖问题 ✗：
			 *    · 此刻状态已经明确 = "已停机"✗
			 *      ★★ "停机后发现卡没了"不是"运行期故障"✓
			 *    · ★ 该不该报警由 `g_shutdown_saved` 决定：
			 *      没存住 ⇒ 屏上 `NOT SAVED !!` ✓（信息一点没丢 ✓）
			 *    · ★★ 而且故障细节已经记进事件文件（`# LOSS` / `# SD_RECOVER`）✓ */
			g_sd_error    = 0u;
			g_sd_last_err = 0u;

			}   /* ★ 结束：if (shutdown_handled == 0u) ✓ */

			g_shutdown_done = 1u;               /* ★ ⑤ 【最后】才说"可以断电"✓ */
			vTaskDelay(pdMS_TO_TICKS(200));
			continue;                           /* ★★ 绝不往下走 ✓ */
		}

		/* ★★ 请求被撤销了（★ 用户又长按了一次 ✓）✗
		 *   → 【清掉 done ✗ 下面"未挂载态"那段【会自己把卡重新挂上】】✓✓
		 *     （★ 这就是【不写新代码就能"恢复记录"**的原因 ✓）*/
		g_shutdown_done  = 0u;
		g_shutdown_saved = 0u;
		shutdown_handled = 0u;              /* ★ 撤销 ⇒ 下次长按可以重新走一遍 ✓ */

		/*------------------------------------------------------------
		 * 未挂载态：周期性重试挂载（开机无卡 / 中途拔卡都会到这里）
		 *----------------------------------------------------------*/
		if (!mounted)
		{
			/* ★★★★★ 2026-10-19 加：**先把工作队列里的批搬进缓存 + 还池** ✗✓
			 *
			 *  ★ 这是本次最值钱的一改 ✗（★ 实测数据推动 ✓）：
			 *    · 实测：拔卡 23 秒 ≈ 230 条 ⇒ 缓存只抓到 **6 条**
			 *      ⇒ ★★ 其余约 224 条【仍然丢了】（96% ✗）✓✓
			 *
			 *  ★ 根因（想清楚才敢改）✗：
			 *    · 未挂载态原来【循环开头就 continue】✗
			 *      ⇒ 【完全不去收工作队列】✓
			 *    · ★★ 而工作队列只有 2 个位置 ⇒ 两块 512B 缓冲
			 *      【永久卡在队列里】✓
			 *    · ★★★ 于是 PROC 每次申请缓冲都失败 ⇒ 一路 `no_buf` ✓
			 *      ⇒ 那些批【从来没走到"写失败"那条路】✗
			 *        ⇒ 【缓存当然抓不到它们】✓✓
			 *
			 *  ★ 修法 ✗：每轮把队列抽干 ⇒ 【批进缓存 + 缓冲还池】✓
			 *    · 还池之后 PROC 马上能拿到空缓冲 ⇒ 【不再 no_buf】✓✓
			 *    · 缓存失败才真的记 LOST ✓（口径保持"真的丢了才记"✓）
			 *  ★ 顺序铁律 ✗：先存缓存，再还池 ✓
			 *    （★ 反过来：缓冲一旦还池就可能被 PROC 覆写 ✗ 数据就错了 ✓）*/
			/* ★★ 抽干队列（★ 实现见 `Sd_DrainToCache` ✓）*/
			(void)Sd_DrainToCache();

			fr = sd_mount_and_open(&file_idx);
			if (fr == FR_OK)
			{
				/* ★★★★ 2026-10-18 加：故障恢复 ⇒ 往事件文件记一笔"断了多久"✗✓
				 * （★ 复盘 §7 ★★ 第 6 条："恢复即抹掉故障史"✓）
				 *
				 *  ★ 原来的错 ✗：挂载成功就把 `g_sd_last_err` 清 0 ✓
				 *    → ★★ 事后屏上永远 `E:00` ✗
				 *      【曾经断记过多久、断过几次 ⇒ 查不出来】✓✓
				 *
				 *  ★ 现在的做法 ✗：
				 *    · 屏上【仍回到 E:00】（当前健康 ✓ 显示语义不变 ✓）
				 *    · ★★ 但卡上留一行：
				 *      `# SD_RECOVER,<rtc>,fr=<码>,down=<断了几秒>`
				 *    ⇒ 事后能查"这台机器运行期间断过几次、每次多久"✓✓ */
				if (s_err_since != 0u)
				{
					char line[120];
					uint32_t now_ts = MyRTC_GetUnix();
					/* ★★★ 2026-10-19 修：`down` 【下溢保护】✗✓
					 *   ★ 实测抓到的错 ✗ 卡上出现过
					 *     `# SD_RECOVER,1792296021,fr=3,down=4294893665s` ✓
					 *   · ★★ 原因：对时把 RTC 往回拨（`T<更早时间>`）⇒
					 *     `now_ts < s_err_since` ⇒ 无符号减法【下溢】✓✓
					 *   · 4.29e9 秒 ≈ 136 年 ✗ 一眼假 ✓
					 *   ⇒ ★ 回拨时【按 0 处理】✗ 绝不写假数 ✓ */
					uint32_t down = (now_ts >= s_err_since) ? (now_ts - s_err_since) : 0u;

					(void)sprintf(line, "# SD_RECOVER,%lu,fr=%u,down=%lus",
					              (unsigned long)now_ts, (unsigned)s_err_code,
					              (unsigned long)down);
					(void)SD_AppendEventLine(line);
					Serial_Printf("SD: recovered after %lus (fr was %u)\r\n",
					              (unsigned long)down,
					              (unsigned)s_err_code);
					s_err_since = 0u;
				}

				mounted     = 1;
				fail_streak = 0;
				g_sd_error  = 0;            /* 复位故障标志：DISP 上摘掉报警 */
				g_sd_last_err = 0;          /* ★ 屏上语义仍是 E:00 = 当前无错 ✓ */

				/* TODO(你): 挂载成功想报个信也行（栈 **384 字**装得下 Serial_Printf，
				 * 但别放进循环高频打）。示例：
				 *   Serial_Printf("SD ready: LOG%04u.CSV\r\n", file_idx); */
			}
			else
			{
				g_sd_error = 1;             /* 无卡/坏卡：亮错误标志（OLED 验收项） */
				/* ★ Y2：把 FRESULT 原码留给 OLED —— 没插卡时 f_mount 会触发
				 *   disk_initialize=完整 SD_Init，失败返回 FR_NOT_READY(3)，
				 *   屏上 E:03 就是"没插卡/接触不良"，而不是笼统的"坏了" */
				sd_set_err((uint8_t)fr, 3u);
				if (s_err_since == 0u)
				{
					s_err_since = MyRTC_GetUnix();
					/* ★★ 2026-10-19 修：这里【也要记原因码】✗✓
					 *   · 原来只在"写失败"那条路设 `s_err_code` ✗
					 *   · ★★ 于是挂载失败导致的故障
					 *     恢复时写出的 `# SD_RECOVER` 是 **`fr=0`** ✓
					 *   · ★ 实测证据：卡上 `# SD_RECOVER,...,fr=0,down=18s`
					 *     ⇒ 【0 = 未知 ✗ 白记一行】✓✓ */
					s_err_code = (uint8_t)fr;
				}
				/* ★★★★ 2026-10-19 改：把 3 秒节流【切成 30 × 100ms】✗✓
				 *
				 *  ★ 原来是什么行为 ✗：
				 *    · `vTaskDelay(SD_REMOUNT_PERIOD_MS)` = 3 秒【裸睡】✗
				 *    · ★★ 这 3 秒里【完全不抽队列】✓
				 *    · 而队列只有 2 位 ⇒ 拔卡 23 秒只救回 78 条（34%）✓✓
				 *
				 *  ★ 现在 ✗：每 100ms 抽一次 ⇒ 抽 10 次/秒 ✓
				 *    · 生产速度 0.55 批/秒 ⇒ 【理论上全救】✓✓
				 *  ★ 为什么不是"不睡、一直抽" ✗：
				 *    · 铁律：任务循环必须有阻塞点 ✓
				 *    · ★★ 而且 100ms 的延迟正是给 PROC/ACQ 让出 CPU ✓
				 *  ★ 总时长不变 ✗：30 × 100ms = 3 秒 ✓（重试节奏不变 ✓）*/
				{
					uint8_t t;

					for (t = 0u; t < 30u; t++)
					{
						(void)Sd_DrainToCache();
						vTaskDelay(pdMS_TO_TICKS(100));
					}
				}
				continue;
			}
		}

		/*------------------------------------------------------------
		 * 挂载态：收缓冲 → 写卡 → sync → 还缓冲
		 * 超时 1s：既是阻塞点（铁律：任务循环必有阻塞），又顺便兜底 sync
		 *----------------------------------------------------------*/
		/* ★★★ 2026-10-18 加：把 RAM 里攒的"丢失记录"落进事件文件 ✓
		 *   · 位置：挂载态、每轮一次 ✓（没卡时攒着 ✗ 有卡了补记 ✓✓）
		 *   · ★ 全为 0 时不做任何 FatFs 操作 ⇒ 平时零开销 ✓ */
		Loss_Flush();

		/* ★★★★ 2026-10-18 加（② 用户拍板 A）：运行期检查【单文件上限】✗✓
		 *
		 *  ★ 原来只在【挂载时】判一次 ✗：
		 *    · 一直写同一个文件（不跨天、不复位）⇒ 【永远不检查】✓
		 *    · ★★ 于是"200 MB 上限"在连续运行时【形同虚设】✓✓
		 *  ★ 现在每轮（约 1 秒）查一次 ✓
		 *    · `f_size()` 在 FatFs 里只是读 `FIL.fsize` 缓存字段
		 *      ⇒ 【不是真 IO】✗ 成本几乎为零 ✓
		 *  ★★ 超限后怎么办 ✗：关文件 + 卸卷 ⇒ 【`mounted=0`】✓
		 *    → 下一轮走"未挂载态重挂" ⇒ 挂载逻辑看到本文件已超限
		 *      ⇒ 【自动试下一个 NN】✓✓（复用现成逻辑，不写新代码 ✓）
		 *  ★ 注意：这里的 `continue` 是【跳过本轮收批】✗ 不影响数据 ✓
		 *    （PROC 侧的缓冲还在队列里 ✓ 下一轮照收 ✓）*/
		if (f_size(&s_log_file) >= (FSIZE_t)SD_FILE_SIZE_LIMIT)
		{
			Serial_Printf("SD: file over %uMB -> roll to next NN\r\n",
			              (unsigned)(SD_FILE_SIZE_LIMIT / (1024u * 1024u)));
			(void)f_sync(&s_log_file);
			(void)f_close(&s_log_file);
			(void)f_mount(NULL, "0:", 0);
			mounted = 0;
			continue;
		}

		/* ★★★★ 2026-10-18 加：**先把缓存搬回来，再收新批** ✗✓
		 *
		 *  ★ 为什么必须"先搬" ✗：新数据源源不断 ⇒ 缓存【永远排不空】✓
		 *  ★★ 每轮搬【一批数据 + 一条事件】✗ 不搞 while 循环
		 *    · 一次最多几十 ms ⇒ 不会把正常落盘拖住 ✓
		 *    · ★ 生产速度 10 条/秒 ≈ 0.55 批/秒 < 1 批/秒 ⇒ 【追得上】✓✓
		 *  ★★★ 顺序铁律 ✗：**Peek → 写成功 → 才 Pop** ✓
		 *    （反过来会出现"指针前进了但数据没写进去"⇒ 真丢 ✓）*/
		/* ★★★ 2026-10-19 加：**回写之前，先把攒着的那点也存进缓存** ✓
		 *   · 卡回来了 ⇒ `s_ds_buf` 里可能还压着几条没攒满的 ✓
		 *   · ★ 不刷的话它们会【卡在 RAM 里】✗ 而那正是"故障最后时刻"的数据 ✓
		 *   · ★★ 刷在这里的好处 ✗：刷完立刻被下面的回写搬进卡 ✓ 一次搬完 ✓✓ */
		(void)Sd_DsFlush();                     /* ★ 这里的战果用不上 ✗ 显式丢弃 ✓ */

		if (DropCache_PendingBatches() != 0u)
		{
			static uint8_t s_cbuf[SD_BUF_SIZE];   /* ★ 静态：task_SD 栈只有 1.5KB ✓ */
			uint16_t clen = 0u;

			/* ★★★★ 2026-10-19 PROBE：先打一行"到底进没进回写块"✓
			 *   ★ 诊断已完成 ⇒ 【探针撤掉】✗
			 *   ★★★ 血的教训 ✗：我原来在这里写了 `if (pk == 0u) { continue; }`
			 *     ⇒ `peek=0` 时【循环里没有任何阻塞点】✗✗
			 *     ⇒ `task_SD` 满速空转 ⇒ 刷爆串口 + 饿死空闲任务
			 *     ⇒ ★ **IWDG 复位**（实测 `G: trigger IWDG reset` ✓）
			 *     ⇒ 然后 Boot 把板子【回滚到金副本】✓✓
			 *   ★ 铁律再念一遍 ✗：**任务循环必须有阻塞点** ✓
			 *     （★ 这个循环下面本来就有 `xQueueReceive(..., 1000ms)` ✓
			 *       我那个 `continue` 把它绕过去了 ✗✓）*/
			if (DropCache_PeekBatch(s_cbuf, &clen) != 0u)
			{
				fr = f_write(&s_log_file, s_cbuf, (UINT)clen, &bw);
				if ((fr == FR_OK) && (bw == (UINT)clen))
				{
					fr = f_sync(&s_log_file);
				}

				if (fr == FR_OK)
				{
					/* ★★★ 2026-10-19 加：统计 + 排空后的总结行 ✓
					 *   ★ 字段含义 ✗：
					 *     `# SD_REPLAY,<rtc>,batches=N,recs=M,seq=A-B`
					 *     · batches = 这次连续搬了几批 ✓
					 *     · recs    = 一共多少条记录（数 `\n` 得到 ✓）
					 *     · seq=A-B = ★ 被回写的【行号范围】✓✓
					 *       ⇒ PC 侧靠它就能【精确圈出"哪几行是补回来的"】✓
					 *         （★ 这些行的 rtc 往往【比周围小】✗ 光看时间认不出来 ✓）*/
					uint32_t sq1 = 0u;
					uint32_t sq2 = 0u;
					uint32_t r1  = 0u;
					uint32_t r2  = 0u;
					uint16_t k;

					Sd_ParseRowRange(s_cbuf, clen, &sq1, &r1, &sq2, &r2);
					if (s_rp_batches == 0u)
					{
						s_rp_first = sq1;
						s_rp_rtc1  = r1;
					}
					s_rp_last = sq2;
					s_rp_rtc2 = r2;
					s_rp_batches++;

					for (k = 0u; k < clen; k++)
					{
						if (s_cbuf[k] == (uint8_t)'\n') { s_rp_recs++; }
					}

					DropCache_PopBatch();

					if (DropCache_PendingBatches() == 0u)
					{
						char rline[180];
						/* ★★ 2026-10-19 改：**加 rtc 范围**✗✓
						 *   ★ 为什么 ✗：`seq` 每次重启从 1 重数 ⇒
						 *     实测同一个 `seq=258~267` 在文件里【出现 10 次】✓
						 *   · ★★ 只有 `(rtc, seq)` 组合才能唯一定位 ✓✓ */
						(void)sprintf(rline,
						              "# SD_REPLAY,%lu,batches=%lu,recs=%lu,seq=%lu-%lu,rtc=%lu-%lu",
						              (unsigned long)MyRTC_GetUnix(),
						              (unsigned long)s_rp_batches,
						              (unsigned long)s_rp_recs,
						              (unsigned long)s_rp_first,
						              (unsigned long)s_rp_last,
						              (unsigned long)s_rp_rtc1,
						              (unsigned long)s_rp_rtc2);

						/* ★ 写不进卡（刚恢复也可能又不稳）⇒ 先塞回缓存 ✓ */
						if (SD_AppendEventLine(rline) != FR_OK)
						{
							(void)DropCache_PutEvent(rline);
						}
						Serial_Printf("SD: replay done %lu batches / %lu recs\r\n",
						              (unsigned long)s_rp_batches,
						              (unsigned long)s_rp_recs);
						s_rp_batches = 0u;
						s_rp_recs    = 0u;
						s_rp_first   = 0u;
						s_rp_last    = 0u;
						s_rp_rtc1    = 0u;
						s_rp_rtc2    = 0u;
					}
					else
					{
						Serial_Printf("SD: cache restored 1 batch (left=%u)\r\n",
						              (unsigned)DropCache_PendingBatches());
					}
				}
				else
				{
					/* ★ 卡又不稳了 ⇒ 指针【不前进】✓ 走错误流程 ✓ */
					sd_set_err((uint8_t)fr, 4u);    /* ★ site 4 = 缓存回写失败 ✓ */
					mounted = 0;
					continue;
				}
			}
			else
			{
				/* ★★★★★ 2026-10-19 修（★ 审计 G10 ✗ 坏项把回写永久卡死 ✓）：
				 *   ★ 什么样的坏项会走到这里 ✗：
				 *     · 上面 `pending != 0` 已经保证"确实有项"✓
				 *     · ★★ 而 `PeekBatch` 仍返回 0 ⇒ 只能是【长度前缀不合法】✓
				 *       （★ 典型值 0xFFFF：槽被擦过 / 被写坏 ✓）
				 *   ★★★ 原来的行为（★ 2026-10-19 实测复现 ✓）✗：
				 *     · 【没有 else】⇒ 什么都不做 ✓
				 *     · ★★ 而 `rd` 【只在写成功后】才前进 ✓
				 *       ⇒ 每一轮都读到同一个坏项 ⇒ **永远卡死** ✓✓
				 *     · 实测：满环后卡回来 ✗ 32 批缓存【被无声遗弃】
				 *       `cache restored` 一条没有 ✗ `replay done` 永远不出现 ✓✓
				 *   ★★★★ 修法 ✗：**跳过它**（Pop 前进）+ **如实记账** ✓
				 *     · ★ 只能按【1 条】记 ✗ 因为槽已坏 ⇒ 里面几条【查不到了】✓
				 *       （★ 宁可少报 ✗ 也不编一个数出来 ✓）
				 *     · ★★ 必须打一行串口 ✗ 这种"整段数据作废"的事
				 *       【绝不能静默】✓✓ */
				DropCache_PopBatch();
				Loss_Add(LOSS_SD_WRITE, 1u);
				Serial_Printf("SD: !! BAD CACHE ITEM -> skipped (recs unknown, counted 1)\r\n");
			}
		}

		/* ★ 事件行也搬一条（★ 目标是 90 文件，和数据批分流 ✓）*/
		if (DropCache_PendingEvents() != 0u)
		{
			char ev[200];

			if (DropCache_PeekEvent(ev, (uint16_t)sizeof(ev)) != 0u)
			{
				if (SD_AppendEventLine(ev) == FR_OK)
				{
					DropCache_PopEvent();
				}
			}
		}

		if (xQueueReceive(s_xQueueSdPtr, &msg, pdMS_TO_TICKS(1000)) == pdPASS)
		{
			/* ★★★★ 2026-10-19 撤探针（★ 审计 D7 ✗ 它【每批都打】✗ 约 2 行/秒噪声 ✓）：
			 *   ★ 它当年是【破 E:09 案的功臣】✗✓ 打的是 FatFs 对象状态 ✓
			 *     · `fstype=0` ⇒ 卷已被 `f_mount(NULL)` 卸掉 ✓
			 *     · `objok=0` ⇒ 文件对象已被 `f_close` 关掉 ✓
			 *     · ★★ 两者都正常却 `fr=9`（FR_INVALID_OBJECT）
			 *       ⇒ 那是 FatFs 的【粘性错误】✗
			 *         （`fp->err` 一旦被 `ABORT` 置上 ✗ 【只由 f_open 清】✓
			 *           ★ 见 `ff.c:3616 / :455 / :3454` ✓）
			 *   · ★★★ 早就定案了 ✗ 现在靠【站点码】定位：
			 *     `sd_set_err(fr, site)` ⇒ 只在【真失败时】打一行 `SD: ERR fr=.. site=..`
			 *     （★ 站点码表见上面 762 行那段 ✓）
			 *   ★ 留着这段说明 ✗ 免得以后有人再去"查一遍 E:09" ✓✓ */
			fr = f_write(&s_log_file, msg.buf, (UINT)msg.len, &bw);

			/* ★★★ 2026-10-18 加：整批丢了 ⇒ 按【条】记账 ✓
			 *   · `msg.nrec` 是 PROC 报上来的"这批几条" ✓
			 *   · ★★ 原来是【完全不计】✗ 丢了也不知道 ✓✓ */
			if (fr != FR_OK)
			{

				/* ★★★★★ 2026-10-19 修（★ 回归：`# LOST` 恒空 ✓）：
				 *   ★★★ 2026-10-18 加的这一步【本意】✗：
				 *     · 写卡失败 ⇒ 先试着存进 W25 缓存 ✗
				 *       【从"丢数据 + 记账"变成"数据保住了"】✓
				 *     · ★ 它和抽队列那条路【共享同一套状态】✗
				 *       `s_ds_last_rtc` + `s_ds_buf` + `s_ds_len` ✓
				 *       ⇒ 两条路混着走【不重复留同一秒 ✗ 也不丢攒着的】✓✓
				 *
				 *   ★★★★ 回归现场 ✗：这里写着 `dc = 1u;`【写死常量】✓
				 *     · 紧跟的 `if (dc == 1u)` 永远成立 ⇒
				 *       `else if (dc == 2u)` 与 `else` 【两支全成死代码】✓✓
				 *     · ★★ 于是【全文件仅有的两处 `Loss_Add()` 都不可达】✗
				 *       ⇒ 【写卡失败再也不会记 `# LOST`】✗
				 *       ⇒ `# LOST` 恒空 ⇒ 【"没丢数据"这个判据自己失效】✓✓
				 *
				 *   ★★★★ 现在怎么记 ✗：
				 *     · `Sd_DsPush()` 返回值【只表示"这次真存进缓存几批"】✗
				 *       0 = 还在攒（★ 数据在 RAM ✗ 没丢 ✓）
				 *       ⇒ 【它【分辨不出】"还在攒"和"存失败"】✓✓
				 *     · ★ 真失败在【内部刷缓存那里】✗
				 *       ⇒ 记账【挪进 `Sd_DsPush` / `Sd_DsFlush`】✓
				 *         （★ 只有那里知道"丢了几条"✗ 见 `s_ds_lines` ✓）
				 *   ★ 所以这里只做【看得见的事】✗ 打一行"交给攒批了" ✓ */
				(void)Sd_DsPush(msg.buf, msg.len);
				Serial_Printf("SD: batch -> cache (%u recs) pending=%u ram=%u recs\r\n",
				              (unsigned)msg.nrec,
				              (unsigned)DropCache_PendingBatches(),
				              (unsigned)s_ds_lines);

				if (s_err_since == 0u)
				{
					s_err_since = MyRTC_GetUnix();
					s_err_code  = (uint8_t)fr;
				}
				/* ★★★ 2026-10-18 加：失败【立刻上屏】✗✓
				 *   ★ 实测痛点（用户 2026-10-18 报）✗：
				 *     · 拔卡后要【约 10 秒】屏上才出现 E:xx ✓
				 *     · ★★ 因为"判离场"要【连续 3 批失败】✓
				 *       （3 × (1.8s 批间隔 + 1.5s 写超时) ≈ 10 秒 ✓）
				 *     · ★★★ 而这 10 秒里【数据一直在静默丢弃】✓
				 *   ⇒ ★ 显示可以快，判离场仍保持"3 连击"不变 ✓
				 *     （`g_sd_last_err` 只驱动屏上那个数字 ✗
				 *       `g_sd_error` 才驱动蜂鸣器和"离场"✓）
				 * ★ 效果：拔卡后【第一批就变 E:xx】✓✓ */
				sd_set_err((uint8_t)fr, 1u);        /* ★ site 1 = 批次写失败（主路径）✓ */
			}

			/* ★ 无论成败先把缓冲还回空闲队列：数据是丢是留是下一行的事，
			 *   缓冲泄漏会让 PROC 拿不到空缓冲、整条链慢慢停摆。
			 * ★ R5 修复：还池失败（队列竟满）说明所有权已被破坏，必须喊出来 */
			if (xQueueSend(s_xQueueFree, &msg.buf, 0) != pdPASS)
			{
				g_sd_error = 1;
			}

			if ((fr != FR_OK) || (bw != (UINT)msg.len))
			{
				/* 写失败（拔卡/坏卡/写保护）——本批数据已丢，只能计数。
				 * ★ Y2：喂给上浮通道的必须是"真原因"——fr 有值用 fr；
				 *   fr==FR_OK 但 bw 短了（理论不出现，防御性兜底）按 FR_DISK_ERR 记，
				 *   绝不能把 0（FR_OK）当错误码存进去，屏上会假显示 E:00
				 *
				 * ★★★★★ 2026-10-19 改（★ 探针实测定案 ✓）：
				 *   ★ 实测（ver 19 探针）✗：
				 *     `SD: pre-write fstype=3 objok=1` ⇒ 【卷有效 ✗ 对象也有效】✓
				 *     可 `f_write` 却返回 **9**（FR_INVALID_OBJECT）✓✓
				 *   ★★★ 真因 = **FatFs 的粘性错误**✗（看 `ff.c` ✓）：
				 *     · `f_write` 开头（ff.c:3616）：
				 *         `if (res != FR_OK || (res = (FRESULT)fp->err) != FR_OK)`
				 *       ⇒ 【直接把对象里粘着的错误码返回】✓
				 *     · 而第一次真失败时内部 `ABORT` 会
				 *         `fp->err = (BYTE)(res)`（ff.c:455）✓
				 *     · ★★ 只有 `f_open` 里那句 `fp->err = 0`（ff.c:3454）
				 *       才能清掉它 ✓✓
				 *   ⇒ ★★★ 所以【后面两次必然也失败】✗ 等 3 连击毫无意义 ✓
				 *     而且屏上会显示【第二次以后的假象】✗
				 *     真实首错（E:01）被盖成 E:09 ✓✓
				 *   ★ 改法 ✗：
				 *     · `fr != FR_OK`（FatFs 层真报错）⇒ **立刻**回未挂载态 ✓
				 *       下一轮重挂载会 `f_open` ⇒ 粘性错误被清 ✓✓
				 *     · `fr == FR_OK` 但短写 ⇒ 仍保留 3 连击容忍 ✓
				 *       （★ 这种是"没写全"✗ 可能瞬时 ✓ 不该立刻重挂 ✓）*/
				if (fr != FR_OK)
				{
					(void)sd_note_failure(&fail_streak, (uint8_t)fr, 5u);
					mounted = 0;        /* ★ 致命错 ⇒ 立刻重挂载（不等 3 连击）✓ */
				}
				else if (sd_note_failure(&fail_streak, (uint8_t)FR_DISK_ERR, 5u))
				{
					mounted = 0;        /* ★ 短写：仍按 3 连击 ✓ */
				}
			}
			else
			{
				fail_streak = 0;            /* 成功一笔，失败连击清零 */
				sync_counter++;

				if (sync_counter >= SD_SYNC_EVERY_BATCH)
				{
					sync_counter = 0;

					/* ★ R3 修复：f_sync 才是"真的进卡"的那一步，它的错必须计数。
					 *   ★ Y2：先接住返回值再判 —— 原来直接 f_sync(...) != FR_OK，
					 *     错误码拿不到，上浮通道无从喂起 */
					fr = f_sync(&s_log_file);
					if (fr != FR_OK)
					{
						if (sd_note_failure(&fail_streak, (uint8_t)fr, 6u))
						{
							mounted = 0;
						}
					}
				}

				/*========================================================================
				 *  ★★ 跨天滚动检查（2026-10-10 加）—— 必须【在运行中也判断】✗ 不只在挂载时 ✓
				 *  为什么：板子连续跑过午夜时【不会重新挂载】✗
				 *    → 若只在挂载时判断，跨天后会【继续写昨天的文件】✗✓
				 *  做法：取"今天"✗ → 与当前文件归属日期比 ✗ → 不同就回未挂载态 ✓
				 *    → 下一轮循环顶部的 `if (!mounted)` 会用【新日期】重新挂载 ✓✓
				 *    → 文件名 YYMMDDNN.CSV 自带日期 ✗ → 自然就是新文件 ✓ 无需额外逻辑 ✓
				 *  ★ 放过期检查的代价：只多一次 RTC 读（寄存器读，微秒级 ✓ 可忽略 ✓）
				 *========================================================================*/
				if (mounted)
				{
					RTC_DateTime_t now;
					MyRTC_GetDateTime(&now);
					if (SD_ROLL_KEY(now) != s_cur_key)      /* ★ 周期变了 → 换文件 ✓ */
					{
#if SD_DEBUG_ROLL_MINUTE
						SD_TRACE("SD: [DEBUG] minute changed -> roll file (old key=%lu)\r\n",
						              (unsigned long)s_cur_key);
#else
						Serial_Printf("SD: day changed -> roll file (old %04u-%02u-%02u)\r\n",
						              (unsigned)((s_cur_key >> 16) & 0xFFFFu),
						              (unsigned)((s_cur_key >>  8) & 0xFFu),
						              (unsigned)( s_cur_key        & 0xFFu));
#endif
						fr = f_sync(&s_log_file);      /* ★ 旧文件先落盘 ✓ 再关 ✓（别丢尾巴 ✓）*/
				/* ★★★ 2026-10-18 改：返回值【不许丢】✗✓（复盘 T5）
				 *   · 原来 `(void)f_sync` ✗ 而它正是【R3 修复专门改过】的地方 ✓
				 *   · ★★ 跨天这两处【漏改】⇒ 若拔着卡跨天
				 *     落盘失败【不报错、不响蜂鸣、不置 g_sd_error】✓✓
				 *     紧接着 `mounted` 被手工置 0 ⇒ 【错误被淹没】✓ */
				if (fr != FR_OK)
				{
					sd_set_err((uint8_t)fr, 8u);
					g_sd_error    = 1u;
					if (s_err_since == 0u) { s_err_since = MyRTC_GetUnix(); }
				}
						(void)f_close(&s_log_file);
						mounted = 0;                    /* ★ 回未挂载态 → 下一轮按新周期重挂 ✓✓ */
					}
				}
			}
		}
		else
		{
			/* 1 秒没收到数据（暂停采集/空闲）：兜底 sync，保证秒级落盘口径。
			 * 文件没脏（wflag=0）时 f_sync 内部是空操作，白调一次不伤卡
			 * ★ R3 修复：这里的错同样要计数 —— 正是"暂停时拔卡"那条路径 */
			if (mounted)
			{
				/* ★ Y2：同上，先接住 f_sync 返回值，失败码喂进上浮通道
				 *   （正是"暂停采集时拔卡"那条路径：E:01=FR_DISK_ERR） */
				fr = f_sync(&s_log_file);
				if (fr != FR_OK)
				{
					if (sd_note_failure(&fail_streak, (uint8_t)fr, 7u))
					{
						mounted = 0;
					}
				}
			}
		}
	}
}
