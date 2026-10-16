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
 *    "SD: all N names failed"     —— 99 个全试完 ✓
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

/* 交接消息：指针 + 有效长度（len 单独传，CSV 尾批不满 512B 也能写对） */
typedef struct {
    uint8_t  *buf;
    uint16_t len;
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
BaseType_t SD_LogEnqueue(uint8_t *buf, uint16_t len)
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

	msg.buf = buf;
	msg.len = len;

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
					break;                  /* ★ 今天的这个还没到兜底 → 续写它 ✓ */
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
				/* ★★★ ③ > 1 天 ✗ → 【报警 + 留证据 + 【切换到 90】】✓✓ */
				Serial_Printf("SD: TIME BACKWARD %lds (>1d) - SWITCH to evidence file\r\n",
				              (long)diff);

				fr = SD_AppendEvidence(name, last_rtc, now_ts, lastline,
				                       evname, (uint16_t)sizeof(evname));
				if (fr == FR_OK)
				{
					/* ★ 切换 ✗：关主文件 → 打开 90（★ 里面已有表头 ✓）→ 定位末尾 ✓ */
					(void)f_close(&s_log_file);
					fr = f_open(&s_log_file, evname, FA_WRITE | FA_READ | FA_OPEN_ALWAYS);
					if (fr == FR_OK)
					{
						/* ★★ 把 name 换成 90 ✗ 让后面的 "SD: log" 显示对的名字 ✓
						 *   （★ 手工拷贝 ✗ 不为这个引入 string.h 依赖 ✓）*/
						uint8_t k = 0u;
						while ((evname[k] != '\0') && (k < 23u))
						{
							name[k] = evname[k];
							k++;
						}
						name[k] = '\0';
						fr = f_lseek(&s_log_file, f_size(&s_log_file));
					}
				}
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
static uint8_t sd_note_failure(uint8_t *streak, uint8_t err)
{
	(*streak)++;
	g_sd_last_err = err;                    /* ★ Y2：失败原因上浮（OLED E:xx） */
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
void task_SD(void *pvParameters)
{
	SdMsg_t  msg;
	FRESULT  fr;
	UINT     bw;                                /* f_write 的"实际写入字节数" */
	uint8_t  mounted      = 0;                  /* 0=未挂载态 1=挂载态 */
	uint8_t  fail_streak  = 0;                  /* 连续写失败批数 */
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
			if (mounted)
			{
				/* ★ ① 把还在队列里的最后一批写完 ✓
				 *   （★ 缓冲【必须还池**✗ 否则 PROC 以后拿不到空缓冲 ✓）*/
				while (xQueueReceive(s_xQueueSdPtr, &msg, 0) == pdPASS)
				{
					(void)f_write(&s_log_file, msg.buf, (UINT)msg.len, &bw);
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

				/* ★ 关键路径【不许静默**✗ 关得成不成功都要说出来 ✓ */
				Serial_Printf("SD: SAFE SHUTDOWN fr=%u (file closed)\r\n", (unsigned)fr);
			}

			g_shutdown_done = 1u;               /* ★ ⑤ 【最后】才说"可以断电"✓ */
			vTaskDelay(pdMS_TO_TICKS(200));
			continue;                           /* ★★ 绝不往下走 ✓ */
		}

		/* ★★ 请求被撤销了（★ 用户又长按了一次 ✓）✗
		 *   → 【清掉 done ✗ 下面"未挂载态"那段【会自己把卡重新挂上】】✓✓
		 *     （★ 这就是【不写新代码就能"恢复记录"**的原因 ✓）*/
		g_shutdown_done = 0u;

		/*------------------------------------------------------------
		 * 未挂载态：周期性重试挂载（开机无卡 / 中途拔卡都会到这里）
		 *----------------------------------------------------------*/
		if (!mounted)
		{
			fr = sd_mount_and_open(&file_idx);
			if (fr == FR_OK)
			{
				mounted     = 1;
				fail_streak = 0;
				g_sd_error  = 0;            /* 复位故障标志：DISP 上摘掉报警 */
				g_sd_last_err = 0;          /* ★ Y2：错误史一并清 0（E:00 = 无错误史） */

				/* TODO(你): 挂载成功想报个信也行（栈 256 字装得下 Serial_Printf，
				 * 但别放进循环高频打）。示例：
				 *   Serial_Printf("SD ready: LOG%04u.CSV\r\n", file_idx); */
			}
			else
			{
				g_sd_error = 1;             /* 无卡/坏卡：亮错误标志（OLED 验收项） */
				/* ★ Y2：把 FRESULT 原码留给 OLED —— 没插卡时 f_mount 会触发
				 *   disk_initialize=完整 SD_Init，失败返回 FR_NOT_READY(3)，
				 *   屏上 E:03 就是"没插卡/接触不良"，而不是笼统的"坏了" */
				g_sd_last_err = (uint8_t)fr;
				/* 重试不能裸转：SD_Init 失败链本身 ~1s，再垫 3s 节流，
				 * 保证本任务在故障期 CPU 占用可忽略 */
				vTaskDelay(pdMS_TO_TICKS(SD_REMOUNT_PERIOD_MS));
				continue;
			}
		}

		/*------------------------------------------------------------
		 * 挂载态：收缓冲 → 写卡 → sync → 还缓冲
		 * 超时 1s：既是阻塞点（铁律：任务循环必有阻塞），又顺便兜底 sync
		 *----------------------------------------------------------*/
		if (xQueueReceive(s_xQueueSdPtr, &msg, pdMS_TO_TICKS(1000)) == pdPASS)
		{
			fr = f_write(&s_log_file, msg.buf, (UINT)msg.len, &bw);

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
				 * 连续 3 批都失败 = 卡离场（驱动层每批已各自重试 3 次）：
				 * 置错误标志、强制卸载，回未挂载态等插回（T2.6③ 验收路径）
				 * ★ Y2：喂给上浮通道的必须是"真原因"——fr 有值用 fr；
				 *   fr==FR_OK 但 bw 短了（理论不出现，防御性兜底）按 FR_DISK_ERR 记，
				 *   绝不能把 0（FR_OK）当错误码存进去，屏上会假显示 E:00 */
				if (sd_note_failure(&fail_streak,
				                    (fr != FR_OK) ? (uint8_t)fr : (uint8_t)FR_DISK_ERR))
				{
					mounted = 0;
				}
				/* 不足 3 次：吞掉本次失败继续收下一批 —— 瞬时干扰在驱动层
				 * 重试里大概率已被救回，到这里的失败多半真是卡要走了 */
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
						if (sd_note_failure(&fail_streak, (uint8_t)fr))
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
						(void)f_sync(&s_log_file);      /* ★ 旧文件先落盘 ✓ 再关 ✓（别丢尾巴 ✓）*/
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
					if (sd_note_failure(&fail_streak, (uint8_t)fr))
					{
						mounted = 0;
					}
				}
			}
		}
	}
}
