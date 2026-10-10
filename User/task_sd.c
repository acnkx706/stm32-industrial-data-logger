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
  *        │  每 3 秒重试：f_mount→(disk_initialize=完整 SD_Init)→开/建 LOGxxxx.CSV
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
#include <stdio.h>                      /* sprintf：拼 "0:LOG0001.CSV" 文件名 */
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
#define SD_FILE_IDX_MAX         9999/* LOG%04u 的上限，到头回绕到 1（TODO 见下） */

/*============================================================================
 *  FatFs 对象（★ 必须全局 static 放置，不能进任务栈：
 *  TINY=1 后 FATFS≈550B + FIL≈40B，放栈上 1KB 直接爆）
 *==========================================================================*/
static FATFS s_fs;                      /* 卷对象（f_mount 登记） */
static FIL   s_log_file;                /* 日志文件（挂载态唯一打开的写文件） */
static FIL   s_probe_file;              /* 探测序号用的临时文件对象 */

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
static FRESULT sd_mount_and_open(uint32_t *pidx)
{
	FRESULT fr;
	char name[16];                      /* "0:LOG0001.CSV" = 13 字符 + '\0' */

	/* f_mount(opt=1)：强制挂载 —— 会触发 disk_initialize，即完整 SD_Init
	 * （74 时钟/CMD0/ACMD41/切 9MHz 全在里头）。没插卡在这里就会失败 */
	fr = f_mount(&s_fs, "0:", 1);
	if (fr != FR_OK)
	{
		return fr;                      /* FR_NOT_READY/FR_DISK_ERR = 卡不在或坏 */
	}

	/* 找空位：从 LOG0001.CSV 起逐个探测，"打不开"才说明这个序号空闲。
	 * 为什么不用 f_getfree/读目录：_FS_MINIMIZE=2 已裁，探测是裁剪后最省的招
	 * ★ R2 修复（2026-10-05 审查）：扫描必须带上限！
	 *   原来是无界 for(;;) 且"序号超过 9999 就回绕到 1" —— 而 LOG0001.CSV 必然存在，
	 *   于是永远转下去：循环里没有 vTaskDelay、也没有队列操作，prio 1 的 task_SD
	 *   空转占满 CPU，工作队列再没人取 -> PROC 两块缓冲只进不出 -> 全链路静默丢数据，
	 *   而且 g_sd_error 恒 0（OLED 不报警）。触发条件很现实：FAT 根目录满 / 卡满。
	 *   现在：整圈扫完仍无空位 -> 明确返回失败，由未挂载态重试 + 报警，不静默覆盖旧日志。 */
	{
		uint32_t scanned = 0;

		for (;;)
		{
			sprintf(name, "0:LOG%04u.CSV", (unsigned int)(*pidx));

			fr = f_open(&s_probe_file, name, FA_READ | FA_OPEN_EXISTING);
			if (fr == FR_NO_FILE)
			{
				break;                      /* 这个序号空闲，用它 */
			}
			if (fr != FR_OK)
			{
				return fr;                  /* 其他错误（读着读着卡没了等）：上浮 */
			}
			(void)f_close(&s_probe_file);   /* 已存在 -> 试下一个序号 */

			(*pidx)++;
			if (*pidx > SD_FILE_IDX_MAX)
			{
				*pidx = 1;                  /* 回绕：只作为"继续找空位"的起点 */
			}

			if (++scanned >= (uint32_t)SD_FILE_IDX_MAX)
			{
				return FR_DENIED;           /* 整圈占满：明确失败（上层报警，不覆盖旧日志） */
			}
		}
	}

	/* ★ R6 语义定版（2026-10-05 审查后拍板）：**滚动新文件**，不做续写。
	 *   一次"挂载" = 一个 LOGxxxx.CSV（上电/拔插后自然新开一个）。
	 *   为什么选它而不是"追加续写"：
	 *     ① 不怕"文件尾部被写坏后还往上追加"；② 会话边界清晰（tick 复位点 = 断电点）；
	 *     ③ PC 端按序号拼接即可；④ 少一个"续写状态"要维护。
	 *   注意：上面的探测用的是"第一个不存在的序号"，所以这里 f_open 必然走新建分支、
	 *        f_lseek 恒为 0 —— 保留它只是为了"文件若已存在也不截断"。
	 *   （28 号验收③ 的表述随之改为"新开下一个 LOGxxxx.CSV 且不丢数据"） */
	fr = f_open(&s_log_file, name, FA_WRITE | FA_OPEN_ALWAYS);
	if (fr == FR_OK)
	{
		fr = f_lseek(&s_log_file, f_size(&s_log_file));
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
			}
		}
	}
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
	uint32_t file_idx     = 1;                  /* LOG 序号，从 0001 起 */

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
