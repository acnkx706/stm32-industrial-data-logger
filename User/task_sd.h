#ifndef __TASK_SD_H
#define __TASK_SD_H

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "queue.h"

/*==============================================================================
 * task_SD —— SD 卡落盘任务（执行表 T2.3 / T2.5）
 *
 *  架构位置（已评审采纳）：
 *    task_ACQ(6) --队列--> task_PROC(2) --乒乓缓冲传指针--> task_SD(1) --> FatFs
 *
 *  ★ 优先级 1（最低业务级）的理由：卡写 busy 最坏 250ms，只有把 SD 操作压在
 *    最低优先级，这段慢才被隔离在 task_SD 里 —— ACQ→PROC 采集链永不阻塞。
 *  ★ 栈 256 字（=1KB）的理由：f_open/f_write 调用链 + sprintf 拼文件名
 *    实测占用几百字节；先按惯例给 256，用 uxTaskGetStackHighWaterMark()
 *    实测后按"×1.5 定容"惯例调整（skill 规范）。
 *
 *  ★ 铁律（_FS_REENTRANT=0 的前提）：全工程【只有本文件】允许调用 f_xxx
 *    和 diskio；其他任务想写卡，只能走 SD_LogEnqueue 交缓冲。
 *
 *  ── 与 task_PROC 的交接（乒乓双缓冲，改动点见 README 集成步骤 §3）──
 *    缓冲指针在两条队列间循环流转，任一时刻只在一个人手里（无拷贝、无竞争）：
 *    1. PROC 侧：SD_BufAcquire(&buf) 申请空缓冲 → 填 CSV 行 →
 *       SD_LogEnqueue(buf, len) 交给 task_SD
 *    2. task_SD 侧：从队列收 {指针,长度} → f_write → f_sync → 还回空闲队列
 *    缓冲定义在本文件（static 512B×2，全局放置不占任务栈，见执行表 RAM 总账）
 *============================================================================*/

#define TASK_SD_PRIO            1   /* 最低业务优先级：SD 慢被隔离在此，不碰采集链 */
#define TASK_SD_STACK_SIZE      256 /* 单位：字（=1KB）。上线后用高水位实测定容 */

#define SD_BUF_SIZE             512 /* 一块 = SD 卡一个扇区：写大块最省协议开销 */

/* 乒乓双缓冲：PROC 填其中一块，task_SD 写另一块，交替无拷贝
 * ★ 定义在 task_sd.c（全局 static 区），extern 只是为了 PROC 侧能填它 */
extern uint8_t g_sd_buf[2][SD_BUF_SIZE];

/* SD 卡故障标志：1 = 卡不在/连续写失败（丢批中），插回自动恢复后清 0
 * ★ volatile：DISP/PROC 只读它，task_SD 写它，不加会被编译器缓存进寄存器 */
extern volatile uint8_t g_sd_error;

/* ★ Y2 修复（第 6 课）：最近一次失败的原因码 —— g_sd_error 只能说"坏了"，
 *   它才能说"为什么"。当前喂进来的都是 FRESULT（f_mount/f_write/f_sync 的
 *   返回值，0~19；FR_OK=0/FR_NOT_READY=3/FR_DISK_ERR=1/FR_DENIED=7...），
 *   将来若接 SD 驱动原始错误码（SD_Result_t），需要再扩一位"来源域"区分。
 *   挂载成功时随 g_sd_error 一起清 0 —— OLED 上 E:00 = 无错误史。
 *   volatile：task_SD 写、DISP 只读，跨任务可见性由它保证 */
extern volatile uint8_t g_sd_last_err;

void task_sd_Init(void);                /* 建交接队列+信号量（调度器启动【前】调一次） */
void task_SD(void *pvParameters);       /* 任务本体（start_task 里创建） */

/* ── PROC 专用的交接接口（两步走，配对使用，见 task_sd.c 顶注释的时序图）──
 * ① SD_BufAcquire(&buf)：申请一块空缓冲的所有权（拿不到=pdFAIL，两块都在 task_SD 手里）
 * ② 填好后 SD_LogEnqueue(buf, len) 交出去；交不出去（pdFAIL）就换一块再试
 * ★ 拿了不交 = 缓冲泄漏，整条链会慢慢停摆 —— 申请和交接必须成对出现在代码里 */
BaseType_t SD_BufAcquire(uint8_t **pbuf);
BaseType_t SD_LogEnqueue(uint8_t *buf, uint16_t len);

#endif
