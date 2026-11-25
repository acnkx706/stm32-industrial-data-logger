#ifndef __ISO_TP_H
#define __ISO_TP_H

/*==============================================================================
 *  ISO 15765-2（ISO-TP）传输层 —— CAN-OTA 的地基
 *
 *  ★ 做什么
 *    把一条长报文拆成若干个 8 字节 CAN 帧发出去，接收侧再拼回来。
 *    本模块实现：接收方向的多帧重组 + 流控帧；发送方向只做单帧。
 *
 *  ★ 为什么发送只做单帧
 *    UDS 里 Tester 到 ECU 是长报文（要拆帧），ECU 到 Tester 的响应几乎都短：
 *    50 02 / 7F 36 73 / 76 01 / 74 20 01 02 / 67 01 + 4 字节种子，全部不超过 7 字节。
 *    一个单帧装得下 ⇒ 不提前写用不到的多帧发送，等真要 ECU 发长报文再补。
 *
 *  ★ 影响什么
 *    CAN_EN 打开才编译。静态缓冲 288 字节（ZI 区），这部分 RAM 正式版也要留。
 *
 *  ★ 怎么验证
 *    PC 发一条 256 字节报文，把 TEST_ISO_TP_DUMP 开成 1，
 *    串口应打回长度、CRC16 和头 16 字节，且与 PC 算的一致。
 *    详细判据见 77 号文档 §7 第 1 条。
 *============================================================================*/

#include "can_tx.h"          /* ★ 拿 CAN_EN，避免两处宏各说各话 */

#if (CAN_EN != 0)

#include <stdint.h>

/* ---- CAN ID ------------------------------------------------------------- */

/* ★ 沿用本工程的编号规律：0x2xx 网关 -> 记录仪，0x3xx 记录仪 -> 网关
 *   · 0x201/0x301 是配置命令/应答，保持原义，不用来传固件
 *   · ISO-TP 需要一问一答两条独立 ID，所以另开 0x2E0/0x3E0 */
#define ISOTP_ID_REQ     0x2E0u    /* 网关 -> 记录仪：ISO-TP 物理请求 */
#define ISOTP_ID_RSP     0x3E0u    /* 记录仪 -> 网关：ISO-TP 物理响应 */

/* ---- 长度与时间参数 ----------------------------------------------------- */

/* ★ 288 = UDS 块长上限 258 + 30 的余量
 *   · 258 来自 77 号文档 §3.4 定的 maxNumberOfBlockLength
 *   · 取 288 是因为 F103 只有 20 KB RAM，这块钱要省着花 */
#define ISOTP_RX_BUF     288u

#define ISOTP_BS         8u        /* 收 8 个连续帧回一个流控帧 */
/* ★ 2026-11-24 压力测试记录（127 那次）
 * · 把这里临时改成 127 可以让网关每帧多等 127 ms，
 *   配合 F407 的 STmin 钳位（5 ms）就能量出钳位是否真的生效
 * · 实测：钳位前 PING 响应 4601 ms，钳位后 334 ms（差 13.8 倍）
 * · 正常值必须是 0：F103 的 CAN 只有 3 个邮箱，限速只会拖慢 OTA */
#define ISOTP_STMIN      0u        /* 连续帧最小间隔（毫秒），0 = 不限速 */

/* ★ 超时按"毫秒"算，不按"轮询次数"算
 *   · 原来写成 ISOTP_N_CR_POLLS = 1000 / ISOTP_POLL_MS，两个宏必须成对改
 *   · 2026-11-23 实测就吃了这个亏：把容忍度从 10 次改成 30 次想绕过问题，
 *     结果无效 —— 因为真正要改的是"轮询够不够勤"，不是"次数够不够多"
 *   · 改成毫秒之后，以后换轮询周期只需改 ISOTP_POLL_MS（仅作文档说明），
 *     超时行为不会跟着漂
 *   ★ 中断里仍然一个时间函数都不调：时间戳只在任务侧的 IsoTp_Poll 里打 */
#define ISOTP_POLL_MS    20u      /* 推荐的轮询周期，实际由调用者的任务节拍决定 */
#define ISOTP_N_CR_MS    3000u    /* 等下一个连续帧最多等多久（毫秒） */

/* ---- PCI（协议控制信息）：看每帧首字节的高 4 位 ------------------------ */

#define ISOTP_PCI_SF     0x00u     /* 单帧：首字节 0x0L，L = 数据长度 1~7 */
#define ISOTP_PCI_FF     0x10u     /* 首帧：首字节 0x1H + 第二字节 L，共 12 位长度 */
#define ISOTP_PCI_CF     0x20u     /* 连续帧：首字节 0x2S，S = 序号 0~15 */
#define ISOTP_PCI_FC     0x30u     /* 流控帧：首字节 0x3F，F = 流控状态 */

#define ISOTP_FC_CTS     0x00u     /* ClearToSend：继续发 */
#define ISOTP_FC_WAIT    0x01u     /* 先等一下 */
#define ISOTP_FC_OVFLW   0x02u     /* 缓冲放不下，别发了 */

/* ---- 接口 --------------------------------------------------------------- */

/* ★ 打开接收中断
 * · 必须在 CAN_TX_Init() 之后调：它依赖 CAN_TX_Init 配好的过滤器和位时序
 * · 内部只配 NVIC 和 FIFO0 中断使能，不动引角和波特率 */
void IsoTp_Init(void);

/* ★ 任务侧轮询：只做超时检查
 * · 必须按 ISOTP_POLL_MS 的周期调用，否则超时判据失真
 * · 取包交给 IsoTp_Take()，两件事分开，调用者才能按自己的节奏取 */
void IsoTp_Poll(void);

/* ★ 取一包收完的报文，直接把内部缓冲的指针给出去（不拷贝）
 * · 返回 NULL = 没有整包；否则把长度写进 *len
 * ★ 用完必须调一次 IsoTp_Release()，否则下一包会被"缓冲还占着"挡掉
 * · 开 TEST_ISO_TP_DUMP 时，取走的那一刻串口会打一行长度+CRC16+头16字节 */
const uint8_t *IsoTp_Take(uint16_t *len);

/* ★ 交还缓冲，Take 成功之后必须调用一次 */
void IsoTp_Release(void);

/* ★ 发一个单帧，最多 7 字节
 * · 返回 1 = 装进邮箱；0 = 长度非法或没有空邮箱
 * · UDS 的所有肯定响应和否定响应都走它 */
uint8_t IsoTp_SendSf(uint16_t id, const uint8_t *data, uint8_t len);

/* ---- 统计量：从串口命令观察，不占 Flash 里的字符串 ---------------------- */

extern volatile uint32_t g_isotp_rx_sf;     /* 收到的单帧数 */
extern volatile uint32_t g_isotp_rx_msg;    /* 重组完成的报文数 */
extern volatile uint32_t g_isotp_rx_drop;   /* 序号错 / 超时 / 溢出 / 缓冲满 */
extern volatile uint32_t g_isotp_fc_tx;     /* 发出的流控帧数 */

/* ★ 最后一次丢包的原因码（2026-11-23 加）
 * · 只记"为什么丢"，不记次数；次数看 g_isotp_rx_drop
 * · 1=帧太短 2=单帧长度非法 3=单帧时上包没取走 4=首帧长度非法
 *   5=首帧时上包没取走 6=没在接收态却收到连续帧 7=连续帧序号不符
 *   8=等连续帧超时
 * · 加它的原因：传整包时出现"某一块无响应"，F407 侧 drop=0 说明帧发出去了，
 *   但没法区分是 F103 的 ISO-TP 丢了、还是 UDS 层没回 —— 只能靠这个码定位 */
extern volatile uint32_t g_isotp_drop_why;
/* ★ 2026-11-24 加：上一条收完的报文（长度 + CRC16），供 UDS 0x22 读
 * · 回归靠它判断"256 字节有没有原样收回来"，不再依赖串口打印 */
extern volatile uint16_t g_isotp_prev_len;
extern volatile uint16_t g_isotp_prev_crc;

#endif /* CAN_EN */

#endif /* __ISO_TP_H */
