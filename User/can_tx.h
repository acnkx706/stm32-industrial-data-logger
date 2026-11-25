#ifndef __CAN_TX_H
#define __CAN_TX_H

/*==============================================================================
 *  CAN 发送模块（阶段 B：记录仪 -> 网关）
 *
 *  ★ 编译开关
 *    CAN_EN = 0：整个模块不参与编译，产物与 ver79 逐字节一致
 *    CAN_EN = 1：启用 CAN1，PB8/PB9 重映射，500 kbps
 *
 *  ★ 为什么用开关而不是直接改
 *    ver79 是 72 小时连续运行验证过的版本，不能随手改坏。
 *    先用开关隔离，验证"开关关掉后产物完全一致"，确认隔离干净，再打开。
 *
 *  ★ 为什么用 CAN 而不是串口
 *    串口是点对点、线短、没有寻址；CAN 是差分、多主、有硬件 CRC 和错误界定，
 *    工业现场的总线形态。
 *============================================================================*/

#define CAN_EN   1

/* ★ 回环自测开关
 * · 1 = CAN 内核内部回环，不接收发器也能自己发自己收，用来验证位时序和配置
 * · 0 = 正常总线模式，必须有第二个节点应答 ACK
 * · 第一次上板时先用 1 验证一遍，通过了再改回 0
 */
#define CAN_TX_LOOPBACK   0

#if (CAN_EN != 0)

#include <stdint.h>

/* ★ 报文 ID 分配
 * · 0x1xx 是记录仪上报，0x2xx 是网关下发命令，0x3xx 是应答
 * · 2026-11-24 补上命令和应答两条：F407 从"只会收"变成"能主动控制"
 */
#define CAN_ID_TELEMETRY   0x101u   /* 记录仪 -> 网关：温度 / 湿度 / 压力 */
#define CAN_ID_STATUS      0x102u   /* 记录仪 -> 网关：序号 / 状态 */
#define CAN_ID_CMD         0x201u   /* 网关 -> 记录仪：命令 */
#define CAN_ID_ACK         0x301u   /* 记录仪 -> 网关：命令应答 */
#define CAN_ID_ALARM       0x080u   /* 记录仪 -> 网关：故障告警 */
#define CAN_ID_EXTRA       0x103u   /* 记录仪 -> 网关：ax ay az vbat */
#define CAN_ID_EXTRA2      0x104u   /* 记录仪 -> 网关：gx gy gz light */

/* ★ 命令码：和 F407 的 can_gw.h 里的 CAN_CMD_* 必须一一对应
 * · 改了任何一边都要同步改另一边，否则 ACK 会回一个对端不认识的码
 */
#define CAN_CMD_REC_START   0x01u   /* 开始记录 */
#define CAN_CMD_REC_STOP    0x02u   /* 停止记录 */
#define CAN_CMD_PAGE_NEXT   0x03u   /* 翻页 */
#define CAN_CMD_BUZZ        0x04u   /* 蜂鸣一声 */
#define CAN_CMD_REPORT_NOW  0x10u   /* 立即上报一次状态 */

/* ★ 命令接收（2026-11-24 加）
 * · 中断里调 CAN_CmdOnFrame，只把命令码存进一个单槽
 * · 任务里调 CAN_CmdTake 取走执行，然后用 CAN_CmdAck 回一条 0x301
 * · 为什么单槽不是队列：现场命令最多每秒几条，队列要多占 RAM，单槽够用
 * · 覆盖式：上一条还没执行完又来一条，保留最新的那条
 */
void CAN_CmdOnFrame(uint16_t id, const uint8_t *d, uint8_t dlc);
uint8_t CAN_CmdTake(uint8_t *cmd);
void CAN_CmdAck(uint8_t cmd, uint8_t result);

/* ★ 初始化：配时钟、重映射引脚、位时序、过滤器
 * · 必须在调度器启动之前调用，和队列、互斥量同一个时序要求
 * · 失败时静默返回（不打印），靠统计量从串口命令观察
 */
void CAN_TX_Init(void);

/* ★ 发一帧标准数据帧
 * · 返回 1 = 成功装进发送邮箱，0 = 彻底没位置（会先撤一帧遥测再试）
 * · 返回成功只代表装进邮箱，不代表总线上已经发完
 */
uint8_t CAN_TX_SendStd(uint16_t id, const uint8_t *data, uint8_t len);
/* ★ 发一帧，邮箱满了就等一下再试
 * · retry 是总尝试次数，两次尝试之间睡 1 ms
 * · 返回 1 = 已装进邮箱，0 = 试满 retry 次还是没有空邮箱
 * · 只能在任务里调，中断里用上面那个
 */
uint8_t CAN_TX_SendStdWait(uint16_t id, const uint8_t *data, uint8_t len, uint8_t retry);

/* ★ 回环自测：发一帧并等它发完
 * · 只在 CAN_TX_LOOPBACK = 1 时有意义
 * · 返回 1 = 邮箱完成（回环下说明位时序和配置都对），0 = 超时
 */
uint8_t CAN_TX_SelfTest(void);

/* ★ 统计量
 * · ok = 成功装进邮箱的帧数，fail = 没有空邮箱的次数
 * · 单节点挂在总线上时，帧发不出去、邮箱不释放，fail 会一直涨
 */
extern volatile uint32_t g_can_tx_ok;
extern volatile uint32_t g_can_tx_fail;

#endif /* CAN_EN */

#endif /* __CAN_TX_H */
