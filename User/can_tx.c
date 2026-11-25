/**
  ******************************************************************************
  * @file    can_tx.c
  * @brief   记录仪 CAN1 发送（PB8/PB9 重映射，500 kbps）
  ******************************************************************************
  * ★ 位时序怎么算
  * · APB1 是 36 MHz，预分频 4 得到 9 MHz 的 tq 时钟
  * · 一位 18 个 tq，9 MHz 除以 18 就是 500 kbps
  * · 采样点在第 14 个 tq，占比 77.8%
  *
  * ★ 为什么是 18 个 tq
  * · bxCAN 规定 BS1 最多 16 tq、BS2 最多 8 tq，一位最多 25 tq
  * · 36 MHz 下要凑 500 kbps，18 tq 是唯一解：1 同步 + 13 BS1 + 4 BS2
  *
  * ★ 影响什么
  * · 网关那边是 42 MHz 下 21 tq，采样点 80.95%
  * · 两边差 3.1%（约 0.65 个 tq），短总线上在容差范围内
  *
  * ★ 怎么验证
  * · CAN_EN 打开后编译，串口命令里看 g_can_tx_ok 是否随采集节拍增长
  * · 接上网关后，网关侧接收计数应该和这里的 ok 同步
  ******************************************************************************
  */

#include "can_tx.h"

#if (CAN_EN != 0)

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"

volatile uint32_t g_can_tx_ok   = 0u;
volatile uint32_t g_can_tx_fail = 0u;

/* ★ 时钟、重映射、引脚
 * · CAN1 默认在 PA11/PA12，这两个脚是 USB 的 D-/D+，所以改到 PB8/PB9
 * · 重映射必须开 AFIO 时钟，漏了这一步 GPIO_PinRemapConfig 会静默失败
 */
void CAN_TX_Init(void)
{
    GPIO_InitTypeDef      gpio;
    CAN_InitTypeDef       can;
    CAN_FilterInitTypeDef filter;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);

    /* PB9 = CAN_TX，复用推挽输出 */
    gpio.GPIO_Pin   = GPIO_Pin_9;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    /* PB8 = CAN_RX，上拉输入
     * ★ 上拉是必须的：没有收发器时这条线悬空会被读成低电平（显性），
     *   bxCAN 要检测到 11 个连续隐性位才能退出初始化模式，悬空就永远卡住
     */
    gpio.GPIO_Pin  = GPIO_Pin_8;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &gpio);

    CAN_DeInit(CAN1);

    can.CAN_TTCM      = DISABLE;
    can.CAN_ABOM      = ENABLE;   /* bus-off 自动恢复，现场被干扰后能自己回来 */
    can.CAN_AWUM      = DISABLE;
    can.CAN_NART      = DISABLE;  /* 允许自动重传，保证送达 */
    can.CAN_RFLM      = DISABLE;
    can.CAN_TXFP      = DISABLE;  /* 按 ID 优先级发送，不按入队顺序 */
    can.CAN_Mode      = (CAN_TX_LOOPBACK != 0) ? CAN_Mode_LoopBack : CAN_Mode_Normal;
    can.CAN_SJW       = CAN_SJW_1tq;
    can.CAN_BS1       = CAN_BS1_13tq;
    can.CAN_BS2       = CAN_BS2_4tq;
    can.CAN_Prescaler = 4;

    if (CAN_Init(CAN1, &can) != CAN_InitStatus_Success)
    {
        return;
    }

    /* ★ 过滤器全通过
     * · 这次只做发送，过滤器留着是为了以后接收网关命令
     * · 32 位掩码模式，ID 和掩码全填 0 就是"什么都收"
     */
    filter.CAN_FilterNumber         = 0;
    filter.CAN_FilterMode           = CAN_FilterMode_IdMask;
    filter.CAN_FilterScale          = CAN_FilterScale_32bit;
    filter.CAN_FilterIdHigh         = 0x0000;
    filter.CAN_FilterIdLow          = 0x0000;
    filter.CAN_FilterMaskIdHigh     = 0x0000;
    filter.CAN_FilterMaskIdLow      = 0x0000;
    filter.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    filter.CAN_FilterActivation     = ENABLE;
    CAN_FilterInit(&filter);
}

/* ★ 最近一次装进邮箱的遥测帧占了哪个邮箱（0 到 2，0xFF = 还不知道）
 * · 只记遥测帧，从不记关键帧，所以撤掉它一定是可以丢的那种
 * · 关键帧挤不进去时靠它抢一个位置
 */
static volatile uint8_t s_mb_tele = 0xFFu;

/* ★ 投递一帧，不计统计（内部用）
 * · 返回 1 = 已装进邮箱，0 = 三个邮箱全满
 * · out_mb 在成功时带回邮箱号（0 到 2），调用者不要传空
 */
static uint8_t can_tx_raw(uint16_t id, const uint8_t *data, uint8_t len, uint8_t *out_mb)
{
    CanTxMsg msg;
    uint8_t  mb;
    uint8_t  i;

    if (len > 8u)
    {
        len = 8u;
    }

    msg.StdId = (uint16_t)(id & 0x7FFu);
    msg.ExtId = 0u;
    msg.IDE   = (uint8_t)CAN_Id_Standard;
    msg.RTR   = CAN_RTR_Data;
    msg.DLC   = len;

    for (i = 0u; i < len; i++)
    {
        msg.Data[i] = data[i];
    }

    mb = CAN_Transmit(CAN1, &msg);

    if (mb == CAN_TxStatus_NoMailBox)
    {
        return 0u;
    }

    *out_mb = mb;
    return 1u;
}

/* ★ 发一帧，不等待；挤不进去就撤掉一帧遥测
 *
 * ★ 为什么要抢占
 *   遥测一拍发四帧，占了三个邮箱还要再等 1 ms，这 1 ms 里邮箱一直是满的。
 *   应答帧 0x3E0 撞进这个窗口就被丢掉：遥测丢一帧只是曲线少个点，
 *   应答丢一帧 PC 得整块重传，还会被判成"块序号错"（NRC 0x73）。
 *   实测：四帧同拍后 315 块 OTA 第 8 块、第 20 块各挂一次；三帧交替时一次过。
 *
 * ★ 为什么只撤遥测
 *   s_mb_tele 只在遥测那条路上写，所以撤掉的一定是周期重发、丢了不心疼的帧。
 *
 * ★ 为什么中断里也敢调
 *   全程只有寄存器操作，没有 vTaskDelay。
 */
uint8_t CAN_TX_SendStd(uint16_t id, const uint8_t *data, uint8_t len)
{
    uint8_t mb;

    if (can_tx_raw(id, data, len, &mb) == 0u)
    {
        if (s_mb_tele != 0xFFu)
        {
            CAN_CancelTransmit(CAN1, s_mb_tele);
            s_mb_tele = 0xFFu;
        }

        if (can_tx_raw(id, data, len, &mb) == 0u)
        {
            g_can_tx_fail++;
            return 0u;
        }
    }

    g_can_tx_ok++;
    return 1u;
}

/* ★ 发一帧，邮箱满了睡 1 ms 再试（2026-11-24 加）
 *
 * ★ 为什么要这个函数
 *   一帧 8 字节标准帧占总线 111 位，500 kbps 下要走 222 us，
 *   而打包一帧只要几十 us。所以一拍连发四帧时，第 4 帧去要邮箱那一刻，
 *   前三个都还在发，必然撞满；以前的做法是让它直接失败，0x104 一帧都没到过网关。
 *
 * ★ 为什么等 1 ms 这么粗
 *   一个 tick 就是 1 ms，够 222 us 的帧时间空出至少一个邮箱；再细要动定时器，不值。
 *   代价：最坏一次 PROC 多花 1 ms，占 100 ms 周期的百分之一。
 *
 * ★ 只能在任务里调
 *   内部调了 vTaskDelay，中断里调会触发断言。中断里继续用 CAN_TX_SendStd。
 *
 * ★ 怎么验证
 *   网关侧 0x103 和 0x104 的到达速率应该一样（各每秒 10 帧），改之前 0x104 是每秒 0 帧。
 */
uint8_t CAN_TX_SendStdWait(uint16_t id, const uint8_t *data, uint8_t len, uint8_t retry)
{
    uint8_t n;
    uint8_t mb;

    for (n = 0u; n < retry; n++)
    {
        if (can_tx_raw(id, data, len, &mb) != 0u)
        {
            s_mb_tele = mb;
            g_can_tx_ok++;
            return 1u;
        }

        if ((uint8_t)(n + 1u) < retry)
        {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    g_can_tx_fail++;
    return 0u;
}
/* ★ 回环自测
 * · 发一帧然后等发送邮箱完成
 * · 回环模式下 CAN 内核自己给自己应答，不需要第二个节点
 * · 返回 1 说明位时序、邮箱、过滤器全部配置正确
 * · 注意：这里是忙等，只允许在调度器启动之前调用一次
 */
uint8_t CAN_TX_SelfTest(void)
{
    CanTxMsg msg;
    uint8_t  mb;
    uint8_t  i;
    uint32_t t;

    msg.StdId = CAN_ID_TELEMETRY;
    msg.ExtId = 0u;
    msg.IDE   = (uint8_t)CAN_Id_Standard;
    msg.RTR   = CAN_RTR_Data;
    msg.DLC   = 4u;
    msg.Data[0] = 0x11u;
    msg.Data[1] = 0x22u;
    msg.Data[2] = 0x33u;
    msg.Data[3] = 0x44u;
    for (i = 4u; i < 8u; i++)
    {
        msg.Data[i] = 0u;
    }

    mb = CAN_Transmit(CAN1, &msg);
    if (mb == CAN_TxStatus_NoMailBox)
    {
        return 0u;
    }

    for (t = 0u; t < 200000u; t++)
    {
        if (CAN_TransmitStatus(CAN1, mb) == CAN_TxStatus_Ok)
        {
            return 1u;
        }
    }

    return 0u;
}

/*==============================================================================
 *  ★ 命令接收与应答（2026-11-24 加）
 *
 *  ★ 做什么
 *    网关往 0x201 发一帧命令，本模块在中断里把命令码存进一个单槽；
 *    任务侧取走执行，再用 0x301 回一条应答。
 *
 *  ★ 为什么命令码只存一个不是队列
 *    现场命令最多每秒几条，队列要多占几十字节 RAM，单槽就够。
 *    覆盖式写入：上一条还没执行完又来一条，保留最新的那条 —— 后到的才是当前意图。
 *
 *  ★ 为什么先读码再清标志
 *    反过来会有一个窗口：标志已清、码还没读，这时中断写进来的新码
 *    会被下一次 Take 读到，但 pending 已经没了，命令就丢。先读后清没有这个窗口。
 *============================================================================*/

static volatile uint8_t s_cmd_code;      /* 待执行的命令码 */
static volatile uint8_t s_cmd_pending;   /* 1 = 有命令待取 */

void CAN_CmdOnFrame(uint16_t id, const uint8_t *d, uint8_t dlc)
{
    if ((id != CAN_ID_CMD) || (dlc < 1u))
    {
        return;
    }

    s_cmd_code    = d[0];
    s_cmd_pending = 1u;
}

/* ★ 2026-11-24 审计修：读码与清标志必须在同一个临界区里
 * · 原来的注释说"先读后清没有窗口"，这是错的：
 *   中断若在 c = s_cmd_code 之后、s_cmd_pending = 0 之前写入新码并置 pending=1，
 *   随后任务把 pending 清 0 —— 那条新命令就永久丢了
 * · 关中断只有几条指令，对 CAN 收发影响可忽略
 * · 这里用 __disable_irq 而不是 taskENTER_CRITICAL：can_tx.c 不依赖 FreeRTOS，
 *   而且本函数只在任务上下文调用，关中断的代价是几百纳秒
 */
uint8_t CAN_CmdTake(uint8_t *cmd)
{
    uint8_t c;
    uint8_t got;

    __disable_irq();

    got = s_cmd_pending;
    c   = s_cmd_code;
    s_cmd_pending = 0u;

    __enable_irq();

    if (got == 0u)
    {
        return 0u;
    }

    if (cmd != 0)
    {
        *cmd = c;
    }

    return 1u;
}

void CAN_CmdAck(uint8_t cmd, uint8_t result)
{
    uint8_t d[8];

    d[0] = cmd;
    d[1] = result;
    d[2] = 0u;
    d[3] = 0u;
    d[4] = 0u;
    d[5] = 0u;
    d[6] = 0u;
    d[7] = 0u;

    /* 发不出去就算了：丢一条 ACK 不影响功能，网关那边有超时兜底 */
    (void)CAN_TX_SendStd(CAN_ID_ACK, d, 8u);
}

#endif /* CAN_EN */
