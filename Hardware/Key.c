#include "stm32f10x.h"          // Device header
#include "Key.h"

/*
 * 非阻塞按键扫描（三态状态机版）
 *
 * 旧版的问题（改版原因，防重蹈）：
 *  - while 死等松手：按住按键时把调用任务绑死在原地，低优先级任务被饿
 *  - 消抖残缺：vTaskDelay(20) 后没有再验证电平，短毛刺也会误报按下
 *  - 每键一个函数 + 都返回 1：调用方要挨个轮询，容易漏扫/重复消费
 *
 * 新版机制：
 *  - 每个键一个三态状态机，消抖时间基准 = 任务的 20ms 扫描周期
 *      空闲(IDLE) --读到按下--> 确认(CONFIRM) --连续2周期仍按下--> 报事件,等松手(WAIT_RELEASE)
 *          ^                        |                                       |
 *          +-------- 已松开 <-------毛刺(不报事件)                读到松手 + 回到空闲
 *  - "按住不放只报一次"由 WAIT_RELEASE 态保证；松手后才能报下一次
 *  - cnt 在 WAIT_RELEASE 态持续自增，是将来做"长按"的现成扩展位（50 周期 = 1s）
 */

/* 状态机状态编号 */
#define KEY_STATE_IDLE             0u   /* 空闲：等按下 */
#define KEY_STATE_CONFIRM          1u   /* 消抖确认：连续 2 个扫描周期都是按下才算数 */
#define KEY_STATE_WAIT_RELEASE     2u   /* 已报事件：等松手，松手前不再报 */

/* 消抖判定：从"第一次读到按下"算起，连续第 2 个周期仍按下才确认
 * → 实际响应延迟 ≈ 2×20ms = 40ms，抖动窗口 <20ms 的毛刺全部被吃掉 */
#define KEY_CONFIRM_TICKS          1u   /* CONFIRM 态里还需再数 1 次按下 */

/*==============================================================================
 *  ★★★★ 电源键（★ PA0 ✗ 2026-10-13 新增 ✓）—— 接法/为什么用它见 Key.h
 *
 *  ★★ 为什么【【不复用】上面那套三态状态机】✗✓：
 *    · ★ 那套状态机报事件的时机是【"确认按下"✗ 约 40ms**】✓
 *    · ★★ 而电源键要的是【"长按满 1 秒"】✗ 两者【判据完全不同】✓
 *    · ★★★ 【硬塞进去【会把 KEY1 的"按下即暂停"手感也一起改掉】】✗✓
 *    → ★★★★ 【所以【单独写一个小计数器 ✗ 十几行 ✗ 互不干扰】】✓✓
 *============================================================================*/
#define KEY_PWR_PORT               GPIOA
#define KEY_PWR_PIN                GPIO_Pin_0
#define KEY_PWR_HOLD_TICKS         50u  /* ★ 20ms × 50 = 【1 秒】✓ */

typedef struct {
    uint8_t state;      /* 当前状态：KEY_STATE_xxx */
    uint8_t cnt;        /* 周期计数器（消抖用 1 次；WAIT_RELEASE 态自增留作长按扩展） */
} Key_Fsm_t;

static Key_Fsm_t fsm_key1;      /* PB1 */
static Key_Fsm_t fsm_key11;     /* PB11 */

/* ★★ 电源键的两个状态位 ✗✓（★ 为什么不用上面那个结构体：见本节开头 ✓）*/
static uint8_t s_pwr_cnt   = 0u;    /* ★ 已经【连续按下】几拍 ✓ */
static uint8_t s_pwr_fired = 0u;    /* ★ 本轮按住【已经报过事件】✗ 松手才复位 ✓ */

/* 读一次电平。★ 本板按键接 VCC、GPIO 配下拉输入(IPD)：按下 = 读到 1。
 * 若换接线（按键接 GND），只需把这里反过来，状态机一行都不用动
 *
 * ★★ 2026-10-13 改成【带端口】✗✓：
 *   · ★ 原来【硬编码 GPIOB**✗ —— 那时两个键都在 B 口 ✓
 *   · ★★ 而【电源键在 **PA0**】✗ → 【不参数化就够不着】✓✓
 *   ★★★ 注 ✗：本板的"接 VCC"是【实测确认**的 ✗ 见 Key.h 顶部 ✓ */
static uint8_t Key_Read(GPIO_TypeDef *port, uint16_t GPIO_Pin)
{
    return (uint8_t)GPIO_ReadInputDataBit(port, GPIO_Pin);
}

/* 单个按键状态机推进一步（每 20ms 一次），返回本次产生的事件位（0 或 event_bit） */
static uint8_t Key_FsmStep(Key_Fsm_t *pFsm, GPIO_TypeDef *port, uint16_t GPIO_Pin, uint8_t event_bit)
{
    uint8_t events = 0;
    uint8_t pressed = Key_Read(port, GPIO_Pin);

    switch (pFsm->state)
    {
        case KEY_STATE_IDLE:            /* 空闲：只关心"有没有按下" */
            if (pressed)
            {
                pFsm->state = KEY_STATE_CONFIRM;
                pFsm->cnt = 0;
            }
            break;

        case KEY_STATE_CONFIRM:         /* 消抖确认：第二次采样说了算 */
            if (pressed)
            {
                if (++pFsm->cnt >= KEY_CONFIRM_TICKS)
                {
                    events |= event_bit;            /* 确认按下，向上报事件 */
                    pFsm->state = KEY_STATE_WAIT_RELEASE;
                    pFsm->cnt = 0;
                }
            }
            else
            {
                pFsm->state = KEY_STATE_IDLE;       /* 松开了 → 刚才是毛刺，不报 */
            }
            break;

        case KEY_STATE_WAIT_RELEASE:    /* 等松手：保证按住不放只报一次 */
            if (!pressed)
            {
                pFsm->state = KEY_STATE_IDLE;       /* 松手，一轮结束 */
            }
            else
            {
                pFsm->cnt++;    /* 按住期间持续计数。扩展位：>=50 即长按 1s，
                                     目前项目用不到，只留计数不做动作 */
            }
            break;

        default:                        /* 异常值兜底：回到空闲自愈 */
            pFsm->state = KEY_STATE_IDLE;
            break;
    }

    return events;
}

/*==============================================================================
 *  ★★★★ 电源键：长按满 1 秒 → 【报一次】✗ 松手才重新武装 ✓
 *
 *  ★ 判据（★ 三条缺一不可 ✓）：
 *    · ★ 连续按下 ≥ `KEY_PWR_HOLD_TICKS`（50 拍 = 1 秒）✓
 *    · ★★ 本轮【还没报过】（★ 否则按住不放会【每秒报一次 ✗ 反复开关机】✓）✓
 *    · ★★★ 【松手就把计数清零 ✗ 并且重新武装】✓✓
 *
 *  ★★ 为什么【计数到 255 就封顶】✗✓：
 *    · ★ 它【没用到 255 ✗ 封顶只是为了【不让它回绕成 0】】✓
 *    · ★★ 回绕的后果 ✗：【按住 5 秒后计数变回 0 ✗ 又"够 50 了"✗ 再报一次】✓✓
 *      （★ 虽然 `s_pwr_fired` 已经挡住了 ✗ 但【两处都设防【更稳】】✓）
 *
 *  ★★★ 短按【什么都不做】✗✓ —— ★ 这是【故意的】✗ 见 Key.h ✓
 *============================================================================*/
static uint8_t Key_PwrStep(void)
{
    if (Key_Read(KEY_PWR_PORT, KEY_PWR_PIN) != 0u)     /* ★ 按下 = 读到 1 ✓ */
    {
        if (s_pwr_fired == 0u)
        {
            if (s_pwr_cnt < 255u) { s_pwr_cnt++; }

            if (s_pwr_cnt >= (uint8_t)KEY_PWR_HOLD_TICKS)
            {
                s_pwr_fired = 1u;                       /* ★ 本轮只报一次 ✓ */
                return (uint8_t)KEY_PWR_EVENT;
            }
        }
    }
    else
    {
        s_pwr_cnt   = 0u;                               /* ★ 松手：清零 + 重新武装 ✓ */
        s_pwr_fired = 0u;
    }

    return 0u;
}

/* 周期扫描入口：由 task_KEY 每 20ms 调一次，一次扫完全部按键 */
uint8_t Key_Scan(void)
{
    uint8_t events = 0;

    events |= Key_FsmStep(&fsm_key1,  GPIOB, GPIO_Pin_1,  KEY1_EVENT);
    events |= Key_FsmStep(&fsm_key11, GPIOB, GPIO_Pin_11, KEY11_EVENT);
    events |= Key_PwrStep();

    return events;
}

void Key_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);   /* ★ 电源键在 PA0 ✓ */

    /* ★★ 三个键【同一个极性 ✗ 同一份配置】**✗✓
     *   · ★ 下拉输入：★【空闲 = 0 ✗ 按下 = 1】✓
     *   · ★★ 接线 ✗：★ 一端接引脚 ✗ 另一端接 **3.3V** ✓
     *     （★ 实测确认 ✗ 见 Key.h ✗ 【不是接 GND】✓）✓✓
     *   · ★★★ 【不需要外部电阻**✗ 内部下拉负责"松开保证是 0"✓ */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = KEY_PWR_PIN;            /* ★ PA0 ✓ */
    GPIO_Init(KEY_PWR_PORT, &GPIO_InitStructure);
}
