#include "stm32f10x.h"
#include "HC_SR04.h"
#include "DWT_Delay.h"
#include <stddef.h>                 /* ★ NULL：AC5 的 stm32f10x.h 不保证带它 ✗ 自己包 ✓ */

/*==============================================================================
 *  ★★★★★ HC-SR04 超声波测距 —— 定时器输入捕获版（2026-10-20）
 *
 *  ★★ 原理（一句话 ✓）：
 *    · 给 TRIG 一个 **≥10µs 的高脉冲** ⇒ 模块自动发 8 个 40kHz 方波 ✓
 *    · 收到回波后 ✗ ECHO 输出**一个高电平** ✗ 宽度 = 声波**往返**时间 ✓
 *    · 距离(mm) = 宽度(µs) × **0.1715** ✓（343 m/s ÷ 2 ⇒ 1µs 往返约 0.1715mm ✓）
 *      ★ 常见写法 `cm = us / 58` 是同一个东西 ✗ 只是把 0.1715 折算成 1/58 ✓
 *
 *  ★★★ 为什么用**输入捕获**（而不是"读 GPIO 电平等它变低"）✗✓：
 *    · 等电平 = **阻塞** ✗ 回波最长 25ms ⇒ 25ms 里这个任务死等 ✓
 *    · ★★ 输入捕获 = **硬件打时间戳** ✗ 上升沿/下降沿到了才进中断 ✓
 *      ⇒ CPU 占用几乎为 0 ✓ 精度 = 定时器 1µs ✓✓
 *    · ★ 为什么不做成"两次 EXTI 中断 + DWT 读周期"✗：
 *      · 也能用 ✓ 但中断里要读 `DWT->CYCCNT` ✗ 还得多写一个 EXTI 回调 ✓
 *      · 而且 EXTI 只有一条线一个入口 ✗ 两个边沿要自己判方向 ✓
 *      ⇒ 有 TIM4_CH1 这个现成的硬件，就用硬件 ✓✓
 *
 *  ★★★★ 三个必须注意的细节（★ 都踩过/算过 ✓）：
 *    ① **定时器时基 = 1µs**：PSC = 72-1（72MHz ÷ 72 = 1MHz ✓）
 *       ARR = 0xFFFF ⇒ 最长量 65.5ms ✓（4 米只要 25ms ✗ 够用 ✓）
 *    ② **中断优先级必须 < 5**（即数字 0~4 ✓）：
 *       · FreeRTOS 的 `configMAX_SYSCALL_INTERRUPT_PRIORITY = 5`
 *         ⇒ 优先级 ≥5 的中断【会被临界区屏蔽】✓
 *       · ★★ 而本工程 DHT22/DS18B20 有【最长约 12ms 的临界区】
 *         ⇒ 若 TIM4 用 5+ ⇒ 上升沿被打断延后 12ms ⇒ **测距直接错** ✓✓
 *       · ★ 本 ISR **不调任何 FreeRTOS API** ⇒ 放 3 是合法且安全的 ✓
 *    ③ **ECHO 配成下拉输入**（`GPIO_Mode_IPD`）✗：
 *       · 传感器没接/没上电时引脚**悬空** ⇒ 会乱跳 ⇒ 乱进中断 ✓
 *       · 下拉之后 = 稳定低电平 ⇒ 安静 ✓✓
 *============================================================================*/

/*------------------------------------------------------------------------------
 *  引脚与定时器（★ 换脚只改这几行 ✓）
 *------------------------------------------------------------------------------*/
#define HC_TRIG_PORT    GPIOB
#define HC_TRIG_PIN     GPIO_Pin_5
#define HC_TRIG_RCC     RCC_APB2Periph_GPIOB

#define HC_ECHO_PORT    GPIOB
#define HC_ECHO_PIN     GPIO_Pin_6      /* ★ 必须 = TIM4_CH1 的引脚 ✓ */
#define HC_ECHO_RCC     RCC_APB2Periph_GPIOB

#define HC_TIM          TIM4
#define HC_TIM_RCC      RCC_APB1Periph_TIM4
#define HC_TIM_IRQn     TIM4_IRQn

/* 触发脉冲宽度（★ 手册要求 ≥10µs ✗ 给 12µs 留余量 ✓）*/
#define HC_TRIG_US          12u
/* 软件超时（★ 4 米 ≈ 25ms ✗ 给 45ms ✓）*/
#define HC_TIMEOUT_MS       45u
/* 有效脉宽窗口（★ 2cm ≈ 116µs ✗ 4m ≈ 2330µs ✓）*/
#define HC_PULSE_MIN_US     110u
#define HC_PULSE_MAX_US     2400u

/*------------------------------------------------------------------------------
 *  状态机（★ 全部 volatile ✗ ISR 写、任务读 ✓）
 *    0 = 空闲（没在测）
 *    1 = 等上升沿（触发已发，等回波开始）
 *    2 = 等下降沿（回波已开始，等它结束）
 *    3 = 完成（脉宽已存进 s_us）
 *------------------------------------------------------------------------------*/
static volatile uint8_t  s_state = 0u;
static volatile uint16_t s_t0    = 0u;      /* 上升沿时刻（µs 计数）*/
static volatile uint16_t s_t1    = 0u;      /* 下降沿时刻 */
static volatile uint16_t s_us    = 0u;      /* 脉宽（µs）*/
static volatile uint16_t s_edges = 0u;      /* ★ 累计进过多少次捕获中断（诊断用 ✓）*/
static uint32_t          s_t0_cyc = 0u;     /* 软件超时起点（DWT 周期数）*/

static TIM_ICInitTypeDef s_ic;

/* 改捕获边沿（★ ISR 里也会调 ✗ 所以必须很短：只写 5 个寄存器 ✓）*/
static void HC_SetEdge(uint16_t pol)
{
	s_ic.TIM_Channel     = TIM_Channel_1;
	s_ic.TIM_ICPolarity  = pol;
	s_ic.TIM_ICSelection = TIM_ICSelection_DirectTI;
	s_ic.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	s_ic.TIM_ICFilter    = 0x3u;             /* ★ 轻过滤：滤掉 <3 个采样点的毛刺 ✓ */
	TIM_ICInit(HC_TIM, &s_ic);
	TIM_ClearITPendingBit(HC_TIM, TIM_IT_CC1);
}

uint8_t HC_SR04_Init(void)
{
	GPIO_InitTypeDef       g;
	TIM_TimeBaseInitTypeDef tb;
	NVIC_InitTypeDef       nv;

	RCC_APB2PeriphClockCmd(HC_TRIG_RCC, ENABLE);
	RCC_APB2PeriphClockCmd(HC_ECHO_RCC, ENABLE);
	RCC_APB1PeriphClockCmd(HC_TIM_RCC,  ENABLE);

	/* ① TRIG：推挽输出，**默认拉低**（★ 空闲时不能是高 ✗ 否则模块乱发 ✓）*/
	g.GPIO_Pin   = HC_TRIG_PIN;
	g.GPIO_Mode  = GPIO_Mode_Out_PP;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(HC_TRIG_PORT, &g);
	GPIO_ResetBits(HC_TRIG_PORT, HC_TRIG_PIN);

	/* ② ECHO：下拉输入（★ 见文件头"细节 ③" ✓）*/
	g.GPIO_Pin  = HC_ECHO_PIN;
	g.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(HC_ECHO_PORT, &g);

	/* ③ TIM4 时基 = 1µs ✓ */
	tb.TIM_Prescaler         = 72u - 1u;
	tb.TIM_CounterMode       = TIM_CounterMode_Up;
	tb.TIM_Period            = 0xFFFFu;
	tb.TIM_ClockDivision     = TIM_CKD_DIV1;
	tb.TIM_RepetitionCounter = 0u;
	TIM_TimeBaseInit(HC_TIM, &tb);

	HC_SetEdge(TIM_ICPolarity_Rising);
	TIM_ClearITPendingBit(HC_TIM, TIM_IT_CC1);
	TIM_ITConfig(HC_TIM, TIM_IT_CC1, ENABLE);

	/* ④ 中断优先级 = 3（★ 见文件头"细节 ②" ✗ 千万别改成 5 及以上 ✓）*/
	nv.NVIC_IRQChannel                   = HC_TIM_IRQn;
	nv.NVIC_IRQChannelPreemptionPriority  = 3u;
	nv.NVIC_IRQChannelSubPriority         = 0u;
	nv.NVIC_IRQChannelCmd                 = ENABLE;
	NVIC_Init(&nv);

	TIM_Cmd(HC_TIM, ENABLE);
	s_state = 0u;
	return 1u;
}

void HC_SR04_Start(void)
{
	s_state = 1u;                            /* ★ 先置状态 ✗ 再发脉冲 ✓ */
	s_us    = 0u;
	HC_SetEdge(TIM_ICPolarity_Rising);

	/* ★ 12µs 高电平：用 DWT 忙等（★ 不碰 SysTick ✓ 见文件头）*/
	GPIO_SetBits(HC_TRIG_PORT, HC_TRIG_PIN);
	DWT_Delay_us(HC_TRIG_US);
	GPIO_ResetBits(HC_TRIG_PORT, HC_TRIG_PIN);

	s_t0_cyc = DWT->CYCCNT;                  /* ★ 超时基准 ✓ */
}

uint8_t HC_SR04_Poll(uint16_t *mm)
{
	uint32_t el_ms;

	if (s_state == 3u)                       /* ★ 硬件已测完 ✓ */
	{
		s_state = 0u;
		if ((s_us >= HC_PULSE_MIN_US) && (s_us <= HC_PULSE_MAX_US))
		{
			/* ★ 脉宽 → 毫米：us × 172 / 1000 ✓（整数运算，无浮点 ✓）*/
			if (mm != NULL)
			{
				*mm = (uint16_t)(((uint32_t)s_us * 172u) / 1000u);
			}
			return 1u;
		}
		return 2u;                           /* ★ 太近/太远/不是真回波 ✗ */
	}

	if (s_state == 0u) { return 2u; }        /* ★ 没在测 ✓ */

	/* ★ 软件超时（★ 硬件不会自己报"没有回波" ✗ 必须软件兜 ✓）*/
	el_ms = (uint32_t)((DWT->CYCCNT - s_t0_cyc) / (SystemCoreClock / 1000u));
	if (el_ms > HC_TIMEOUT_MS)
	{
		s_state = 0u;
		HC_SetEdge(TIM_ICPolarity_Rising);
		return 2u;
	}
	return 0u;
}

uint16_t HC_SR04_LastPulseUs(void)
{
	return s_us;
}

/* ★ 2026-10-20 加（诊断 ✓）：累计捕获中断次数 + ECHO 当前电平
 *   ★ 判据 ✗：
 *     · `edges` 一直 **0** ⇒ ECHO 线上**从来没有边沿** ✓
 *       ⇒ 传感器没供电 / 没接 / 接错脚 / 或已坏 ✓✓
 *     · `edges` 在涨但 `pulse 0` ⇒ 边沿来了但状态机没接住（软件问题 ✓）
 *   ★ 手测法：把 ECHO 那根线**短一下 3.3V 再松开** ✗ `edges` 应该 +1~2 ✓
 *     （★ 不用示波器就能验"脚通不通、中断在不在" ✓✓）*/
uint16_t HC_SR04_EdgeCount(void)
{
	return s_edges;
}

uint8_t HC_SR04_EchoLevel(void)
{
	return (GPIO_ReadInputDataBit(HC_ECHO_PORT, HC_ECHO_PIN) != 0u) ? 1u : 0u;
}

/*==============================================================================
 *  ★★★★ TIM4 中断（★ 只做两件事：记时间戳 + 翻边沿 ✓）
 *
 *  ★ 为什么这个 ISR 必须极短 ✗：
 *    · 它的优先级是 3（★ 比 FreeRTOS 的临界区还高 ✓）
 *      ⇒ 它每多花 1µs，被它打断的那段代码就多延迟 1µs ✓
 *    · ★★ 本函数全部动作：读一个捕获寄存器、存两个变量、写 5 个寄存器 ✓
 *      ⇒ 实测量级 **1~2µs** ✓✓
 *
 *  ★ 为什么两个边沿共用一个通道 ✗：
 *    · TIM4_CH1 只连一根引脚（PB6）✗ 要么捕获上升沿、要么捕获下降沿 ✓
 *    · ⇒ 上升沿到了【在 ISR 里把极性翻成下降沿】✓ 这就是"单通道测脉宽" ✓✓
 *    · ★ 另一种做法是 TI1 同时映射到 TI2（PWM 输入模式）✗
 *      能用但配置复杂得多 ✓ 对一个测距足够简单的场景不值得 ✓
 *
 *  ★ 注意 ✗ 这里**不能调任何 FreeRTOS API**（★ 优先级高于 max syscall ✓）
 *============================================================================*/
void TIM4_IRQHandler(void)
{
	uint16_t cap;

	if (TIM_GetITStatus(HC_TIM, TIM_IT_CC1) == RESET) { return; }

	cap = (uint16_t)TIM_GetCapture1(HC_TIM);
	s_edges++;                               /* ★ 诊断计数器（只加不读 ✓ 无副作用）*/

	if (s_state == 1u)                       /* ★ 上升沿：回波开始 ✓ */
	{
		s_t0    = cap;
		s_state = 2u;
		HC_SetEdge(TIM_ICPolarity_Falling);
	}
	else if (s_state == 2u)                  /* ★ 下降沿：回波结束 ✓ */
	{
		s_t1    = cap;
		s_us    = (uint16_t)(s_t1 - s_t0);   /* ★ 16 位相减自动处理回绕 ✓ */
		s_state = 3u;
		HC_SetEdge(TIM_ICPolarity_Rising);   /* ★ 为下一次测量备好 ✓ */
	}
	else
	{
		/* ★ 空闲/已完成时来的边沿：忽略（可能是上一次的尾巴 ✓）*/
	}

	TIM_ClearITPendingBit(HC_TIM, TIM_IT_CC1);
}
