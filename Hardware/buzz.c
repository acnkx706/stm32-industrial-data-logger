#include "stm32f10x.h"                  // Device header
#include "buzz.h"

void Buzz_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin= GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_SetBits(GPIOB,GPIO_Pin_12);
}

void Buzz_ON(void)
{
	GPIO_ResetBits(GPIOB,GPIO_Pin_12 );
}

void Buzz_OFF(void)
{
	GPIO_SetBits(GPIOB,GPIO_Pin_12 );
}


void Buzz_Turn(void)
{
	if (GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_12) == 0)
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_12);
	}
	else
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_12);
	}
}


/*==============================================================================
 *  ★★★★ 蜂鸣器服务层实现（2026-10-13 加 ✓ 理由见 buzz.h）
 *
 *  ★ 设计要点 ✗：
 *    · ★ **请求与节奏分离**✗✓ ——
 *      任务只置标志位 ✗ 【不直接开关 GPIO】✓
 *    · ★★ **两类事件区别对待**✗✓：
 *      · 持续性（SD 故障）→ 【翻转出"滴滴"✗ 一直响到 Clear】✓
 *      · 一次性（按键）→ 【响固定帧数 ✗ 自动结束】✓
 *    · ★★★ **只有一个地方写 GPIO**✗✓（Buzz_Poll ✓）
 *      → 【不会出现"两个任务抢着开关"】✓✓
 *============================================================================*/
#define BUZZ_ON_FRAMES   3u      /* ★ 一次性事件响几帧 ✓
                                  *   ★ 调用者 task_DISP 是【每 200ms 一帧】✓
                                  *   → 3 帧 ≈ 【600ms】✓ 足够听清 ✓✓ */

#define BUZZ_LONG_FRAMES 8u      /* ★★ 长响的帧数（★ 2026-10-13 加 ✓）
                                  *   → 8 帧 ≈ 【1.6 秒】✓
                                  *   ★ 为什么【不复用 3 帧**✗：
                                  *     【"可以拔电了"和"按键收到了"【必须听得出区别】】✓✓ */

static uint8_t s_sd_err = 0u;    /* ★ SD 故障标志（★ 持续型 ✓）*/
static uint8_t s_key_fr = 0u;    /* ★ 按键剩余帧数（★ 一次型 ✓）*/
static uint8_t s_silent = 0u;    /* ★ 静音 ✓ */
static uint8_t s_on     = 0u;    /* ★ 当前 GPIO 状态（★ 避免重复写寄存器 ✓）*/
static uint8_t s_phase  = 0u;    /* ★ 持续型的翻转相位 ✓ */

void Buzz_Request(uint8_t ev)
{
	/* ★ 一次性事件 ✗ → 【只折算成帧数 ✗ 不进持续标志】✓
	 *   ★★ 为什么必须分开 ✗：
	 *     如果按键也放进持续标志 ✗ → 【它会一直响下去 ✗ 没人清】✓✓ */
	if ((ev & BUZZ_EV_KEY) != 0u)
	{
		s_key_fr = BUZZ_ON_FRAMES;
	}
	if ((ev & BUZZ_EV_LONG) != 0u)
	{
		/* ★★ 长响也是【一次性】✗ 只是帧数更长 ✓
		 *   · ★ 为什么【取 max ✗ 而不是直接赋值】✗：
		 *     【短响正在响时又来了长响 ✗ 不该【把长响缩短成 600ms】】✓✓
		 *   · ★★ 反过来：长响进行中来了短响 ✗ 【保持长响】✓
		 *     （★ 因为长响承载的信息更多 ✗ "可以拔电了"✓）✓✓ */
		if (s_key_fr < BUZZ_LONG_FRAMES) { s_key_fr = BUZZ_LONG_FRAMES; }
	}
	if ((ev & BUZZ_EV_SD_ERR) != 0u)
	{
		s_sd_err = 1u;
	}
}

void Buzz_Clear(uint8_t ev)
{
	if ((ev & BUZZ_EV_SD_ERR) != 0u)
	{
		s_sd_err = 0u;
		s_phase  = 0u;           /* ★ 复位相位 ✗ 下次从"响"开始 ✓ */
	}
	/* ★ 按键是一次性的 ✗ 没有"清除"语义 ✓ */
}

void Buzz_Silence(uint8_t on)
{
	s_silent = (on != 0u) ? 1u : 0u;
	if (s_silent != 0u)
	{
		Buzz_OFF();
		s_on = 0u;
	}
}

uint8_t Buzz_IsSilent(void)
{
	return s_silent;
}

void Buzz_Poll(void)
{
	/* ★ 静音 ✗ → 【强制关 ✗ 直接返回】✓ */
	if (s_silent != 0u)
	{
		if (s_on != 0u) { Buzz_OFF(); s_on = 0u; }
		return;
	}

	/* ★★ ① 【一次性事件优先】✗✓
	 *   为什么优先 ✗：★ 按键是【用户刚按下的 ✗ 应该立刻有反馈】✓
	 *   ★★ 否则会被 SD 的持续响淹没 ✗ → 【用户以为按键没反应】✓✓ */
	if (s_key_fr > 0u)
	{
		s_key_fr--;
		if (s_on == 0u) { Buzz_ON(); s_on = 1u; }
		return;
	}

	/* ★★★ ② 【持续性事件：SD 故障】✗ → 每帧翻转出"滴滴"✓
	 *   ★ 频率 ✗：★ 200ms 一帧 ✗ 每帧翻转 ✓
	 *     → 【200ms 响 + 200ms 停】= 周期 400ms ≈ **2.5Hz** ✓✓ 很醒目 ✓
	 *     ★★ 2026-10-19 修正 ✗：原注释写"100ms 响 + 100ms 停 ≈ 5Hz"✗
	 *       那是【把 200ms 一帧误当成 100ms 半周期】✓ 实际帧长 200ms ✓ */
	if (s_sd_err != 0u)
	{
		s_phase = (uint8_t)(1u - s_phase);
		if (s_phase != 0u) { Buzz_ON();  s_on = 1u; }
		else               { Buzz_OFF(); s_on = 0u; }
		return;
	}

	/* ★★★★ ③ 【什么事件都没有 ✗ → 确保关着】✓ */
	if (s_on != 0u) { Buzz_OFF(); s_on = 0u; }
}
	
