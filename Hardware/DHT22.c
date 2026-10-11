#include "stm32f10x.h"
#include "DHT22.h"
#include "DWT_Delay.h"
#include "FreeRTOS.h"
#include "task.h"

/*==============================================================================
 * 时序常量（微秒）—— 取自 AM2302 数据手册，取"留余量"的值
 *   手册：启动低 ≥1ms / 释放 20~40us / 响应低 80us 高 80us
 *         每位 = 低 50us + 高 26~28us(=0) 或 70us(=1)
 *============================================================================*/
#define DHT_START_LOW_US    1200    /* 主机启动信号：拉低 ≥1ms，取 1.2ms */
#define DHT_RELEASE_US      30      /* 释放后等 20~40us，交给器件 */
#define DHT_TIMEOUT_US      200     /* 等电平翻转的上限（正常最长 80us） */
#define DHT_SAMPLE_US       40      /* ★ 采样点：>28(=0的高) 且 <70(=1的高)，取中点 40 */
#define DHT_BITS            40      /* 湿度16 + 温度16 + 校验8 */

/*------------------------------------------------------------------------------
 * 引脚控制
 *   ★ 固定【开漏输出】—— 模块自带 10k 上拉（R=103），所以：
 *       写 1 = 释放（靠外部上拉拉高）/ 写 0 = 拉低
 *     ★★ 开漏模式下 IDR 仍反映引脚真实电平 → 读电平【不需要切换模式】✓
 *        （这正是它比 DS18B20 那套 IPU/Out_PP 动态切换更干净的原因）
 *------------------------------------------------------------------------------*/
static void DHT_PullLow(void)  { GPIO_ResetBits(DHT22_GPIO, DHT22_PIN); }
static void DHT_Release(void)  { GPIO_SetBits(DHT22_GPIO, DHT22_PIN);   }
static uint8_t DHT_Level(void) { return GPIO_ReadInputDataBit(DHT22_GPIO, DHT22_PIN); }

/* 忙等直到引脚等于 level；超时返回 0（★ 纯忙等，临界区内调用安全） */
static uint8_t DHT_WaitLevel(uint8_t level, uint16_t timeout_us)
{
	uint16_t t;
	for (t = 0u; t < timeout_us; t++)
	{
		if (DHT_Level() == level) { return 1u; }
		DWT_Delay_us(1u);
	}
	return 0u;
}

void DHT22_Init(void)
{
	GPIO_InitTypeDef g;

	/* PB15 —— 和 DS18B20(PB14) 同属 GPIOB，时钟在那边已开，这里再开一次无害 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	g.GPIO_Pin   = DHT22_PIN;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	g.GPIO_Mode  = GPIO_Mode_Out_OD;        /* ★ 开漏：靠模块上的 10k 上拉 */
	GPIO_Init(DHT22_GPIO, &g);

	DHT_Release();                          /* 空闲时释放总线（高） */
	/* ★ 器件上电稳定约 1s —— 由调用时机保证（系统初始化到第一次读早已超过 ✓） */
}

/*------------------------------------------------------------------------------
 * 读一次：成功 1 / 失败 0（失败时不改动输出参数）
 *
 * ★★ 为什么整帧包一次临界区（和 DS18B20"每字节一次"不同 ✗）：
 *    DHT22 的位间隙没有容错空间 —— 被打断一次，后面 40 位全部错位。
 *    代价：4.8ms 关中断。任务侧按 2 秒一次调用 → 平均 <0.3%
 *------------------------------------------------------------------------------*/
uint8_t DHT22_Read(uint16_t *humi_x10, int16_t *temp_x10)
{
	uint8_t  d[5];
	uint8_t  i, b;
	uint16_t t;
	uint8_t  sum;

	if ((humi_x10 == NULL) || (temp_x10 == NULL)) { return 0u; }

	d[0] = 0u; d[1] = 0u; d[2] = 0u; d[3] = 0u; d[4] = 0u;

	taskENTER_CRITICAL();

	/* ① 启动信号 */
	DHT_PullLow();
	DWT_Delay_us(DHT_START_LOW_US);
	DHT_Release();
	DWT_Delay_us(DHT_RELEASE_US);

	/* ② 器件响应：低 80us → 高 80us → 再低（数据开始） */
	if (DHT_WaitLevel(0u, DHT_TIMEOUT_US) == 0u) { taskEXIT_CRITICAL(); return 0u; }
	if (DHT_WaitLevel(1u, DHT_TIMEOUT_US) == 0u) { taskEXIT_CRITICAL(); return 0u; }
	if (DHT_WaitLevel(0u, DHT_TIMEOUT_US) == 0u) { taskEXIT_CRITICAL(); return 0u; }

	/* ③ 40 位 —— 每位都是"低 50us 开始"，高电平的长短表示 0/1 */
	for (i = 0u; i < (uint8_t)DHT_BITS; i++)
	{
		if (DHT_WaitLevel(1u, DHT_TIMEOUT_US) == 0u) { taskEXIT_CRITICAL(); return 0u; }
		DWT_Delay_us(DHT_SAMPLE_US);                    /* ★ 40us 后采样 */
		b = DHT_Level();
		if (DHT_WaitLevel(0u, DHT_TIMEOUT_US) == 0u) { taskEXIT_CRITICAL(); return 0u; }

		d[i >> 3] = (uint8_t)((uint8_t)(d[i >> 3] << 1) | b);
	}

	taskEXIT_CRITICAL();

	/* ④ 校验：前 4 字节之和的低 8 位 */
	sum = (uint8_t)((uint16_t)d[0] + (uint16_t)d[1] + (uint16_t)d[2] + (uint16_t)d[3]);
	if (sum != d[4]) { return 0u; }

	*humi_x10 = (uint16_t)(((uint16_t)d[0] << 8) | (uint16_t)d[1]);   /* 0.1 %RH */

	t = (uint16_t)(((uint16_t)d[2] << 8) | (uint16_t)d[3]);            /* 0.1 °C */
	*temp_x10 = (t & 0x8000u) ? (int16_t)(-(int16_t)(t & 0x7FFFu))     /* ★ bit15 = 符号 */
	                          : (int16_t)t;
	return 1u;
}
