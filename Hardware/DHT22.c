#include "stm32f10x.h"
#include "DHT22.h"
#include "Serial.h"
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
#define DHT_TIMEOUT_US      200     /* 等电平翻转的上限（正常最长 80us）
                                     * ★ 2026-11-04 试过放到 1000us：失败率没降
                                     *   （3 分钟 4/62 vs 2/61），⇒ 不是上升沿快慢
                                     *   ⇒ 已改回 200，不白等 */
#define DHT_SAMPLE_US       40      /* ★ 采样点：取在"0 的高电平(26~28us)"之后、
                                     *   "1 的高电平(70us)"之前
                                     *   ★ 40 落在窗口内，但它不是中点
                                     *     中点 = (28+70)/2 = 49
                                     *   ★ 原注释写"取中点 40"，自相矛盾
                                     *     （2026-10-19 修正口径，值本身没改） */
#define DHT_BITS            40      /* 湿度16 + 温度16 + 校验8 */

/*------------------------------------------------------------------------------
 * 引脚控制
 *   ★ 固定开漏输出—— 模块自带 10k 上拉（R=103），所以：
 *       写 1 = 释放（靠外部上拉拉高）/ 写 0 = 拉低
 *     ★ 开漏模式下 IDR 仍反映引脚真实电平 → 读电平不需要切换模式
 *        （这正是它比 DS18B20 那套 IPU/Out_PP 动态切换更干净的原因）
 *------------------------------------------------------------------------------*/
static void DHT_PullLow(void)  { GPIO_ResetBits(DHT22_GPIO, DHT22_PIN); }
static void DHT_Release(void)  { GPIO_SetBits(DHT22_GPIO, DHT22_PIN);   }
static uint8_t DHT_Level(void) { return GPIO_ReadInputDataBit(DHT22_GPIO, DHT22_PIN); }

/* 忙等直到引脚等于 level；超时返回 0（纯忙等，临界区内调用安全） */
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
	/* ★ 器件上电稳定约 1s —— 由调用时机保证（系统初始化到第一次读早已超过） */
}

/*------------------------------------------------------------------------------
 * 读一次：成功 1 / 失败 0（失败时不改动输出参数）
 *
 * ★ 为什么整帧包一次临界区（和 DS18B20"每字节一次"不同）：
 *    DHT22 的位间隙没有容错空间 —— 被打断一次，后面 40 位全部错位。
 *    代价：4.8ms 关中断。任务侧按 2 秒一次调用 → 平均 <0.3%
 *------------------------------------------------------------------------------*/
/* ★ 2026-11-04 加：失败原因记账（查"约 15% 偶发失败"用，查完可删）
 *   码值对应 DHT22_Read() 里的步骤：
 *   0 = 上一次成功
 *   1 = 启动后没等到低电平（器件根本没应答）
 *   2 = 应答低之后没等到高
 *   3 = 应答高之后没等到数据起始的低
 *   4 = 40 位里某一位的"高"没等到
 *   5 = 40 位里某一位的"低"没等到（这一位结束后没回低）
 *   6 = 校验和不对（帧收全了，但数据是错的）
 *   ★ 分这些码的意义：1~3 是"没人应答"，4~5 是"位流被打断"，
 *     6 是"位流收全但比特错了"——三种原因的修法完全不同 */
volatile uint8_t  g_dht_reason = 0u;
volatile uint16_t g_dht_ok     = 0u;
volatile uint16_t g_dht_fail   = 0u;

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
	if (DHT_WaitLevel(0u, DHT_TIMEOUT_US) == 0u) { g_dht_reason = 1u; g_dht_fail++; taskEXIT_CRITICAL(); return 0u; }
	if (DHT_WaitLevel(1u, DHT_TIMEOUT_US) == 0u) { g_dht_reason = 2u; g_dht_fail++; taskEXIT_CRITICAL(); return 0u; }
	if (DHT_WaitLevel(0u, DHT_TIMEOUT_US) == 0u) { g_dht_reason = 3u; g_dht_fail++; taskEXIT_CRITICAL(); return 0u; }

	/* ③ 40 位 —— 每位都是"低 50us 开始"，高电平的长短表示 0/1 */
	for (i = 0u; i < (uint8_t)DHT_BITS; i++)
	{
		if (DHT_WaitLevel(1u, DHT_TIMEOUT_US) == 0u) { g_dht_reason = 4u; g_dht_fail++; taskEXIT_CRITICAL(); return 0u; }
		DWT_Delay_us(DHT_SAMPLE_US);                    /* ★ 40us 后采样 */
		b = DHT_Level();
		if (DHT_WaitLevel(0u, DHT_TIMEOUT_US) == 0u) { g_dht_reason = 5u; g_dht_fail++; taskEXIT_CRITICAL(); return 0u; }

		d[i >> 3] = (uint8_t)((uint8_t)(d[i >> 3] << 1) | b);
	}

	taskEXIT_CRITICAL();

	/* ④ 校验：前 4 字节之和的低 8 位 */
	sum = (uint8_t)((uint16_t)d[0] + (uint16_t)d[1] + (uint16_t)d[2] + (uint16_t)d[3]);
	if (sum != d[4]) { g_dht_reason = 6u; g_dht_fail++; return 0u; }

	g_dht_reason = 0u;
	g_dht_ok++;

	*humi_x10 = (uint16_t)(((uint16_t)d[0] << 8) | (uint16_t)d[1]);   /* 0.1 %RH */

	t = (uint16_t)(((uint16_t)d[2] << 8) | (uint16_t)d[3]);            /* 0.1 °C */
	*temp_x10 = (t & 0x8000u) ? (int16_t)(-(int16_t)(t & 0x7FFFu)) /* bit15 = 符号 */
	                          : (int16_t)t;
	return 1u;
}

/*------------------------------------------------------------------------------
 *  2026-11-03 重建：原实现（2026-10-24 从 main.c 搬来）在 11-03 丢失，
 *  按当时的串口输出格式重写。开机读一次，把值打出来供人对照 DS18B20。
 *----------------------------------------------------------------------------*/
void DHT22_BootTest(void)
{
	uint16_t humi = 0u;
	int16_t  temp = 0;
	int16_t  t;

	DHT22_Init();                     /* 引脚配成开漏并释放总线（原实现也在读之前做这一步）*/

	if (DHT22_Read(&humi, &temp) == 0u)
	{
		Serial_Printf("DHT22: read FAILED (no response or bad checksum)\r\n");
		return;
	}

	t = (temp < 0) ? (int16_t)(-temp) : temp;
	Serial_Printf("DHT22: humi=%u.%u %%RH  temp=%s%d.%d C\r\n",
	              (unsigned)(humi / 10u), (unsigned)(humi % 10u),
	              (temp < 0) ? "-" : "",
	              (int)(t / 10), (int)(t % 10));
	Serial_Printf("DHT22: (compare with DS18B20 below; DHT22 self-heats a bit)\r\n");
}
