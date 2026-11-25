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
/* ★ 2026-11-24：原 DHT_SAMPLE_US = 40（固定 40us 采样点）已删除。
 *   现在改成"量高电平有多宽"，不再需要在某个时刻去采样。
 *   删掉的理由见 DHT22_Read 里 ③ 那段说明。 */
#define DHT_BIT1_MIN_US     50      /* ★ 判 1 的门限（us）
                                     *   0 位高电平 26~28us，1 位 70us
                                     *   取 50 落在两簇中间：离 0 位 22us、离 1 位 20us
                                     *   不取 40：那离 0 位只剩 12us，等于换汤不换药 */
#define DHT_BITS            ((uint8_t)DHT22_BITS)  /* 湿度16+温度16+校验8；数值取自 DHT22.h，别在这里改 */

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

/* ★ 2026-11-04 加：reason=2 那次的"加长观察"结果（查责任方用）
 *   · g_dht_extra_us  —— 超时之后又过了多少微秒线才变高（65535=一直没高）
 *   · g_dht_line_high —— 最后读到的电平（1=自己回去了 0=一直低）
 *   ★ 判读：自己回去了 ⇒ 器件慢；一直低到 20ms ⇒ 线被拉死，再查是谁拉的 */
volatile uint16_t g_dht_extra_us = 0u;
volatile uint8_t  g_dht_line_high = 0u;

/* ★ 2026-11-24 加：判决余量体检（成功也记，不用等失败）
 *
 * ★ 为什么要有它：
 *   · 之前的探针只在"失败"时留证据，而失败率只有 3~6%
 *     ⇒ 等一次要几十分钟，那是赌运气，不是诊断
 *   · 但每一次成功读取都留下了"余量还剩多少"：
 *     40 位各自的高电平宽度、器件应答延迟、两个应答脉冲宽度
 *   · 余量最薄的那一项，就是将来失败的地方 —— 这不用等它坏
 *
 * ★ 影响什么：
 *   · 正常读多 40 次 CYCCNT 差值 + 40 次除法（M3 有硬件除法，约 3~7us）
 *   · 关中断总时长不变：一帧的时长由器件决定，我们只是不再空等 40us
 *   · RAM 多 80 + 40 + 6 + 14 字节
 *
 * ★ 怎么验证：
 *   · 串口发 H，下一次读取（≤3 秒）后打出 40 位宽度
 *   · 0 位应聚在 26~28us、1 位应聚在 70us；门限 50 离两簇都不小于 20us
 *  */
volatile uint16_t g_dht_width[DHT22_BITS];   /* 每位高电平宽度 us */
volatile uint8_t  g_dht_bits[DHT22_BITS];    /* 每位解出来的 0/1 */
volatile uint16_t g_dht_ack_delay_us = 0u;   /* 释放 -> 器件拉低 us（手册 20~40） */
volatile uint16_t g_dht_ack_low_us   = 0u;   /* 应答低电平宽度 us（手册 80） */
volatile uint16_t g_dht_ack_high_us  = 0u;   /* 应答高电平宽度 us（手册 80） */
volatile uint16_t g_dht_rc[7] = {0u};        /* reason 0..6 各出现几次 */
volatile uint8_t  g_dht_diag_ready = 0u;     /* 1 = 这批数据来自一次完整帧 */
volatile uint16_t g_dht_last_humi = 0u;      /* 上次成功解出的湿度 ×10 */
volatile int16_t  g_dht_last_temp = 0;       /* 上次成功解出的温度 ×10 */
	volatile uint16_t g_dht_reinit_cnt = 0u;    /* ★ 连续失败后重配引脚了几次 */
	volatile uint16_t g_dht_humi_ffff  = 0u;    /* ★ 校验通过但湿度字段就是 0xFFFF 的次数 */

uint8_t DHT22_Read(uint16_t *humi_x10, int16_t *temp_x10)
{
	uint8_t  d[5];
	uint8_t  i, b;
	uint16_t t;
	uint8_t  sum;
	uint32_t cyc, c;
	uint16_t w;

	if ((humi_x10 == NULL) || (temp_x10 == NULL)) { return 0u; }

	d[0] = 0u; d[1] = 0u; d[2] = 0u; d[3] = 0u; d[4] = 0u;

	/* 每微秒几个内核时钟。临界区里不再反复算：少一次除法就少一点关中断时间 */
	cyc = (uint32_t)(SystemCoreClock / 1000000u);

	taskENTER_CRITICAL();

	/* ① 启动信号 */
	DHT_PullLow();
	DWT_Delay_us(DHT_START_LOW_US);
	DHT_Release();
	DWT_Delay_us(DHT_RELEASE_US);

	/* ② 器件响应：拉低 80us → 高 80us → 再低（数据开始）
	 *   ★ 2026-11-24 加：三段时长全部量下来
	 *     · ack_delay —— 释放到器件拉低的总时间（手册 20~40us）
	 *       注意它含前面那 30us 固定等待，所以存的是绝对值，不是余量
	 *     · ack_low   —— 应答低电平宽度（手册 80us）
	 *     · ack_high  —— 应答高电平宽度（手册 80us）
	 *     · 为什么要量：这三段离超时上限 200us 有多远，就是"器件反应有多慢"的
	 *       直接证据 —— 不用等它慢到失败才看得见 */
	c = DWT->CYCCNT;
	if (DHT_WaitLevel(0u, DHT_TIMEOUT_US) == 0u)
	{
		g_dht_ack_delay_us = 999u;          /* 999 = 根本没应答，不是测出来的 */
		g_dht_reason = 1u; g_dht_rc[1]++; g_dht_fail++;
		taskEXIT_CRITICAL();
		return 0u;
	}
	g_dht_ack_delay_us = (uint16_t)(DHT_RELEASE_US + ((DWT->CYCCNT - c) / cyc));

	c = DWT->CYCCNT;
	if (DHT_WaitLevel(1u, DHT_TIMEOUT_US) == 0u)
	{
		/* ★ 2026-11-04 诊断：别急着放弃，继续看到底多久才放开（最多 20ms）
		 *   · 这段只在失败路径上跑，正常帧不受影响
		 *   · 用 CYCCNT 计时，换算成微秒后存进全局，由任务侧打印
		 *     （临界区里不许打印，只能存） */
		uint32_t c0 = DWT->CYCCNT;
		uint8_t  rose = DHT_WaitLevel(1u, 20000u);
		uint32_t dt = (DWT->CYCCNT - c0) / cyc;
		g_dht_extra_us  = (uint16_t)((dt > 65000u) ? 65000u : dt);
		g_dht_line_high = rose;
		g_dht_reason = 2u; g_dht_rc[2]++; g_dht_fail++;
		taskEXIT_CRITICAL();
		return 0u;
	}
	g_dht_ack_low_us = (uint16_t)((DWT->CYCCNT - c) / cyc);

	c = DWT->CYCCNT;
	if (DHT_WaitLevel(0u, DHT_TIMEOUT_US) == 0u)
	{
		g_dht_reason = 3u; g_dht_rc[3]++; g_dht_fail++;
		taskEXIT_CRITICAL();
		return 0u;
	}
	g_dht_ack_high_us = (uint16_t)((DWT->CYCCNT - c) / cyc);

	/* ③ 40 位 —— 每位都以"低约 50us"开头，高电平的长短表示 0/1
	 *   ★ 2026-11-24 改：原来"死等 40us 后采一个点"，现在"量高电平多宽，>50us 判 1"
	 *     · 为什么改：采样点准不准，取决于这颗粒子的 0 位是不是真落在手册的 26~28us。
	 *       若它的 0 位偏宽到 40us 以上，40us 那个点会落在高电平里 ⇒ 0 被判成 1
	 *       ⇒ 帧收全了、校验却错（reason=6），且只在某些温度/电压下偶发，最难查
	 *     · 影响什么：判决点从"离 0 位只 12us"变成"离两簇都不小于 20us"，
	 *       顺手留下 40 个宽度给体检；一帧的关中断总时长不变（时长由器件决定）
	 *     · 怎么验证：发 H 看 0 位是否聚在 26~28us、1 位是否聚在 70us 附近 */
	for (i = 0u; i < (uint8_t)DHT_BITS; i++)
	{
		if (DHT_WaitLevel(1u, DHT_TIMEOUT_US) == 0u)
		{
			g_dht_reason = 4u; g_dht_rc[4]++; g_dht_fail++;
			taskEXIT_CRITICAL();
			return 0u;
		}
		c = DWT->CYCCNT;
		if (DHT_WaitLevel(0u, DHT_TIMEOUT_US) == 0u)
		{
			g_dht_reason = 5u; g_dht_rc[5]++; g_dht_fail++;
			taskEXIT_CRITICAL();
			return 0u;
		}

		/* 高电平宽度 = 从"变高"到"变低"之间的内核周期数 */
		w = (uint16_t)((DWT->CYCCNT - c) / cyc);
		g_dht_width[i] = w;
		b = (w > DHT_BIT1_MIN_US) ? 1u : 0u;
		g_dht_bits[i] = b;

		d[i >> 3] = (uint8_t)((uint8_t)(d[i >> 3] << 1) | b);
	}

	taskEXIT_CRITICAL();

	/* ④ 校验：前 4 字节之和的低 8 位 */
	sum = (uint8_t)((uint16_t)d[0] + (uint16_t)d[1] + (uint16_t)d[2] + (uint16_t)d[3]);
	if (sum != d[4])
	{
		g_dht_reason = 6u; g_dht_rc[6]++; g_dht_fail++;
		return 0u;
	}

	g_dht_reason = 0u;
	g_dht_rc[0]++;
	g_dht_ok++;
	g_dht_diag_ready = 1u;          /* ★ 40 位宽度与应答时序都完整，可以体检 */

	*humi_x10 = (uint16_t)(((uint16_t)d[0] << 8) | (uint16_t)d[1]);   /* 0.1 %RH */

	/* ★ 2026-11-24 加：帧是好的、校验也对，但湿度字段本身就是 0xFFFF
	 *   · 这不是"读失败"，现在的代码会把它当正常值一路写进记录
	 *   · 单独计数是为了分清两种 65535：读不到 vs 器件说"我测不出来"
	 *   · 判据：这个数若远大于 g_dht_fail，说明短掉点走的是这条路 */
	if (*humi_x10 == 0xFFFFu)
	{
		if (g_dht_humi_ffff < 0xFFFFu) { g_dht_humi_ffff++; }
	}
	g_dht_last_humi = *humi_x10;

	t = (uint16_t)(((uint16_t)d[2] << 8) | (uint16_t)d[3]);            /* 0.1 °C */
	*temp_x10 = (t & 0x8000u) ? (int16_t)(-(int16_t)(t & 0x7FFFu)) /* bit15 = 符号 */
	                          : (int16_t)t;
	g_dht_last_temp = *temp_x10;
	return 1u;
}

/*------------------------------------------------------------------------------
 *  ★ 2026-11-24 加：把上一次读的判决余量打出来（只在任务里调，临界区内禁止）
 *
 * ★ 打四组：
 *   · 40 位各自的高电平宽度 —— 判 0/1 的原始依据
 *   · 两簇的边界，以及它们离门限还有多远（margin）
 *   · 器件应答时序：释放->拉低、应答低、应答高
 *   · reason 0..6 的累计分布（成功也算 reason=0）
 *
 * ★ 判读（重点是 margin 那一行）：
 *   · 两项 margin 都 ≥ 15us ⇒ 时序不是嫌疑，去查供电与器件本身
 *   · 任一项 < 8us          ⇒ 判决点本来就贴边，这颗粒子的位宽偏出手册
 *   · ack_delay 接近 200us  ⇒ 器件反应慢到快超时，上拉或供电有问题
 *----------------------------------------------------------------------------*/
void DHT22_DumpDiag(void)
{
	uint8_t  i;
	uint16_t w0min = 0xFFFFu, w0max = 0u;
	uint16_t w1min = 0xFFFFu, w1max = 0u;
	uint8_t  n0 = 0u, n1 = 0u;
	uint16_t d0, d1;

	if (g_dht_diag_ready == 0u)
	{
		Serial_Printf("DHT diag: no complete frame yet (send H after a read)\r\n");
		return;
	}

	Serial_Printf("DHT diag: 40-bit high widths (us), decide 1 when > %u\r\n",
	              (unsigned)DHT_BIT1_MIN_US);
	for (i = 0u; i < (uint8_t)DHT_BITS; i++)
	{
		if ((i % 10u) == 0u) { Serial_Printf("  bit %u:", (unsigned)i); }
		Serial_Printf(" %u", (unsigned)g_dht_width[i]);
		if ((i % 10u) == 9u) { Serial_Printf("\r\n"); }
	}

	for (i = 0u; i < (uint8_t)DHT_BITS; i++)
	{
		if (g_dht_bits[i] != 0u)
		{
			n1++;
			if (g_dht_width[i] < w1min) { w1min = g_dht_width[i]; }
			if (g_dht_width[i] > w1max) { w1max = g_dht_width[i]; }
		}
		else
		{
			n0++;
			if (g_dht_width[i] < w0min) { w0min = g_dht_width[i]; }
			if (g_dht_width[i] > w0max) { w0max = g_dht_width[i]; }
		}
	}

	if ((n0 != 0u) && (n1 != 0u))
	{
		Serial_Printf("DHT diag: 0-bit n=%u w=%u..%u us | 1-bit n=%u w=%u..%u us\r\n",
		              (unsigned)n0, (unsigned)w0min, (unsigned)w0max,
		              (unsigned)n1, (unsigned)w1min, (unsigned)w1max);

		d0 = (uint16_t)((DHT_BIT1_MIN_US > w0max) ? (DHT_BIT1_MIN_US - w0max) : 0u);
		d1 = (uint16_t)((w1min > DHT_BIT1_MIN_US) ? (w1min - DHT_BIT1_MIN_US) : 0u);
		Serial_Printf("DHT diag: margin below=%u us above=%u us (both >=15 => timing OK)\r\n",
		              (unsigned)d0, (unsigned)d1);
	}
	else
	{
		Serial_Printf("DHT diag: all 40 bits decoded as %u, frame is wrong, ignore widths\r\n",
		              (unsigned)((n0 != 0u) ? 0u : 1u));
	}

	Serial_Printf("DHT diag: ack delay=%u us (spec 20..40) low=%u high=%u us (80/80)\r\n",
	              (unsigned)g_dht_ack_delay_us,
	              (unsigned)g_dht_ack_low_us, (unsigned)g_dht_ack_high_us);

	Serial_Printf("DHT diag: ok=%u fail=%u | r0=%u r1=%u r2=%u r3=%u r4=%u r5=%u r6=%u\r\n",
	              (unsigned)g_dht_ok, (unsigned)g_dht_fail,
	              (unsigned)g_dht_rc[0], (unsigned)g_dht_rc[1], (unsigned)g_dht_rc[2],
	              (unsigned)g_dht_rc[3], (unsigned)g_dht_rc[4], (unsigned)g_dht_rc[5],
	              (unsigned)g_dht_rc[6]);

	Serial_Printf("DHT diag: reinit=%u humi_ffff=%u\r\n",
	              (unsigned)g_dht_reinit_cnt, (unsigned)g_dht_humi_ffff);

	{
		int16_t ta = (g_dht_last_temp < 0) ? (int16_t)(-g_dht_last_temp) : g_dht_last_temp;
		Serial_Printf("DHT diag: this frame h=%u.%u %%RH t=%s%d.%d C\r\n",
		              (unsigned)(g_dht_last_humi / 10u), (unsigned)(g_dht_last_humi % 10u),
		              (g_dht_last_temp < 0) ? "-" : "",
		              (int)(ta / 10), (int)(ta % 10));
	}
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
