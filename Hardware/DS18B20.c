#include "stm32f10x.h"
#include "DS18B20.h"
#include "DWT_Delay.h"
#include "FreeRTOS.h"
#include "task.h"

/*==============================================================================
 * 1-Wire 时序常量（单位：微秒）
 *   参考 DS18B20 数据手册的 min/typ/max 列，这里取「留余量且互相不冲突」的值：
 *   一个时隙 >=60us；写 1 拉低 1~15us；写 0 拉低 60~120us；读时隙拉低 1~15us 后采样。
 *============================================================================*/
#define OW_RESET_LOW_US   500    /* 复位脉冲拉低：规范 480~960us */
#define OW_PRESENCE_US    70     /* 释放后等 60~70us 再采样存在脉冲 */
#define OW_W1_LOW_US      6      /* 写 1：拉低 1~15us */
#define OW_W0_LOW_US      65     /* 写 0：拉低 60~120us */
#define OW_SLOT_US        70     /* 单个时隙总长（>=60us） */
#define OW_RD_LOW_US      3      /* 读时隙：拉低 1~15us 后释放 */
#define OW_RD_SAMPLE_US   2      /* ★ 9 -> 2：实测 OW_Release() 里的 GPIO_Init 要 4.26us
                                  *   （SPL 会遍历 16 个引脚），加上拉低 3.5us，采样点被推到
                                  *   ~17us —— 已超出 DS18B20 的"下降沿后 1~15us 采样"窗口，
                                  *   于是每位都读到 1（read FAILED / sp9 全 FF）。
                                  *   改成 2us 后采样点回到 ~10us（窗口中段）。
                                  *   ★ 彻底修法是改用固定 GPIO_Mode_Out_OD + 外部 4.7k 上拉，
                                  *     每 bit 零重配 —— 见 30 号 §十八。 */

/*------------------------------------------------------------------------------
 * 总线电平控制（IPU / Out_PP 动态切换，免外部上拉）
 *------------------------------------------------------------------------------*/
static void OW_Release(void)
{
	GPIO_InitTypeDef g;
	g.GPIO_Pin   = DS18B20_PIN;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	g.GPIO_Mode  = GPIO_Mode_IPU;          /* 输入上拉：把线拉到高 */
	GPIO_Init(DS18B20_GPIO, &g);
}

static void OW_PullLow(void)
{
	GPIO_InitTypeDef g;
	g.GPIO_Pin   = DS18B20_PIN;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	g.GPIO_Mode  = GPIO_Mode_Out_PP;       /* 推挽输出低 */
	GPIO_Init(DS18B20_GPIO, &g);
	GPIO_ResetBits(DS18B20_GPIO, DS18B20_PIN);
}

static uint8_t OW_Level(void)
{
	return GPIO_ReadInputDataBit(DS18B20_GPIO, DS18B20_PIN);
}

/*------------------------------------------------------------------------------
 * 字节收发（1-Wire 是 LSB 先出）
 *
 * ★★ 临界区代价量化（2026-10-11 独立审查 F20 补 ✗ 2026-10-19 数字重算 ✓）
 *   下面两个函数的【整字节 8 个时隙都在 taskENTER_CRITICAL 里】✗
 *     · 每字节 ≈ 8 × **78us** ≈ **630us**（★ 原写 8×70 ≈ 560us ✗ 偏低 ✓）
 *     · 一次 DS18B20_Read 要读 9 字节暂存器 + 2 条命令 → 约 **7.5ms**
 *       （★ 原写 6ms ✓）
 *     · 加 DS18B20_Start 的 **570us**（★ 原注释【漏算】了这一项 ✓）
 *       → ★ 每秒约 **9.3ms**，占空比 **0.93%**
 *       （★ 原写 7.2ms / 0.72% ✓）
 *     ⚠️ ★ 结论不变（★ 仍可接受 ✓）✗ 只是【数字要准】—— 余量算错会误导定容 ✓
 *
 *  影响评估（★ 可接受 ✓ —— 审查确认不必改 ✗）：
 *   · taskENTER_CRITICAL 在 Cortex-M3 上是设 BASEPRI ✗
 *     → SysTick（优先级 15）被屏蔽 ✗ → ★ 这期间【节拍会丢】✗
 *   · 但 task_sensor 每秒才读一次 ✓ → ACQ 的 100ms 节拍平均漂移仅 0.7% ✓
 *     ★ 实测印证：1 小时长稳里 tick 步进 98.8% 严格 +100 ✓ 真异常 0 处 ✓
 *   · ★★ rec.rtc 来自 RTC 硬件计数器 ✗ → 时间戳【不受节拍丢失影响】✓✓
 *   · 临界区内只有 DWT_Delay_us 忙等 ✓ 没有调用任何可能阻塞的 FreeRTOS API ✓ 不会死锁 ✓
 *
 *  ★ 什么时候要重新评估 ✗：挂多个 DS18B20 / 提高读取频率 → 这里会先成为瓶颈 ✓
 *------------------------------------------------------------------------------*/
static void OW_WriteByte(uint8_t b)
{
	uint8_t i;

	taskENTER_CRITICAL();
	for (i = 0; i < 8; i++)
	{
		OW_PullLow();
		if (b & 0x01)
		{
			DWT_Delay_us(OW_W1_LOW_US);
			OW_Release();
			DWT_Delay_us(OW_SLOT_US - OW_W1_LOW_US);
		}
		else
		{
			DWT_Delay_us(OW_W0_LOW_US);
			OW_Release();
			DWT_Delay_us(OW_SLOT_US - OW_W0_LOW_US);
		}
		b >>= 1;
	}
	taskEXIT_CRITICAL();
}

static uint8_t OW_ReadByte(void)
{
	uint8_t i, b = 0;

	taskENTER_CRITICAL();
	for (i = 0; i < 8; i++)
	{
		OW_PullLow();
		DWT_Delay_us(OW_RD_LOW_US);
		OW_Release();
		DWT_Delay_us(OW_RD_SAMPLE_US);
		if (OW_Level())
		{
			b |= (uint8_t)(1u << i);       /* LSB 先出 */
		}
		DWT_Delay_us(OW_SLOT_US - OW_RD_LOW_US - OW_RD_SAMPLE_US);
	}
	taskEXIT_CRITICAL();
	return b;
}

/*------------------------------------------------------------------------------
 * Dallas/Maxim CRC-8（多项式 x^8+x^5+x^4+1，反射写法 0x8C）
 * 用于校验读回的 9 字节暂存器（前 8 字节算出的 CRC 应等于第 9 字节）
 *------------------------------------------------------------------------------*/
static uint8_t OW_Crc8(const uint8_t *d, uint8_t len)
{
	uint8_t crc = 0, i, b;

	for (i = 0; i < len; i++)
	{
		crc ^= d[i];
		for (b = 0; b < 8; b++)
		{
			crc = (crc & 0x01) ? (uint8_t)((crc >> 1) ^ 0x8C) : (uint8_t)(crc >> 1);
		}
	}
	return crc;
}

/*------------------------------------------------------------------------------
 * 对外接口
 *------------------------------------------------------------------------------*/
uint8_t DS18B20_Init(void)
{
	uint8_t present;
	uint32_t g;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	/* 复位 + 存在脉冲：主机拉低 >=480us，释放后 60~70us 采样；从机在座会把线拉低 */
	taskENTER_CRITICAL();
	OW_PullLow();
	DWT_Delay_us(OW_RESET_LOW_US);
	OW_Release();
	DWT_Delay_us(OW_PRESENCE_US);
	present = (OW_Level() == 0) ? 1 : 0;
	taskEXIT_CRITICAL();

	/* ★★ 关键（2026-10-05 修）：必须【等存在脉冲真正结束】再发命令。
	 *   器件脉冲：释放后 15~60us 才开始，宽 60~240us（实测本器件 123us）
	 *             -> 结束于释放后 138~183us；
	 *   而"采样 70us + 固定再等 70us + 拉低/释放开销 ~9us" ≈ 释放后 150us
	 *             -> 第一条命令撞在脉冲中间！器件此刻仍按住总线拉低，
	 *                我们"释放"时线还是低 -> 命令每位被读成 0 -> 整条命令作废。
	 *   现象就是"复位有应答、但 0x33 / 0xCC+0xBE 全部不应答"。
	 *   照正点原子参考驱动的做法：循环等线回到高（最多 400us）再往下走。 */
	if (present)
	{
		for (g = 0; g < 400u; g++)
		{
			if (OW_Level() != 0) { break; }
			DWT_Delay_us(1);
		}
	}
	DWT_Delay_us(OW_SLOT_US);              /* 再留一个时隙的余量 */
	return present;
}

void DS18B20_Start(void)
{
	if (!DS18B20_Init())
	{
		return;                            /* 器件不在就别空发命令 */
	}
	OW_WriteByte(0xCC);                    /* Skip ROM：总线上只有一个器件 */
	OW_WriteByte(0x44);                    /* Convert T：启动温度转换 */
	DWT_Delay_us(OW_SLOT_US);
}

uint8_t DS18B20_Read(int16_t *raw)
{
	uint8_t sp[9];
	uint8_t i;

	if (!DS18B20_Init())
	{
		return 0;
	}
	OW_WriteByte(0xCC);                    /* Skip ROM */
	OW_WriteByte(0xBE);                    /* Read Scratchpad：读 9 字节 */
	for (i = 0; i < 9; i++)
	{
		sp[i] = OW_ReadByte();
	}

	if (OW_Crc8(sp, 8) != sp[8])
	{
		return 0;                          /* CRC 不过：数据不可信 */
	}

	*raw = (int16_t)(((uint16_t)sp[1] << 8) | sp[0]);   /* 温度寄存器：低字节在前 */
	return 1;
}

int16_t DS18B20_ToX100(int16_t raw)
{
	/* 12 位分辨率、0.0625 C/LSB -> 放大 100 倍：raw * 6.25 = raw * 25 / 4 */
	return (int16_t)(((int32_t)raw * 25) / 4);
}

/*------------------------------------------------------------------------------
 * 临时调试：位采样探针
 *   在一个读时隙里，释放后每隔 2us 采一次电平，共 7 点（约等于下降沿后 4~16us），
 *   组成 7 位掩码：bit0 = 最早那点，1 = 高电平。
 *   判读：
 *     全 1 (0x7F)          -> 器件在这个时隙完全没驱动总线（命令阶段没通 或 器件不响应）
 *     形如 0000011 / 0000111 -> 器件确实拉低了、并在某个时刻抬起 -> 可据此定出采样点
 *     全 0 (0x00)          -> 总线一直低（上拉/接触问题）
 *------------------------------------------------------------------------------*/
static uint8_t OW_ReadBit7(void)
{
	uint8_t m = 0, k;

	OW_PullLow();
	DWT_Delay_us(2);
	OW_Release();
	for (k = 0; k < 7; k++)
	{
		DWT_Delay_us(2);
		if (OW_Level())
		{
			m |= (uint8_t)(1u << k);
		}
	}
	DWT_Delay_us(OW_SLOT_US - 2 - 14);
	return m;
}

/* 临时调试用：复位 + SkipROM + ReadScratchpad，但每一位用 OW_ReadBit7 采 7 个点。
 * m8: 输出 8 位的掩码数组；present: 输出存在脉冲检测结果。测好可删。 */
void DS18B20_BitProbe(uint8_t *m8, uint8_t *present)
{
	uint8_t i;

	taskENTER_CRITICAL();
	OW_PullLow();
	DWT_Delay_us(OW_RESET_LOW_US);
	OW_Release();
	DWT_Delay_us(OW_PRESENCE_US);
	*present = (OW_Level() == 0) ? 1 : 0;
	taskEXIT_CRITICAL();
	DWT_Delay_us(OW_SLOT_US);

	OW_WriteByte(0xCC);                    /* Skip ROM */
	OW_WriteByte(0xBE);                    /* Read Scratchpad */

	for (i = 0; i < 8; i++)
	{
		m8[i] = OW_ReadBit7();
	}
}

/* 临时调试用：量我们自己时隙的实际宽度（单位 = DWT 周期，72MHz 下 1us = 72 周期）
 *   pull_ovh : OW_PullLow() 本身耗时（SPL 的 GPIO_Init 会遍历 16 个引脚，不可忽略）
 *   lo6us    : DWT_Delay_us(6) 实测
 *   one_ms   : DWT_Delay_us(1000) 实测（应为 72000）
 *   slot70us : DWT_Delay_us(70) 实测
 * 期望：one_ms ≈ 72000；pull_ovh 若达到几百周期以上，就说明"每 bit 两次 GPIO_Init"
 *       把时隙整体推长了 —— 那是把采样点推出 15us 窗口的元凶。测好可删。 */
void DS18B20_TimingCheck(uint32_t *pull_ovh, uint32_t *lo6us, uint32_t *one_ms, uint32_t *slot70us,
                         uint32_t *rd_release, uint32_t *rd_sample)
{
	uint32_t t0;

	t0 = DWT->CYCCNT;  OW_PullLow();                *pull_ovh = DWT->CYCCNT - t0;

	t0 = DWT->CYCCNT;  DWT_Delay_us(OW_W1_LOW_US);  *lo6us    = DWT->CYCCNT - t0;

	t0 = DWT->CYCCNT;  DWT_Delay_us(1000);          *one_ms   = DWT->CYCCNT - t0;

	OW_PullLow();
	t0 = DWT->CYCCNT;  DWT_Delay_us(OW_SLOT_US);    *slot70us = DWT->CYCCNT - t0;
	OW_Release();

	/* 读时隙实测：t0 取在 OW_PullLow() 返回之后（引脚刚变低的时刻） -> 下面两个值
	 * 就是"下降沿之后多少周期"。期望 rd_sample < 15*72=1080（窗口上限）。 */
	OW_PullLow();
	t0 = DWT->CYCCNT;
	DWT_Delay_us(OW_RD_LOW_US);
	OW_Release();
	*rd_release = DWT->CYCCNT - t0;                 /* 下降沿 -> 释放 */
	DWT_Delay_us(OW_RD_SAMPLE_US);
	*rd_sample  = DWT->CYCCNT - t0;                 /* 下降沿 -> 采样 */
}

/* 临时静态测试（测完删）：low=1 -> DQ 一直拉低；low=0 -> 一直释放（上拉为高）。
 * 用途：程序停在这里，用万用表量 PB14 —— 拉低应 0V、释放应 ≈VDD，
 *       以此证明"代码对 PB14 的驱动/释放都正常"。 */
void DS18B20_PinTest(uint8_t low)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	if (low)
	{
		OW_PullLow();
	}
	else
	{
		OW_Release();
	}
	for (;;) { }
}

/* 临时静态测试（测完删）：死循环只做"复位脉冲"（500us 拉低 + 约 140us 释放）。
 * 用途：万用表量 PB14 的【平均电压】应明显下降（约 0.2~0.9V，而不是 VDD）——
 *       证明"在真实时序里代码确实在拉低总线"，而不是只在静态下能拉低。 */
void DS18B20_ResetLoop(void)
{
	for (;;)
	{
		(void)DS18B20_Init();
	}
}

/* 临时调试用（测好可删）：测"存在脉冲"的宽度（单位 us），做法照正点原子参考驱动：
 *   复位拉低后【循环等待】线变低，再【循环等待】线变高 —— 两段之间的时长就是脉冲宽度。
 *   真 DS18B20 的脉冲宽 60~240us。
 * 返回：0 = 一直没变低（无应答）；0xFFFF = 变低后一直不放开（线卡死）；否则为脉冲宽度 us。 */
uint16_t DS18B20_MeasurePresence(void)
{
	uint32_t t0, t1;
	uint32_t g;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	OW_PullLow();
	DWT_Delay_us(OW_RESET_LOW_US);
	OW_Release();

	for (g = 0; g < 300u; g++)                     /* 等它变低，最多 ~300us */
	{
		if (OW_Level() == 0) { break; }
		DWT_Delay_us(1);
	}
	if (OW_Level() != 0) { return 0; }             /* 一直是高：没有应答 */

	t0 = DWT->CYCCNT;
	for (g = 0; g < 300u; g++)                     /* 等它放开，最多 ~300us */
	{
		if (OW_Level() != 0) { break; }
		DWT_Delay_us(1);
	}
	t1 = DWT->CYCCNT;
	if (OW_Level() == 0) { return 0xFFFF; }        /* 一直是低：线卡死 */

	return (uint16_t)((t1 - t0) / 72u);
}

/* 临时调试用：读 ROM（0x33 Read ROM）—— 一步分辨两件事，测好可删：
 *   ① 写命令到底通不通：若能回 8 字节有效 ROM，说明 0xCC/0xBE 这类单字节命令也能被收到
 *   ② 这颗芯片到底是什么：真 DS18B20 的 ROM[0] = 0x28（家族码），ROM[7] = 其 CRC
 * 返回 1 = 存在脉冲检测到。 */
uint8_t DS18B20_ReadRom(uint8_t *rom8)
{
	uint8_t i;

	if (!DS18B20_Init())
	{
		return 0;
	}
	OW_WriteByte(0x33);                    /* Read ROM */
	for (i = 0; i < 8; i++)
	{
		rom8[i] = OW_ReadByte();
	}
	return 1;
}

/* 临时调试用：不做 CRC 校验，把 9 字节原样给出（CRC 失败时定位用，测好可删） */
uint8_t DS18B20_ReadScratchpad(uint8_t *sp9)
{
	uint8_t i;

	if (!DS18B20_Init())
	{
		return 0;
	}
	OW_WriteByte(0xCC);                    /* Skip ROM */
	OW_WriteByte(0xBE);                    /* Read Scratchpad */
	for (i = 0; i < 9; i++)
	{
		sp9[i] = OW_ReadByte();
	}
	return 1;
}
