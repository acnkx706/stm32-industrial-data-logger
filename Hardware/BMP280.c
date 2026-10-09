#include "stm32f10x.h"
#include "DWT_Delay.h"
#include "MyI2C.h"
#include "Serial.h"
#include "BMP280.h"

/*==============================================================================
 *  寄存器与常量
 * ============================================================================*/
#define BMP_REG_ID        0xD0u     /* 应回 0x58 ✓ */
#define BMP_REG_RESET     0xE0u
#define BMP_REG_STATUS    0xF3u     /* bit3 = measuring ✓ */
#define BMP_REG_CTRL      0xF4u     /* [7:5]osrs_t [4:2]osrs_p [1:0]mode */
#define BMP_REG_CONFIG    0xF5u     /* [7:5]t_sb [4:2]filter [0]spi3w */
#define BMP_REG_DATA      0xF7u     /* 0xF7..0xFC = press(3) + temp(3) ✓ */
#define BMP_REG_CALIB     0x88u     /* 0x88..0xA1 = 26 字节校准 ✓ */

/* ★ 采样与滤波配置（bring-up 阶段取值 ✓ 后面可按噪声实测调 ✓）
 *   osrs_t = ×2 (010) / osrs_p = ×4 (011) / mode = forced 单次 (01)
 *   → 0xF4 = 0x4D ✓
 *   ★ 为什么用 forced 单次而不是连续（normal）：
 *     我们 1 秒读一次 ✓ —— forced 只在需要时测一次、测完回到睡眠 ✓ 最省电 ✓
 *     （continuous 会一直测，白耗电 ✓）*/
#define BMP_CTRL_FORCED   ((uint8_t)((0x02u << 5) | (0x03u << 2) | 0x01u))   /* 0x4D */
#define BMP_CONFIG_VAL    0x08u     /* filter = ×4 (010) ✓ t_sb 无关（forced）✓ */

/*==============================================================================
 *  低位 I2C 读写（只操作本器件 ✓ 与 MPU6050 共用同一总线 ✓）
 * ============================================================================*/
static uint8_t BMP_WriteReg(uint8_t reg, uint8_t val)
{
	MyI2C_Start();
	MyI2C_SendByte((uint8_t)(BMP280_I2C_ADDR << 1));
	if (MyI2C_ReceiveAck() != 0u) { MyI2C_Stop(); return 1u; }   /* 无应答 ✓ */
	MyI2C_SendByte(reg);
	MyI2C_ReceiveAck();
	MyI2C_SendByte(val);
	MyI2C_ReceiveAck();
	MyI2C_Stop();
	return 0u;
}

static uint8_t BMP_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len)
{
	uint8_t i;
	MyI2C_Start();
	MyI2C_SendByte((uint8_t)(BMP280_I2C_ADDR << 1));
	if (MyI2C_ReceiveAck() != 0u) { MyI2C_Stop(); return 1u; }   /* 无应答 ✓ */
	MyI2C_SendByte(reg);
	MyI2C_ReceiveAck();
	MyI2C_Start();                                    /* ★ 重复起始 ✓ */
	MyI2C_SendByte((uint8_t)((BMP280_I2C_ADDR << 1) | 0x01u));
	if (MyI2C_ReceiveAck() != 0u) { MyI2C_Stop(); return 1u; }
	for (i = 0u; i < len; i++)
	{
		buf[i] = MyI2C_ReceiveByte();
		MyI2C_SendAck((i == (uint8_t)(len - 1u)) ? 1u : 0u);      /* ★ 最后 NACK ✓ */
	}
	MyI2C_Stop();
	return 0u;
}

static uint8_t BMP_ReadReg(uint8_t reg)
{
	uint8_t v = 0u;
	(void)BMP_ReadRegs(reg, &v, 1u);
	return v;
}

/*==============================================================================
 *  校准系数
 *  ★★ 这里是最容易抄错的地方 ✗：寄存器里都是【无符号】读法 ✓，
 *     但数据手册里 dig_T2/T3 与 dig_P2..P9 是【有符号】✗
 *     → 读成 uint16 之后必须【按表转成 int16】✓
 *     抄错 = 气压完全乱 ✗（温度那条也一起崩 ✓）
 * ============================================================================*/
static uint16_t dig_T1;
static int16_t  dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

/* ★★ 2026-10-09 修复（自检抓出来的 ✗）：t_fine 必须是【32 位】✗✗
 *   原来写成 int16_t ✗ —— 而 t_fine 的典型值是十万量级（手册算例 = 128422 ✓）
 *   塞进 16 位会被截断：128422 → -2650 ✗
 *   → 温度算出 -52（应为 2508）✗，气压跟着一起崩 ✗
 *   ★ 教训：这是【类型宽度】这一类 bug ✓（和 SD 那次 uint8_t 循环变量同族 ✓）
 *     数据手册说了位宽就照着写 ✓ 不要凭"看起来够用"缩窄 ✗ */
static int32_t s_tfine;                                 /* 温度补偿的中间量 ✓ */

static void BMP_ReadCalib(void)
{
	uint8_t b[26];

	if (BMP_ReadRegs(BMP_REG_CALIB, b, 26u) != 0u)
	{
		return;
	}
	/* 小端：低字节在前 ✓；每个系数 2 字节 ✓（b[24]/b[25] 是保留寄存器 ✓ 不用 ✓）*/
	dig_T1 = (uint16_t)((uint16_t)b[1]  << 8 | b[0]);
	dig_T2 = (int16_t) ((uint16_t)b[3]  << 8 | b[2]);
	dig_T3 = (int16_t) ((uint16_t)b[5]  << 8 | b[4]);
	dig_P1 = (uint16_t)((uint16_t)b[7]  << 8 | b[6]);
	dig_P2 = (int16_t) ((uint16_t)b[9]  << 8 | b[8]);
	dig_P3 = (int16_t) ((uint16_t)b[11] << 8 | b[10]);
	dig_P4 = (int16_t) ((uint16_t)b[13] << 8 | b[12]);
	dig_P5 = (int16_t) ((uint16_t)b[15] << 8 | b[14]);
	dig_P6 = (int16_t) ((uint16_t)b[17] << 8 | b[16]);
	dig_P7 = (int16_t) ((uint16_t)b[19] << 8 | b[18]);
	dig_P8 = (int16_t) ((uint16_t)b[21] << 8 | b[20]);
	dig_P9 = (int16_t) ((uint16_t)b[23] << 8 | b[22]);
	/* b[24]/b[25] 是保留寄存器（0xA0/0xA1）✓ 不用 ✓ */
}

/*==============================================================================
 *  补偿运算（★ 全部定点整数 ✗ 无浮点 ✓ —— 项目铁律）
 *  ★ 公式取自 Bosch BMP280 数据手册的【32-bit 定点版】✓ 逐字搬运 ✓
 * ============================================================================*/
static int32_t BMP_CompensateT(int32_t adc_T)
{
	int32_t var1, var2, T;

	var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
	        ((int32_t)dig_T3)) >> 14;
	s_tfine = var1 + var2;                              /* ★ 32 位直接存 ✓ 不再强转 ✗ */
	T = (int32_t)((s_tfine * 5 + 128) >> 8);            /* ★ 单位 0.01 °C ✓ */
	return T;
}

static uint32_t BMP_CompensateP(int32_t adc_P)
{
	int64_t var1, var2, p;

	var1 = ((int64_t)s_tfine) - 128000;
	var2 = var1 * var1 * (int64_t)dig_P6;
	var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
	var2 = var2 + (((int64_t)dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
	if (var1 == 0) { return 0u; }                       /* 防除零 ✓ */
	p = 1048576 - adc_P;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);

	/* ★★★ 2026-10-09 修复（又是自检抓的 ✗）：定点版气压公式的输出是【Q24.8】✗
	 *   即结果是【Pa × 256】✓ —— 必须自己再 >> 8 才是 Pa ✓✓
	 *   实测铁证：自检返回 25767233，而 25767233 / 256 = 100653.25
	 *             ★ 正好等于手册期望值 100653 ✓✓ → 就是漏了这一次移位 ✓
	 *   ★ 注意两条公式的输出单位【不对称】✗（最易漏的地方 ✓）：
	 *     温度那条 → 0.01 °C ✓（无额外缩放）
	 *     气压那条 → Pa × 256 ✗（需自己 >> 8）*/
	return (uint32_t)(p >> 8);                          /* ★ Q24.8 → Pa ✓ */
}

/*==============================================================================
 *  对外接口
 * ============================================================================*/
uint8_t BMP280_Init(void)
{
	uint8_t id;

	if (BMP_ReadReg(BMP_REG_ID) == 0xFFu)               /* 全 1 = 无应答（读失败）✓ */
	{
		/* ★ 注意：真器件读 0xD0 不可能得到 0xFF ✓ 所以这个判据够用 ✓
		 *   （更严格应该看 ACK 位 ✓ 见 main.c 里的探针 ✓）*/
		return 2u;
	}
	id = BMP_ReadReg(BMP_REG_ID);
	if (id != BMP280_ID_EXPECT) { return 1u; }          /* 型号不符 ✓ */

	BMP_ReadCalib();                                    /* 读 26 字节校准 ✓ */
	(void)BMP_WriteReg(BMP_REG_CONFIG, BMP_CONFIG_VAL);
	(void)BMP_WriteReg(BMP_REG_CTRL, 0x00u);            /* 先进睡眠，等读数时再触发 ✓ */
	return 0u;
}

uint8_t BMP280_Read(int32_t *press_x10pa, int32_t *temp_c100)
{
	uint8_t  b[6];
	uint8_t  t;
	int32_t  adc_P, adc_T;
	int32_t  T;
	uint32_t P;

	/* ① 触发一次 forced 测量 ✓ */
	if (BMP_WriteReg(BMP_REG_CTRL, BMP_CTRL_FORCED) != 0u) { return 1u; }

	/* ② 等测量完成（轮询 status 的 bit3 ✓ 带超时 ✗ 不能死等 ✓）
	 *    ×2/×4 过采样的典型测量时间约 10ms 量级 ✓ → 给 50ms 余量 ✓ */
	for (t = 0u; t < 100u; t++)
	{
		if ((BMP_ReadReg(BMP_REG_STATUS) & 0x08u) == 0u) { break; }   /* bit3=0 = 测完 ✓ */
		DWT_Delay_us(500);                              /* 100 × 500us = 50ms 上限 ✓ */
	}
	if (t >= 100u) { return 2u; }                       /* 超时 ✓ */

	/* ③ 读 6 字节：press(3) + temp(3) ✓ */
	if (BMP_ReadRegs(BMP_REG_DATA, b, 6u) != 0u) { return 3u; }

	adc_P = (int32_t)(((uint32_t)b[0] << 12) | ((uint32_t)b[1] << 4) | ((uint32_t)b[2] >> 4));
	adc_T = (int32_t)(((uint32_t)b[3] << 12) | ((uint32_t)b[4] << 4) | ((uint32_t)b[5] >> 4));

	/* ④ 补偿（★ 必须先算温度 ✓ —— 气压公式依赖 s_tfine ✓）*/
	T = BMP_CompensateT(adc_T);
	P = BMP_CompensateP(adc_P);

	if (temp_c100    != NULL) { *temp_c100    = T; }
	if (press_x10pa  != NULL) { *press_x10pa  = (int32_t)(P * 10u); }   /* Pa → ×10 Pa ✓
	                                                                     * 101325×10 = 1013250 ✓ 不溢出 ✓ */
	return 0u;
}

/*==============================================================================
 *  ★ 自检：用数据手册的官方算例验证补偿公式
 *  期望：t_fine = 128422 → T = 2508（25.08 °C）✓
 *        P = 100653 Pa ✓
 *  ⚠️ 下面这些算例常量是【凭记忆写入手册值】✗ ——
 *     若自检不过 ✗，先怀疑"我抄的算例值"✗，再怀疑公式 ✓（两处分开验证 ✓）
 * ============================================================================*/
uint8_t BMP280_SelfTest(void)
{
	int32_t  T;
	uint32_t P;
	uint8_t  ok = 1u;

	/* 手册算例的校准系数 ✓ */
	dig_T1 = 27504u;  dig_T2 = 26435;   dig_T3 = -1000;
	dig_P1 = 36477u;  dig_P2 = -10685;  dig_P3 = 3024;   dig_P4 = 2855;
	dig_P5 = 140;     dig_P6 = -7;      dig_P7 = 15500;  dig_P8 = -14600;  dig_P9 = 6000;

	T = BMP_CompensateT(519888);                        /* 期望 2508 ✓ */
	P = BMP_CompensateP(415148);                        /* 期望 100653 ✓ */

	Serial_Printf("BMP280 selftest: T=%ld (expect 2508)  P=%lu (expect 100653)\r\n",
	              (long)T, (unsigned long)P);
	if (T != 2508) { ok = 0u; }
	if (P != 100653u) { ok = 0u; }
	Serial_Printf("BMP280 selftest: %s\r\n", ok ? "PASS" : "FAIL");

	BMP_ReadCalib();                                    /* ★ 自检用完把真校准读回来 ✓
	                                                     *   （否则后面读数会全错 ✗）*/
	return ok;
}
