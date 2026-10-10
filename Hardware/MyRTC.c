#include "stm32f10x.h"
#include "DWT_Delay.h"
#include "Serial.h"
#include "MyRTC.h"

/*==============================================================================
 *  ★ F103 的 RTC 有四个"静默失败"型的坑 ✗（不报错、最难查 ✓）
 *    ① 访问 BDCR / RTC 属于【备份域】✗ → 必须先 PWR_BackupAccessCmd(ENABLE) ✓
 *    ② ★★ 每次写 RTC 寄存器后都要 RTC_WaitForLastTask() ✗
 *       —— RTC 在 32.768kHz 域、APB 在 72MHz 域 ✗
 *          不等的话写入会【静默丢失】✗✓（编译不报、运行不提示 ✓）
 *    ③ 设预分频必须先进配置模式（RTC_EnterConfigMode ✓）
 *    ④ ★ BKP 里放【版本化魔数】✓ —— 已配过就不要再配 ✗
 *       （否则每次复位都把时间清零 ✗；改初始化逻辑就要换魔数 ✓）
 * ============================================================================*/

/* ★ 魔数带"版本"语义：改了初始化逻辑就换一个 ✓
 *   0xA5A5 = 第一步（只做走时验证）—— 已废弃 ✗
 *   0x5A5B = 第二步（编译时刻设初值 ✓）—— 当前 ✓ */
#define RTC_BKP_MAGIC       0x5A5Bu
#define RTC_BKP_DR          BKP_DR1

/* LSE 起振检查（★ 与 RTC 逻辑分开验 ✓ 一次只动一个变量 ✓）*/
static uint8_t MyRTC_CheckLSE(void)
{
	uint32_t t;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	PWR_BackupAccessCmd(ENABLE);                 /* ★ 备份域解锁 ✓ */

	RCC_LSEConfig(RCC_LSE_ON);

	for (t = 0u; t < 3000u; t++)                 /* ★ 最多等 ~3 秒（不死等 ✗）*/
	{
		if (RCC_GetFlagStatus(RCC_FLAG_LSERDY) != RESET) { return RTC_OK; }
		DWT_Delay_ms(1u);
	}
	return RTC_ERR_NO_LSE;                       /* ★ 超时：LSE 没起振 ✗ */
}

/* 解析 __DATE__ / __TIME__ → 年月日时分秒
 * ★ __DATE__ = "Oct  9 2026" ✗ —— 日字段是【宽度 2、空格补齐】✗
 *   "Oct  9"（两个空格 ✓）与 "Oct 19"（一个空格 ✓）都必须吃得下 ✓✓ */
static void MyRTC_CompileTime(RTC_DateTime_t *dt)
{
	static const char *mon = "JanFebMarAprMayJunJulAugSepOctNovDec";
	const char *d = __DATE__;
	const char *t = __TIME__;
	uint8_t i, p;
	uint16_t v;

	for (i = 0u; i < 12u; i++)
	{
		if ((d[0] == mon[i * 3u]) && (d[1] == mon[i * 3u + 1u]) && (d[2] == mon[i * 3u + 2u])) { break; }
	}
	dt->month = (uint8_t)(i + 1u);

	p = 4u;                                      /* ★ 跳过 "Mon " 这前 4 个字符 ✓ */
	v = 0u;
	while (d[p] == ' ') { p++; }                 /* ★ 吃掉空格补齐 ✓ */
	while ((d[p] >= '0') && (d[p] <= '9')) { v = (uint16_t)(v * 10u + (uint8_t)(d[p] - '0')); p++; }
	dt->day = (uint8_t)v;
	p++;                                         /* ★ 跳过分隔空格 ✓ */
	dt->year = (uint16_t)((d[p] - '0') * 1000u + (d[p + 1] - '0') * 100u +
	                      (d[p + 2] - '0') * 10u + (d[p + 3] - '0'));

	dt->hour   = (uint8_t)((t[0] - '0') * 10u + (t[1] - '0'));
	dt->minute = (uint8_t)((t[3] - '0') * 10u + (t[4] - '0'));
	dt->second = (uint8_t)((t[6] - '0') * 10u + (t[7] - '0'));
}

static uint8_t MyRTC_IsLeap(uint16_t y)
{
	return (uint8_t)(((y % 4u == 0u) && ((y % 100u != 0u) || (y % 400u == 0u))) ? 1u : 0u);
}

/* ---- 年月日时分秒 → Unix 时间戳（★ 纯整数 ✓ 无浮点 ✓）---- */
uint32_t MyRTC_YmdToUnix(uint16_t y, uint8_t mo, uint8_t d,
                         uint8_t hh, uint8_t mm, uint8_t ss)
{
	/* 每月【之前】累计的天数（按平年 ✓；闰年的补偿在后面单独加 ✓）*/
	static const uint16_t mdays[12] = {0u,31u,59u,90u,120u,151u,181u,212u,243u,273u,304u,334u};
	uint32_t days = 0u;
	uint16_t yy;

	for (yy = 1970u; yy < y; yy++)               /* ★ 逐年累加（简单、看得懂 ✓）*/
	{
		days += (MyRTC_IsLeap(yy) != 0u) ? 366u : 365u;
	}
	days += mdays[mo - 1u];
	if ((mo > 2u) && (MyRTC_IsLeap(y) != 0u)) { days += 1u; }   /* ★ 闰年 2 月多一天 ✓ */
	days += (uint32_t)(d - 1u);

	return days * 86400u + (uint32_t)hh * 3600u + (uint32_t)mm * 60u + (uint32_t)ss;
}

/* ---- Unix 时间戳 → 年月日时分秒（★ 与上面互为反函数 ✓ 显示/自检都要用 ✓）---- */
void MyRTC_UnixToYmd(uint32_t ts, RTC_DateTime_t *dt)
{
	static const uint16_t mdays[12] = {0u,31u,59u,90u,120u,151u,181u,212u,243u,273u,304u,334u};
	uint32_t days = ts / 86400u;
	uint32_t rem  = ts % 86400u;
	uint8_t  i, leap;

	dt->hour   = (uint8_t)(rem / 3600u);
	dt->minute = (uint8_t)((rem % 3600u) / 60u);
	dt->second = (uint8_t)(rem % 60u);

	dt->year = 1970u;
	for (;;)
	{
		uint16_t ylen;
		leap = MyRTC_IsLeap(dt->year);
		ylen = (leap != 0u) ? 366u : 365u;
		if (days < (uint32_t)ylen) { break; }
		days -= (uint32_t)ylen;
		dt->year++;
	}

	/* ★ 从 12 月往回找：第一个"累计天数 <= 剩余天数"的月份 ✓
	 *   例：1 月时循环一次都不命中 → i 停在 0 → month = 1 ✓ 天然正确 ✓ */
	for (i = 11u; i > 0u; i--)
	{
		uint16_t md = (uint16_t)(mdays[i] + (((i >= 2u) && (leap != 0u)) ? 1u : 0u));
		if (days >= (uint32_t)md) { days -= (uint32_t)md; break; }
	}
	dt->month = (uint8_t)(i + 1u);
	dt->day   = (uint8_t)(days + 1u);
}

/*==============================================================================
 *  对外接口
 * ============================================================================*/
uint8_t MyRTC_Init(void)
{
	if (MyRTC_CheckLSE() != RTC_OK)
	{
		return RTC_ERR_NO_LSE;                   /* ★ 时钟源不成立，其他都别做 ✓ */
	}

	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);      /* ★ RTC 时钟源 = LSE ✓ */
	RCC_RTCCLKCmd(ENABLE);                       /* 使能 RTC 时钟 ✓ */
	RTC_WaitForSynchro();                        /* ★ 等 RTC 与 APB 同步 ✓ */

	/* ★ 打诊断：BDCR 的原始值信息量最大 ✓
	 *   bit0 LSEON / bit1 LSERDY / bit2 LSEBYP / bit8-9 RTCSEL / bit15 RTCEN ✓ */
	Serial_Printf("MyRTC: BDCR=0x%08X (bit0 LSEON / bit1 LSERDY / bit2 LSEBYP)\r\n",
	              (unsigned)RCC->BDCR);

	if (BKP_ReadBackupRegister(RTC_BKP_DR) != RTC_BKP_MAGIC)     /* ★ 只在第一次配 ✓ */
	{
		RTC_DateTime_t ct;
		uint32_t ts;

		MyRTC_CompileTime(&ct);
		ts = MyRTC_YmdToUnix(ct.year, ct.month, ct.day, ct.hour, ct.minute, ct.second);

		Serial_Printf("MyRTC: 1st time -> set from compile time %04u-%02u-%02u %02u:%02u:%02u\r\n",
		              (unsigned)ct.year, (unsigned)ct.month, (unsigned)ct.day,
		              (unsigned)ct.hour, (unsigned)ct.minute, (unsigned)ct.second);

		RTC_WaitForLastTask();
		RTC_EnterConfigMode();                   /* ★ 进配置模式才能改分频 ✓ */
		RTC_SetPrescaler(32767u);                /* ★ 32768/(32767+1) = 1 Hz ✓ */
		RTC_WaitForLastTask();
		RTC_ExitConfigMode();
		RTC_SetCounter(ts);                      /* ★ 编译时刻 ✓ */
		RTC_WaitForLastTask();
		BKP_WriteBackupRegister(RTC_BKP_DR, RTC_BKP_MAGIC);      /* ★ 打标记 ✓ */
	}
	else
	{
		Serial_Printf("MyRTC: already configured (BKP magic hit) -> keep counter\r\n");
	}

	return RTC_OK;
}

uint32_t MyRTC_GetUnix(void)
{
	uint32_t a, b;
    do { a = RTC_GetCounter(); b = RTC_GetCounter(); } while (a != b);
    return b;                     /* ★ RTC 已在跑，直接读 ✓ ，对跨时钟域问题进行保护，避免高八位跳变*/
}

void MyRTC_GetDateTime(RTC_DateTime_t *dt)
{
	if (dt != NULL) { MyRTC_UnixToYmd(RTC_GetCounter(), dt); }
}

void MyRTC_SetUnix(uint32_t ts)
{
	RTC_SetCounter(ts);
	RTC_WaitForLastTask();                       /* ★ 写完必须等 ✓ 否则静默丢失 ✗ */
}
