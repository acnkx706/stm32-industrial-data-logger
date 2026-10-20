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

/* ★★★★★ 2026-10-17 新增：【对过时没有】的独立标志 ✗✓
 *
 *  ★★★★ 为什么【必须】和 `RTC_BKP_DR`（DR1）分开 ✗✓：
 *    · `DR1` 只回答"RTC 被【初始化】过吗" ✗
 *    · 而初始化用的可能是【编译时刻】——
 *      一个可能【很旧】、而且被当成 UTC 用的值 ✓
 *    ⇒ ★★【"配过" ≠ "对过"】✗✓ 两者必须分开记 ✓✓
 *
 *  ★ 寄存器选择 ✗：`DR1`=RTC 魔数 / `DR2`=升级标志 / `DR3`=回退标志
 *    / `DR4`=低功耗预留 ⇒ ★【`DR5` 是空的】✓
 *  ★★ 语义 ✗：★ 只有 `MyRTC_SetUnix()`（串口 `T<unix>` 对时）才置它 ✓
 *    · 没装 VBAT ⇒ 【一断电就丢】⇒ 自动回到"不可信"✓✓
 *    · 而【软复位/看门狗复位】不会丢 ⇒ 时间还连着 ✗ 标记也该留着 ✓
 *      （★ 这正是我们要的语义 ✓）
 *  ★★★ 有没有 VBAT 都一样对 ✗：装了电池 ⇒ 断电也保留 ✗ 更准 ✓ */
#define RTC_TRUST_MAGIC     0x5A5Au
#define RTC_TRUST_DR        BKP_DR5

/* LSE 起振检查（★ 与 RTC 逻辑分开验 ✓ 一次只动一个变量 ✓）*/
/* ★ 前向声明（★ 降级路径在下面用到了定义在【更后面】的两个函数 ✓
 *   AC5 会把没声明的当"外部函数"✗ 而它们是 static ⇒ 链接报 L6218E ✓）*/
static void MyRTC_CompileTime(RTC_DateTime_t *dt);

/*==============================================================================
 *  ★★★★★ 2026-10-20 重写：**LSE 三次重试 + 不行就降级 LSI**（★ 别再一枪打死 ✓）
 *
 *  ★ 为什么必须改 ✗（这次的现场教训 ✓）：
 *    · 旧版：只试 **一次**、等 3 秒 ⇒ 不起就 `return RTC_ERR_NO_LSE` ✓
 *    · ★★ 后果：RTC 计数器 = 0、`BKP_DR1` 魔数不写
 *      ⇒ `MyRTC_IsConfigured()` = 0 ⇒ **`T<unix>` 直接被拒**（`TIME BAD`）✓✓
 *      ⇒ 连"手动对时"这条退路都被堵死 ⇒ **时间永远 0** ✓✓✓
 *    · ★ 而"3.3V 稍微低一点/纹波大一点"就可能让 32768 晶振起不来
 *      （LSE 是微功耗小信号振荡器 ✗ 对电源最敏感 ✓）
 *
 *  ★★★ 新的三段策略 ✗：
 *    ① 试 3 次（每次先 `LSE_OFF` 再 `LSE_ON`，间隔 50ms）
 *       ★ 边缘晶振常常第二次就起振（第一次失败会留下残余电荷 ✓）
 *    ② 都不行 ⇒ **降级到 LSI**（内部 ~40kHz）
 *       ⇒ 至少时间能走、`T` 能对时、`CLK:` 能看出漂多少 ✓
 *    ③ ★ 降级时**大声说**：`!!! RTC on LSI: time WILL DRIFT !!!`
 *       （★ 绝不静默降级 ✗ 那等于假装时间是对的 ✓✓）
 *
 *  ★★★ LSI 的代价（★ 说清楚 ✗ 别粉饰 ✓）：
 *    · LSI 出厂容差就有 **±50%**（30~60kHz）✗ 温漂更大 ✓
 *    · 实测：一天可能差 **几分钟到几十分钟** ✓
 *    · ⇒ 只适合"能记就行"✗ 要当证据用必须修回 LSE ✓
 *============================================================================*/
static uint8_t MyRTC_CheckLSE(void)
{
	uint32_t t;
	uint8_t  attempt;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	PWR_BackupAccessCmd(ENABLE);                 /* ★ 备份域解锁 ✓ */

	/* ★ 三次尝试 −− 每次 1.5 秒 ⇒ 最坏 4.5 秒（★ 开机多等这点时间值得 ✓）*/
	for (attempt = 0u; attempt < 3u; attempt++)
	{
		RCC_LSEConfig(RCC_LSE_OFF);
		DWT_Delay_ms(50u);
		RCC_LSEConfig(RCC_LSE_ON);

		for (t = 0u; t < 1500u; t++)
		{
			if (RCC_GetFlagStatus(RCC_FLAG_LSERDY) != RESET) { return RTC_OK; }
			DWT_Delay_ms(1u);
		}
	}
	return RTC_ERR_NO_LSE;                       /* ★ 三次都不行 ⇒ 降级 ✓ */
}

/* ★★★★★ 2026-10-20 新增：**用 LSI 把 RTC 跑起来**（降级路径 ✓）
 *   ★ 关键 ✗：RTCSEL 只能在【备份域复位之后】改 ✓
 *     ⇒ 先 `RCC_BackupResetCmd(ENABLE/DISABLE)` ✓
 *     （★ 代价：RTC 计数器清 0、BKP 寄存器也清 ✗ 反正现在也是 0 ✓）*/
static uint8_t MyRTC_UseLSI(void)
{
	uint32_t t;
	RTC_DateTime_t ct;
	uint32_t ts;

	/* ① 开 LSI 并等它稳 ✓ */
	RCC_LSICmd(ENABLE);
	for (t = 0u; t < 1000u; t++)
	{
		if (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) != RESET) { break; }
		DWT_Delay_ms(1u);
	}
	if (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET) { return 0u; }

	/* ② 备份域复位 ⇒ 才能改 RTCSEL ✓ */
	RCC_BackupResetCmd(ENABLE);
	RCC_BackupResetCmd(DISABLE);

	/* ③ 选 LSI 作为 RTC 时钟源 ✓ */
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
	RCC_RTCCLKCmd(ENABLE);
	RTC_WaitForSynchro();

	/* ④ 分频：LSI 典型 40kHz ⇒ 40000/40000 = 1Hz ✓ */
	RTC_WaitForLastTask();
	RTC_EnterConfigMode();
	RTC_SetPrescaler(39999u);
	RTC_WaitForLastTask();
	RTC_ExitConfigMode();

	/* ⑤ 先塞个编译时刻（★ 至少不是 1970 ✓ 之后可以用 `T` 校 ✓）*/
	MyRTC_CompileTime(&ct);
	ts = MyRTC_YmdToUnix(ct.year, ct.month, ct.day, ct.hour, ct.minute, ct.second);
	RTC_SetCounter(ts);
	RTC_WaitForLastTask();

	/* ⑥ 打标记 ⇒ `IsConfigured()` 变 1 ⇒ **`T` 命令能用了** ✓✓ */
	BKP_WriteBackupRegister(RTC_BKP_DR, RTC_BKP_MAGIC);

	Serial_Printf("!!! RTC on LSI: time WILL DRIFT (minutes/day) !!!\r\n");
	Serial_Printf("!!! RTC: check LSE hardware (32.768k crystal / VBAT / 3.3V ripple) !!!\r\n");
	return 1u;
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
		/* ★★★ 2026-10-20：起不来【不再直接放弃】✗ 先切 LSI 让时间能走 ✓
		 *   ★★ 理由：`rtc=0` 是"完全不可用"（CSV 时间戳全废、`T` 还被拒）
		 *     而"LSI 会飘"是"能用但要校" ✓✓ 后者远好于前者 ✓ */
		if (MyRTC_UseLSI() != 0u) { return RTC_OK; }
		return RTC_ERR_NO_LSE;                   /* LSI 也不行 ⇒ 只能报错 ✓ */
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
		/* ★★ 注意 ✗：`DR1` 命中【只说明"配过"✗ 不代表"对过"】✓✓
		 *   · 可能是上次上电时用【编译时刻】配的 ✓
		 *   · 也可能是本次【软复位】✗ 计数器还连着 ✓
		 *   ⇒ 真正的判据看下面的 `MyRTC_IsTrusted()`（`BKP_DR5`）✓ */
	}

	/* ★★★★★ 2026-10-17 加：把"这个时间【可不可信】"明确报出来 ✗✓
	 *
	 *  ★★★★ 为什么必须在【驱动里】报 ✗：
	 *    · 调用者（main）可能忘了查 ✗
	 *    · 而"忘了查"的代价是【22 小时 35 分的错时间戳 + 界面毫无异常】✓✓
	 *      （★ 2026-10-17 实测，见 `长稳验收报告.md` §八 ✓）
	 *  ★★ 为什么不能靠"编译时刻自检"兜底 ✗：
	 *    那是【自洽式自检】✗ 永远 PASS ✓
	 *    （★ 见 `main.c` 里 `RTC now =` 之后那段
	 *      `NOTE: this self-check only proves the counter ROUND-TRIPS` ✓
	 *      ★★ 按【字符串】找 ✗ 别写行号 ✓✓）
	 *  ★★★ 判据 ✗：`BKP_DR5` 有魔数 = 【本次上电后被对过时】✓ */
	if (MyRTC_IsTrusted() == 0u)
	{
		Serial_Printf("!!! MyRTC: ** TIME NOT TRUSTED ** (BKP_DR5 has no magic) !!!\r\n");
		Serial_Printf("!!! this time may be WRONG by hours -> run time_sync on the PC !!!\r\n");
	}

	return RTC_OK;
}

/* ★ 安全读计数器：读两次直到一致 ✓
 *   RTC 在 32.768kHz 域、APB1 在 36MHz 域 ✗ 跨时钟域 ✗
 *   CNTL 与 CNTH 是两次独立读，若之间发生进位（如 0x0000FFFF → 0x00010000）
 *   会拼出 0x0001FFFF ✗ → 时间瞬跳 65535 秒（约 18.2 小时）✗✗
 *   读两次一致才采信 ✓
 *   ★★ 2026-10-19 修正 ✗：原注释写"两次读间隔【远大于】进位窗口"✗
 *     · 实测两次读的间隔只有**【约 1µs】✗ 而进位周期是【1 秒】**✓
 *       ⇒ 间隔【远小于】进位窗口 ✗ 【原话正好说反了】✓✓
 *     · ★★★ 那它为什么还安全 ✗：靠的是【循环重试】——
 *       真撞上进位那一瞬 ✗ 两次读到不同值 ⇒ `while (a != b)` 【再来一次】✓
 *       而下一秒的进位【在 1µs 尺度上是极小概率】⇒ 几乎必然一次通过 ✓✓
 *   ★ RTC 停走时两次恒等 → 立即返回 ✓ 无死循环风险 ✓ */
static uint32_t MyRTC_ReadCntSafe(void)
{
	uint32_t a, b;
	do { a = RTC_GetCounter(); b = RTC_GetCounter(); } while (a != b);
	return b;
}

uint32_t MyRTC_GetUnix(void)
{
	return MyRTC_ReadCntSafe();
}

void MyRTC_GetDateTime(RTC_DateTime_t *dt)
{
	if (dt != NULL) { MyRTC_UnixToYmd(MyRTC_ReadCntSafe(), dt); }
}

/* ★★ MyRTC_SetUnix —— 补回（2026-10-11 独立审查 F1 修）✗
 *   审查发现：MyRTC.h 里一直有这个声明 ✗ 但定义在
 *     "把 GetUnix 的 do-while 抽成 MyRTC_ReadCntSafe()" 那次重构里被一起替换掉了 ✓
 *   ★ 因为【当前无人调用】✗ → 编译链接都通过 ✓ 属于【潜伏型】✗
 *     → 将来做"串口/上位机对时"（50 号文档 2.2 写明的意图）时会直接
 *       L6218E: Undefined symbol MyRTC_SetUnix ✓
 *   ★ 教训：头文件与源文件接口必须一一对应 ✗
 *     （同类问题还有 MPU6050.h 的 Calibrate/GetOffset/ReadRaw ✓ 一并清了 ✓）*/
void MyRTC_SetUnix(uint32_t ts)
{
	RTC_SetCounter(ts);
	RTC_WaitForLastTask();          /* ★ 写完必须等 ✓ 否则 RTC 跨时钟域写入会静默丢失 ✗ */

	/* ★★★★★ 2026-10-17 加：标上"这个值是【人对过的】"✓
	 *   ★ 为什么放在【写完计数器之后】✗：先置标记再写 ✗
	 *     万一写失败（RTC 没配好 / 跨时钟域丢）⇒ 会留下【假的"已对时"】✓✓
	 *   ★★ 写 BKP 需要备份域解锁 ✗ —— 但 `MyRTC_Init` 里已经
	 *     `PWR_BackupAccessCmd(ENABLE)` 过（★ 见 `MyRTC_CheckLSE` ✓）
	 *     ⇒ 这里【不用再解一次】✓
	 *   ★★★ 唯一的调用点：`freertos_demo.c` 的 `T<unix>` 处理器
	 *     （★ 它【已经先查过 `MyRTC_IsConfigured()`】✓
	 *       ⇒ 走到这里时 RTC 一定是配好的 ✓✓）*/
	BKP_WriteBackupRegister(RTC_TRUST_DR, RTC_TRUST_MAGIC);
}

/* ★★★ MyRTC_IsConfigured —— RTC 是否【曾被正确初始化过】
 *   ★ 为什么需要它 ✗：★ 串口对时前要确认【RTC 真的在跑】✗✓
 *     · ★ 判据 = 【备份寄存器里的魔数在不在】✓
 *     · ★★ 不在 ✗ → 说明【MyRTC_Init 因为 LSE 问题提前返回了】✓
 *       → 【这时写计数器毫无意义 ✗ 该报警】✓✓
 *   ★★★ 这也解释了【为什么烧录不会重设时间】**✗：
 *     魔数【属于备份域 ✗ 只要有电就一直在】✓
 *     → 所以【Rebuild + 下载 ✗ 都不会触发重设】✓✓ */
uint8_t MyRTC_IsConfigured(void)
{
	return (BKP_ReadBackupRegister(RTC_BKP_DR) == RTC_BKP_MAGIC) ? 1u : 0u;
}

/* ★★★★★ 2026-10-17 新增：RTC 的值【可不可信】✓
 *   ★ 判据 = `BKP_DR5` 上的魔数 ✗（★ 只有 `MyRTC_SetUnix` 会置它 ✓）
 *   ★★ 和 `MyRTC_IsConfigured()` 的区别（★ 见 MyRTC.h 的为什么 ✓）✗
 *     · 配过 ≠ 对过 ✓ */
uint8_t MyRTC_IsTrusted(void)
{
	return (BKP_ReadBackupRegister(RTC_TRUST_DR) == RTC_TRUST_MAGIC) ? 1u : 0u;
}

