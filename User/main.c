/**
  ******************************************************************************
  * @file    main.c
  * @brief   主函数：只做硬件初始化 + 调用 FreeRTOS 例程入口
  *
  *  结构对齐正点原子：
  *    main.c          → 硬件初始化（GPIO / OLED ...）
  *    freertos_demo.c → 所有 FreeRTOS 相关代码
  *
  *  请注意：原来写在 main.c 里的任务函数、定时器中断、xTaskCreate 都要
  *          挪到 freertos_demo.c 里去（动态方式实现，见该文件头注释）
  ******************************************************************************
  */
#include "stm32f10x.h"                  /* Device header */
#include "OLED.h"
#include "freertos_demo.h"
#include "freertos.h"
#include "Key.h"
#include "MPU6050.h"
#include "task.h"
#include "Serial.h"
#include "AD.h"
#include "DWT_Delay.h"
#include "SD_SPI.h"
#include "DS18B20.h"
#include "DHT22.h"                 /* ★ DHT22 温湿度（PB15，模块自带 10k 上拉） */
#include "BMP280.h"
#include "MyRTC.h"  
#include "buzz.h"                  /* ★ 蜂鸣器服务（PB12，低电平触发） */
#include "HC_SR04.h"               /* ★ 2026-10-20：超声波（PB5=TRIG / PB6=ECHO） */
#include "W25Q64.h"                /* ★ W25Q64 8MB SPI Flash
                                    *   ★ 引脚（2026-10-17 从 W25Q64.c:31~39 核出的真值）
                                    *     SCK=PA8 / MISO=PA11 / MOSI=PA12 / CS=PB13
                                    *   ⚠️ 原来这里写着 PB5/6/7/10，那是错的
                                    *     （而 PB5/PB6 正是给 HC-SR04 预留的 TRIG/ECHO
                                    *       信了那条注释 ⇒ 会给超声波重选引脚 ⇒ 改错硬件）
                                    *   ⇒ 教训：改板子前回代码核引脚，别信注释。 */
#include "Crc16.h"                 /* ★ 2026-10-17 加 
                                    *   main.c 里用了 Crc16_Kermit / Crc16_Xmodem
                                    *   （那两个 CRC 契约自检）
                                    *   但一直没包含这个头，靠"隐式声明"侥幸通过
                                    *   ⇒ 编译器一直在报 warning #223-D，已修 */
#include "ProbeConfig.h"

/* ★ 2026-10-18 改：ver 3 → ver 4 
 *
 *  ★ 为什么必须升：
 *    · 10-18 又改了两批App 代码（WFI 睡眠 + OLED 超时黑屏）
 *    · 而版本串还写着 10-17 ⇒ 从串口根本分不清板子里是哪一版
 *
 *  ★ 为什么写功能名而不是只写编号：
 *    · 现场排障第一句话就是"你这版有没有黑屏功能"
 *    · 编号要靠记忆，名字不用
 *
 *  ★ 这条是人手维护的，改代码时必须一起改
 *    ★ 别指望用 __DATE__ 自动：
 *      · 它是该 .o 的编译时间，增量编译时不更新
 *      · 本项目已经在 RTC 初值上踩过这个坑（净偏 22h35m 那次）
 *    ⇒ 宁可人手改，也不要一个"看起来自动、其实会骗人"的版本号。 */
#define APP_VER_STR   "APP: ver 73 (2026-11-05 dsh: W25 lock wait probe)"
/*==============================================================================
 *  ★ 2026-10-22 main() 拆分（纯搬代码，不改任何逻辑）
 *
 *  ★ 为什么拆：
 *    · 原来 main() 是一根 600 多行的直线
 *    · 想找"RTC 是哪一步初始化的"得靠滚屏找注释
 *    · 而初始化是有先后依赖的（串口要最先
 *      因为后面每一步失败都要能打印出来）⇒ 这个顺序本身就是设计
 *      埋在 600 行里就等于没写
 *
 *  ★ 为什么不新建 .c 文件：
 *    · 本工程铁律：不动 .uvprojx（新文件进不了编译）
 *    · 而 main.c 里的 static 函数完全够用
 *    ⇒ 零构建风险 
 *
 *  ★ 拆分原则（按用户要求）：
 *    ① 一个函数 = 一个"子系统"（名字即文档）
 *    ② 函数名统一 App_InitXxx()，全部 static
 *    ③ 注释跟着代码一起搬，一个字都不删
 *    ④ 每个函数不返回值：失败就在里面打印 + 降级
 *       （符合本工程"能跑就继续跑、但要说出来"的风格）
 *
 *  ★ 调用顺序不能乱，原因都写在 main() 里
 *============================================================================*/

/* ① 控制台：NVIC 分组 + 串口 + 版本横幅 + CRC 契约自检
 *   ★ 必须是第一步，否则后面任何一步失败都打不出来。 */
static void App_InitConsole(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	Serial_Init();
	Serial_Printf("=== boot ===\r\n");

	/* ★ 2026-10-14 加：版本字符串（一眼看出跑的是哪一版）
	 *   ★ 为什么现在才加：以前 App 只剩 536 字节，加不起
	 *   ★ 现在有 112KB 了，而"分不清版本"是今天最大的折腾来源 
	 *   ★ 附带好处：改这一行 = 镜像 CRC 变了
	 *       → 正好用来测 R 回退（备份里那份是旧 CRC）
	 *   ★ 换版本时只改这里的数字和日期。 */
	Serial_Printf(APP_VER_STR "\r\n");

	/* ★ 2026-10-24：CRC 契约自检搬进 Crc16.c → Crc16_SelfTest()，变体对不对打出来才算数。 */
	Crc16_SelfTest();

}

/* ② 人机接口：AD / 屏 / 按键 / 蜂鸣器（光敏 2026-10-23 摘除，见 LightSensor.c）
 *   ★ 为什么排在存储之前：这几样是"看得见"的诊断能力
 *     万一后面 W25/SD 初始化卡住，至少有屏和串口能显示进度。 */
static void App_InitUi(void)
{
	AD_TestVdd();
	AD_Init();
	OLED_SelfCheck();
	OLED_Init();
	OLED_BusStress();     /* ★ 2026-11-04 临时：全屏写并数丢应答（查雪花） */

	Key_Init();
	/* ★ 2026-10-23：LightSensor_Init() 已删，PB13 = W25Q64 的 CS，详见 LightSensor.c。 */

	/* ★ 蜂鸣器（2026-10-13 加）
	 *   ★ 为什么以前没有：审查时发现Buzz_* 从未被任何 .c 调用
	 *     → 连 Buzz_Init 都没调，PB12 根本没配置成输出
	 *     → 蜂鸣器一点作用都没起
	 *   ★ 用途（见 buzz.h）：
	 *     · SD 故障 → 持续间歇响（无人值守时唯一的"叫人"手段）
	 *     · 按键反馈 → 短促一声 */
	Buzz_Init();
}

/* ★ 2026-10-24 从 App_InitRtc 提出：时间不可信的双通道告警
 *   ★ 串口打 10 行框（现场有 PC 时看）+ OLED 留 4 行（无人值守时唯一能看见的）
 *   ★ 屏上常驻 T? 由 task_DISP 接棒（见 freertos_demo.c） */
static void App_WarnRtcNotTrusted(void)
{
	Serial_Printf("\r\n");
	Serial_Printf("********************************************************************\r\n");
	Serial_Printf("*** RTC: TIME NOT TRUSTED !!!  (BKP_DR5 has no magic)\r\n");
	Serial_Printf("*** cause: no VBAT -> on every power-up the RTC is set from\r\n");
	Serial_Printf("***        MyRTC.c's compile time -- an OLD .o stamp\r\n");
	Serial_Printf("***        (measured 8h35m stale) -- and it is used as UTC\r\n");
	Serial_Printf("***        => real error measured: 22h35m\r\n");
	Serial_Printf("*** fix  : run time_sync.bat on the PC now (or send T<unix>)\r\n");
	Serial_Printf("***        data written before that has WRONG timestamps\r\n");
	Serial_Printf("********************************************************************\r\n");
	Serial_Printf("\r\n");

	/* ★ 屏上也留一条，直到对时为止
	 *   （之后 task_DISP 会用 T? 常驻提示，见 freertos_demo.c） */
	OLED_Clear();
	/* ★ 2026-10-17 修：每行补空格到 16 列
	 *   本工程的规矩是"状态字定长防残影"
	 *   ★ 我第一版忘了补 ⇒ 11~16 列会留上一屏的残影
	 *     （实测现象：第 1 行末尾冒出 ET!
	 *       第 2 行末尾冒出 ync —— 正是这两行的尾巴）
	 *   ★ 但只补空格还不够：
	 *     本任务正常画面的第 1 行只写 1~10 列
	 *     ⇒ 11~16 列仍是这里写下的内容
	 *     ⇒ 真正的修法是接管屏幕的那一帧整屏清一次
	 *       （见 freertos_demo.c 的 frame == 1u） */
	OLED_ShowString(1, 1, " RTC NOT SET !  ");   /* 14 + 2 = 16。 */
	OLED_ShowString(2, 1, "RUN time_sync   ");   /* 13 + 3 = 16。 */
	OLED_ShowString(3, 1, "TIME IS WRONG   ");   /* 13 + 3 = 16。 */
	OLED_ShowString(4, 1, " on your PC !  ");    /* 14 + 2 = 16。 */
}

/* ④ RTC：初始化 + 读回自检（纯搬代码，逻辑一字未改）
 *   ★ 自检原理（不依赖任何外部设备）：
 *     MyRTC_Init 用编译时刻设初值→ 立刻读回来再换算成年月日
 *     → 两行应当基本一致（差几秒 = 编译到烧录的耗时）
 *       差几小时 / 几年，= 换算算法错。 */
static void App_InitRtc(void)
{
	RTC_DateTime_t dt;

	if (MyRTC_Init() != RTC_OK)
	{
		Serial_Printf("!!! MyRTC_Init FAILED: LSE not ready !!!\r\n");
		/* LSE 没起振 ⇒ 计数器恒为 0 ⇒ 每条记录 rtc=0、文件名变成 700101。
		 * 现场没有串口，所以这里同时上屏，否则整段时间是假的也没人知道。
		 * （2026-11-03 加：E 盘那个 8.7 MB 的 70010101.CSV 就是这么来的）*/
		App_WarnRtcNotTrusted();
	}
	else
	{
		MyRTC_GetDateTime(&dt);
		Serial_Printf("RTC now = %04u-%02u-%02u %02u:%02u:%02u   (unix=%lu)\r\n",
		              (unsigned)dt.year, (unsigned)dt.month, (unsigned)dt.day,
		              (unsigned)dt.hour, (unsigned)dt.minute, (unsigned)dt.second,
		              (unsigned long)MyRTC_GetUnix());

		/* ★ 2026-10-17 改：这一行原来是骗人的自检
		 *   ★ 原来写的是
		 *     「^^ must match 'set from compile time' above (within seconds)」
		 *   ★ 为什么它永远通过（实测过一次 22 小时的错，它照样 PASS）
		 *     · 写进去用 MyRTC_YmdToUnix(__DATE__, __TIME__)
		 *     · 读回来用 MyRTC_UnixToYmd(RTC_GetCounter())
		 *     · 这两个是互为反函数，而且同一套无时区换算
		 *     ⇒ 那个 32 位数无论是什么意思，两边都相等 
		 *     ⇒ 它只证明了计数器能往返，证明不了时间对不对
		 *   ★ 这和 10-17 那个金副本 CRC bug 是同一类
		 *     交接文档 §10.2 的教训原文：
		 *     「自检和被检用同一套错逻辑 ⇒ 永远自洽 ⇒ 永远查不出来」
		 *   ★ 所以真正的校验必须在 PC 端用外部参照做
		 *     ⇒ time_sync.py 的 wait_ack() 会断言与 PC 差 ≤ 2 秒 */
		Serial_Printf("  ^^ NOTE: this self-check only proves the counter ROUND-TRIPS,\r\n");
		Serial_Printf("         NOT that the time is correct. The real check is on the PC:\r\n");
		Serial_Printf("         time_sync.py asserts |diff| <= 2 s against the PC clock.\r\n");

		/* ★ 2026-10-24：告警段提成 App_WarnRtcNotTrusted()，双通道喊话逻辑集中一处
		 *   ★ 判据仍是 BKP_DR5 魔数（只有串口对时成功才会置它） */
		if (MyRTC_IsTrusted() == 0u)
		{
			App_WarnRtcNotTrusted();
		}
	}
}

/* ④ 传感器：MPU6050（姿态）+ BMP280（气压）
 *   ★ 顺序不能换：MyI2C_Init() 是在 MPU6050_Init() 里调的
 *     而 BMP280 也要用同一条软件 I2C（PA2/PA3） */
static void App_InitSensors(void)
{
	int16_t oax, oay, oaz, ogx, ogy, ogz;   /* ★ 校准偏移量（只在函数内用） */

	/* ---------- 第二批：传感器 ---------- */
	MPU6050_Init();
	Serial_Printf("MPU ID = %02X (expect 68)\r\n", MPU6050_GetID());
	/* ★ 硬件探针：I2C 全地址扫描 + 认型号—— 只在排查期跑
	 *   ★ 关掉它不影响BMP280 的正式流程：下面那段 Init/SelfTest/Read
	 *      是无条件跑的 
	 *   ⚠️ 这里踩过一次坑（2026-10-10 自查抓到）：一开始把 #endif 放到了整段末尾
	 *      → 关掉开关会把正式流程也一起关掉
	 *      → 后果是静默故障：BMP280_Init 没跑 → 校准系数没读 →
	 *        气压全错，但一切"看起来正常"（正是工程里最怕的那类） */
	#if PROBE_BMP280_SCAN
	BMP280_Probe();                        /* ★ 顺手确认 I2C 总线 + 认 BMP280 型号
	                                        *   （MyI2C_Init 已在 MPU6050_Init 里调过） */
	#endif

	/* ★ 2026-10-24：开机三件套（Init/SelfTest/Read）搬进 BMP280.c → BMP280_BootTest()。 */
	BMP280_BootTest();


	/* ⚠️ 2026-10-14 起这一行失效了，不是 bug
	 *   · 光敏的数字量 DO接在 PB13
	 *   · 而 PB13 被 W25Q64 的 CS 占用了（用户确认"用不上"）
	 *     → 所以 LightSensor_Get() 现在读到的是CS 线的电平
	 *       不是光照
	 *   · 记录里的 light 字段来自 PA1 的 ADC，完全不受影响
	 *   · 之所以不删 LightSensor.c
	 *     万一以后换引脚还能用，先只把这一行停掉。 */
	/* Serial_Printf("LIGHT DO = %u\r\n", LightSensor_Get()); */
	Serial_Printf("gyro calibrating, keep still for 2s ...\r\n");
	DWT_Delay_ms(2000);
	MPU6050_CalibrateGyro();

	MPU6050_GetAccOffset(&oax, &oay, &oaz);
	MPU6050_GetGyroOffset(&ogx, &ogy, &ogz);
	Serial_Printf("MPU OFF acc=(%d,%d,%d) fixed, gyro=(%d,%d,%d) boot-cal\r\n",
	              oax, oay, oaz, ogx, ogy, ogz);
	/* ★ 2026-10-24：残差核查搬进 MPU6050.c → MPU6050_GyroResidual()。 */
	MPU6050_GyroResidual();
	Serial_Printf("AD init: light=%u vbat=%u (raw 0-4095)\r\n", AD_Value[0], AD_Value[1]);
	
		

}

/* ⑤ 1-wire 传感器 + 超声波（它们都是"先配引脚再读"，放一起看得清）
 *   · DHT22（PB15，开漏 + 模块自带 10k 上拉）
 *   · HC-SR04（PB5 触发 / PB6 回波）
 *   · 末尾的"线路体检"会临时改 PB14/PB15 的模式
 *     ⇒ 所以它必须在 DHT22 初始化之后、并且自己把引脚还回去 */
static void App_InitWire(void)
{
	/* ★ 2026-10-24：开机自检读搬进 DHT22.c → DHT22_BootTest()（任务侧仍每 2 秒读） */
	DHT22_BootTest();


	/* ★ 2026-10-20：超声波（PB5=TRIG / PB6=ECHO）
	 *   ★ 放在这里：和其它传感器初始化挨着 ⇒ 一眼看出"板上有哪些传感器"
	 *   ★ 它只配两个引脚 + TIM4，不采样、不发脉冲（采样在 task_sensor 里） */
	HC_SR04_Init();

	/* ★ 2026-10-24：1-wire 线路体检搬进 DS18B20.c → DS18B20_WireCheck()
	 *   （PB14/PB15 浮空+上拉双采样 + 存在脉冲测量，体检完把引脚还给驱动） */
	DS18B20_WireCheck();

	/* ★ 2026-11-04 修：体检会把 PB15 改成输入，必须还回 DHT22 手里
	 *   · Wire_SampleHigh() 把 PB15 配成 IN_FLOATING / IPU
	 *   · 而 DS18B20_WireCheck 收尾只调 DS18B20_Init()，那只管 PB14
	 *   ⇒ 体检之后 PB15 一直是输入 ⇒ DHT22_Read 的启动脉冲发不出去
	 *   ⇒ 运行时湿度永远 65535，屏上 H 那一格一直显示 --.-%
	 *   （开机测试读在体检之前，所以"开机那次是好的"，最容易误导人） */
	DHT22_Init();

	/* ★ 2026-10-24：单次读自检搬进 DS18B20.c → DS18B20_BootTest()
	 *   开关仍是 ProbeConfig.h 的 TEST_DS18B20_ON，与驱动内探针同一开关。 */
#if (TEST_DS18B20_ON)
	DS18B20_BootTest();
#endif

	
}

/* ⑥ SD 卡：初始化 + MBR 校验 + 硬件探针（探针开关在 ProbeConfig.h）
 *   ★ 为什么整段包在 #if 里：探针会反复折腾卡（发 CMD0 / 翻时钟）
 *     而它跑在 SD_Init() 之前⇒ 会把卡留在失步状态
 *     ⇒ 正式跑记录时探针全关
 *   ★ 为什么条件写 TEST_SD_ON：
 *     原来 main() 里有一句 #define TEST_SD TEST_SD_ON
 *     ★ 宏是文本替换 ⇒ 代码搬到函数里之后那个 define 就不在作用域了
 *     ⇒ #if (TEST_SD) 会被当成 0 ⇒ 整段静默失效
 *     （这种"搬家把宏搬丢了"的坑，编译器一声不响） */
static void App_InitSd(void)
{
	/* ★ 2026-10-24：SD 上电测试整段搬进 SD_SPI.c → SD_BootTest()
	 *   含 MBR/分区表解析 + 分区前空档写读回验证 + 假卡检测，开关仍是 TEST_SD_ON。 */
#if (TEST_SD_ON)
	SD_BootTest();
#endif
}

/* ③ W25Q64（8MB SPI Flash）：接线验证（它马上要承担金副本/回滚）
 *   ★ 判据三条：JEDEC=0xEF4017 + selftest=0（擦写读比对）
 *   ★ 只读 ID 就宣布"接线没问题"是自欺欺人，写方向可能坏 */
static void App_InitW25(void)
{
	/* ★ 2026-10-24：接线验证搬进 W25Q64.c → W25Q64_BootTest()
	 *   判据三条：JEDEC=0xEF4017 + selftest=0（擦写读比对，写方向也通才算接线 OK） */
	W25Q64_BootTest();

	}

int main(void)
{
/*==============================================================================
 *  ★ 开机初始化：七步，顺序不能乱（每一步的原因见对应函数）
 *
 *   ① App_InitConsole()  串口 + 版本 + CRC 契约自检
 *        ★ 必须第一步，后面任何一步失败都要能打出来
 *   ② App_InitUi()       AD / 屏 / 按键 / 蜂鸣器
 *        ★ 排在存储前，它是"看得见"的诊断能力
 *   ③ App_InitW25()      W25Q64 接线验证（金副本/回滚的地基）
 *   ④ App_InitRtc()      RTC（LSE 三次重试 -> 备份域复位 -> 重新启动 LSE -> 实在不行才降 LSI）
 *        ★ 2026-11-04 修：备份域复位会把 BDCR 清掉（LSEON 归零），
 *          所以复位之后必须重新把 LSE 起起来，否则会一直掉在 LSI
 *   ⑤ App_InitSensors()  MPU6050 + BMP280（共用 PA2/PA3 软件 I2C）
 *   ⑥ App_InitWire()     DHT22 + 超声波 + 1-wire 线路体检
 *   ⑦ App_InitSd()       SD 初始化 + MBR 校验（硬件探针默认关）
 *
 *  ★ 为什么这个顺序是设计而不是随手排的：
 *    · ②~⑦ 每一步失败都只打印 + 降级，不 return
 *      ⇒ 一台设备缺个传感器也得照常记录（这是记录仪的本分）
 *    · 而 ① 的串口如果不先起来，上面那句就没有地方打印
 *
 *  ★ 最后才是看门狗 + 调度器：
 *    · IWDG 一旦使能不能停，而初始化可能耗好几秒（SD 的 ACMD41 重试）
 *      ⇒ 早开 = 初始化阶段自己复位（见下面那段注释）
 *============================================================================*/
	App_InitConsole();          /* ① 串口 + 版本 + CRC 自检。 */
	App_InitUi();               /* ② AD / 屏 / 按键 / 蜂鸣器。 */

	App_InitW25();              /* ③ W25Q64 接线验证（金副本/回滚的地基） */
	App_InitRtc();              /* ④ RTC + 自检 + 时间可信度告警。 */

	App_InitSensors();          /* ⑤ MPU6050 + BMP280（共用软件 I2C） */
	App_InitWire();             /* ⑥ DHT22 + 超声波 + 线路体检。 */
	App_InitSd();               /* ⑦ SD 初始化 + MBR 校验（探针默认关） */


	Serial_Printf("=== Init Done ===\r\n");

	/* ★ 启动 IWDG 独立看门狗（2026-10-12 加，阶段 3 最后一块）
	 *
	 *  ★ 为什么必须放在这里：
	 *    · IWDG 一旦 Enable() 就不能停
	 *    · 而上面那些初始化可能耗好几秒
	 *      （实测 SD 的 ACMD41 重试过 142 次，那就是好几秒）
	 *    · → 如果更早启动→ 初始化阶段就会自己复位
	 *    · 所以放在所有耗时初始化之后、调度器启动之前
	 *
	 *  ★ 喂狗点是 vApplicationIdleHook()（见 freertos_demo.c）
	 *    超时约 6.55 秒（LSI 公差 → 4.4~8.7 秒） */
	WD_Init();

	/* ★ 关于"怎么验证 IWDG 真的有效"（2026-10-12 结论）
	 *
	 *  ★ 我一开始想加一个"故意死循环不喂狗"的自测段—— 那是错的
	 *    原因（实测踩到）：
	 *      ① while(1){} 之后的一切都不可达
	 *         → 编译器把 freertos_demo() 和所有任务都优化掉了
	 *         → Code 从 45,272 暴跌到 22,672，并报 statement is unreachable
	 *      ② 更要命的是测错了对象：
	 *         · 那种测法只证明IWDG 硬件会咬人
	 *           —— 而这是ST 的硅片保证，根本不用测
	 *         · 它没证明"喂狗钩子接对了"
	 *           （钩子被优化掉了，压根没参与）
	 *
	 *  ★ 正确的验证 = 直接烧正式版，然后看：
	 *    · 跑起来且不反复复位→ 喂狗路径是通的 
	 *    · 每 6.55 秒复位一次→ 钩子没接上 / 配置错了 
	 *  → 零额外代码，而且测的正是有风险的地方 */

	freertos_demo();
    while (1)
    {
		
    }
}
