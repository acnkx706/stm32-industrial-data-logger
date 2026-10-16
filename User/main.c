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
  *          挪到 freertos_demo.c 里去（静态方式实现）
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
#include "LightSensor.h"
#include "AD.h"
#include "DWT_Delay.h"
#include "SD_SPI.h"
#include "DS18B20.h"
#include "DHT22.h"                 /* ★ DHT22 温湿度（PB15 ✓ 模块自带 10k 上拉 ✓）*/
#include "MyI2C.h"                 /* ★ BMP280 与 MPU6050 共用这条软件 I2C（PA2=SCL / PA3=SDA）*/
#include "BMP280.h"
#include "MyRTC.h"  
#include "buzz.h"                  /* ★ 蜂鸣器服务（PB12 ✓ 低电平触发 ✓）*/
#include "W25Q64.h"                /* ★★ W25Q64 8MB SPI Flash（PB5/6/7/10 ✓ 2026-10-14 加 ✓）*/
#include "ProbeConfig.h"

/*==============================================================================
 *  ★ BMP280 探针（2026-10-09）—— 只做"扫地址 + 读 ID"，先认型号再写驱动
 *
 *  为什么先做这个（`30` 号 §15.2 第 462 行的既定办法）：
 *    BMP280 与 BMP580 是【两套完全不同的驱动】✗ —— 寄存器、测量流程、校准系数都不同 ✓
 *    → 型号认错 = 白写一版驱动 ✓ → 所以【先认型号，不靠丝印猜】✓✓
 *
 *  怎么认：
 *    ① I2C 扫描：谁应答（ACK=0）就是总线上的器件地址 ✓
 *       0x76 / 0x77 → BMP280 家族 ✓   0x46 / 0x47 → BMP58x 家族 ✓
 *    ② 读 ID 寄存器确认：BMP280 的 0xD0 应回 0x58 ✓（BMP58x 的 ID 在 0x01 ✓）
 *
 *  ★ 判据纪律（踩过的坑）：★ "器件在不在"看的是【ACK 位】✗
 *    不能看"读回来的值不是 0xFF"✓ —— 有些寄存器本来就可能是 0xFF ✗ 会误判 ✓
 *
 *  ★ 附带作用：全地址扫描会顺带扫出 MPU6050（0x68）✓
 *     → 若 0x68 应答而 0x76/0x77 不应答 = 【总线是好的，只是 BMP280 没接上】✓✓
 * ============================================================================*/
 #if PROBE_BMP280_SCAN
static uint8_t I2C_Probe(uint8_t addr)              /* 返回 1 = 有器件应答 ✓ */
{
	uint8_t ack;
	MyI2C_Start();
	MyI2C_SendByte((uint8_t)(addr << 1));           /* 写地址帧 ✓ */
	ack = MyI2C_ReceiveAck();                       /* ★ 0 = 应答 / 1 = 无 ✓ */
	MyI2C_Stop();
	return (ack == 0) ? 1 : 0;
}

static uint8_t I2C_ReadReg(uint8_t addr, uint8_t reg)
{
	uint8_t d;
	MyI2C_Start();
	MyI2C_SendByte((uint8_t)(addr << 1));
	MyI2C_ReceiveAck();
	MyI2C_SendByte(reg);
	MyI2C_ReceiveAck();
	MyI2C_Start();                                  /* ★ 重复起始 ✓（MPU6050 已验证的写法 ✓）*/
	MyI2C_SendByte((uint8_t)((addr << 1) | 0x01));
	MyI2C_ReceiveAck();
	d = MyI2C_ReceiveByte();
	MyI2C_SendAck(1);                               /* ★ 最后一个字节发 NACK ✓ */
	MyI2C_Stop();
	return d;
}

static void BMP280_Probe(void)
{
	uint8_t a, id, found = 0;

	Serial_Printf("---- BMP280 probe (I2C: PA2=SCL PA3=SDA) ----\r\n");

	for (a = 0x08u; a <= 0x77u; a++)                /* ★ 全地址扫（不只 0x76/0x77）✓ */
	{
		if (I2C_Probe(a))
		{
			found++;
			id = I2C_ReadReg(a, 0xD0u);             /* 读 ID 寄存器 ✓ */
			Serial_Printf("  ACK at 0x%02X : reg[0xD0]=0x%02X%s\r\n", a, id,
			              (id == 0x58u) ? "  <== BMP280 (ID=0x58) -- FOUND" : "");
		}
	}

	if (found == 0u)
	{
		Serial_Printf("  NO device on I2C bus!\r\n");
		Serial_Printf("  check: VIN=3.3V / GND / SCL=PA2 / SDA=PA3, MPU6050 also missing?\r\n");
	}
	else
	{
		Serial_Printf("  %u device(s) on bus. BMP280 expected at 0x76/0x77 with ID 0x58\r\n",
		              (unsigned)found);
	}
}
#endif
#define APP_VER_STR   "APP: ver 2  (2026-10-14 CBT6 + new partition)"
int main(void)
{int16_t oax, oay, oaz, ogx, ogy, ogz;
/* ---------- 第一批：先把"能显示/能通信"的立起来（诊断能力优先） ---------- */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	Serial_Init();
	Serial_Printf("=== boot ===\r\n");
			/* ★★★★ 2026-10-14 加：版本字符串（★ 一眼看出跑的是哪一版 ✓）
	 *   ★ 为什么现在才加 ✗：以前 App 只剩 536 字节 ✗ 加不起 ✓
	 *   ★★ 现在有 112KB 了 ✗ 而"分不清版本"是今天最大的折腾来源 ✓✓
	 *   ★★★ 附带好处：改这一行 = 【镜像 CRC 变了】✗
	 *       → 正好用来测 `R` 回退（★ 备份里那份是旧 CRC ✓）✓✓
	 *   ★ 换版本时只改这里的数字和日期 ✓ */
	
	Serial_Printf(APP_VER_STR "\r\n");
		/* ★★★ 2026-10-14 加：两个 CRC 的契约自检（★ 上电打一行 ✓）
	 *   ★ 为什么值得 ✗：CRC 变体看代码看不出来 ✗ 打出来才算数 ✓✓ */
	{
		static const uint8_t anchor[] = "123456789";
		Serial_Printf("CRC: KERMIT=0x%04X(want 2189)  XMODEM=0x%04X(want 31C3)\r\n",
		              (unsigned)Crc16_Kermit (anchor, 9u),
		              (unsigned)Crc16_Xmodem(anchor, 9u));
	}
	AD_TestVdd();
	AD_Init();
	OLED_SelfCheck(); 
	OLED_Init();
	
	Key_Init();
	LightSensor_Init();
	/* ★★★★ 蜂鸣器（2026-10-13 加 ✓）
	 *   ★ 为什么以前没有 ✗：★ 审查时发现【Buzz_* 从未被任何 .c 调用】✗✓
	 *     → 【连 Buzz_Init 都没调 ✗ PB12 根本没配置成输出】✓
	 *     → ★★ 【蜂鸣器一点作用都没起】✓✓
	 *   ★★★ 用途（★ 见 buzz.h ✓）：
	 *     · SD 故障 → 持续间歇响（★ 无人值守时唯一的"叫人"手段 ✓）
	 *     · 按键反馈 → 短促一声 ✓✓ */
	Buzz_Init();
	
	/*==========================================================================
	 *  ★★★★ W25Q64（8MB SPI Flash）接线验证（★ 2026-10-14 加 ✓）
	 *
	 *  ★★ 为什么【开机就要探一次**✗✓：
	 *    · ★ 它马上要承担【金副本 / 自动回滚】✗ 【是"救命"的那块存储】✓
	 *    · ★★ 【"接线到底通不通"必须【开机一眼可见】】✗✓
	 *      → 【不能等到【真要救砖的那一刻】才发现它没接上】✓✓
	 *
	 *  ★★★ 判据（★ 三条 ✗ 缺一不可 ✓）✗：
	 *    · ★ `JEDEC = 0xEF4017` ✗ 【这只是第一条】✓
	 *    · ★★ `selftest = 0` ✗ 【擦 + 写 + 读回比对】✓
	 *      → 【这条才证明【写的方向（MOSI）】也通】✓✓
	 *    ⚠️ ★★★★ 【只读 ID 就宣布"接线没问题"✗ 【是自欺欺人】】✓✓
	 *      （★ 现象 ✗：ID 读得出来 ✗ 但一写就坏 ✗ 【能查一天】✓）
	 *
	 *  ★★★ 引脚 ✗：★ **SCK=PA8 ✗ MISO=PA11 ✗ MOSI=PA12 ✗ CS=PB13** ✓
	 *    （★ 为什么不用 SPI1 / 为什么避开 PA15 ✗ 见 `W25Q64.h` 顶部 ✓）
	 *    ⚠️ ★★ 【CS 占了 PB13**✗ 那是原来光敏 DO 的脚 ✓
	 *      → 【本文件下面那行 `LIGHT DO` 打印【已经停掉】✓
	 *        （★ 记录里的 `light` 字段【来自 PA1 ✗ 不受影响】✓）✓
	 *========================================================================*/
	{
		uint8_t  w25_ok  = W25Q64_Init();
		uint32_t w25_jid = W25Q64_ReadJEDEC();

		Serial_Printf("W25Q64: JEDEC = 0x%06X (expect 0xEF4017) -> %s\r\n",
		              (unsigned)w25_jid, (w25_ok != 0u) ? "PRESENT" : "NOT FOUND");
		Serial_Printf("W25Q64: pins SCK=PA8  MISO=PA11  MOSI=PA12  CS=PB13\r\n");

#if W25Q64_SELFTEST
		if (w25_ok != 0u)
		{
			uint8_t st = W25Q64_SelfTest();
			Serial_Printf("W25Q64: selftest = %u (0=PASS 1=erase 2=write 3=read 4=mismatch)\r\n",
			              (unsigned)st);
		}
#endif
	}

		/* ★ RTC（阶段 3）—— 初始化 + 读回自检 ✓
	 *   ★ 自检原理（不依赖任何外部设备 ✓）：
	 *     MyRTC_Init 用【编译时刻】设初值 ✓ → 立刻读回来再换算成年月日 ✓
	 *     → 两行应当基本一致（差几秒 = 编译到烧录的耗时 ✓）
	 *       差几小时 / 几年 ✗ = 换算算法错 ✓ */
	{
		RTC_DateTime_t dt;

		if (MyRTC_Init() != RTC_OK)
		{
			Serial_Printf("!!! MyRTC_Init FAILED: LSE not ready !!!\r\n");
		}
		else
		{
			MyRTC_GetDateTime(&dt);
			Serial_Printf("RTC now = %04u-%02u-%02u %02u:%02u:%02u   (unix=%lu)\r\n",
			              (unsigned)dt.year, (unsigned)dt.month, (unsigned)dt.day,
			              (unsigned)dt.hour, (unsigned)dt.minute, (unsigned)dt.second,
			              (unsigned long)MyRTC_GetUnix());
			Serial_Printf("  ^^ must match 'set from compile time' above (within seconds)\r\n");
		}
	}

	/* ---------- 第二批：传感器 ---------- */
	MPU6050_Init();
	Serial_Printf("MPU ID = %02X (expect 68)\r\n", MPU6050_GetID());
	/* ★ 硬件探针：I2C 全地址扫描 + 认型号 ✗ —— 只在排查期跑 ✓
	 *   ★★ 关掉它【不影响】BMP280 的正式流程 ✗：下面那段 Init/SelfTest/Read
	 *      是【无条件】跑的 ✓✓
	 *   ⚠️ 这里踩过一次坑（2026-10-10 自查抓到）：一开始把 #endif 放到了整段末尾 ✗
	 *      → 关掉开关会把【正式流程也一起关掉】✗✓
	 *      → 后果是【静默故障】✗：BMP280_Init 没跑 → 校准系数没读 →
	 *        气压全错，但一切"看起来正常" ✓（★ 正是工程里最怕的那类 ✗） */
	#if PROBE_BMP280_SCAN
	BMP280_Probe();                        /* ★ 顺手确认 I2C 总线 + 认 BMP280 型号 ✓
	                                        *   （MyI2C_Init 已在 MPU6050_Init 里调过 ✓）*/
	#endif

	/* ================= ★ BMP280 正式驱动测试（2026-10-09）=================
	 * 顺序：Init（认型号+读校准）→ SelfTest（手册算例，验公式）→ Read（真读数）
	 * ★ 两个锚点：
	 *   ① SelfTest：期望 T=2508（25.08°C）/ P=100653 Pa
	 *      ⚠️ 算例常量是凭记忆写的手册值 ✗ —— 不过就先怀疑它 ✗
	 *   ② ★ 真读数：温度应 ≈ 上面 DS18B20 的读数（±1°C 内 ✓）
	 * ==================================================================== */
	{
		int32_t p_x10, t_c100;
		uint8_t r;

		r = BMP280_Init();
		Serial_Printf("BMP280 init = %u (0=OK / 1=ID wrong / 2=no bus)\r\n", (unsigned)r);

		if (r == 0u)
		{
			(void)BMP280_SelfTest();

			if (BMP280_Read(&p_x10, &t_c100) == 0u)
			{
				Serial_Printf("BMP280 raw : press=%ld (x10Pa)  temp=%ld (x100C)\r\n",
				              (long)p_x10, (long)t_c100);
				Serial_Printf("BMP280 real: press=%ld.%01lu hPa   temp=%ld.%02lu C\r\n",
				              (long)(p_x10 / 1000), (unsigned long)((p_x10 % 1000) / 100),
				              (long)(t_c100 / 100), (unsigned long)(t_c100 % 100));
				Serial_Printf("  ^^ compare temp with DS18B20 above (expect within ~1C)\r\n");
			}
			else
			{
				Serial_Printf("BMP280 read FAILED\r\n");
			}
		}
	}

	/* ⚠️ ★★★★ 【2026-10-14 起【这一行【失效】了】✗ 不是 bug】✗✓
	 *   · ★ 光敏的【数字量 DO】接在 **PB13** ✗
	 *   · ★★ 而 PB13 【被 W25Q64 的 CS 占用了】（★ 用户确认"用不上"✓）
	 *     → 【所以 `LightSensor_Get()` 现在读到的是【CS 线的电平**
	 *       【不是光照】】✓✓
	 *   · ★★★ 【记录里的 `light` 字段【来自 PA1 的 ADC ✗ 完全不受影响】】✓
	 *   · ★★★★ 【之所以【不删 `LightSensor.c**✗
	 *     【万一以后换引脚还能用 ✗ 先只把这一行停掉】✓ */
	/* Serial_Printf("LIGHT DO = %u\r\n", LightSensor_Get()); */
	Serial_Printf("gyro calibrating, keep still for 2s ...\r\n");
	DWT_Delay_ms(2000);
	MPU6050_CalibrateGyro();

	MPU6050_GetAccOffset(&oax, &oay, &oaz);
	MPU6050_GetGyroOffset(&ogx, &ogy, &ogz);
	Serial_Printf("MPU OFF acc=(%d,%d,%d) fixed, gyro=(%d,%d,%d) boot-cal\r\n",
	              oax, oay, oaz, ogx, ogy, ogz);
	{
		int32_t sgx = 0, sgy = 0, sgz = 0;
		int16_t ax, ay, az, gx, gy, gz, tchip;
		uint8_t i;
		for (i = 0; i < 20; i++)              /* 20 × 5.24ms ≈ 105ms */
		{
			MPU6050_GetData(&ax, &ay, &az, &gx, &gy, &gz, &tchip);
			sgx += gx; sgy += gy; sgz += gz;
		}
		Serial_Printf("GYRO residual=(%d,%d,%d)  (abs>5 = calibration NOT trustworthy)\r\n",
		              (int)(sgx / 20), (int)(sgy / 20), (int)(sgz / 20));
	}
	Serial_Printf("AD init: light=%u vbat=%u (raw 0-4095)\r\n", AD_Value[0], AD_Value[1]);
	
		

	/* ============ 调试期测试开关（★ 2026-10-11 更新：全部集中到 User\ProbeConfig.h ✓）============
	 *   ★★ 这里只做【别名】✗ —— 真正的开关值请去 ProbeConfig.h 改 ✓
	 *     TEST_DS18B20_ON / TEST_SD_ON 的定义都在那里 ✓
	 *
	 *   ★ 硬件排查探针（回环 / PinFinder / MISO / 位翻转 / 详细初始化 / 假卡）
	 *     同样在 ProbeConfig.h 里用 PROBE_SD_xxx 控制 ✓
	 *     ★★ 关掉时那些函数【根本不编译】✗ → Flash 占用为零 ✓✓
	 *        （比原来"贴回 SD_SPI.c 末尾"的手工作业好 ✓ 现在改一个宏即可 ✓）
	 *
	 *   ★ 探针跑在 SD_Init() 之前 ✗ 它自己会反复折腾卡（发 CMD0 / 翻时钟 ✓）
	 *     → 会把卡留在【失步状态】✗ → 紧接着的 SD_Init 连 CMD0 都答不上 ✗
	 *       （实测踩过：开着探针那一轮 = NO_CARD ✓）
	 *     ★ 所以：正式跑记录时【探针全关】✗；任何探针跑完后要再测 SD，
	 *       都必须【先给 SD 模块断电重上电】✓
	 * ======================================================================================= */
	#define TEST_DS18B20   TEST_DS18B20_ON      /* ★ 开关值见 ProbeConfig.h ✓ */
	#define TEST_SD        TEST_SD_ON           /* ★ 开关值见 ProbeConfig.h ✓ */

	/* ★★ DHT22 初始化 + 开机自检读取（2026-10-11 加 ✓）
	 *   · DHT22_Init 只调一次 ✗（配 PB15 为开漏 + 释放总线 ✓）
	 *   · ★ 这里读一次是为了【开机就知道它在不在】✗ ——
	 *     存在性靠"能不能拿到合法校验和"判断 ✓
	 *     （★ DHT22 没有 DS18B20 那种独立的存在脉冲 ✗ 只能靠整帧成功与否 ✓）
	 *   · ★★ 任务侧仍在 task_sensor 里每 2 秒读一次 ✓（这里是额外的开机自检 ✓）
	 *     两次读之间的间隔【远超 2 秒】✓ —— 后面还有 MPU 校准(2s) + SD 初始化 ✓✓ */
	{
		uint16_t h_x10 = 0xFFFF;
		int16_t  t_x10 = 0;

		DHT22_Init();
		if (DHT22_Read(&h_x10, &t_x10))
		{
			Serial_Printf("DHT22: humi=%u.%u %%RH  temp=%d.%d C\r\n",
			              (unsigned)(h_x10 / 10u), (unsigned)(h_x10 % 10u),
			              (int)(t_x10 / 10), (int)((t_x10 % 10 + 10) % 10));
			Serial_Printf("DHT22: (compare with DS18B20 below; DHT22 self-heats a bit)\r\n");
		}
		else
		{
			Serial_Printf("DHT22: READ FAILED (check PB15 wiring / module pull-up)\r\n");
		}
	}

#if (TEST_DS18B20)
	Serial_Printf("---- DS18B20 test (PB14) ----\r\n");
	Serial_Printf("presence = %u (1=found)\r\n", DS18B20_Init());
	Serial_Printf("presence width = %u us (0=no resp, 65535=stuck low, real chip 60~240)\r\n",
	              (unsigned)DS18B20_MeasurePresence());
	DS18B20_Start();
	DWT_Delay_ms(750);                     /* 12 位转换规格 <=750ms */
	{
		int16_t raw, t;
		if (DS18B20_Read(&raw))
		{
			t = DS18B20_ToX100(raw);
			Serial_Printf("raw=0x%04X  temp=%d.%02d C\r\n",
			              (unsigned)raw, t / 100, (t % 100 + 100) % 100);
		}
		else
		{
			uint8_t sp[9];
			uint8_t k;

			Serial_Printf("read FAILED (presence=%u)\r\n", DS18B20_Init());
			if (DS18B20_ReadScratchpad(sp))
			{
				Serial_Printf("sp9:");
				for (k = 0; k < 9; k++) { Serial_Printf(" %02X", sp[k]); }
				Serial_Printf("   (sp[0..1]=temp lo/hi, sp[8]=chip CRC)\r\n");
			}
			else
			{
				Serial_Printf("scratchpad read: no presence\r\n");
			}

			{
				uint8_t rom[8], k;
				if (DS18B20_ReadRom(rom))
				{
					Serial_Printf("rom8:");
					for (k = 0; k < 8; k++) { Serial_Printf(" %02X", rom[k]); }
					Serial_Printf("   (real DS18B20: rom[0]=28, rom[7]=CRC)\r\n");
				}
				else
				{
					Serial_Printf("rom read: no presence\r\n");
				}
			}

			{
				uint32_t ovh, lo6, ms1, sl70, rdrel, rdsmp;
				DS18B20_TimingCheck(&ovh, &lo6, &ms1, &sl70, &rdrel, &rdsmp);
				Serial_Printf("timing(cycles @72MHz): pulloverhead=%lu  one_ms=%lu(exp 72000)\r\n",
				              (unsigned long)ovh, (unsigned long)ms1);
				Serial_Printf("  read slot after falling edge: release=%lu  sample=%lu (must be <1080)\r\n",
				              (unsigned long)rdrel, (unsigned long)rdsmp);
			}

			{
				uint8_t m8[8], pres, j;
				DS18B20_BitProbe(m8, &pres);
				Serial_Printf("bit7: present=%u  (sampled ~4/6/8/10/12/14/16us after falling edge)\r\n", pres);
				for (j = 0; j < 8; j++)
				{
					Serial_Printf("  bit%u: %u%u%u%u%u%u%u\r\n", (unsigned)j,
					    (unsigned)((m8[j] >> 0) & 1), (unsigned)((m8[j] >> 1) & 1),
					    (unsigned)((m8[j] >> 2) & 1), (unsigned)((m8[j] >> 3) & 1),
					    (unsigned)((m8[j] >> 4) & 1), (unsigned)((m8[j] >> 5) & 1),
					    (unsigned)((m8[j] >> 6) & 1));
				}
			}
		}
	}
#endif

#if (TEST_SD)
	{
		static uint8_t s_sd_buf[512];      /* ★ 512B 必须 static：main 的栈装不下 */
		SD_Result_t rr;
		uint8_t i;

		/* ★ 硬件探针（实现在 SD_SPI.c 末尾 ✓ 跑完自己恢复硬件 SPI ✓ 所以不影响下面 ✓）*/
		extern void SD_BitBangProbe(void);
		extern void SD_LoopbackTest(void);
		extern void SD_PinFinder(void);
		extern void SD_MisoProbe(void);
		extern void SD_VerboseInit(void);      /* ★ 逐字节慢节奏初始化诊断 ✓ */

#if   PROBE_SD_LOOPBACK
		extern void SD_LoopbackTest(void);
		SD_LoopbackTest();                 /* 5 行全 OK = MCU 侧清白 ✓ */
#elif PROBE_SD_PINFINDER
		extern void SD_PinFinder(void);
		SD_PinFinder();                    /* 量模块端：1s=SCK 2s=MOSI 4s=CS ✓ */
#elif PROBE_SD_MISO
		extern void SD_MisoProbe(void);
		SD_MisoProbe();                    /* MISO 针接 GND -> 读数应变 0x00 ✓ */
#elif PROBE_SD_BITBANG
		extern void SD_BitBangProbe(void);
		Serial_Printf("---- BitBang probe: 50kHz CMD0 (no rewiring needed) ----\r\n");
		SD_BitBangProbe();                 /* 能答 -> 信号完整性 ✗ / 不能答 -> 电源·线·卡·模块 ✓ */
#elif PROBE_SD_VERBOSE
		extern void SD_VerboseInit(void);  /* ★ 逐字节慢节奏初始化诊断 ✓ */
		/* ★★ 逐字节慢节奏初始化诊断：每步抬 CS + 插 2ms，ACMD41 多读 8 字节全打出来 ✓
		 *   看点：CMD55 的 R1 是多少；ACMD41 那 8 个字节长什么样
		 *   （如果这个慢版本能过 -> 问题是"节奏太紧" ✓；过不了 -> 看它吐的字节 ✓）*/
		SD_VerboseInit();
#endif

		rr = SD_Init();
		Serial_Printf("---- SD test ----\r\n");
		Serial_Printf("SD_Init=%d (%s) cardtype=%u (1=V1 2=V2 4=SDHC)\r\n",
		              (int)rr, SD_ErrorStr(rr), SD_GetCardType());

		if (rr == SD_OK)
		{
			rr = SD_ReadBlocks(s_sd_buf, 0, 1);          /* 读扇区 0 */
			Serial_Printf("read sector0 = %d (%s)\r\n", (int)rr, SD_ErrorStr(rr));

			if (rr == SD_OK)
			{
				uint32_t start_sec, n_sec;

				Serial_Printf("sector0[0..15]:");
				for (i = 0; i < 16; i++) { Serial_Printf(" %02X", s_sd_buf[i]); }
				Serial_Printf("\r\nMBR sig = %02X %02X (expect 55 AA)\r\n",
				              s_sd_buf[510], s_sd_buf[511]);

				/* MBR 分区表第 1 项在偏移 446：[4]=类型 [8..11]=起始扇区 [12..15]=扇区数（都小端） */
				start_sec = (uint32_t)s_sd_buf[446+8]  | ((uint32_t)s_sd_buf[446+9]  << 8)
				          | ((uint32_t)s_sd_buf[446+10] << 16) | ((uint32_t)s_sd_buf[446+11] << 24);
				n_sec     = (uint32_t)s_sd_buf[446+12] | ((uint32_t)s_sd_buf[446+13] << 8)
				          | ((uint32_t)s_sd_buf[446+14] << 16) | ((uint32_t)s_sd_buf[446+15] << 24);

				Serial_Printf("part1 type=0x%02X start=%lu sec, sectors=%lu (~%lu MB)\r\n",
				              s_sd_buf[446+4], (unsigned long)start_sec, (unsigned long)n_sec,
				              (unsigned long)(n_sec / 2048u));

				/* ============ 写/读回验证（2026-10-05 改：写【两处】做对比）============
				 * ① 分区【前】的空档（start_sec-1）—— 有些卡把这块当自身保留区 ✗
				 * ② 分区【后】的空闲区（start_sec+n_sec）—— 一定在文件系统之外 ✓✓
				 * 判据：
				 *   ① 失败而 ② 成功 ✓ -> 是"卡保护了前部空档"，【不是】写路径问题 ✓
				 *   两个都失败 ✗        -> 写路径本身有问题（CRC/时序/卡）✓✓
				 * 两个都安全：都在分区之外，不会动到文件系统 ✓
				 * 清零再读是为了排除"根本没读到、缓冲里还是旧的" ✗ */
				{
					uint32_t targets[3];
					uint8_t  ntargets = 0;
					uint8_t  t;

					/* ★ 2026-10-05 修正（血泪算术 ✓）：
					 *   卡容量 3,965,190,144B / 512 = 【7,744,512】扇区（有效 0..7,744,511 ✓）
					 *   分区 start=32 + sectors=7,744,480 = 7,744,512 -> ★ 分区【正好填满整卡】✓
					 *   -> 原来写的"分区后"目标 7,744,512 = 【第一个越界扇区】✗
					 *      -> 报 CMD FAILED（R1 bit5 = address error ✓）【不是假卡】✓
					 *   -> 改成写【分区之内且为空】的位置 ✓（卡上没有文件 ✓ 安全 ✓）*/
					/* ★★★★ 2026-10-11 修复：删掉两个"分区内"的写目标 ✗✗
					 *   这是本机历史上最隐蔽的一个 bug ✓（伪装成"测试通过" ✗）
					 *
					 *   ★ 原来的三个目标（用实测数据推 ✓）：
					 *     part1 start = 8192 ✗（type=0x0C = FAT32 ✓）
					 *     ① start-1       = 8191     分区【之前】✓ 安全 ✓
					 *     ② start+100     = 8292   ✗ ★★ 分区内偏移 100
					 *     ③ start+n-1000  = 7743512 ✗ 分区内靠尾
					 *
					 *   ★★ FAT32 的分区布局（关键 ✗）：
					 *     扇区 0 = 引导扇区 BPB ✓ / 1 = FSInfo ✓ / 2~5 = 保留 ✓
					 *     ★ 扇区 6 起 = 【FAT1】✗（FAT 表大小随容量，通常几百个扇区 ✓）
					 *     → ★★★ 【偏移 100 正好落在 FAT1 里】✗✓!!
					 *
					 *   ★★★★ 后果（实测链条 ✓）：
					 *     · 测试段往 FAT 表写了 0xA5^j / 0x5A+j 的垃圾 ✗
					 *     · ★★ 但它【自己读回验证 PASS】✗ —— 它验的是自己刚写的图案 ✓
					 *       → 【看起来完全正常】✗✓ 这是它最坑的地方 ✓
					 *     · FatFs 之后挂载 → 读到垃圾 FAT ✗
					 *       → ★【f_open 全部返回 FR_DENIED(7)】✗✓
					 *         （实测：sd_last=7 恒定 ✓ drop 每秒 +10 ✓ 全链路丢数据 ✓）
					 *     · ★★★ 而且每次上电都写一次 ✗ → 【格式化也救不了】✗✓
					 *       （格式化后测试段立刻又写坏 ✓ —— 所以"格式化没用"✓）
					 *
					 *   ★ 原注释写着"空卡安全"✗ —— 它只考虑了"不破坏已有数据"✗
					 *     没考虑"写完垃圾之后 FatFs 还挂不挂得上"✗✓（这就是盲点 ✓）
					 *
					 *   ★★ 修法：
					 *     · ★ 只保留【分区之前】那个 ✗ —— 那里与文件系统无关 ✓
					 *     · ★★ 而且它能回答"SD_WriteBlocks 本身能不能工作"✓
					 *     · ★★★ "能不能稳定写"交给【正式功能】去验 ✓
					 *       （task_sd 真写卡 ✗ 就是最好的写测试 ✓✓ 不需要往管理区写 ✓）*/
					if (start_sec >= 2u)
					{
						targets[ntargets++] = start_sec - 1u;         /* ★ 只留这个：分区前空档 ✓ */
					}
					else
					{
						Serial_Printf("  (no safe write target: start_sec=%lu too low)\r\n",
						              (unsigned long)start_sec);
					}

					for (t = 0; t < ntargets; t++)
					{
						uint32_t sec = targets[t];
						uint8_t  ok = 1;
						uint16_t j;                 /* ★★ 必须 ≥512！uint8_t 会回绕成死循环 ✗ */

						for (j = 0; j < 16; j++)
						{
							s_sd_buf[j]       = (uint8_t)(0xA5u ^ j);   /* 头部图案 */
							s_sd_buf[496 + j] = (uint8_t)(0x5Au + j);   /* 尾部也放，防"只写了一部分" */
						}
						rr = SD_WriteBlocks(s_sd_buf, sec, 1);
						Serial_Printf("write sec %lu [%s] = %d (%s)\r\n",
						              (unsigned long)sec,
						              (t == 0u) ? "before-part" : "after-part",
						              (int)rr, SD_ErrorStr(rr));

						if (rr == SD_OK)
						{
							/* ★ 定位打印（临时）：写成功后卡住位置不明 -> 三行夹住 ✓ */
							Serial_Printf("  [dbg] write ok, blanking buffer...\r\n");
							for (j = 0; j < 512; j++) { s_sd_buf[j] = 0x00; }   /* 先清零 ✓ */
							Serial_Printf("  [dbg] calling SD_ReadBlocks(sec=%lu)...\r\n", (unsigned long)sec);
							rr = SD_ReadBlocks(s_sd_buf, sec, 1);
							Serial_Printf("  [dbg] SD_ReadBlocks returned %d\r\n", (int)rr);
							Serial_Printf("  read back = %d (%s)\r\n", (int)rr, SD_ErrorStr(rr));

							if (rr == SD_OK)
							{
								for (j = 0; j < 16; j++)
								{
									if (s_sd_buf[j]       != (uint8_t)(0xA5u ^ j)) { ok = 0; }
									if (s_sd_buf[496 + j] != (uint8_t)(0x5Au + j)) { ok = 0; }
								}
								Serial_Printf("  verify: %s\r\n", ok ? "PASS" : "FAIL (mismatch)");
							}
						}
					}
				}
			}
		}
	}
#endif

	/* ★★ 扩容假卡检测（只读 ✓ 绝对安全 ✓）
	 *   放在 SD 测试之后：此时若 SD_Init 成功，卡已初始化 ✓
	 *   （若 SD_Init 失败，下面的读会直接报错，也无害 ✓）*/
#if (TEST_SD) && PROBE_SD_FAKECARD
	{
		extern void SD_FakeCardProbe(void);
		SD_FakeCardProbe();
	}
#endif

	
	Serial_Printf("=== Init Done ===\r\n");

	/* ★★★ 启动 IWDG 独立看门狗（2026-10-12 加 ✓ 阶段 3 最后一块）
	 *
	 *  ★★ 为什么【必须放在这里】✗：
	 *    · IWDG 一旦 Enable() 就【不能停】✗✓
	 *    · 而上面那些初始化【可能耗好几秒】✗
	 *      （★ 实测 SD 的 ACMD41 重试过 142 次 ✓ 那就是好几秒 ✓）
	 *    · → 如果更早启动 ✗ → ★【初始化阶段就会自己复位】✗✓
	 *    · ★★ 所以放在【所有耗时初始化之后、调度器启动之前】✓✓
	 *
	 *  ★ 喂狗点是 vApplicationIdleHook() ✗（见 freertos_demo.c ✓）
	 *    超时约 6.55 秒（★ LSI 公差 → 4.4~8.7 秒 ✓）*/
	WD_Init();

	/* ★★★★ 关于"怎么验证 IWDG 真的有效"（2026-10-12 结论 ✓）
	 *
	 *  ★ 我一开始想加一个"故意死循环不喂狗"的自测段 ✗ —— 【那是错的】✓
	 *    原因（实测踩到 ✓）：
	 *      ① ★ `while(1){}` 之后的一切都【不可达】✗
	 *         → 编译器把 freertos_demo() 和【所有任务】都优化掉了 ✓
	 *         → Code 从 45,272 【暴跌到 22,672】✗ 并报 `statement is unreachable` ✓
	 *      ② ★★ 更要命的是【测错了对象】✗✓：
	 *         · 那种测法只证明【IWDG 硬件会咬人】✗
	 *           —— 而这是【ST 的硅片保证】✓ 根本不用测 ✓
	 *         · ★★★ 它【没证明"喂狗钩子接对了"】✗✓
	 *           （★ 钩子被优化掉了 ✗ 压根没参与 ✓）
	 *
	 *  ★★ 正确的验证 = 【直接烧正式版】✗ 然后看 ✗：
	 *    · ★ 跑起来且【不反复复位】✗ → 喂狗路径是通的 ✓✓
	 *    · ★★ 每 6.55 秒复位一次 ✗ → 钩子没接上 / 配置错了 ✓✓
	 *  → ★★★ 零额外代码 ✗ 而且【测的正是有风险的地方】✓✓ */

	freertos_demo();
    while (1)
    {
		
    }
}
