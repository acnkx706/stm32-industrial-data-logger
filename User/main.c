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
#include "MyI2C.h"                 /* ★ BMP280 与 MPU6050 共用这条软件 I2C（PA2=SCL / PA3=SDA）*/
#include "BMP280.h"

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


int main(void)
{int16_t oax, oay, oaz, ogx, ogy, ogz;
/* ---------- 第一批：先把"能显示/能通信"的立起来（诊断能力优先） ---------- */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	Serial_Init();
	Serial_Printf("=== boot ===\r\n");
	AD_TestVdd();
	AD_Init();
	OLED_Init();
	Key_Init();
	LightSensor_Init();

	/* ---------- 第二批：传感器 ---------- */
	MPU6050_Init();
	Serial_Printf("MPU ID = %02X (expect 68)\r\n", MPU6050_GetID());
	BMP280_Probe();                        /* ★ 顺手确认 I2C 总线 + 认 BMP280 型号 ✓
	                                        *   （MyI2C_Init 已在 MPU6050_Init 里调过 ✓）*/
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
	Serial_Printf("LIGHT DO = %u\r\n", LightSensor_Get());
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
	/* ============ 调试期测试开关（正式跑记录时两个都设 0）============
	 * TEST_DS18B20 = 1：单次读一次 DS18B20（PB14）并打印
	 * TEST_SD      = 1：SD 卡初始化 + 读扇区 0 验 MBR（换好板子/卡之后再打开）
	 * 硬件排查探针（回环 / PinFinder / MISO 探针）已存档到
	 *   stm32\_临时脚本\SD_排查探针.md —— 需要时贴回 SD_SPI.c 末尾
	 * ============================================================= */
	#define TEST_DS18B20   0      /* DS18B20 已调通（探针待清理）-> 关掉，让 SD 输出干净 ✓ */
	#define TEST_SD        1     /* ★ 2026-10-05 换 USB 线后电压恢复正常 -> 重测 SD 卡 ✓ */

	/* ★ SD 硬件探针选择（实现在 Hardware\SD_SPI.c 末尾 ✓ 跑完自动恢复硬件 SPI ✓）
	 *   0 = 不跑（正式测试用）
	 *   1 = 回环自检      —— 需【拔掉模块 4 根线】+ 短接 PA6<->PA7 ✓
	 *   2 = 引脚识别      —— 需万用表（死循环，不返回 ✓）
	 *   3 = MISO 通路     —— 需把模块 MISO 针接 GND（串 1k~10k 电阻 ✓ 死循环 ✓）
	 *   4 = ★ 位翻转 50kHz CMD0 —— 【不用动任何线】✓ 结果直接打在串口上 ✓ 推荐先用它 ✓ */
	/* ★★ 2026-10-05：测 SD 一律用 0 ✗ —— 探针跑在 SD_Init() 之前，它自己会反复折腾卡
	 *    （发 CMD0 / 翻时钟 ✓），会把卡留在【失步状态】✗
	 *    → 紧接着的 SD_Init 连 CMD0 都答不上 ✗（实测踩过：开着探针那一轮 = NO_CARD ✓）
	 *    ★ 任何探针跑完后，要再测 SD，都必须【先给模块断电重上电】✓ */
	#define TEST_SD_PROBE  0

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

#if   (TEST_SD_PROBE == 1)
		SD_LoopbackTest();                 /* 5 行全 OK = MCU 侧清白 ✓ */
#elif (TEST_SD_PROBE == 2)
		SD_PinFinder();                    /* 量模块端：1s=SCK 2s=MOSI 4s=CS ✓ */
#elif (TEST_SD_PROBE == 3)
		SD_MisoProbe();                    /* MISO 针接 GND -> 读数应变 0x00 ✓ */
#elif (TEST_SD_PROBE == 4)
		Serial_Printf("---- BitBang probe: 50kHz CMD0 (no rewiring needed) ----\r\n");
		SD_BitBangProbe();                 /* 能答 -> 信号完整性 ✗ / 不能答 -> 电源·线·卡·模块 ✓ */
#elif (TEST_SD_PROBE == 5)
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
					if (start_sec >= 2u)
					{
						targets[ntargets++] = start_sec - 1u;         /* ① 分区前空档（可能受保护 ✗）*/
					}
					targets[ntargets++] = start_sec + 100u;           /* ② 分区内 FAT 区（空卡安全 ✓）*/
					targets[ntargets++] = start_sec + n_sec - 1000u;  /* ③ 分区内靠尾（空卡安全 ✓）*/

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
#if (TEST_SD)
	{
		extern void SD_FakeCardProbe(void);
		SD_FakeCardProbe();
	}
#endif

	Serial_Printf("=== Init Done ===\r\n");
	freertos_demo();
    while (1)
    {
		
    }
}
