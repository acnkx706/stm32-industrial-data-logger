/**
  ******************************************************************************
  * @file    SD_SPI.c
  * @brief   SD 卡 SPI 模式驱动（硬件 SPI1，标准库）—— 执行表 T2.1
  *
  *  ┌─ 为什么要"软件 CS + 硬件 SPI1" ─────────────────────────────────────┐
  *  │ SD 卡 SPI 协议是全双工、帧长不固定（命令 6 字节、数据 512+N 字节、   │
  *  │ token 靠 MISO 实时回读），硬件 SPI 负责字节收发，CS 由 GPIO 手动拉： │
  *  │ 因为 SD 协议里"一条命令"和"一次数据"的边界由 CS 的起落定义，而硬件   │
  *  │ NSS 只能自动管理"一次 SPI 传输"的边界，粒度对不上 —— 所以 NSS_Soft。 │
  *  └──────────────────────────────────────────────────────────────────────┘
  *
  *  ★ SPI 用 mode 3（CPOL=1 空闲高 + CPHA=2Edge）的原因：
  *    SD 卡规定 DI 在 SCK 上升沿采样、DO 在下降沿更新。空闲电平为高、
  *    上升沿是"第 2 个边沿"（2Edge），恰好是 SPI mode 3。
  *
  *  ★ MISO(PA6) 必须配内部上拉（IPU）的原因：
  *    卡在"未选中/识别完成前"的 DO 是高阻态，悬空时读到的是随机电平 ——
  *    驱动靠"读到 0xFF = 无响应"来判定卡的应答，悬空会让判定时灵时不灵。
  *    上拉保证 DO 空闲时是确定的 0xFF。（硬件上模块若已带 10k~100k 上拉，
  *    内部上拉与其并联，无害。）
  *
  *  ★ 上电低速 281kHz、初始化后 9MHz 的原因：
  *    卡上电只保证在 ≤400kHz 下能工作（卡内部分频器还在爬升），所以
  *    CMD0~ACMD41 阶段必须 72M/256=281.25kHz；初始化通过后切 72M/8=9MHz
  *    （远低于高速卡 25MHz 上限，杜邦线/电平转换模块都扛得住，留足余量）。
  *
  *  ★ 命令 CRC：SD 卡上电默认要求命令 CRC 有效（SPI 模式里唯一强制 CRC 的
  *    阶段）。本驱动所有命令都现算 CRC7（函数 SD_Crc7，已用规范已知值
  *    CMD0=0x95 / CMD8(0x1AA)=0x87 验证）；初始化完成后发 CMD59(0) 关卡内
  *    CRC 校验，此后【数据块】的 2 字节 CRC 直接发 0xFF 哑元（省掉一套
  *    CRC16 运算）。命令 CRC 仍保留现算 —— 有些卡对 CMD59 并不完全认账，
  *    命令 CRC 继续给有效值最稳。
  *
  *  ★ 线程边界：本驱动【非重入】（内部有全局卡类型、共享 SPI 总线）。
  *    全工程只允许 task_sd（经 FatFs 的 diskio）一个调用者 —— 执行表铁律。
  *
  *  延时全部走 DWT（system\DWT_Delay.c）：本驱动可能在调度器启动后运行，
  *  裸机 Delay.c 的 SysTick 版是 FreeRTOS 雷（见 DWT_Delay.c 头注释）。
  ******************************************************************************
  */
#include "stm32f10x.h"                  /* Device header */
#include "SD_SPI.h"
#include "DWT_Delay.h"
#include "Serial.h"        /* 临时：只为回环自检打印，测完连这个 include 一起删 */
#include "ProbeConfig.h"
#include "Crc16.h"          /* ★ 2026-10-11（审查 F13）：数据块 CRC 改用公共实现 ✓
                             *   原来本文件里有一份 static SD_Crc16 ✗
                             *   而 Crc16.h 自己立着"一处实现、避免两份悄悄跑偏"的规矩 ✗
                             *   → 现在统一走 Crc16_Kermit() ✓✓ */
#include "AD.h"             /* ★ 2026-10-11（审查 F23）：AD_ReadVrefint 改用正式声明 ✓
                             *   原来这里写的是【局部 extern】✗ ——
                             *   那是独立于 AD.h 的第二份签名 ✓
                             *   AD.c 哪天改了参数/返回类型 ✗ → 这行不会跟着变 ✗
                             *   而编译器【也不会报错】✗（两个编译单元各信自己的声明 ✓）
                             *   → 只有链接期/运行期才炸 ✓✓ 现在挪进头文件 = 单一事实来源 ✓
                             *
                             *   ★ 当时的注释写着"为什么写成 extern 而不去改 AD.h：
                             *     少改一个文件、少一处和别人的冲突面"✗
                             *     —— 那个取舍是错的 ✗：省一个 include 换来一个静默风险 ✓ */

/* ★★ 原来的局部 extern 已删（同审查 F23）✗：
 *   extern uint16_t AD_ReadVrefint(void);                                        */
/*============================================================================
 *  内部参数（集中放一起，调参不用翻代码）
 *==========================================================================*/
#define SD_DUMMY            0xFF    /* 哑元字节：全双工总线上"只收不发"就喂 0xFF */

#define SD_INIT_SPEED       SPI_BaudRatePrescaler_256   /* 72M/256 = 281.25kHz ≤400kHz */
#define SD_FAST_SPEED       SPI_BaudRatePrescaler_8     /* 72M/8   = 9MHz              */

#define SD_PWR_SETTLE_MS    10      /* 上电稳定等待（模块稳压 + 卡内上电时序） */
#define SD_IDLE_RETRY       200     /* CMD0 重发次数上限（换卡瞬间/慢启动卡） */
#define SD_INIT_TIMEOUT_MS  2000    /* ACMD41 timeout widened 1s->2s for marginal-contact debug */    /* ACMD41 总超时：规范典型 ~500ms，给一倍余量 */
#define SD_READ_TIMEOUT_MS  100     /* 读数据 token 等待（规范 NAC=100ms） */
#define SD_BUSY_TIMEOUT_MS  500     /* 写 busy 等待（规范单块编程最长 250ms，留一倍） */
#define SD_OP_RETRY         3       /* 读/写整操作重试次数（执行表 T2.1：3 次） */

/*============================================================================
 *  命令表（SD 规范：命令字节 = 0x40 | 命令号）
 *  ★ CMD55/CMD12/CMD59/ACMD23 虽不在执行表点名清单里，但分别是
 *    ACMD 的前导、多块读的收尾、数据 CRC 开关、多块写提速 —— 缺一不可
 *==========================================================================*/
#define CMD0    (0)     /* GO_IDLE_STATE   ：复位进 IDLE，响应 R1=0x01 */
#define CMD8    (8)     /* SEND_IF_COND    ：探测 SDv2（参数 0x1AA，响应 R7） */
#define CMD12   (12)    /* STOP_TRANSMISSION：停多块读（响应前有 1 个 stuff 字节） */
#define CMD16   (16)    /* SET_BLOCKLEN    ：设块长 512（SDHC 固定 512，忽略之） */
#define CMD17   (17)    /* READ_SINGLE_BLOCK */
#define CMD18   (18)    /* READ_MULTIPLE_BLOCK */
#define CMD24   (24)    /* WRITE_BLOCK       */
#define CMD25   (25)    /* WRITE_MULTIPLE_BLOCK */
#define CMD55   (55)    /* APP_CMD 前导：下一条命令按 ACMD 解释 */
#define CMD58   (58)    /* READ_OCR        ：看 CCS 位定卡类型/寻址方式 */
#define CMD59   (59)    /* CRC_ON_OFF      ：参数 0 = 关数据 CRC 校验 */
#define ACMD23  (23)    /* SET_WR_BLK_ERASE_COUNT：多块写前预擦除（可选优化） */
#define ACMD41  (41)    /* SD_SEND_OP_COND ：初始化轮询，0x00 = 就绪 */

/*============================================================================
 *  R1 响应位定义（1 字节状态）
 *==========================================================================*/
#define R1_IDLE         0x01    /* 卡在 IDLE 状态（初始化中） */
#define R1_ILLEGAL_CMD  0x04    /* 非法命令（CMD8 探测 SDv1/MMC 靠它） */
#define R1_READY        0x00    /* 就绪：无任何错误位 */

/*============================================================================
 *  数据 token / 数据响应
 *==========================================================================*/
#define TOK_READ        0xFE    /* 单块读 & 单块写的起始 token */
#define TOK_WRITE_MULTI 0xFC    /* 多块写每块的起始 token */
#define TOK_STOP_MULTI  0xFD    /* 多块写的结束 token */
#define DAT_ACCEPTED    0x05    /* 数据响应 xxx00101 = 卡已接受本块 */

/*============================================================================
 *  私有变量
 *==========================================================================*/
static uint8_t s_card_type = SD_CT_NONE;    /* 当前卡类型（SD_CT_xxx） */

/*============================================================================
 *  底层：CS / 字节交换 / CRC7
 *==========================================================================*/

/* CS 是 PA4 上的普通 GPIO（NSS_Soft 的另一半：电平由我们说了算） */
static void SD_CS(uint8_t bit)
{
	GPIO_WriteBit(GPIOA, GPIO_Pin_4, (BitAction)bit);
}

/* 一个字节的收发：SPI 全双工，发一个字节的同时必然收一个字节。
 * "发哑元 0xFF" = 制造时钟让卡把数据吐过来 —— SD 驱动里到处是这个套路 */
static uint8_t SD_SwapByte(uint8_t tx)
{
	uint32_t t;

	/* 等 TXE：发送缓冲空，可以塞新字节。
	 * ★ 带上限：SPI 配错/未使能时若死等，整机会静默卡死（30 号 §十一.10 的教训）。
	 *   超时后返回 0xFF（"总线上什么都没有"）→ 命令层判无响应 → 错误自然上浮，无需额外标志 */
	for (t = 0; t < 100000u && SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET; t++);
	if (t >= 100000u) return SD_DUMMY;

	SPI_I2S_SendData(SPI1, tx);

	/* 等 RXNE：移位寄存器收完一个字节 */
	for (t = 0; t < 100000u && SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET; t++);
	if (t >= 100000u) return SD_DUMMY;

	return (uint8_t)SPI_I2S_ReceiveData(SPI1);
}

/* 等 SPI 真正发完（BSY 清零）：拉 CS 前必须调，否则最后一个字节
 * 还在移位寄存器里没走完，CS 先抬起来 = 卡收到半个字节 = 偶发诡异错误 */
static void SD_WaitSpiIdle(void)
{
	uint32_t t;
	/* ★ 同样带上限：这里是"尽力等"，超时也继续（后面的 SD_Release 还会补 8 时钟），
	 *   但绝不允许在这里把整机卡死 */
	for (t = 0; t < 100000u && SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET; t++);
}

/* 事务收尾：CS 抬高 + 补 8 个时钟。
 * 为什么补时钟：卡在 CS 上升沿释放 DO，且部分内部操作（如 CMD12 收尾）
 * 需要"CS 高电平期间的额外时钟"才能完成 —— 缺了这 8 个时钟，会出现
 * "下一条命令偶发无响应"这种查一晚上不出来的玄学问题 */
static void SD_Release(void)
{
	SD_CS(1);
	SD_WaitSpiIdle();
	(void)SD_SwapByte(SD_DUMMY);
}

/* CRC7 生成（命令用）。
 * 多项式 x^7+x^3+1（0x89），SD 命令帧 = 0x40|cmd + 4 参数字节，共 5 字节；
 * 结果占发送字节的 bit7..1，bit0 是恒 1 的停止位。
 * 自检锚点（与 SD 规范附录一致）：CMD0 帧算出 0x95，CMD8(0x1AA) 帧算出 0x87 */
static uint8_t SD_Crc7(const uint8_t *buf, uint8_t len)
{
	uint8_t crc = 0;
	uint8_t i, bit;

	for (i = 0; i < len; i++)
	{
		crc ^= buf[i];
		for (bit = 0; bit < 8; bit++)
		{
			if (crc & 0x80)
			{
				crc = (uint8_t)((crc << 1) ^ 0x12); /* 0x12 = 多项式低 7 位左移一位后的反馈掩码 */
			}
			else
			{
				crc = (uint8_t)(crc << 1);
			}
		}
	}
	return (uint8_t)(crc | 0x01);       /* 补上恒 1 的停止位 */
}

/* ★★ 本文件原来的 static SD_Crc16 已删（2026-10-11 独立审查 F13）✗
 *   它与 Hardware\Crc16.c 的 Crc16_Kermit() 是【逐行相同的两份副本】✓
 *   （poly 反射 0x8408 / init 0 / 输入输出均反射 / 无输出异或 ✓）
 *   而 Crc16.h 顶部写着"一处实现、多处使用 ✓（避免两份实现悄悄跑偏 ✗）"✗
 *   → 自己立的规矩自己没守住 ✓ 现在统一调用 Crc16_Kermit() ✓✓
 *
 *   ★ 对拍结论（删之前核过 ✓）：
 *     Crc16_Kermit 的锚点 "123456789" → 0x2189 ✓（与 SD 那份逐行同源 ✓）
 *     两份代码参数与实现完全一致 ✓ 替换无行为变化 ✓✓ */

/*============================================================================
 *  命令层
 *==========================================================================*/

/* 发一条命令并收 R1（R3/R7 在 R1 之后还有 4 个数据字节，由调用者另收）。
 * 返回：R1 值；0xFF = 卡无响应（没卡/卡死）
 * ★ 本函数把 CS 拉低但不抬 —— 多块读的块与块之间不能抬 CS，由调用者决定
 *   何时 SD_Release()（单发命令后立刻 Release，连续读块则读完全部再 Release） */
static uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg)
{
	uint8_t frame[6];
	uint8_t r1;
	uint8_t i;

	frame[0] = (uint8_t)(0x40 | cmd);
	frame[1] = (uint8_t)(arg >> 24);    /* 参数大端序发送 */
	frame[2] = (uint8_t)(arg >> 16);
	frame[3] = (uint8_t)(arg >> 8);
	frame[4] = (uint8_t)(arg);
	frame[5] = SD_Crc7(frame, 5);       /* 初始化阶段卡强制要求有效 CRC，索性全算 */

	SD_CS(0);
	for (i = 0; i < 6; i++)
	{
		(void)SD_SwapByte(frame[i]);
	}

	/* CMD12 特例：响应之前卡先吐 1 个 stuff 字节，必须跳过它再收 R1 */
	if (cmd == CMD12)
	{
		(void)SD_SwapByte(SD_DUMMY);
	}

	/* ★★★★ 等响应（2026-10-12 修正 ✗ 这是一个【真 bug】✓✓）
	 *
	 *  【错在哪】✗：判据原来是 `if (r1 != 0xFF) break;` ✗✓
	 *    —— 那等于"任何非 0xFF 的字节都当成 R1"✓
	 *
	 *  【为什么错】✗：★ R1 响应的【bit7 必须为 0】✗（R1 ∈ {0x00..0x7F} ✓）
	 *    所以【合法判据是 `(r1 & 0x80) == 0`】✗✓
	 *    ★★ 卡在"还没准备好"时吐出的 0xC0 / 0xE0 / 0x85 这类中间值 ✗
	 *      会被【原样返回】✗ → 调用者按位判断 → 【误判】✓
	 *
	 *  【实测症状】✗：CMD59 读到含 bit2 的字节 ✗
	 *    → `r1 & R1_ILLEGAL_CMD(0x04)` 成立 ✗
	 *    → ★★★ 误报 SD_ERR_UNSUPPORTED(03) ✓✓
	 *    → 表现：热拔卡（拔久一点）后插回必犯 ✗ 必须给模块断电重上电才好 ✓
	 *
	 *  【为什么"拔久了才犯"】✗：
	 *    · 短时间拔插 → 卡内还有残余电荷 ✗ → 上电后【立刻能用】✓
	 *    · 长时间拔下 → 【完全放电】✗ → 上电后内部初始化慢 ✓
	 *      → ★★ 响应延迟【超过 8 字节】✗ → 更容易读到中间值 ✓✓
	 *
	 *  ★ 本文件行 741 的注释早就记过"实测 0xE0 ✗，R1 非法"✗ ——
	 *    那时只在 ACMD41 那里打了补丁 ✗ 【没修根】✓ 现在修根 ✓✓
	 *
	 *  ★★ 顺带把等待上限从 8 放宽到 16 ✗
	 *    （规范 Ncr = 0~8 字节 ✓ 但【上电初期】可能更宽 ✓）*/
	for (i = 0; i < 16; i++)
	{
		r1 = SD_SwapByte(SD_DUMMY);
		if ((r1 & 0x80u) == 0u)         /* ★ bit7 = 0 才是合法 R1 ✓ */
		{
			break;
		}
	}
	return r1;                          /* CS 保持低，收尾交给调用者 */
}

/* ACMD 命令 = 先发 CMD55 前导，再发真命令
 * ★★★ 2026-10-05 关键修复：CMD55 与 ACMD 之间【必须给卡一个间隙】✗✗
 *   原来两条命令背靠背发送（CS 全程压低、中间不补时钟）✗
 *     -> 实测：卡对 ACMD41 的响应变成 0xE0（bit6=参数错/bit5=地址错，非法 R1）✗
 *        随后永久沉默（状态机卡死，只能断电恢复）✗
 *   同一时刻用"慢节奏"版本（每条命令后 SD_Release() + 2ms 延时）实测：
 *     线上字节 = 【FF 01 FF FF FF FF FF FF】-> ★ 卡正确回了 R1=0x01（"忙，等我"）✓✓
 *   => 根因 = 这张卡不接受 CMD55/ACMD 背靠背，需要抬 CS + 8 时钟的间隙 ✓
 *   （ChaN 的驱动每条命令后都 deselect，天然有这个间隙 —— 这也是它到处能跑的原因 ✓）
 *   为什么只在这里加：其它调用点本来就是"一条命令 + 立刻 Release"，只有 ACMD 是两条 ✗ */
static uint8_t SD_SendAcmd(uint8_t cmd, uint32_t arg)
{
	(void)SD_SendCmd(CMD55, 0);

	SD_Release();                  /* ★ 抬 CS + 补 8 时钟 ✓ */
	DWT_Delay_us(100);             /* ★ 再给 100us 让卡处理完 CMD55 ✓
	                                *   （verbose 版用 2ms 也能通；这里取小值，
	                                *    保证 ACMD41 轮询的整体节奏仍在超时预算内 ✓）*/

	return SD_SendCmd(cmd, arg);
}

/*============================================================================
 *  等待层（超时全部用 DWT 计时）
 *  ★ 为什么不用 vTaskDelay：SD 的 busy/token 等待必须"原地空转"——本驱动
 *    只被最低优先级 task_sd 调用，空转只拖慢自己，卡 250ms 的慢被隔离在
 *    底层（执行表 T2.3 负载核算的前提）；更不能用裸机 Delay.c（SysTick 雷）
 *  ★ 为什么用 DWT->CYCCNT 原始计数：起点取一次、每圈做无符号减法，
 *    72MHz 下 (now-start)/72000 = 已流逝毫秒数；unsigned 减法天然免疫回绕
 *==========================================================================*/

/* 等卡写编程结束：卡在内部烧写期间把 DO 拉低，读到 0xFF 才算完。
 * 返回 1=就绪，0=超时（卡坏/拔卡） */
static uint8_t SD_WaitBusy(uint32_t timeout_ms)
{
	uint32_t start = DWT->CYCCNT;

	while (SD_SwapByte(SD_DUMMY) != 0xFF)
	{
		if ((DWT->CYCCNT - start) > (timeout_ms * 72000u))
		{
			return 0;
		}
	}
	return 1;
}

/* 等数据起始 token（0xFE）：读操作时卡内部可能要先翻查找表（NAC 延迟）。
 * 返回 1=等到，0=超时 */
static uint8_t SD_WaitToken(uint32_t timeout_ms)
{
	uint32_t start = DWT->CYCCNT;
	uint8_t  d;

	do
	{
		d = SD_SwapByte(SD_DUMMY);
		if (d == TOK_READ)
		{
			return 1;
		}
		if ((DWT->CYCCNT - start) > (timeout_ms * 72000u))
		{
			return 0;
		}
	} while (1);
}

/*============================================================================
 *  硬件初始化 / 变速
 *==========================================================================*/

/* GPIO + SPI1 外设初始化：出厂即 281kHz 低速（SD_SPI.h 顶部的引脚表） */
static void SD_LowLevelInit(void)
{
	GPIO_InitTypeDef gpio;
	SPI_InitTypeDef  spi;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1, ENABLE);

	/* PA4 = CS：软件控制推挽输出，空闲拉高（不选中卡） */
	gpio.GPIO_Pin   = GPIO_Pin_4;
	gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio);
	GPIO_SetBits(GPIOA, GPIO_Pin_4);

	/* PA5 SCK / PA7 MOSI：复用推挽 —— 电平交给 SPI1 外设生成 */
	gpio.GPIO_Pin  = GPIO_Pin_5 | GPIO_Pin_7;
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &gpio);

	/* PA6 MISO：上拉输入 —— 关键见文件头注释（卡 DO 高阻期要读到确定的 0xFF） */
	gpio.GPIO_Pin  = GPIO_Pin_6;
	gpio.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &gpio);

	/* SPI1 主机，mode 3（CPOL=1 空闲高 / CPHA=2Edge），8bit，MSB 先行 */
	spi.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
	spi.SPI_Mode              = SPI_Mode_Master;
	spi.SPI_DataSize          = SPI_DataSize_8b;
	spi.SPI_CPOL              = SPI_CPOL_High;
	spi.SPI_CPHA              = SPI_CPHA_2Edge;
	spi.SPI_NSS               = SPI_NSS_Soft;           /* CS 由 PA4 手动管理 */
	spi.SPI_BaudRatePrescaler = SD_INIT_SPEED;          /* 上电 281kHz */
	spi.SPI_FirstBit          = SPI_FirstBit_MSB;
	spi.SPI_CRCPolynomial     = 7;                      /* 硬件 CRC 不用，占位必填 */
	SPI_Init(SPI1, &spi);
	SPI_Cmd(SPI1, ENABLE);
}

/* 变速：改 CR1 的波特率分频必须先关 SPI 再开（参考手册：SPI 使能期间
 * CR1 大多位只读，直接写不生效 —— 玄学问题预备队二号） */
static void SD_SetSpeed(uint16_t prescaler)
{
	SPI_InitTypeDef spi;

	SPI_Cmd(SPI1, DISABLE);

	spi.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
	spi.SPI_Mode              = SPI_Mode_Master;
	spi.SPI_DataSize          = SPI_DataSize_8b;
	spi.SPI_CPOL              = SPI_CPOL_High;
	spi.SPI_CPHA              = SPI_CPHA_2Edge;
	spi.SPI_NSS               = SPI_NSS_Soft;
	spi.SPI_BaudRatePrescaler = prescaler;
	spi.SPI_FirstBit          = SPI_FirstBit_MSB;
	spi.SPI_CRCPolynomial     = 7;
	SPI_Init(SPI1, &spi);

	SPI_Cmd(SPI1, ENABLE);
}

/*============================================================================
 *  裸读写（不带重试）：sector 是 512B 块号，内部按卡类型翻译成地址
 *==========================================================================*/

/* 块号 → 命令参数：SDHC/SDXC 是块寻址（参数=块号）；
 * V1/V2 是字节寻址（参数=块号×512）。混用是"读出来全错但不报错"的经典坑 */
static uint32_t SD_SectorToAddr(uint32_t sector)
{
	if (s_card_type == SD_CT_SDHC)
	{
		return sector;
	}
	return sector * 512u;
}

/* 读 1~N 块。多块用 CMD18 连读，读完美发 CMD12 停传输 */
static SD_Result_t SD_ReadRaw(uint8_t *buf, uint32_t sector, uint32_t count)
{
	uint32_t addr, i, n;
	uint8_t  r1;

	addr = SD_SectorToAddr(sector);

	if (count == 1)                         /* ---- 单块读 CMD17 ---- */
	{
		r1 = SD_SendCmd(CMD17, addr);
		if (r1 != R1_READY)
		{
			SD_Release();
			return SD_ERR_CMD_FAILED;
		}

		if (!SD_WaitToken(SD_READ_TIMEOUT_MS))
		{
			SD_Release();
			return SD_ERR_READ_TIMEOUT;
		}

		for (i = 0; i < 512; i++)           /* 数据 512 字节 */
		{
			buf[i] = SD_SwapByte(SD_DUMMY);
		}
		(void)SD_SwapByte(SD_DUMMY);        /* 2 字节 CRC 占位丢弃：CMD59 已关 CRC，
											   卡发什么是它的自由，我们不校验 */
		(void)SD_SwapByte(SD_DUMMY);
		SD_Release();
	}
	else                                    /* ---- 多块读 CMD18 ---- */
	{
		r1 = SD_SendCmd(CMD18, addr);
		if (r1 != R1_READY)
		{
			SD_Release();
			return SD_ERR_CMD_FAILED;
		}

		for (n = 0; n < count; n++)
		{
			if (!SD_WaitToken(SD_READ_TIMEOUT_MS))
			{
				/* 中途超时必须发 CMD12 把传输掐断，否则卡会一直占着总线 */
				(void)SD_SendCmd(CMD12, 0);
				SD_Release();
				return SD_ERR_READ_TIMEOUT;
			}
			for (i = 0; i < 512; i++)
			{
				*buf++ = SD_SwapByte(SD_DUMMY);
			}
			(void)SD_SwapByte(SD_DUMMY);    /* CRC 占位 ×2 */
			(void)SD_SwapByte(SD_DUMMY);
		}

		(void)SD_SendCmd(CMD12, 0);         /* 停止多块读 */
		SD_Release();
	}
	return SD_OK;
}

/* 写 1~N 块。多块用 CMD25，每块之间都要等 busy（卡逐块烧写） */
static SD_Result_t SD_WriteRaw(const uint8_t *buf, uint32_t sector, uint32_t count)
{
	uint32_t addr, i, n;
	uint8_t  r1, res;

	addr = SD_SectorToAddr(sector);

	if (count == 1)                         /* ---- 单块写 CMD24 ---- */
	{
		r1 = SD_SendCmd(CMD24, addr);
		if (r1 != R1_READY)
		{
			SD_Release();
			return SD_ERR_CMD_FAILED;
		}

		(void)SD_SwapByte(SD_DUMMY);        /* 命令与数据之间留 1 字节空隙（Nwr 时序） */
		(void)SD_SwapByte(TOK_READ);        /* 单块写起始 token 也是 0xFE */
		for (i = 0; i < 512; i++)
		{
			(void)SD_SwapByte(buf[i]);
		}
		/* ★ 2026-10-05：改发【真 CRC16】（原来发哑元 0xFF ✗，只在 CMD59 生效时才成立 ✓）
		 *   大端序（高字节先）✓ */
		{
			uint16_t wcrc = Crc16_Kermit(buf, 512u);   /* ★ 改用公共实现（审查 F13）*/
			(void)SD_SwapByte((uint8_t)(wcrc >> 8));
			(void)SD_SwapByte((uint8_t)(wcrc & 0xFFu));
		}

		/* 数据响应 token：规范形状 = xxx0 sss 1（bit0 恒 1、bit4 恒 0）
		 * ★★★ 2026-10-05 修复：原来判据是"找到一个 MSB=0 的字节就停" ✗
		 *   结果把【卡 busy 时拉低的 0x00】当成了 token ✗ -> 误判"数据被拒"✓
		 *   实测：token=0x00，连打 3 次（驱动内部重试 3 次 ✓）
		 *   -> 0x00 = 卡把 DO 拉低 = 【它正在写】✓ 说明数据其实被收下了 ✓
		 *   ★ 两种情形都要兼容：
		 *     A. 拿到合法 token（0x05/0x0B/0x0D）-> 严格按它判 ✓
		 *     B. 读到 0x00（token 已经过去、我们赶上的是 busy）-> 不判错，
		 *        交给后面的 SD_WaitBusy + 调用者的【读回验证】来定案 ✓✓
		 *        （读回验证才是最终真相：数据对不对，读一遍就知道 ✓）*/
		res = 0xFF;
		for (i = 0; i < 8; i++)
		{
			res = SD_SwapByte(SD_DUMMY);
			if ((res & 0x11u) == 0x01u) { break; }   /* A. 合法 token ✓ */
			if (res == 0x00u)           { break; }   /* B. busy-low（token 错过 ✓）*/
		}
		if (((res & 0x11u) == 0x01u) && ((res & 0x1Fu) != DAT_ACCEPTED))
		{
			/* ★ 诊断（临时，2026-10-05）：把 token 原样打出来 ——
			 *   规范：token = xxx0 sss 1，sss = 010 已接受 ✓ / 101 CRC 错 ✗ / 110 写错 ✗
			 *   -> 0x05 = 已接受 / 0x0B = CRC 错 / 0x0D = 卡内部写错 ✓✓ */
			Serial_Printf("SD: write resp token=0x%02X (0x05=ok, 0x0B=CRC err, 0x0D=write err)\r\n",
			              (unsigned)res);
			SD_Release();
			return SD_ERR_WRITE_RESPONSE;
		}

		/* 卡烧写期间 DO 拉低（busy），必须等到它抬起来才准抬 CS */
		if (!SD_WaitBusy(SD_BUSY_TIMEOUT_MS))
		{
			SD_Release();
			return SD_ERR_WRITE_TIMEOUT;
		}
		SD_Release();
	}
	else                                    /* ---- 多块写 CMD25 ---- */
	{
		/* ACMD23 预擦除【可选优化】：提前告诉卡"接下来要写 count 块"，卡把
		 * 这些块一次擦好，多块写耗时能降一半以上、磨损也更均匀。
		 * 卡不认识就忽略（返回值不看）；对单块写无意义所以只在多块前发 */
		(void)SD_SendAcmd(ACMD23, count);

		r1 = SD_SendCmd(CMD25, addr);
		if (r1 != R1_READY)
		{
			SD_Release();
			return SD_ERR_CMD_FAILED;
		}

		for (n = 0; n < count; n++)
		{
			(void)SD_SwapByte(SD_DUMMY);            /* 块间空隙 */
			(void)SD_SwapByte(TOK_WRITE_MULTI);     /* 多块写起始 token 0xFC */
			for (i = 0; i < 512; i++)
			{
				(void)SD_SwapByte(buf[n * 512u + i]);
			}
			(void)SD_SwapByte(SD_DUMMY);            /* CRC 哑元 ×2 */
			(void)SD_SwapByte(SD_DUMMY);

			/* 数据响应（同单块写） */
			for (i = 0; i < 8; i++)
			{
				res = SD_SwapByte(SD_DUMMY);
				if ((res & 0x80) == 0)
				{
					break;
				}
			}
			if ((res & 0x1F) != DAT_ACCEPTED)
			{
				(void)SD_SendCmd(CMD12, 0);         /* 出错强停：卡必须退出写状态 */
				SD_Release();
				return SD_ERR_WRITE_RESPONSE;
			}

			/* ★ 每写完一块都要等 busy：下一块的数据只能等卡烧完上一块才收 */
			if (!SD_WaitBusy(SD_BUSY_TIMEOUT_MS))
			{
				(void)SD_SendCmd(CMD12, 0);
				SD_Release();
				return SD_ERR_WRITE_TIMEOUT;
			}
		}

		(void)SD_SwapByte(TOK_STOP_MULTI);  /* 结束 token 0xFD */
		if (!SD_WaitBusy(SD_BUSY_TIMEOUT_MS))   /* 卡还要烧最后一块，继续等 */
		{
			SD_Release();
			return SD_ERR_WRITE_TIMEOUT;
		}
		SD_Release();
	}
	return SD_OK;
}

/*============================================================================
 *  对外接口
 *==========================================================================*/

/**
  * @brief  SD 卡上电初始化（可反复调用：拔卡重插恢复就是再调一遍）
  *         流程：74 时钟预热 → CMD0 → CMD8 探卡型 → ACMD41 轮询就绪 →
  *               CMD58 定寻址 → CMD16 设块长 → CMD59 关 CRC → 切 9MHz
  * @param  无
  * @retval SD_Result_t
  */
SD_Result_t SD_Init(void)
{
	uint8_t     r1, i;
	uint8_t     is_sd2 = 0;
	uint8_t     ocr[4];
	uint32_t    t_start;
	uint16_t    tries = 0;             /* ★ 诊断：ACMD41 轮询次数（2026-10-05 加）*/
	SD_Result_t ret;

	s_card_type = SD_CT_NONE;

	/* 后面所有毫秒级等待全走 DWT（调度器启动后调用本函数也安全） */
	DWT_Delay_Init();
	SD_LowLevelInit();
	DWT_Delay_ms(SD_PWR_SETTLE_MS);     /* 模块稳压/卡上电需要几毫秒稳定期 */

	/* ★ 诊断（2026-10-05）：进门先打一次 Vref，这样【即使 CMD0 就失败】也能看到电压 ✓
	 *   （上一版把打印都放在 CMD0 成功之后 ✗ -> 卡失步时一行都看不到 ✓ 那样诊断等于没有 ✗）*/
	Serial_Printf("SD: begin, Vref=%u (1489=3.3V, bigger=lower)\r\n",
	              (unsigned)AD_ReadVrefint());

	/* ① 预热：CS 保持【高】（不选中）连发 ≥74 个时钟。
	 *    这是规范要求的上电序列 —— 卡靠这 74+ 个时钟同步内部状态机，
	 *    之后 CMD0 才有效。10 字节 = 80 clock，凑够还带余量 
	 	 *    ★★ 2026-10-10 加长：10 字节(80 clock) → 200 字节(1600 clock) ✗
	 *    为什么：★★ 热拔卡后重插，卡可能【悬停在半途】✗
	 *      （正在等上一条命令的剩余字节 ✓）
	 *      → 80 clock 不够冲干净 ✗ → 后续命令偶尔回垃圾 ✗
	 *        ★ 实测症状：CMD59 回"非法命令" → 误报 SD_ERR_UNSUPPORTED(03) ✗
	 *        ★ 而手动给卡断电重上电就正常 ✓ ← 正是"状态机没复位"的铁证 ✓✓
	 *    ★ 这是 SD 驱动的标准"软复位"做法 ✓（dummy clocks ✓）
	 *    ★ 代价：281kHz 下 200 字节 ≈ 5.7ms ✓（只在初始化时一次 ✓ 可接受 ✓）*/
	SD_CS(1);
	for (i = 0; i < 200; i++)
	{
		(void)SD_SwapByte(SD_DUMMY);
	}
	/* ★★★ 2026-10-12 修正（★ 我上一版改过头了 ✗）：
	 *   上一版我把它加到 800 字节 ✗ —— 【方向错了】✓
	 *   ★ SD 规范要求的是两个东西 ✗：
	 *     ① 【至少 74 个时钟 ✗】—— 200 字节 = 1600 个时钟 ✓ 【早就远超 ✓】
	 *     ② 【上电后 ≥1ms 的稳定时间 ✗】—— ★★ 【这才是真正缺的】✓✓
	 *   → ★★★ 所以时钟数改回 200 ✗ 只保留下面这个【真实延时】✓
	 *   ★★ 而且 800 字节还有个隐患 ✗：万一 SPI 异常 ✗
	 *     → 每次 SD_SwapByte 会跑满它的 100000 次超时循环 ✓
	 *     → 【字节数越多 ✗ 卡得越久】✓✓ */
	DWT_Delay_ms(10);                  /* ★ 给卡【真正的时间】去稳定 ✓ */

	/* ★ 2026-10-12 加打印（诊断 ✓）：确认软复位这段【过没过】✗
	 *   （★ 上一版卡在这里 ✗ 而它【一句打印都没有】✓ → 只能靠猜 ✓）*/
	Serial_Printf("SD: reset clocks done\r\n");

	/* ② CMD0 复位进 IDLE：期望 R1=0x01。
	 *    一次不成功很正常（卡刚上电还在走内部流程），最多重发 200 次；
	 *    200 次都无响应 = 没插卡 / 接线错 / 接触不良 */
	for (i = 0; i < SD_IDLE_RETRY; i++)
	{
		r1 = SD_SendCmd(CMD0, 0);
		SD_Release();
		if (r1 == R1_IDLE)
		{
			break;
		}
		{
			uint16_t k;
			SD_CS(1);
			for (k = 0; k < 32u; k++) { (void)SD_SwapByte(SD_DUMMY); }
		}
		DWT_Delay_ms(1);
	}
	if (r1 != R1_IDLE)
	{
		/* ★ 诊断：CMD0 失败也要给出【重试次数 + 当时的电压】✓ */
		Serial_Printf("SD: CMD0 NO RESPONSE (r1=0x%02X) after %u tries; Vref=%u\r\n",
		              r1, (unsigned)i, (unsigned)AD_ReadVrefint());
		return SD_ERR_NO_CARD;
	}

	/* ③ CMD8 探卡型：参数 0x1AA（电压窗 2.7-3.6V + 校验字 0xAA）。
	 *    ★ 三种应答必须分开处理（2026-10-09 修复，T2.6③ 演练抓到）✗✗
	 *      · R1 = 0x01 → 正常应答，继续读 R7 校验回显 → SDv2/v1 ✓
	 *      · R1 = 0x05（illegal command）→ ★ 这才是真正的 SDv1/MMC ✓
	 *      · 其它任何值（0x00 / 0xFF / 垃圾 ✓）→ ★★ 通信异常 ✗
	 *        实测证据：拔插卡过程中捕获到
	 *          `CMD8 R1=0x00 echo=00 00 is_sd2=0` → 旧代码判成 "assume SDv1/MMC" ✗
	 *          → ACMD41 连续收到垃圾（0xFF / 0xF7 / 0x1F ✓）→ 第 4 次撞上 0x00
	 *          → 打印 "ACMD41 ready after 4 tries" 但这是【假就绪】✗✓
	 *          → 卡被当成 v1 → 寻址方式可能错 → "读出来内容全错却不报错" ✗✗
	 *    旧代码的错：只打了一句 "assume SDv1/MMC" 就【继续往下走】✗ 不重试不返回 ✓
	 *    ★ 正确做法：异常就【判失败返回】，让上层 3 秒后重跑一次干净的初始化 ✓✓ */
	r1 = SD_SendCmd(CMD8, 0x000001AAu);
	if (r1 == R1_IDLE)
	{
		ocr[0] = SD_SwapByte(SD_DUMMY);     /* R7：R1 之后跟 4 字节 */
		ocr[1] = SD_SwapByte(SD_DUMMY);     /*   [2]=电压窗 [3]=校验字回显 */
		ocr[2] = SD_SwapByte(SD_DUMMY);
		ocr[3] = SD_SwapByte(SD_DUMMY);
		if ((ocr[2] == 0x01) && (ocr[3] == 0xAA))
		{
			is_sd2 = 1;
		}
		SD_Release();
		/* ★ 诊断打印：只有这个分支才是"卡正常应答"✓ 此时 ocr 才有意义 ✓ */
		Serial_Printf("SD: CMD0 ok; CMD8 R1=0x01 echo=%02X %02X is_sd2=%u\r\n",
		              ocr[2], ocr[3], is_sd2);
	}
	else if (r1 == 0x05u)                   /* ★ 非法命令 = 真 v1/MMC ✓ */
	{
		SD_Release();
		is_sd2 = 0;
		Serial_Printf("SD: CMD8 -> R1=0x05 (illegal cmd) => SDv1/MMC\r\n");
	}
	else                                    /* ★★ 其它一切 = 通信异常 ✗ */
	{
		SD_Release();
		Serial_Printf("SD: CMD8 ABNORMAL r1=0x%02X (expect 0x01 or 0x05) -> abort & retry\r\n",
		              r1);
		/* ★ 判失败返回：不带着"来路不明的状态"往下走 ✗✓
		 *   上层（SD_Init 的重试封装 / task_sd 的 3 秒重挂载）会重跑一次干净流程 ✓ */
		return SD_ERR_NO_CARD;
	}
	/* ★ 诊断基线：ACMD41 开跑之前的 VDD（raw 越大 = VDD 越低 ✓ 1489≈3.3V）*/
	Serial_Printf("SD: Vref baseline = %u\r\n", (unsigned)AD_ReadVrefint());

	/* ④ ACMD41 轮询直到就绪（R1 从 0x01 变 0x00）。
	 *    SDv2 的参数带 HCS=1（bit30）：告诉卡"主机支持 SDHC"，
	 *    卡支持高容量就会在就绪后把 OCR 的 CCS 位置 1。
	 *    ★ MMC 卡走 CMD1，本驱动不支持（执行表未要求，市面已绝迹）——
	 *      MMC 在这里会一直返回非法命令直到超时 → SD_ERR_INIT_TIMEOUT */
	t_start = DWT->CYCCNT;
	do
	{
		if (is_sd2)
		{
			/* ★★★ 2026-10-05 修复：ACMD41 的参数必须带上【电压窗口】✗✗
			 *   原来发的是 `1UL << 30` = 0x40000000 —— 只有 HCS，电压窗口位(15~23)全 0 ✗
			 *   而 ACMD41 的参数语义 = "主机的 OCR"，规范要求声明支持的电压范围 ✓
			 *   发全 0 = "我支持 0V" -> 卡回【参数错误】✗
			 *   实测证据：ACMD41 第一发就回 0xE0 = bit6(parameter error)+bit5(address error) ✓✓
			 *   改成 0x40FF8000 = HCS(bit30) + 0xFF8000(2.7~3.6V 电压窗口) —— 规范推荐值 ✓✓ */
			r1 = SD_SendAcmd(ACMD41, 0x40FF8000u);
		}
		else
		{
			/* v1/MMC 分支：不带 HCS，但【电压窗口仍要带】✓ */
			r1 = SD_SendAcmd(ACMD41, 0x00FF8000u);
		}

		/* ★★★ 2026-10-05 修复：ACMD41 轮询必须【每次都收尾】✗✗
		 *   SD_SendCmd 的设计是"拉低 CS 不抬，收尾交给调用者"（见行 185），
		 *   其它每个调用点后面都跟了 SD_Release() ✓ —— 唯独这里漏了 ✗
		 *   后果：CMD55+ACMD41 被连续重复 2000+ 次，CS 一直按着、中间没有那 8 个时钟间隙
		 *     -> 卡第一发就回垃圾（实测 0xE0 ✗，R1 非法），随即彻底失步、全 0xFF ✓
		 *     -> 而且失步会【留在卡里】：下一次 SD_Init 连 CMD0 都不答，必须断电重上电才恢复 ✓
		 *        （这正是"每次都是刚给模块接上电时才走得远"的原因 ✓）
		 *   SD_Release 的注释（行 141-144）早就写了这条规律："缺了这 8 个时钟，
		 *   会出现'下一条命令偶发无响应'这种查一晚上不出来的玄学问题" —— 就是本案 ✓ */
		SD_Release();

		tries++;
		if (tries <= 5u)
		{
			Serial_Printf("SD: ACMD41 #%u R1=0x%02X\r\n", (unsigned)tries, r1);
		}
		/* ★ 诊断（2026-10-05）：每 200 次读一次内部基准 —— 直接看 ACMD41 期间 VDD 塌没塌 ✓
		 *   这一步的价值：不靠万用表、不受表笔/地影响 ✓（注入通道读，对 ADC 常规组无影响 ✓）
		 *   判据：与上面 baseline 比 —— 明显变大（如 1495 -> 2000+）= 电压在掉 ✓✓ */
		if ((tries % 200u) == 0u)
		{
			Serial_Printf("SD:   tries=%u R1=0x%02X Vref=%u\r\n",
			              (unsigned)tries, r1, (unsigned)AD_ReadVrefint());
		}
		if ((DWT->CYCCNT - t_start) > ((uint32_t)SD_INIT_TIMEOUT_MS * 72000u))
		{
			Serial_Printf("SD: ACMD41 TIMEOUT after %u tries, last R1=0x%02X Vref=%u\r\n",
			              (unsigned)tries, r1, (unsigned)AD_ReadVrefint());
			return SD_ERR_INIT_TIMEOUT;
		}
	} while (r1 != R1_READY);
	Serial_Printf("SD: ACMD41 ready after %u tries\r\n", (unsigned)tries);

	/* ⑤ CMD58 读 OCR 定寻址方式：CCS=bit30 → 1=SDHC 块寻址 / 0=字节寻址
	 *
	 *  ★★★★ 2026-10-13 修（dsh）：CMD58 加重试 ✗✓
	 *
	 *  ★ 现象（实测日志 ✓）：
	 *    ```
	 *    SD: ACMD41 ready after 3 tries
	 *    SD: CMD58 R1=0x04 (expect 0x00)      ← ★ 撞在这里
	 *    SD: [1] f_mount FAILED fr=3           ← FR_NOT_READY
	 *    ```
	 *    → ★★ 【整个挂载失败 ✗ 上层【报一串错 ✗ 然后【重试【又好了】】✓
	 *    → ★★★ 用户观察到的 `00→03→01→00` / `00→09→00` ✗
	 *      【就是这一个根因【演化出来的一串**】✓✓
	 *      （★ 03/01/09 是 **FatFs 的 `fr`** ✗ 不是 SD 驱动的错误码 ✓）
	 *
	 *  ★★ 根因 ✗：
	 *    · ★ ACMD41 刚回 `0x00` 的那一瞬间 ✗✓
	 *      【卡【其实【还没【完全进 SPI 模式 / 内部还没收拾好】】✓
	 *    · ★★ 【于是【它对【紧跟其后的命令【回 `0x04`（ILLEGAL_CMD）】】✓
	 *    → ★★★ 【★ 这【不是"卡不认 CMD58"✗ 而是"【等会儿再问】"】】✓✓
	 *
	 *  ★★★ 这和 CMD59 是【同一个病】✗ 所以修法也一样 ✓
	 *
	 *  ★★★★ 做法：失败就【歇 10ms 再试 ✗ 最多 5 次】✓
	 *    （★ 5 × 10ms = 50ms ✗ 远小于 ACMD41 那 2 秒的预算 ✓✓）*/
	for (i = 0u; i < 5u; i++)
	{
		r1 = SD_SendCmd(CMD58, 0);

		if (r1 == R1_READY)
		{
			break;                          /* ★ 成功 ✗ 跳出 ✗ 往下读 OCR ✓ */
		}

		/* ★ 失败 ✗ 先把总线收回命令态 ✗ 歇一下再来 ✓
		 *   ★★ 为什么【必须 `SD_Release()`**✗：
		 *     它【补那 8 个时钟 ✗ 把卡【拽回命令态】✓
		 *     （★ 缺了这一步✗ 会出现"下一条命令偶发无响应"✗
		 *      见文件里行 141-144 的注释 ✓）*/
		SD_Release();
		Serial_Printf("SD: CMD58 try%u R1=0x%02X, retry...\r\n",
		              (unsigned)(i + 1u), r1);
		DWT_Delay_ms(10);
	}

	if (r1 != R1_READY)
	{
		/* ★ 5 次都没成 ✗ 才认输 ✓（★ 这时【才是【真的有问题】✓）*/
		Serial_Printf("SD: CMD58 FAILED after 5 retries, R1=0x%02X\r\n", r1);
		SD_Release();
		return SD_ERR_CMD_FAILED;
	}

	/* ★★ 注意 ✗ 只有【成功】才【读 OCR 那 4 个字节】✓
	 *   （★ 重试的失败分支【绝对不能【读字节 ✗ 那里【没有数据】✓）*/
	ocr[0] = SD_SwapByte(SD_DUMMY);         /* OCR 大端 4 字节，CCS 在 [0] 的 bit6 */
	ocr[1] = SD_SwapByte(SD_DUMMY);
	ocr[2] = SD_SwapByte(SD_DUMMY);
	ocr[3] = SD_SwapByte(SD_DUMMY);
	SD_Release();

	if (is_sd2)
	{
		s_card_type = (ocr[0] & 0x40) ? SD_CT_SDHC : SD_CT_V2;
	}
	else
	{
		s_card_type = SD_CT_V1;
	}

	/* ⑥ 块长 512：V1/V2 卡需要显式设置；SDHC 固定 512 会忽略此命令
	 *
	 *  ★★★ 2026-10-13 加（dsh）：重试 ✗ 和 CMD58 同一个病 ✓
	 *    · ★ 卡【刚进 SPI 模式【还没收拾好】时 ✗ 会对命令回 0x04 ✓
	 *    · ★★ 重试 5 次（★ 50ms ✓）就够 ✗ 而【原本【一次失败就【整个挂载失败】】✓✓ */
	if (s_card_type != SD_CT_SDHC)
	{
		for (i = 0u; i < 5u; i++)
		{
			r1 = SD_SendCmd(CMD16, 512);

			if (r1 == R1_READY)
			{
				SD_Release();               /* ★ 成功 ✗ 正常收尾 ✓ */
				break;
			}

			SD_Release();                   /* ★ 失败 ✗ 拽回命令态再试 ✓ */
			Serial_Printf("SD: CMD16 try%u R1=0x%02X, retry...\r\n",
			              (unsigned)(i + 1u), r1);
			DWT_Delay_ms(10);
		}

		if (r1 != R1_READY)
		{
			Serial_Printf("SD: CMD16 FAILED after 5 retries, R1=0x%02X\r\n", r1);
			return SD_ERR_CMD_FAILED;
		}
	}

	/* ⑦ 关卡内数据 CRC 校验（CMD59 参数 bit0=0）：之后数据块的 CRC 字节
	 *    全部用 0xFF 哑元（原理见文件头注释）。命令 CRC 照旧现算，最稳
	 *
	 *  ★★★ 2026-10-13 加（dsh）：重试 ✗ 和 CMD58 / CMD16 同一个病 ✓
	 *    · ★ 原来【一次失败就返回】✗ 于是【偶发一次卡顿【整个挂载就废了】】✓✓ */
	for (i = 0u; i < 5u; i++)
	{
		r1 = SD_SendCmd(CMD59, 0);

		if (r1 == R1_READY)
		{
			SD_Release();                   /* ★ 成功 ✗ 正常收尾 ✓ */
			break;
		}

		SD_Release();                       /* ★ 失败 ✗ 拽回命令态再试 ✓ */
		Serial_Printf("SD: CMD59 try%u R1=0x%02X, retry...\r\n",
		              (unsigned)(i + 1u), r1);
		DWT_Delay_ms(10);
	}

	/* ★★★ 判据修正（和 SD_SendCmd 的修复配套 ✓）：
	 *   · 0xFF = 【卡无响应】✗ 不是"不认这条命令" ✓ —— 必须分开判 ✓
	 *     （★ 原来的 `r1 & 0x04` 对 0xFF 也成立 ✗ → 误报 03 ✓✓）
	 *   · 只有【合法 R1 且 bit2=1】才是真的"卡不支持 CMD59" ✓ */
	if (r1 == 0xFFu)
	{
		Serial_Printf("SD: CMD59 NO RESPONSE after 5 tries (card busy/slow) -> CMD_FAILED\r\n");
		return SD_ERR_CMD_FAILED;
	}
	if (r1 & R1_ILLEGAL_CMD)
	{
		/* 极个别老卡不认 CMD59 → 数据 CRC 哑元会被它拒收，只能判失败。
		 * 真遇到（几乎不可能）再补 CRC16 运算，现阶段不做 */
		Serial_Printf("SD: CMD59 ILLEGAL after 5 tries (card really rejects it) -> UNSUPPORTED\r\n");
		return SD_ERR_UNSUPPORTED;
	}

	/* ⑧ 初始化通过，切 9MHz 高速干活 */
	SD_SetSpeed(SD_FAST_SPEED);

	ret = SD_OK;
	return ret;
}

/**
  * @brief  读 1~N 块（每块 512B，buf 至少 count*512 字节）
  *         整操作带 3 次重试：单次失败后释放总线、补时钟、歇 2ms 再来
  * @param  buf    : 数据缓冲
  * @param  sector : 起始块号（512B 单位，FatFs 的扇区号直通）
  * @param  count  : 块数（≥1）
  * @retval SD_Result_t
  */
SD_Result_t SD_ReadBlocks(uint8_t *buf, uint32_t sector, uint32_t count)
{
	SD_Result_t r = SD_ERR_PARAM;
	uint8_t     attempt;

	if ((buf == 0) || (count == 0))
	{
		return SD_ERR_PARAM;
	}

	for (attempt = 0; attempt < SD_OP_RETRY; attempt++)
	{
		r = SD_ReadRaw(buf, sector, count);
		if (r == SD_OK)
		{
			return SD_OK;
		}
		/* 失败现场清理：确保 CS 高 + 补 8 时钟，把卡拽回命令态再试 */
		SD_Release();
		DWT_Delay_ms(2);
	}
	return r;
}

/**
  * @brief  写 1~N 块（每块 512B，buf 至少 count*512 字节）
  *         整操作带 3 次重试（执行表 T2.1：上限 3 次，超了上浮错误，
  *         由 diskio → task_sd 丢弃本批，不允许在这里无限死等）
  * @param  buf    : 数据缓冲
  * @param  sector : 起始块号（512B 单位）
  * @param  count  : 块数（≥1）
  * @retval SD_Result_t
  */
SD_Result_t SD_WriteBlocks(const uint8_t *buf, uint32_t sector, uint32_t count)
{
	SD_Result_t r = SD_ERR_PARAM;
	uint8_t     attempt;

	if ((buf == 0) || (count == 0))
	{
		return SD_ERR_PARAM;
	}

	for (attempt = 0; attempt < SD_OP_RETRY; attempt++)
	{
		r = SD_WriteRaw(buf, sector, count);
		if (r == SD_OK)
		{
			return SD_OK;
		}
		SD_Release();
		DWT_Delay_ms(2);
	}
	return r;
}

/**
  * @brief  取当前卡类型（SD_CT_xxx），供 disk_ioctl(MMC_GET_TYPE) 与调试用
  */
uint8_t SD_GetCardType(void)
{
	return s_card_type;
}

/**
  * @brief  错误码 → 可读字符串（串口报错用，不占 RAM 常驻）
  */
const char *SD_ErrorStr(SD_Result_t err)
{
	switch (err)
	{
	case SD_OK:                 return "OK";
	case SD_ERR_NO_CARD:        return "NO CARD";
	case SD_ERR_INIT_TIMEOUT:   return "INIT TIMEOUT";
	case SD_ERR_UNSUPPORTED:    return "UNSUPPORTED";
	case SD_ERR_CMD_FAILED:     return "CMD FAILED";
	case SD_ERR_READ_TIMEOUT:   return "READ TIMEOUT";
	case SD_ERR_READ_CRC:       return "READ CRC";
	case SD_ERR_WRITE_RESPONSE: return "WRITE REJECTED";
	case SD_ERR_WRITE_TIMEOUT:  return "WRITE TIMEOUT";
	case SD_ERR_PARAM:          return "PARAM";
	default:                    return "?";
	}
}

/*==============================================================================
 *  ★ 硬件排查探针（调试用，收尾时整段删掉 ✓）
 *    来源：_临时脚本\SD_排查探针.md（原版已存档 ✓ 这里只加了一处小改进：BitBang 开头
 *          先调 SD_LowLevelInit() 保证 SPI1 时钟/引脚是初始化好的 ✓）
 *    为什么必须贴在【本文件内部】：它们用到 SD_LowLevelInit()/SD_CS()/SD_SwapByte()，
 *          这三个是本文件的 static 函数 ✓ 放别的 .c 文件里链接不到 ✓
 *    ★ 字符串全部 ASCII ✓（工程铁律：串口字符串里不许有中文 ✗ 会乱码 ✓）
 *============================================================================*/

/* ① 回环自检：拔掉模块 4 根信号线，用杜邦线短接 PA6<->PA7，再调用
 *    判读：5 行全 OK = MCU 侧 SPI 完全清白 ✓；全 FF = 短接没接好 ✗ */
#if PROBE_SD_LOOPBACK
void SD_LoopbackTest(void)
{
	uint8_t tx[5] = {0xA5, 0x55, 0x3C, 0x00, 0xFF};
	uint8_t rx[5];
	uint8_t i;

	SD_LowLevelInit();
	SD_CS(1);
	for (i = 0; i < 10; i++) { (void)SD_SwapByte(0xFF); }
	SD_CS(0);
	for (i = 0; i < 5; i++) { rx[i] = SD_SwapByte(tx[i]); }
	SD_CS(1);

	Serial_Printf("---- LOOPBACK (PA6 <-> PA7 shorted) ----\r\n");
	for (i = 0; i < 5; i++)
	{
		Serial_Printf("  tx=%02X -> rx=%02X  %s\r\n",
		              tx[i], rx[i], (tx[i] == rx[i]) ? "OK" : "<-- MISMATCH");
	}
	Serial_Printf("---- end ----\r\n");
}
#endif

/* ② 引脚识别：PA5 每 1s 翻 / PA7 每 2s / PA4 每 4s（死循环，不返回）
 *    用法：万用表直流档，逐个量【模块端】的 CS/SCK/MOSI 三个针
 *      1s 跳 = 接的是 PA5（应是模块 SCK）
 *      2s 跳 = 接的是 PA7（应是模块 MOSI）
 *      4s 跳 = 接的是 PA4（应是模块 CS）
 *    ★ 这个探针的意义：绕开丝印目视，直接用电器行为确认「哪根线接哪个脚」 */
 #if PROBE_SD_PINFINDER
void SD_PinFinder(void)
{
	GPIO_InitTypeDef g;
	uint32_t tick = 0;
	uint8_t i;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	g.GPIO_Pin   = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7;
	g.GPIO_Mode  = GPIO_Mode_Out_PP;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &g);

	Serial_Printf("PIN FINDER: PA5=1s, PA7=2s, PA4=4s\r\n");
	for (;;)
	{
		GPIO_WriteBit(GPIOA, GPIO_Pin_5, (BitAction)(tick & 1));
		GPIO_WriteBit(GPIOA, GPIO_Pin_7, (BitAction)((tick >> 1) & 1));
		GPIO_WriteBit(GPIOA, GPIO_Pin_4, (BitAction)((tick >> 2) & 1));
		Serial_Printf("tick=%lu  PA5=%u  PA7=%u  PA4=%u\r\n",
		              (unsigned long)tick, (unsigned)(tick & 1),
		              (unsigned)((tick >> 1) & 1), (unsigned)((tick >> 2) & 1));
		for (i = 0; i < 4; i++) { DWT_Delay_ms(250); }
		tick++;
	}
}
#endif
/* ③ MISO 通路：持续读 PA6 并打印。人工把【模块的 MISO 针】依次接 GND / 3.3V
 *    接 GND -> 0x00；接 3.3V -> 0xFF；一直不变 = MISO 线不通 ✗
 *    ⚠️ 模块若带缓冲器，必须【串 1k~10k 电阻】再接地（防输出对打）*/
 #if PROBE_SD_MISO
void SD_MisoProbe(void)
{
	uint8_t v;

	SD_LowLevelInit();
	SD_CS(1);

	Serial_Printf("MISO PROBE RUNNING: tie module MISO pin to GND / 3.3V\r\n");
	for (;;)
	{
		v = SD_SwapByte(0xFF);
		Serial_Printf("miso=0x%02X\r\n", v);
		DWT_Delay_ms(300);
	}
}
#endif
/* ④ ★ 位翻转极低速 CMD0（~50kHz，绕开 SPI 外设）—— 判别力最强、且【不用动任何线】
 *    低速能答 R1=0x01  -> 硬件 SPI 那条路没错，是信号完整性（线长/干扰/接触）✗
 *    低速也全 0xFF     -> 排除速度因素，锁定电源/线/卡/模块 ✓
 *    跑完自动恢复硬件 SPI ✓ 然后正常流程继续跑（所以一次烧录能同时看到它和 SD_Init 的结果 ✓）*/
 #if PROBE_SD_BITBANG
static void BB_WriteByte(uint8_t b)
{
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_RESET);
		GPIO_WriteBit(GPIOA, GPIO_Pin_7, (BitAction)(b & 0x80));
		DWT_Delay_us(10);
		GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_SET);
		DWT_Delay_us(10);
		b <<= 1;
	}
}

static uint8_t BB_ReadByte(void)
{
	uint8_t i, v = 0;
	for (i = 0; i < 8; i++)
	{
		GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_RESET);
		DWT_Delay_us(10);
		GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_SET);
		DWT_Delay_us(5);
		v <<= 1;
		if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6)) { v |= 1; }
		DWT_Delay_us(5);
	}
	return v;
}

void SD_BitBangProbe(void)
{
	uint8_t  i, n, r1, got = 0;
	uint8_t  frame[6] = {0x40, 0x00, 0x00, 0x00, 0x00, 0x95};
	GPIO_InitTypeDef g;

	SD_LowLevelInit();                                /* ★ 保证 SPI1 时钟/引脚已初始化 ✓ */
	SPI_Cmd(SPI1, DISABLE);                           /* 外设放手，改成软件翻转 */
	g.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_7;
	g.GPIO_Mode  = GPIO_Mode_Out_PP;
	g.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &g);

	SD_CS(1);
	for (i = 0; i < 10; i++) { BB_WriteByte(0xFF); }  /* 80 时钟预热 */

	for (n = 0; n < 3; n++)
	{
		SD_CS(0);
		for (i = 0; i < 6; i++) { BB_WriteByte(frame[i]); }
		r1 = 0xFF;
		for (i = 0; i < 16; i++)
		{
			r1 = BB_ReadByte();
			if (r1 != 0xFF) { break; }
		}
		SD_CS(1);
		(void)BB_WriteByte(0xFF);

		/* ★ 判据修正（2026-10-05）：原来"任何非 0xFF 都算 answered" ✗ 会把噪声当真应答
		 *   （实测遇到过：卡都不插也报 "card answered"，因为模块在没卡时会把 MISO 拉成 0x00 ✗）
		 *   真正的 CMD0 应答只有一个值：R1 = 0x01（bit0=1「idle 状态」✓ 且 bit7 必须为 0 ✓）
		 *   0xC1 这种 bit7=1 的 → R1 定义上不可能 ✗ 是噪声/失步 ✓ */
		Serial_Printf("BitBang CMD0 #%u: R1=0x%02X %s\r\n",
		              (unsigned)(n + 1), r1,
		              (r1 == 0x01) ? "<-- VALID CMD0 answer (idle)!" :
		              (r1 == 0xFF) ? "(no resp)" :
		              ((r1 & 0x80) ? "(INVALID: bit7 set -> noise/desync)" : "(responded, not idle)"));
		if (r1 == 0x01) { got = 1; }
		DWT_Delay_ms(2);
	}

	SD_LowLevelInit();
	Serial_Printf("BitBang verdict: %s\r\n",
	              got ? "card answered 0x01 -> CARD ALIVE; trouble = signal integrity (wire/contact/noise)"
	                  : "no valid 0x01 -> power / wire / card / module  (mode mismatch not ruled out)");
}
#endif
/*==============================================================================
 *  ★★ 逐字节"慢节奏"初始化诊断（2026-10-05 加）—— 定位到底哪一步失步
 *  与 SD_Init 的唯一区别（故意的）：
 *    ① 每步之间都 SD_Release()（抬 CS + 补 8 时钟）✓
 *    ② 每步之间插 2ms 延时 ✓
 *    ③ ACMD41 那一步【多读 8 个字节】全部打出来 ✓✓
 *    ④ CMD55 的 R1 单独打出来（原驱动把它丢了 ✗ —— 失步是从 CMD55 就有的吗？）
 *  判读：
 *    · 这个慢版本【能过】-> 问题在【节奏太紧】（背靠背命令 / 无间隙）
 *    · 这个慢版本【也过不了】-> 问题不在节奏，看它吐的具体字节
 *    · ACMD41 那 8 个字节：全 FF=没响应 / 01=忙 / 00=就绪 / 其他=垃圾(看什么形状)
 *  用法：User\ProbeConfig.h 里把 PROBE_SD_VERBOSE 置 1 ✓（它自己跑，不调 SD_Init ✓）
 *============================================================================*/
 #if PROBE_SD_VERBOSE
void SD_VerboseInit(void)
{
	uint8_t r1, i, k;

	DWT_Delay_Init();
	SD_LowLevelInit();
	DWT_Delay_ms(SD_PWR_SETTLE_MS);

	SD_CS(1);
	for (i = 0; i < 10; i++) { (void)SD_SwapByte(0xFF); }   /* 80 时钟预热 ✓ */

	/* --- ① CMD0 --- */
	r1 = SD_SendCmd(CMD0, 0);
	Serial_Printf("V: CMD0  -> R1=0x%02X (expect 0x01)\r\n", r1);
	SD_Release();
	DWT_Delay_ms(2);

	/* --- ② CMD8 --- */
	r1 = SD_SendCmd(CMD8, 0x1AA);
	Serial_Printf("V: CMD8  -> R1=0x%02X (expect 0x01)", r1);
	if (r1 == 0x01)
	{
		uint8_t e[4];
		for (i = 0; i < 4; i++) { e[i] = SD_SwapByte(0xFF); }
		Serial_Printf("  echo=%02X %02X %02X %02X (expect 00 00 01 AA)", e[0], e[1], e[2], e[3]);
	}
	Serial_Printf("\r\n");
	SD_Release();
	DWT_Delay_ms(2);

	/* --- ③ CMD55（单独看它的 R1 ✓ 原驱动丢了这条信息 ✗）--- */
	r1 = SD_SendCmd(CMD55, 0);
	Serial_Printf("V: CMD55 -> R1=0x%02X (expect 0x01)\r\n", r1);
	SD_Release();
	DWT_Delay_ms(2);                       /* ★ 故意插延时：如果这样就能过，说明是"太快" ✓ */

	/* --- ④ ACMD41：读 8 个字节全打出来 --- */
	SD_CS(0);
	{
		uint8_t frame[6];
		frame[0] = (uint8_t)(0x40 | ACMD41);
		frame[1] = 0x40; frame[2] = 0xFF; frame[3] = 0x80; frame[4] = 0x00;  /* ★ 0x40FF8000：HCS + 电压窗口 ✓ */
		frame[5] = SD_Crc7(frame, 5);
		for (i = 0; i < 6; i++) { (void)SD_SwapByte(frame[i]); }

		Serial_Printf("V: ACMD41 bytes:");
		for (k = 0; k < 8; k++)
		{
			Serial_Printf(" %02X", SD_SwapByte(0xFF));
		}
		Serial_Printf("   (0xFF=no resp / 01=busy / 00=ready / else=garbage)\r\n");
	}
	SD_Release();

	Serial_Printf("V: done\r\n");
}
#endif
/*==============================================================================
 *  ★★ 扩容假卡检测（只读 ✓ 绝对安全 ✓ 2026-10-05 加）
 *  原理：扩容假卡的高地址会【回绕】到开头 ✗
 *        -> 在很深的扇区读回来的，其实是【开头扇区】的内容 ✓
 *  硬判据：扇区 0 的 510/511 是 MBR 签名 `55 AA` ✓
 *          如果【很深】的扇区也出现 `55 AA` ✗✗
 *            -> 那里其实是扇区 0 的副本 -> 卡在回绕 -> 【假卡】✓✓✓
 *          （真卡上，2GB/3GB 深处应该是文件数据 ✓ 不可能又是一个 MBR ✓）
 *  探测点：0（基准）、1GB、2GB、3GB、分区后第一扇区、800万（远超任何小卡）
 *          ★ 1GB = 2,097,152 扇区 / 2GB = 4,194,304 / 4GB = 8,388,608
 *  为什么要错开多个点：真实容量未知 ✓ 多几个点才能看出"从哪开始回绕" ✓
 *  前提：调用前卡必须已初始化 ✓（本函数不调 SD_Init ✓）
 *  用法：main.c 里在 SD 测试之后调一次 ✓
 *============================================================================*/
 #if PROBE_SD_FAKECARD
void SD_FakeCardProbe(void)
{
	static uint8_t buf[512];              /* ★ 512B 必须 static：栈上放不下 ✓ */
	uint32_t pts[6];
	uint32_t npts = 6;
	uint8_t  k;
	SD_Result_t rr;

	pts[0] = 0u;              /* 基准：这是 MBR ✓ 应有 55 AA */
	pts[1] = 2097152u;        /* 1GB */
	pts[2] = 4194304u;        /* 2GB */
	pts[3] = 6291456u;        /* 3GB */
	pts[4] = 7744512u;        /* 正好是分区之后第一个扇区 ✓ */
	pts[5] = 8000000u;        /* 远超 3.78GB ✓ */

	Serial_Printf("---- fake-card probe (read only) ----\r\n");

	for (k = 0; k < npts; k++)
	{
		uint8_t j;

		for (j = 0; j < 16; j++) { buf[j] = 0x00; }    /* 清头部：防"没读到、缓冲里还是旧的" ✗ */
		buf[510] = 0x00;                               /* 清签名位：防旧值残留 ✗ */
		buf[511] = 0x00;

		rr = SD_ReadBlocks(buf, pts[k], 1);
		Serial_Printf("sec %lu = %d (%s)  ",
		              (unsigned long)pts[k], (int)rr, SD_ErrorStr(rr));

		if (rr == SD_OK)
		{
			Serial_Printf("head:");
			for (j = 0; j < 8; j++) { Serial_Printf(" %02X", buf[j]); }
			Serial_Printf("  sig=%02X %02X%s\r\n", buf[510], buf[511],
			              ((k > 0u) && (buf[510] == 0x55u) && (buf[511] == 0xAAu))
			                  ? "  <== MBR deep inside!! SUSPECT WRAP-AROUND"
			                  : "");
			/* ★ 注意（2026-10-05 教训）：多处分区"内容相同"【不能】当回绕证据 ✗
			 *   如果写进去的图案本身是重复的（如 0x5A^(i&0xFF) 每 256 字节一轮 ✓）
			 *   -> 任何相隔 256 倍数的扇区开头都会一样 ✓ 那是图案造成的假象 ✗
			 *   验假卡请用【PC 侧标记文件法】：写满全卡后看开头的标记有没有被覆盖 ✓✓ */
		}
		else
		{
			Serial_Printf("(read failed -> this address may not exist)\r\n");
		}
	}

	/* ★ 串口字符串一律 ASCII ✗ —— 2026-10-05 我在这里又写过一次中文，串口打出来是乱码 ✓
	 *   （工程铁律：字符串字面量里不许有中文 ✓ 中文只能写进注释 ✓）*/
	Serial_Printf("---- end (real card: only sec 0 has 55 AA at 510) ----\r\n");
}
#endif
