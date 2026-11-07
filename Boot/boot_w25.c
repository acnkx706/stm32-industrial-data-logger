#include "stm32f10x.h"
#include "boot_config.h"
#include "boot_w25.h"

/*==============================================================================
 *  ★ Bootloader 的 W25Q64 驱动（2026-10-14 建，2026-11-07 加写/擦）
 *
 *  ★ 只读 → 可写，为什么变了：
 *    · 原来所有"写 W25"的活都在 App 侧，Boot 只负责把金副本搬回内部
 *    · 现在升级改成"先落盘再切换"：新固件先写进 W25 暂存区，
 *      校验通过后才动内部 Flash ⇒ 传输中途断电不会把 App 弄坏
 *    · 这条路径只能在 Boot 里做（那时 App 还没跑）
 *    → 于是补上页写和扇区擦；引脚、时序、和 App 侧完全一致
 *
 *  ★ 软件 SPI（GPIO 手动打拍，mode 0）
 *    · 为什么不内联 BSRR，用 GPIO_WriteBit
 *      → 和 App 侧同一个选择，故意慢一点
 *      → 实际 SCK 约 1~2MHz，面包板/短线更稳
 *
 *  ★ 本文件不加锁 —— 因为Bootloader 是裸机单线程
 *    （没有任务，没有中断里碰它，无并发可言）
 *============================================================================*/

#define W25_PORT_SCK     GPIOA
#define W25_PORT_MISO    GPIOA
#define W25_PORT_MOSI    GPIOA
#define W25_PORT_CS      GPIOB

#define W25_PIN_SCK      GPIO_Pin_8
#define W25_PIN_MISO     GPIO_Pin_11
#define W25_PIN_MOSI     GPIO_Pin_12
#define W25_PIN_CS       GPIO_Pin_13

#define W25_INS_READ_DATA    0x03u
#define W25_INS_JEDEC_ID     0x9Fu
#define W25_INS_WRITE_ENABLE 0x06u
#define W25_INS_PAGE_PROGRAM 0x02u
#define W25_INS_SECTOR_ERASE 0x20u
#define W25_INS_READ_STATUS1 0x05u
#define W25_DUMMY            0xFFu
#define W25_PAGE_SIZE        256u

#define W25_SIZE             (8u * 1024u * 1024u)

static void W25_CS(uint8_t v)   { GPIO_WriteBit(W25_PORT_CS,   W25_PIN_CS,   (BitAction)v); }
static void W25_SCK(uint8_t v)  { GPIO_WriteBit(W25_PORT_SCK,  W25_PIN_SCK,  (BitAction)v); }
static void W25_MOSI(uint8_t v) { GPIO_WriteBit(W25_PORT_MOSI, W25_PIN_MOSI, (BitAction)v); }
static uint8_t W25_MISO(void)   { return (uint8_t)GPIO_ReadInputDataBit(W25_PORT_MISO, W25_PIN_MISO); }

/* ★ mode 0：MOSI 在 SCK 低时摆好，上升沿采样 MISO。 */
static uint8_t W25_SwapByte(uint8_t out)
{
	uint8_t i;
	uint8_t in = 0u;

	for (i = 0u; i < 8u; i++)
	{
		W25_MOSI((out & 0x80u) ? 1u : 0u);
		out = (uint8_t)(out << 1u);
		W25_SCK(1u);
		if (W25_MISO() != 0u)
		{
			in |= (uint8_t)(0x80u >> i);
		}
		W25_SCK(0u);
	}
	return in;
}

static void W25_SendAddr(uint32_t addr)
{
	W25_SwapByte((uint8_t)(addr >> 16));
	W25_SwapByte((uint8_t)(addr >> 8));
	W25_SwapByte((uint8_t)(addr));
}

uint32_t BootW25_ReadJedec(void)
{
	uint32_t id;

	W25_CS(0u);
	W25_SwapByte(W25_INS_JEDEC_ID);
	id  = (uint32_t)W25_SwapByte(W25_DUMMY) << 16;
	id |= (uint32_t)W25_SwapByte(W25_DUMMY) << 8;
	id |= (uint32_t)W25_SwapByte(W25_DUMMY);
	W25_CS(1u);
	return id;
}

uint8_t BootW25_Read(uint32_t addr, uint8_t *buf, uint32_t len)
{
	uint32_t i;

	if ((addr + len) > W25_SIZE) { return 0u; }

	W25_CS(0u);
	W25_SwapByte(W25_INS_READ_DATA);
	W25_SendAddr(addr);
	for (i = 0u; i < len; i++)
	{
		buf[i] = W25_SwapByte(W25_DUMMY);
	}
	W25_CS(1u);
	return 1u;
}

/*==============================================================================
 *  ★ 写与擦（2026-11-07 加，给升级暂存区用）
 *
 *  ★ 为什么必须等 WIP：
 *    · W25 的页写约 0.7ms、扇区擦约 45ms，期间除了读状态寄存器都不认命令
 *    · 不等就发下一条命令，会被静默丢弃 —— 而且不报错
 *    · 所以每次写完都轮询 status1 的 bit0，等它自己落下
 *
 *  ★ 为什么留 guard：
 *    · 芯片不在位时 MISO 上拉读 0xFF，bit0 永远是 1
 *      ⇒ 没有上限就是一个死循环，而 Bootloader 里死循环 = 变砖
 * ============================================================================*/
static void W25_WaitBusy(void)
{
	uint8_t  st;
	uint32_t guard = 0u;

	W25_CS(0u);
	W25_SwapByte(W25_INS_READ_STATUS1);
	do
	{
		st = W25_SwapByte(W25_DUMMY);
		guard++;
	} while (((st & 0x01u) != 0u) && (guard < 4000000u));
	W25_CS(1u);
}

static void W25_WriteEnable(void)
{
	W25_CS(0u);
	W25_SwapByte(W25_INS_WRITE_ENABLE);
	W25_CS(1u);
}

uint8_t BootW25_EraseSector(uint32_t addr)
{
	if ((addr % 4096u) != 0u) { return 0u; }   /* 必须扇区对齐 */
	if (addr >= W25_SIZE)     { return 0u; }

	W25_WriteEnable();
	W25_CS(0u);
	W25_SwapByte(W25_INS_SECTOR_ERASE);
	W25_SendAddr(addr);
	W25_CS(1u);
	W25_WaitBusy();

	return 1u;
}

uint8_t BootW25_Write(uint32_t addr, const uint8_t *buf, uint32_t len)
{
	uint32_t i = 0u;

	if ((addr + len) > W25_SIZE) { return 0u; }
	if (len == 0u)               { return 1u; }

	while (i < len)
	{
		uint32_t room;
		uint32_t n;
		uint32_t k;

		/* ★ 一条 Page Program 不能跨 256B 页边界，这里自动切 */
		room = W25_PAGE_SIZE - ((addr + i) & (W25_PAGE_SIZE - 1u));
		n    = len - i;
		if (n > room) { n = room; }

		W25_WriteEnable();
		W25_CS(0u);
		W25_SwapByte(W25_INS_PAGE_PROGRAM);
		W25_SendAddr(addr + i);
		for (k = 0u; k < n; k++)
		{
			W25_SwapByte(buf[i + k]);
		}
		W25_CS(1u);
		W25_WaitBusy();

		i += n;
	}

	return 1u;
}

uint8_t BootW25_Init(void)
{
	GPIO_InitTypeDef gi;

	/* ★ 两个端口都要开时钟，SCK/MISO/MOSI 在 A，CS 在 B。 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	gi.GPIO_Mode  = GPIO_Mode_Out_PP;
	gi.GPIO_Pin   = W25_PIN_SCK | W25_PIN_MOSI;
	gi.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(W25_PORT_SCK, &gi);

	gi.GPIO_Pin   = W25_PIN_CS;
	GPIO_Init(W25_PORT_CS, &gi);

	/* ★ MISO 上拉输入，芯片不在位时稳定读 0xFF，一眼知"没接上"。 */
	gi.GPIO_Mode  = GPIO_Mode_IPU;
	gi.GPIO_Pin   = W25_PIN_MISO;
	GPIO_Init(W25_PORT_MISO, &gi);

	W25_CS(1u);
	W25_SCK(0u);

	return (BootW25_ReadJedec() == BOOT_W25_JEDEC_W25Q64) ? 1u : 0u;
}
