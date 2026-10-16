#include "stm32f10x.h"
#include "boot_config.h"
#include "boot_w25.h"

/*==============================================================================
 *  ★★★★ Bootloader 的 W25Q64 只读驱动（★ 2026-10-14 建 ✓）
 *    设计理由（★ 为什么只有读 ✓）见 boot_w25.h
 *
 *  ★★ 软件 SPI（★ GPIO 手动打拍 ✗ mode 0 ✓）
 *    · ★ 为什么【不内联 BSRR ✗ 用 `GPIO_WriteBit`】✗
 *      → 【和 App 侧【同一个选择】**✗ 故意慢一点 ✓
 *      → 【实际 SCK 约 1~2MHz ✗ 【面包板/短线【更稳】】✓✓
 *
 *  ★★★ 本文件【不加锁】✗✓ —— ★ 因为【Bootloader 是【裸机单线程】】✓
 *    （★ 没有任务 ✗ 没有中断里碰它 ✗ 【无并发可言】✓）
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
#define W25_DUMMY            0xFFu

#define W25_SIZE             (8u * 1024u * 1024u)

static void W25_CS(uint8_t v)   { GPIO_WriteBit(W25_PORT_CS,   W25_PIN_CS,   (BitAction)v); }
static void W25_SCK(uint8_t v)  { GPIO_WriteBit(W25_PORT_SCK,  W25_PIN_SCK,  (BitAction)v); }
static void W25_MOSI(uint8_t v) { GPIO_WriteBit(W25_PORT_MOSI, W25_PIN_MOSI, (BitAction)v); }
static uint8_t W25_MISO(void)   { return (uint8_t)GPIO_ReadInputDataBit(W25_PORT_MISO, W25_PIN_MISO); }

/* ★ mode 0 ✗：MOSI 在 SCK 低时摆好 ✗ 上升沿采样 MISO ✓ */
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

uint8_t BootW25_Init(void)
{
	GPIO_InitTypeDef gi;

	/* ★ 两个端口都要开时钟 ✗ SCK/MISO/MOSI 在 A ✗ CS 在 B ✓ */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	gi.GPIO_Mode  = GPIO_Mode_Out_PP;
	gi.GPIO_Pin   = W25_PIN_SCK | W25_PIN_MOSI;
	gi.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(W25_PORT_SCK, &gi);

	gi.GPIO_Pin   = W25_PIN_CS;
	GPIO_Init(W25_PORT_CS, &gi);

	/* ★★ MISO 上拉输入 ✗ 芯片不在位时稳定读 0xFF ✗ 一眼知"没接上" ✓ */
	gi.GPIO_Mode  = GPIO_Mode_IPU;
	gi.GPIO_Pin   = W25_PIN_MISO;
	GPIO_Init(W25_PORT_MISO, &gi);

	W25_CS(1u);
	W25_SCK(0u);

	return (BootW25_ReadJedec() == BOOT_W25_JEDEC_W25Q64) ? 1u : 0u;
}
