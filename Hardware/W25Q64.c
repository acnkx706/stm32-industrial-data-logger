#include "stm32f10x.h"
#include "W25Q64.h"

/*==============================================================================
 *  ★★★★★ W25Q64 驱动（★ 2026-10-14 移植 + 加固 ✓）
 *         移植自 `裸机\11.1软件SPI读写W25Q64\Hardware\MySPI.c + W25Q64.c`
 *
 *  ★★★★ 【移植时改了三处 ✗ 三处都是"会静默出错"的那种】✗✓：
 *    · ★ ① `WaitBusy()` 超时 10000 → **50000** ✓
 *      · ★ 原值只够约 150ms ✗ 而【4KB 扇区擦除最坏 400ms】✓
 *      → ★★ 【擦除中途被判"好了"✗ 接着写 ✗ 【写进一个还没擦干净的扇区】】✓✓
 *    · ★★ ② `PageProgram()` 补 **256B 越页拆包** ✓
 *      · ★ W25Q64 的页写【不跨页】✗ 到页尾会【回绕到页首】✓
 *      → ★★ 【静默覆盖刚写的数据 ✗ 而且【不报任何错】】✓✓
 *    · ★★★ ③ 所有会失败的函数【从 void 改成返回状态】✓
 *      · ★ 原版超时/失败【调用者完全不知道】✓
 *      → ★★ 【往金副本里写进坏镜像 ✗ 然后靠它去救砖 ✗ 必炸】✓✓
 *
 *  ★★★ 引脚见 W25Q64.h ✗ 为什么不用 SPI1 也写在那里了 ✓
 *============================================================================*/

/* ---- 引脚（★ **2026-10-14 第二次定案** ✗ 完整理由见 W25Q64.h ✓）----------
 *  ★★★ 第一版选的 PB5/PB6/PB7/PB10 【代码上确实空闲】✗
 *    但【实物上被 OLED 那片线【物理占死】】✗✓
 *  → ★★★★ 【教训 ✗【实物优先于代码清点】】✓✓
 *
 *  ★★ 【故意避开 PA15**✗✓：
 *    · 它是 JTAG 的 JTDI ✗ 【要用就得关 JTAG】✓
 *    · ★★ 【关错了会把 SWD 一起关掉 ✗ 调试口就没了】✓✓
 *    → 【不值得为省一个脚冒这个险】✓ */
#define W25_PORT_SCK         GPIOA
#define W25_PORT_MISO        GPIOA
#define W25_PORT_MOSI        GPIOA
#define W25_PORT_CS          GPIOB          /* ★ 只有 CS 在 B 口 ✓ */

#define W25_PIN_SCK          GPIO_Pin_8     /* ★ PA8  ✓ */
#define W25_PIN_MISO         GPIO_Pin_11    /* ★ PA11 ✓ */
#define W25_PIN_MOSI         GPIO_Pin_12    /* ★ PA12 ✓ */
#define W25_PIN_CS           GPIO_Pin_13    /* ★ PB13（★ 从光敏 DO 腾出来的 ✓）*/

/* ---- 等 BUSY 的超时上限 ---------------------------------------------------
 *  ★★★★ 为什么是 **50000** ✗✓：
 *    · ★ 一次 `W25_SwapByte` 是【8 拍 × 几次 GPIO 调用】≈ **10~20µs** ✓
 *    · ★★ 【4KB 扇区擦除典型 30~400ms ✗ 64KB 块擦更久】✓
 *    · ★★★ 50000 × 15µs ≈ **0.75 秒** ✗ 【够 4KB ✗ 勉强够 64KB】✓
 *      → ⚠️ 【64KB 块擦若报超时 ✗ 先怀疑【是这个值不够】✗ 不是芯片坏】✓✓
 *    · ★★★★ 宁可【等久一点】✗ 也不能【早早放弃】✓ —— 放弃的后果是
 *      【后面所有操作全在错误状态上继续跑】✓ */
#define W25_WAIT_TIMEOUT     50000u


/*==============================================================================
 *  第 1 层：软件 SPI（★ GPIO 手动打拍 ✗ 模式 0 ✓）
 *
 *  ★★ 模式 0 ✗：★ SCK 空闲 **低** ✗ 【上升沿采样】✓
 *    · ★ W25Q64 手册写明【mode 0 和 mode 3 都支持】✓
 *    → 【软件 SPI 自己打拍 ✗ 用 mode 0 最直观】✓✓
 *
 *  ★★★ 为什么【不内联 BSRR ✗ 而用 `GPIO_WriteBit`】✗✓：
 *    · ★ 【不是偷懒 ✗ 是故意的】✓
 *    · ★★ `GPIO_WriteBit` 是【库函数调用 ✗ 每次要几十个周期】✓
 *      → 【实际 SCK 只有约 1~2MHz】✓
 *    · ★★★ 【而我们现在是【面包板 + 杜邦线】】✗✓
 *      → 【【慢一点【反而稳】✗ 快才是出问题的原因】】✓✓
 *    · ★★★★ 【所以【先不要"优化"成 BSRR】✗
 *      等挪到洞洞板/PCB 之后【再考虑提速】】✓✓
 *============================================================================*/

static void W25_CS(uint8_t v)   { GPIO_WriteBit(W25_PORT_CS,   W25_PIN_CS,   (BitAction)v); }
static void W25_SCK(uint8_t v)  { GPIO_WriteBit(W25_PORT_SCK,  W25_PIN_SCK,  (BitAction)v); }
static void W25_MOSI(uint8_t v) { GPIO_WriteBit(W25_PORT_MOSI, W25_PIN_MOSI, (BitAction)v); }
static uint8_t W25_MISO(void)   { return (uint8_t)GPIO_ReadInputDataBit(W25_PORT_MISO, W25_PIN_MISO); }

/* ★★★ 一个字节的"交换"✗（★ SPI 没有"只读"✗ 发一个才能收一个 ✓）
 *  ★ 顺序【不能改 ✗ mode 0 规定】✗：
 *    ① 在 SCK 低电平期间【摆好 MOSI】✓
 *    ② 拉高 SCK ✗ 【从机在这时采样】✓
 *    ③ 【同时在上升沿之后【读 MISO】】✓
 *    ④ 拉低 SCK ✗ 准备下一位 ✓ */
static uint8_t W25_SwapByte(uint8_t out)
{
	uint8_t i;
	uint8_t in = 0u;

	for (i = 0u; i < 8u; i++)
	{
		W25_MOSI((out & 0x80u) ? 1u : 0u);   /* ★ 发最高位 ✓ */
		out = (uint8_t)(out << 1u);

		W25_SCK(1u);                          /* ★ 上升沿 ✗ 从机采样 ✓ */
		if (W25_MISO() != 0u)
		{
			in |= (uint8_t)(0x80u >> i);      /* ★ 收到的也是【高位在前】✓ */
		}
		W25_SCK(0u);
	}
	return in;
}


/*==============================================================================
 *  第 2 层：W25Q64 指令
 *============================================================================*/

/* ★ 写使能（★ 每一次【页写 / 擦除】之前【都必须发】✓
 *   ★★ 芯片执行完写/擦之后【自己会清掉】✗ 所以【每次都要重发】✓）*/
static void W25_WriteEnable(void)
{
	W25_CS(0u);
	W25_SwapByte(W25_INS_WRITE_ENABLE);
	W25_CS(1u);
}

/* ★★★★ 等 BUSY 清 ✗ 【返回状态 ✗ 原版是 void】✓ */
uint8_t W25Q64_WaitReady(void)
{
	uint32_t t = W25_WAIT_TIMEOUT;

	W25_CS(0u);
	W25_SwapByte(W25_INS_READ_STATUS_1);
	while ((W25_SwapByte(W25_DUMMY) & 0x01u) != 0u)   /* ★ bit0 = BUSY ✓ */
	{
		if (--t == 0u)
		{
			W25_CS(1u);
			return 0u;                                 /* ★★ 超时 ✗ 【明说】✓ */
		}
	}
	W25_CS(1u);
	return 1u;
}


/*==============================================================================
 *  第 3 层：数据操作
 *============================================================================*/

/* ★★★ 读（★ 无长度限制 ✓ ✗ 返回 1 = OK ✓） */
uint8_t W25Q64_ReadData(uint32_t addr, uint8_t *buf, uint32_t len)
{
	uint32_t i;

	if ((addr + len) > W25_SIZE) { return 0u; }         /* ★ 越界护栏 ✓ */

	W25_CS(0u);
	W25_SwapByte(W25_INS_READ_DATA);
	W25_SwapByte((uint8_t)(addr >> 16));                /* ★ 地址 24 位 ✗ 高位在前 ✓ */
	W25_SwapByte((uint8_t)(addr >> 8));
	W25_SwapByte((uint8_t)(addr));
	for (i = 0u; i < len; i++)
	{
		buf[i] = W25_SwapByte(W25_DUMMY);
	}
	W25_CS(1u);

	/* ★★ 【这里【故意不调 WaitReady】】✗✓
	 *   · ★ 读操作【根本不会让芯片变 BUSY】✓
	 *   · ★★ 原版教程代码在末尾等了一次 ✗ 【纯属白等】✓
	 *     → 【而且一次状态轮询要 20~30µs ✗ 读 51KB 时【白花几毫秒】】✓✓ */
	return 1u;
}

/* ★ 写一个"页内"片段（★ 调用者【保证 addr 到页尾之间够放 len】✓）*/
static void W25_PageProgramRaw(uint32_t addr, const uint8_t *buf, uint16_t len)
{
	uint16_t i;

	W25_WriteEnable();
	W25_CS(0u);
	W25_SwapByte(W25_INS_PAGE_PROGRAM);
	W25_SwapByte((uint8_t)(addr >> 16));
	W25_SwapByte((uint8_t)(addr >> 8));
	W25_SwapByte((uint8_t)(addr));
	for (i = 0u; i < len; i++)
	{
		W25_SwapByte(buf[i]);
	}
	W25_CS(1u);
}

/* ★★★★ 写（★ **自动按 256B 页边界拆开**✓ ✗ 返回 1 = OK ✓）
 *
 *  ★★★★ 为什么【必须拆**✗✓（★ 这是【原版最危险的一处】✓）：
 *    · ★ W25Q64 的"页写"指令【只能写【一页之内】（256 字节）】✓
 *    · ★★ 如果【从页中间开始 ✗ 一路写到超过页尾】✗
 *      → 【芯片【不会报错 ✗ 而是【回绕到本页页首继续写】】✓✓
 *      → ★★★ 【把【这一页开头的数据【悄悄覆盖掉】】✗✓
 *    · ★★★★ 现象 ✗：★【写 400 字节 ✗ 前 100 字节变成垃圾 ✗
 *      而函数【返回成功】】✗✓ —— 【这种错【根本查不出来】】✓✓
 *  → ★★★ 所以【按页切 ✗ 一段一段写】✓
 *
 *  ★★ 注意 ✗：★ 调用者【必须【先擦过】】✗
 *    【Flash 只能 1→0 ✗ 往有数据的格子上写 = 旧值 AND 新值 ✗ 静默错】✓✓ */
uint8_t W25Q64_WriteData(uint32_t addr, const uint8_t *buf, uint32_t len)
{
	if ((addr + len) > W25_SIZE) { return 0u; }         /* ★ 越界护栏 ✓ */

	while (len > 0u)
	{
		/* ★ 本页还能放几个字节 ✓ */
		uint16_t page_left = (uint16_t)(W25_PAGE_SIZE - (addr & (W25_PAGE_SIZE - 1u)));
		uint16_t n = (len < (uint32_t)page_left) ? (uint16_t)len : page_left;

		W25_PageProgramRaw(addr, buf, n);

		if (W25Q64_WaitReady() == 0u)
		{
			return 0u;                                  /* ★★ 超时 ✗ 明说 ✓ */
		}

		addr += n;
		buf  += n;
		len  -= n;
	}
	return 1u;
}

/* ★★★ 擦一个 4KB 扇区（★ 返回 1 = OK ✓）*/
uint8_t W25Q64_EraseSector(uint32_t addr)
{
	W25_WriteEnable();
	W25_CS(0u);
	W25_SwapByte(W25_INS_SECTOR_ERASE_4KB);
	W25_SwapByte((uint8_t)(addr >> 16));
	W25_SwapByte((uint8_t)(addr >> 8));
	W25_SwapByte((uint8_t)(addr));
	W25_CS(1u);
	return W25Q64_WaitReady();
}

/* ★★★ 擦一个 64KB 块（★ 比擦 16 个扇区【快得多】✗ 备份大镜像时用 ✓）*/
uint8_t W25Q64_EraseBlock64K(uint32_t addr)
{
	W25_WriteEnable();
	W25_CS(0u);
	W25_SwapByte(W25_INS_BLOCK_ERASE_64KB);
	W25_SwapByte((uint8_t)(addr >> 16));
	W25_SwapByte((uint8_t)(addr >> 8));
	W25_SwapByte((uint8_t)(addr));
	W25_CS(1u);
	return W25Q64_WaitReady();
}


/*==============================================================================
 *  探测 / 自检
 *============================================================================*/

/* ★★★ 读 JEDEC ID（★ 24 位 ✓ ✗ 正常 `0xEF4017` ✓）*/
uint32_t W25Q64_ReadJEDEC(void)
{
	uint32_t id;

	W25_CS(0u);
	W25_SwapByte(W25_INS_JEDEC_ID);                     /* ★ 0x9F ✓ */
	id  = (uint32_t)W25_SwapByte(W25_DUMMY) << 16;      /* ★ MID（华邦 = 0xEF）✓ */
	id |= (uint32_t)W25_SwapByte(W25_DUMMY) << 8;       /* ★ DID 高字节 ✓ */
	id |= (uint32_t)W25_SwapByte(W25_DUMMY);            /* ★ DID 低字节 ✓ */
	W25_CS(1u);
	return id;
}

/* ★★★ 在位吗（★ 判据就是 JEDEC ✓）*/
uint8_t W25Q64_IsPresent(void)
{
	return (W25Q64_ReadJEDEC() == W25_JEDEC_W25Q64) ? 1u : 0u;
}

/* ★ 初始化（★ 配好 GPIO ✗ 返回 1 = 芯片在位 ✓）
 *  ★★ 注意 ✗：★ 即使芯片【不在位】✗ 这一步也【不会卡住】✓
 *    （★ MISO 配了上拉 ✗ 悬空读回 0xFF ✗ JEDEC 自然不等于 0xEF4017 ✓）*/
uint8_t W25Q64_Init(void)
{
	GPIO_InitTypeDef gi;

	/* ★★ 【两个端口都要开时钟**✗✓ ——
	 *   ★ SCK/MOSI/MISO 在 **GPIOA** ✗ CS 在 **GPIOB** ✓
	 *   （★ 为什么不一整条都放一个口 ✗
	 *     【因为【实物上只有这几个位置空着】✗ 不是想选哪就选哪 ✓）*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	/* ★ SCK / MOSI → 推挽输出（★ 同在 GPIOA ✗ 一次配完 ✓）*/
	gi.GPIO_Mode  = GPIO_Mode_Out_PP;
	gi.GPIO_Pin   = W25_PIN_SCK | W25_PIN_MOSI;
	gi.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(W25_PORT_SCK, &gi);

	/* ★ CS → 推挽输出（★ 在 GPIOB ✗ 必须【单独来一次】✓）*/
	gi.GPIO_Pin   = W25_PIN_CS;
	GPIO_Init(W25_PORT_CS, &gi);

	/* ★★ MISO → 上拉输入 ✓
	 *   ★ 为什么【上拉】✗：★ 芯片不在位 / 线掉了的时候【引脚悬空】✓
	 *     → 【浮空输入会【随机抖动 ✗ 读出的 ID 每次都不一样】】✓
	 *     → ★★ 【上拉之后【稳定读回 0xFF ✗ 一眼就知道"没接上"】】✓✓ */
	gi.GPIO_Mode  = GPIO_Mode_IPU;
	gi.GPIO_Pin   = W25_PIN_MISO;
	GPIO_Init(W25_PORT_MISO, &gi);

	/* ★ 空闲态 ✗：CS 高（★ 不选中 ✓）✗ SCK 低（★ mode 0 的空闲电平 ✓）*/
	W25_CS(1u);
	W25_SCK(0u);

	return W25Q64_IsPresent();
}

/* ★★★★ 自检（★ 擦 → 写 → 读回比对 ✓）
 *  ★ 返回 ✗ 0 = 全过 ✗ 1 = 擦失败 ✗ 2 = 写失败 ✗ 3 = 读失败 ✗ 4 = 数据不符 ✓
 *
 *  ★★ 为什么【非要"擦+写+读"全套】✗✓：
 *    · ★ 【只读 JEDEC】只能证明【MISO/MOSI/CS/SCK 里【收的那一半】通】✓
 *    · ★★ 而【写要【MOSI + 擦除时序 + 等 BUSY】全对】✓
 *    → ★★★ 【只读 ID 就宣布"接线没问题"✗ 【是自欺欺人】】✓✓
 *      （★ 现象 ✗：ID 读得出来 ✗ 但一写就坏 ✗ 【查一天】✓）*/
uint8_t W25Q64_SelfTest(void)
{
	static const uint8_t wr[4] = { 0xA5u, 0x5Au, 0xC3u, 0x3Cu };
	uint8_t rd[4];
	uint8_t i;

	if (W25Q64_EraseSector(W25_SELFTEST_ADDR) == 0u) { return 1u; }
	if (W25Q64_WriteData(W25_SELFTEST_ADDR, wr, 4u) == 0u) { return 2u; }
	if (W25Q64_ReadData(W25_SELFTEST_ADDR, rd, 4u) == 0u) { return 3u; }

	for (i = 0u; i < 4u; i++)
	{
		if (rd[i] != wr[i]) { return 4u; }
	}
	return 0u;
}
