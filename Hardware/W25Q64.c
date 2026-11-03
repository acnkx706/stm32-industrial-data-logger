#include "stm32f10x.h"
#include "W25Q64.h"
#include "Serial.h"                    /* ★ 2026-10-24：接线验证要打印。 */

/*==============================================================================
 *  ★ W25Q64 驱动（2026-10-14 移植 + 加固）
 *         移植自 裸机\11.1软件SPI读写W25Q64\Hardware\MySPI.c + W25Q64.c
 *
 *  ★ 移植时改了三处，三处都是"会静默出错"的那种：
 *    · ① WaitBusy() 超时 10000 → 50000
 *      · 原值只够约 150ms，而4KB 扇区擦除最坏 400ms
 *      → 擦除中途被判"好了"，接着写，写进一个还没擦干净的扇区
 *    · ② PageProgram() 补 256B 越页拆包
 *      · W25Q64 的页写不跨页，到页尾会回绕到页首
 *      → 静默覆盖刚写的数据，而且不报任何错
 *    · ③ 所有会失败的函数从 void 改成返回状态
 *      · 原版超时/失败调用者完全不知道
 *      → 往金副本里写进坏镜像，然后靠它去救砖，必炸
 *
 *  ★ 引脚见 W25Q64.h，为什么不用 SPI1 也写在那里了
 *============================================================================*/

/* ---- 引脚（2026-10-14 第二次定案，完整理由见 W25Q64.h）----------
 *  ★ 第一版选的 PB5/PB6/PB7/PB10 代码上确实空闲
 *    但实物上被 OLED 那片线物理占死
 *  → 教训，实物优先于代码清点
 *
 *  ★ 故意避开 PA15：
 *    · 它是 JTAG 的 JTDI，要用就得关 JTAG
 *    · 关错了会把 SWD 一起关掉，调试口就没了
 *    → 不值得为省一个脚冒这个险。 */
#define W25_PORT_SCK         GPIOA
#define W25_PORT_MISO        GPIOA
#define W25_PORT_MOSI        GPIOA
#define W25_PORT_CS          GPIOB          /* ★ 只有 CS 在 B 口。 */

#define W25_PIN_SCK          GPIO_Pin_8     /* ★ PA8。 */
#define W25_PIN_MISO         GPIO_Pin_11    /* ★ PA11。 */
#define W25_PIN_MOSI         GPIO_Pin_12    /* ★ PA12。 */
#define W25_PIN_CS           GPIO_Pin_13    /* ★ PB13（从光敏 DO 腾出来的） */

/* ---- 等 BUSY 的超时上限 ---------------------------------------------------
 *  ★ 为什么是 50000：
 *    · 一次 W25_SwapByte 是8 拍 × 几次 GPIO 调用≈ 10~20µs
 *    · 4KB 扇区擦除典型 30~400ms，64KB 块擦更久
 *    · 200000 × 15µs ≈ 3 秒，盖住 64KB 块擦最坏 2 秒
 *      ★ 2026-10-23 改（原值 50000 ≈ 0.75s，独立审查）：
 *        金副本备份真的会一次连擦两个 64KB 块（golden.c Golden_EraseImageArea）
 *        慢芯片单块最坏 2s，0.75s 会把"慢"误判成"坏"⇒ 备份流程假报错
 *    · 宁可等久一点，也不能早早放弃—— 放弃的后果是
 *      后面所有操作全在错误状态上继续跑。 */
#define W25_WAIT_TIMEOUT     200000u


/*==============================================================================
 *  第 1 层：软件 SPI（GPIO 手动打拍，模式 0）
 *
 *  ★ 模式 0：SCK 空闲 低，上升沿采样
 *    · W25Q64 手册写明mode 0 和 mode 3 都支持
 *    → 软件 SPI 自己打拍，用 mode 0 最直观
 *
 *  ★ 为什么不内联 BSRR，而用 GPIO_WriteBit：
 *    · 不是偷懒，是故意的
 *    · GPIO_WriteBit 是库函数调用，每次要几十个周期
 *      → 实际 SCK 只有约 1~2MHz
 *    · 而我们现在是面包板 + 杜邦线
 *      → 慢一点反而稳，快才是出问题的原因
 *    · 所以先不要"优化"成 BSRR
 *      等挪到洞洞板/PCB 之后再考虑提速
 *============================================================================*/

static void W25_CS(uint8_t v)   { GPIO_WriteBit(W25_PORT_CS,   W25_PIN_CS,   (BitAction)v); }
static void W25_SCK(uint8_t v)  { GPIO_WriteBit(W25_PORT_SCK,  W25_PIN_SCK,  (BitAction)v); }
static void W25_MOSI(uint8_t v) { GPIO_WriteBit(W25_PORT_MOSI, W25_PIN_MOSI, (BitAction)v); }
static uint8_t W25_MISO(void)   { return (uint8_t)GPIO_ReadInputDataBit(W25_PORT_MISO, W25_PIN_MISO); }

/* ★ 一个字节的"交换"（SPI 没有"只读"，发一个才能收一个）
 *  ★ 顺序不能改，mode 0 规定：
 *    ① 在 SCK 低电平期间摆好 MOSI
 *    ② 拉高 SCK，从机在这时采样
 *    ③ 同时在上升沿之后读 MISO
 *    ④ 拉低 SCK，准备下一位。 */
static uint8_t W25_SwapByte(uint8_t out)
{
	uint8_t i;
	uint8_t in = 0u;

	for (i = 0u; i < 8u; i++)
	{
		W25_MOSI((out & 0x80u) ? 1u : 0u);   /* ★ 发最高位。 */
		out = (uint8_t)(out << 1u);

		W25_SCK(1u);                          /* ★ 上升沿，从机采样。 */
		if (W25_MISO() != 0u)
		{
			in |= (uint8_t)(0x80u >> i);      /* ★ 收到的也是高位在前。 */
		}
		W25_SCK(0u);
	}
	return in;
}


/*==============================================================================
 *  第 2 层：W25Q64 指令
 *============================================================================*/

/* ★ 写使能（每一次页写 / 擦除之前都必须发
 *   ★ 芯片执行完写/擦之后自己会清掉，所以每次都要重发） */
static void W25_WriteEnable(void)
{
	W25_CS(0u);
	W25_SwapByte(W25_INS_WRITE_ENABLE);
	W25_CS(1u);
}

/* ★ 等 BUSY 清，返回状态，原版是 void。 */
uint8_t W25Q64_WaitReady(void)
{
	uint32_t t = W25_WAIT_TIMEOUT;

	W25_CS(0u);
	W25_SwapByte(W25_INS_READ_STATUS_1);
	while ((W25_SwapByte(W25_DUMMY) & 0x01u) != 0u)   /* ★ bit0 = BUSY。 */
	{
		if (--t == 0u)
		{
			W25_CS(1u);
			return 0u;                                 /* ★ 超时，明说。 */
		}
	}
	W25_CS(1u);
	return 1u;
}


/*==============================================================================
 *  第 3 层：数据操作
 *============================================================================*/

/* ★ 读（无长度限制，返回 1 = OK） */
uint8_t W25Q64_ReadData(uint32_t addr, uint8_t *buf, uint32_t len)
{
	uint32_t i;

	if ((addr + len) > W25_SIZE) { return 0u; }         /* ★ 越界护栏。 */

	W25_CS(0u);
	W25_SwapByte(W25_INS_READ_DATA);
	W25_SwapByte((uint8_t)(addr >> 16));                /* ★ 地址 24 位，高位在前。 */
	W25_SwapByte((uint8_t)(addr >> 8));
	W25_SwapByte((uint8_t)(addr));
	for (i = 0u; i < len; i++)
	{
		buf[i] = W25_SwapByte(W25_DUMMY);
	}
	W25_CS(1u);

	/* ★ 这里故意不调 WaitReady
	 *   · 读操作根本不会让芯片变 BUSY
	 *   · 原版教程代码在末尾等了一次，纯属白等
	 *     → 而且一次状态轮询要 20~30µs，读 51KB 时白花几毫秒 */
	return 1u;
}

/* ★ 写一个"页内"片段（调用者保证 addr 到页尾之间够放 len） */
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

/* ★ 写（自动按 256B 页边界拆开，返回 1 = OK）
 *
 *  ★ 为什么必须拆（这是原版最危险的一处）：
 *    · W25Q64 的"页写"指令只能写一页之内（256 字节）
 *    · 如果从页中间开始，一路写到超过页尾
 *      → 芯片不会报错，而是回绕到本页页首继续写
 *      → 把这一页开头的数据悄悄覆盖掉
 *    · 现象：写 400 字节，前 100 字节变成垃圾
 *      而函数返回成功 —— 这种错根本查不出来
 *  → 所以按页切，一段一段写
 *
 *  ★ 注意：调用者必须先擦过
 *    Flash 只能 1→0，往有数据的格子上写 = 旧值 AND 新值，静默错 */
uint8_t W25Q64_WriteData(uint32_t addr, const uint8_t *buf, uint32_t len)
{
	if ((addr + len) > W25_SIZE) { return 0u; }         /* ★ 越界护栏。 */

	while (len > 0u)
	{
		/* ★ 本页还能放几个字节。 */
		uint16_t page_left = (uint16_t)(W25_PAGE_SIZE - (addr & (W25_PAGE_SIZE - 1u)));
		uint16_t n = (len < (uint32_t)page_left) ? (uint16_t)len : page_left;

		W25_PageProgramRaw(addr, buf, n);

		if (W25Q64_WaitReady() == 0u)
		{
			return 0u;                                  /* ★ 超时，明说。 */
		}

		addr += n;
		buf  += n;
		len  -= n;
	}
	return 1u;
}

/* ★ 擦一个 4KB 扇区（返回 1 = OK） */
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

/* ★ 擦一个 64KB 块（比擦 16 个扇区快得多，备份大镜像时用） */
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

/* ★ 读 JEDEC ID（24 位，正常 0xEF4017） */
uint32_t W25Q64_ReadJEDEC(void)
{
	uint32_t id;

	W25_CS(0u);
	W25_SwapByte(W25_INS_JEDEC_ID);                     /* ★ 0x9F。 */
	id  = (uint32_t)W25_SwapByte(W25_DUMMY) << 16;      /* ★ MID（华邦 = 0xEF）。 */
	id |= (uint32_t)W25_SwapByte(W25_DUMMY) << 8;       /* ★ DID 高字节。 */
	id |= (uint32_t)W25_SwapByte(W25_DUMMY);            /* ★ DID 低字节。 */
	W25_CS(1u);
	return id;
}

/* ★ 在位吗（判据就是 JEDEC） */
uint8_t W25Q64_IsPresent(void)
{
	return (W25Q64_ReadJEDEC() == W25_JEDEC_W25Q64) ? 1u : 0u;
}

/* ★ 初始化（配好 GPIO，返回 1 = 芯片在位）
 *  ★ 注意：即使芯片不在位，这一步也不会卡住
 *    （MISO 配了上拉，悬空读回 0xFF，JEDEC 自然不等于 0xEF4017） */
uint8_t W25Q64_Init(void)
{
	GPIO_InitTypeDef gi;

	/* ★ 两个端口都要开时钟 ——
	 *   ★ SCK/MOSI/MISO 在 GPIOA，CS 在 GPIOB
	 *   （为什么不一整条都放一个口
	 *     因为实物上只有这几个位置空着，不是想选哪就选哪） */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	/* ★ SCK / MOSI → 推挽输出（同在 GPIOA，一次配完） */
	gi.GPIO_Mode  = GPIO_Mode_Out_PP;
	gi.GPIO_Pin   = W25_PIN_SCK | W25_PIN_MOSI;
	gi.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(W25_PORT_SCK, &gi);

	/* ★ CS → 推挽输出（在 GPIOB，必须单独来一次） */
	gi.GPIO_Pin   = W25_PIN_CS;
	GPIO_Init(W25_PORT_CS, &gi);

	/* ★ MISO → 上拉输入
	 *   ★ 为什么上拉：芯片不在位 / 线掉了的时候引脚悬空
	 *     → 浮空输入会随机抖动，读出的 ID 每次都不一样
	 *     → 上拉之后稳定读回 0xFF，一眼就知道"没接上" */
	gi.GPIO_Mode  = GPIO_Mode_IPU;
	gi.GPIO_Pin   = W25_PIN_MISO;
	GPIO_Init(W25_PORT_MISO, &gi);

	/* ★ 空闲态：CS 高（不选中），SCK 低（mode 0 的空闲电平） */
	W25_CS(1u);
	W25_SCK(0u);

	return W25Q64_IsPresent();
}

/* ★ 自检（擦 → 写 → 读回比对）
 *  ★ 返回，0 = 全过，1 = 擦失败，2 = 写失败，3 = 读失败，4 = 数据不符
 *
 *  ★ 为什么非要"擦+写+读"全套：
 *    · 只读 JEDEC只能证明MISO/MOSI/CS/SCK 里收的那一半通
 *    · 而写要MOSI + 擦除时序 + 等 BUSY全对
 *    → 只读 ID 就宣布"接线没问题"，是自欺欺人
 *      （现象：ID 读得出来，但一写就坏，查一天） */
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

/* ★ 2026-10-24 从 main.c 搬来：接线验证
 *   ★ 为什么开机就要探一次：它承担金副本/回滚，是救命的那块存储
 *   ★ 判据三条：JEDEC=0xEF4017 / selftest=0（擦+写+读回比对）
 *     ⚠️ 只读 ID 就宣布接线没问题 = 自欺欺人，写方向可能坏 */
void W25Q64_BootTest(void)
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
