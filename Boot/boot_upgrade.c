#include "stm32f10x.h"
#include "boot_config.h"
#include "boot_jump.h"
#include "boot_flag.h"
#include "boot_flash.h"
#include "boot_param.h"
#include "boot_upgrade.h"
#include "boot_w25.h"
#include "Serial.h"

/*==============================================================================
 *  ★ 升级协议状态机（2026-10-13 建）
 *        安全模型 / 协议 / 为什么不那样做 —— 全部写在 boot_upgrade.h
 *============================================================================*/

/* ---- 调试输出开关（见 boot_upgrade.h）-------------------------------- */
#if BOOT_UPGRADE_DEBUG
  #define UP_LOG(s)      Serial_SendString(s)
#else
  #define UP_LOG(s)      do { } while (0)
#endif

/* ---- 超时用"空轮次数"，不用毫秒（理由见下）---------------------------
 *
 *  ★ 为什么接收循环里绝不能塞一个粗延时：
 *    · 115200，8N1 → 11.52 字节/毫秒
 *    · 而环形缓冲只有 64 字节（Serial.c 里 SER_RX_BUF_SIZE）
 *      → 64 ÷ 11.52 ≈ 5.6 毫秒就满
 *    · 原版等待循环是100ms 一轮→ 缓冲溢出约 18 倍，必丢字节
 *    → 所以必须是紧密轮询，一轮只做"取一个字节 + 判空"
 *
 *  ★ 那超时怎么算 → 数"空转了多少轮"
 *    · 一轮 ≈ 一次 Serial_RxPop() + 判空 ≈ 30 周期 ≈ 0.4µs
 *    · 5,000,000 轮 ≈ 2 秒
 *    · 精度完全不重要：
 *      → 超时只用来放弃一条卡住的半截命令
 *      → 早 0.5 秒 / 晚 0.5 秒都安全，因为失败的出口永远是"回 IDLE"
 *    · 而且不用 SysTick：stm32f10x_it.c 里已经有一个
 *      SysTick_Handler()→ 我们再定义一次 = 重复符号，链接不过
 *      （更别说跳转前还要记得关掉它，平白多一份状态） */
#define UP_IDLE_POLLS     5000000u    /* ★ 空闲态：约 2 秒没动静 → 打一次心跳。 */
/* ★ 2026-11-08：空闲多久放弃升级、跳回 App
 *   · 心跳是 2 秒一次，15 次 ≈ 30 秒
 *   · 只在"IDLE + App 有效 + 暂存区没有就绪镜像"时才允许跳 —— 见 Up_CanBootInstead()
 *   ★ 为什么需要它：U 设的升级标志只有"升级成功"或 C 才清，
 *     传输阶段断线/断电后板子会一直停在升级模式；现场没人发 C 就再也不会跑。 */
#define UP_IDLE_CANCEL_WAITS  15u
#define UP_RECV_POLLS    10000000u    /* ★ 收数据态：约 4 秒收不满一行 → 判超时。 */

/* ---- 状态 ------------------------------------------------------------------ */
typedef enum
{
	UP_ST_IDLE = 0,                   /* ★ 等 E<len>（或 C） */
	UP_ST_RECV                        /* ★ 正在收 D / X。 */
} UpState_t;

/* ---- 缓冲（全部 static，绝不放栈）---------------------------------
 *  ★ 为什么不放栈：
 *    · 两个 1KB 缓冲 = 512 words
 *    · Bootloader 的栈只有启动文件里那个 Stack_Size（通常 0x400）
 *      → 直接爆栈，而且是静默踩到别的变量，极难查
 *  ★ 为什么不放堆：裸机没有 malloc，也没有 FreeRTOS 的 heap
 *  ★ RAM 完全够：20KB 里，ZI 原本才 1.1KB */
static uint8_t  s_page0[BOOT_FLASH_PAGE_SIZE];   /* ★ 第 0 页暂存，最后才写。 */
static uint8_t  s_page [BOOT_FLASH_PAGE_SIZE];   /* ★ 当前正在攒的页。 */
static char     s_line [UP_LINE_MAX];            /* ★ 一行命令缓冲。 */

static UpState_t s_state;
static uint16_t  s_page_bytes;    /* ★ 当前页已攒字节数。 */
static uint16_t  s_page_index;    /* ★ 当前是第几页（0 = 第 0 页，只暂存） */
static uint32_t  s_expect;        /* ★ E<len> 声明的总字节数。 */
static uint32_t  s_total;         /* ★ 已经收了多少字节。 */
static uint16_t  s_crc;           /* ★ 边收边算的 CRC-16/KERMIT。 */
static uint16_t  s_seq;           /* ★ 期望的下一块块号。 */
static uint8_t   s_stage;         /* ★ 1 = 本次走 W25 暂存（U3），0 = 老直写路径。 */


/*==============================================================================
 *  ★ 小工具
 *============================================================================*/

/* ★ hex 字符 → 4 bit，返回 -1 = 不是 hex。 */
static int8_t Up_HexVal(char c)
{
	if ((c >= '0') && (c <= '9')) { return (int8_t)(c - '0'); }
	if ((c >= 'A') && (c <= 'F')) { return (int8_t)(c - 'A' + 10); }
	if ((c >= 'a') && (c <= 'f')) { return (int8_t)(c - 'a' + 10); }
	return (int8_t)-1;
}

/* ★ CRC-16/KERMIT增量更新
 *
 *  ★ 为什么要"增量"版：
 *    · 最终 CRC 是同一个值，但不能在最后"把 112KB 读一遍再算"
 *    · 边收边算→ 收完的那一刻 CRC 已经好了
 *
 *  ★ 算法和 App 的 Hardware\Crc16.c 一字不差的契约
 *    · 反射算法，每次右移一位
 *    · 最低位为 1 就异或反射多项式 0x8408
 *    · 初值 0，无输出异或
 *    · 判据：CRC("123456789") == 0x2189（由 SelfTest 守着）
 *
 *  ★ 2026-10-14 起：本函数不是 static 了
 *    · 原因，金副本回退（boot_golden.c）也要用同一个 CRC
 *    · 绝不能再写第二份 —— 这条是今天踩出来的教训
 *      （两个变体 0x31C3 / 0x29B1 混起来的混乱，见 30 号）
 *    → 全 Bootloader只有这一份，声明在 boot_upgrade.h */
uint16_t Up_CrcUpdate(uint16_t crc, uint8_t b)
{
	uint8_t i;

	crc ^= (uint16_t)b;
	for (i = 0u; i < 8u; i++)
	{
		crc = (crc & 0x0001u) ? (uint16_t)((crc >> 1) ^ 0x8408u)
		                      : (uint16_t)(crc >> 1);
	}
	return crc;
}

/* ★ 回一条 K<块号>，表示"这块收好了，而且该写的都写完了"。 */
static void Up_SendK(uint16_t seq)
{
	Serial_SendString("K");
	Serial_SendHex((uint32_t)seq, 4u);
	Serial_SendString("\r\n");
}

/* ★ 回一条 N<块号>，表示"块号不对，请从这个号重发"。 */
static void Up_SendN(uint16_t seq)
{
	Serial_SendString("N");
	Serial_SendHex((uint32_t)seq, 4u);
	Serial_SendString("\r\n");
}

/* ★ 回一条 FAIL <原因>
 *   ★ 按约定：关键路径不许静默
 *     → 失败必须带着原因说出来，否则只能干看着 */
static void Up_SendFail(const char *why)
{
	Serial_SendString("FAIL ");
	Serial_SendString(why);
	Serial_SendString("\r\n");
}

/* ★ 清空一轮传输的全部状态
 *   ⚠️ 不会去动 Flash（第 0 页已经被擦掉了，保持无效是对的） */
static void Up_Reset(void)
{
	s_state      = UP_ST_IDLE;
	s_page_bytes = 0u;
	s_page_index = 0u;
	s_expect     = 0u;
	s_total      = 0u;
	s_crc        = 0u;
	s_seq        = 0u;
}


/*==============================================================================
 *  ★ 读一整行（紧密轮询，无粗延时）
 *
 *  ★ 返回：
 *    · 1 = 收到了一整行（*out_len = 长度，0 = 空行）
 *    · 0 = 超时（到了 max_empty_polls 轮一个字节都没来）
 *    · 2 = 这一行太长了，已经丢弃，请重发
 *
 *  ★ 为什么按字节流解析，而不是"先读满 264 字节"：
 *    · 我们不知道 PC 会发多长
 *    · 必须看到 \n 才收工，否则会一直等下去
 *    · 而且环形缓冲只有 64 字节→ 我们必须边到边取
 *
 *  ★ \r 直接丢掉：判行尾只认 \n
 *    → 这样 PC 发 \r\n，或者只发 \n，都能跑
 *============================================================================*/
static uint8_t Up_ReadLine(uint16_t *out_len, uint32_t max_empty_polls)
{
	uint16_t n        = 0u;
	uint8_t  overflow = 0u;
	uint32_t empty    = 0u;
	int16_t  ch;

	for (;;)
	{
		/* ★ 每一轮都喂狗（理由见 boot_config.h） */
		BOOT_FEED_IWDG();

		ch = Serial_RxPop();

		if (ch < 0)
		{
			if (++empty >= max_empty_polls)
			{
				return 0u;                      /* ★ 超时。 */
			}
			continue;
		}

		empty = 0u;                             /* ★ 来了字节，超时计数清零。 */

		if ((char)ch == '\n')
		{
			s_line[n] = '\0';
			*out_len  = overflow ? 0xFFFFu : n;
			return overflow ? 2u : 1u;
		}

		if ((char)ch == '\r') { continue; }      /* ★ 丢掉。 */

		if (n < (uint16_t)(UP_LINE_MAX - 1u))
		{
			s_line[n] = (char)ch;
			n++;
		}
		else
		{
			/* ★ 太长了，但还得继续读到 \n
			 *   → 否则下一行会被当成这一行的尾巴 */
			overflow = 1u;
		}
	}
}


/*==============================================================================
 *  ★ 把攒好的一页"落地"
 *
 *  ★ 规则只有两条：
 *    · 第 0 页（s_page_index == 0）→ 只搬进 s_page0，一个字节都不写 Flash
 *    · 其它页 → 擦 → 写 → 立刻回读比对
 *
 *  ★ 尾部补 0xFF（最后一页常常不满）
 *    · 补在缓冲里，让"写"永远写满一页
 *    · 而 CRC 天然不受影响
 *      → 因为它是在收到的每一个字节上边收边算的
 *      → 补进缓冲的那些 0xFF从来没进过 CRC
 *        （要是"最后把整页喂进 CRC"，就会和 PC 算的不一样，必 FAIL）
 *
 *  ★ 返回 1 = 这一页确认写对了；0 = 失败（已经打了 FAIL，调用者直接回 IDLE）
 *============================================================================*/
/*==============================================================================
 *  ★ U3：升级暂存区（2026-11-07 加）
 *
 *  ★ 为什么要有它：
 *    · 老路径是 PC 直接往内部 Flash 写，传到一半断电 ⇒ App 区是半成品
 *      ⇒ 下次开机 App 无效 ⇒ 只能接电脑救
 *    · 新路径：整包先写进 W25 尾部，整包校验通过后才搬进内部
 *      ⇒ 传输期间内部 App 一个字节都不动，断电照常开机
 *
 *  ★ 暂存头放在单独一个扇区：
 *    · magic 先写成 0xFFFFFFFF，其它字段写完后再单独补 magic
 *      ⇒ 中途断电，magic 没补上 ⇒ 头无效 ⇒ 当没暂存
 *    ⚠️ 这和内部参数区的 0xFF01 是同一个思路：有效标志最后写
 *============================================================================*/
typedef struct
{
	uint32_t magic;
	uint32_t size;
	uint32_t crc;
	uint32_t state;
	uint32_t rsv[12];
} UpStageHdr_t;

/* ★ U5：镜像里的元数据（App 侧一个普通 const 结构，"ATEM" 开头）
 *   ⚠️ 为什么按 magic 扫、而不是钉死地址：
 *     AC5 的 __attribute__((at(addr))) 在这套工程里不可靠 ——
 *     试过 0x08004200 和 0x08004800，两次都能编过、bin 里也对，
 *     但 App 跑到一半就静默（去掉 at 立刻正常）。
 *     与其和链接器较劲，不如让 Boot 扫一遍：只在升级时扫一次，1.7 秒。 */
typedef struct
{
	uint32_t magic;
	uint32_t version;
	uint32_t build;
	uint32_t rsv;
} UpMeta_t;

static uint8_t Up_FindMeta(uint32_t size, UpMeta_t *out)
{
	uint32_t off = 0u;
	uint32_t i;

	if (size < 16u) { return 0u; }

	while (off < size)
	{
		uint32_t n = size - off;
		uint32_t step;

		if (n > (uint32_t)BOOT_FLASH_PAGE_SIZE) { n = (uint32_t)BOOT_FLASH_PAGE_SIZE; }
		if (BootW25_Read(UP_STAGE_IMG_ADDR + off, s_page, n) == 0u) { return 0u; }

		/* ★ 逐字节扫，不能 4 字节跳
		 *   · 分块起点是 off += (n-15)，不是 4 的倍数
		 *   · 结构体在块内的偏移因此也不一定是 4 的倍数
		 *   · 按 4 步长会漏掉（第一次实现就是这么漏的：报 FAIL nometa） */
		for (i = 0u; (i + 16u) <= n; i++)
		{
			/* ★ 小端 "ATEM"：41 54 45 4D */
			if ((s_page[i]      == 0x41u) && (s_page[i + 1u] == 0x54u) &&
			    (s_page[i + 2u] == 0x45u) && (s_page[i + 3u] == 0x4Du))
			{
				uint32_t k;
				uint8_t *p = (uint8_t *)out;

				for (k = 0u; k < 16u; k++) { p[k] = s_page[i + k]; }
				if ((out->version == 0u) || (out->version > 100000u)) { continue; }
				if (out->rsv != 0u) { continue; }
				return 1u;
			}
		}

		/* ★ 留 15 字节重叠：结构体可能正好跨 1KB 边界 */
		step = (n > 15u) ? (n - 15u) : n;
		off += step;
	}

	return 0u;
}

static uint8_t Up_StageHdrWrite(uint32_t state, uint32_t size, uint32_t crc)
{
	UpStageHdr_t h;
	uint32_t i;
	uint32_t magic_only = UP_STAGE_HDR_MAGIC;
	uint32_t off = 0u;                          /* ★ magic 在结构体第一个字 */

	for (i = 0u; i < (uint32_t)(sizeof(h) / 4u); i++)
	{
		((uint32_t *)&h)[i] = 0xFFFFFFFFu;
	}
	h.size  = size;
	h.crc   = crc;
	h.state = state;

	if (BootW25_EraseSector(UP_STAGE_HDR_ADDR) == 0u) { return 0u; }
	if (BootW25_Write(UP_STAGE_HDR_ADDR, (const uint8_t *)&h, (uint32_t)sizeof(h)) == 0u)
	{
		return 0u;
	}

	/* ★ 最后补 magic（只清位不改别的位），失败就整条不算数 */
	if (state == UP_STAGE_NONE) { return 1u; }
	if (BootW25_Write(UP_STAGE_HDR_ADDR + off, (const uint8_t *)&magic_only, 4u) == 0u)
	{
		return 0u;
	}
	return 1u;
}

static uint8_t Up_StageHdrRead(UpStageHdr_t *h)
{
	if (BootW25_Read(UP_STAGE_HDR_ADDR, (uint8_t *)h, (uint32_t)sizeof(*h)) == 0u) { return 0u; }
	if (h->magic != UP_STAGE_HDR_MAGIC) { return 0u; }
	if ((h->size == 0u) || (h->size > (uint32_t)UP_STAGE_IMG_MAX)) { return 0u; }
	return 1u;
}

/* ★ 算暂存区里前 size 字节的 CRC16（和 PC 的 X 命令同一个算法） */
static uint16_t Up_StageCrc(uint32_t size)
{
	uint32_t off = 0u;
	uint32_t n;
	uint32_t i;
	uint16_t crc = 0u;

	while (off < size)
	{
		n = size - off;
		if (n > (uint32_t)BOOT_FLASH_PAGE_SIZE) { n = (uint32_t)BOOT_FLASH_PAGE_SIZE; }

		if (BootW25_Read(UP_STAGE_IMG_ADDR + off, s_page, n) == 0u) { return 0xFFFFu; }
		for (i = 0u; i < n; i++)
		{
			crc = Up_CrcUpdate(crc, s_page[i]);
		}
		off += n;
	}
	return crc;
}

/*==============================================================================
 *  ★ 把暂存区的整包搬进内部 Flash
 *  ★ 顺序：先写 1..N-1 页 → 置 image_ok=0 → 最后写第 0 页
 *    · 第 0 页是"新镜像生效"的提交点，必须最后
 *    · 每页都是 擦→写→回读，错一页就整条中止
 *  ★ 返回 0 = 成功
 *============================================================================*/
static uint8_t Up_InstallFromStage(uint32_t size)
{
	uint32_t pages = (size + BOOT_FLASH_PAGE_SIZE - 1u) / BOOT_FLASH_PAGE_SIZE;
	uint32_t p;
	uint8_t  e;

	/* ★ 2026-11-07：第一步就把第 0 页擦掉
	 *   · 它是"App 是否有效"的唯一判据（只看头两个字）
	 *   · 不擦的话，搬到一半断电 ⇒ 老向量表 + 新代码的混合镜像
	 *     ⇒ 下次开机判"有效"，直接跳进半新半旧的固件，比不跳还糟
	 *   · 擦掉之后 App 必无效 ⇒ 开机命中"App 无效"
	 *     ⇒ 要么被暂存区装回来，要么被金副本救回来，两条都行 */
	e = BootFlash_ErasePage((uint32_t)APP_BASE);
	if (e != BOOTFLASH_OK)
	{
		Serial_SendString("FAIL inst p0.r e=");
		Serial_SendHex((uint32_t)e, 2u);
		Serial_SendString("\r\n");
		return 6u;
	}

	for (p = 1u; p < pages; p++)
	{
		uint32_t waddr = UP_STAGE_IMG_ADDR + (p * BOOT_FLASH_PAGE_SIZE);
		uint32_t faddr = (uint32_t)APP_BASE + (p * BOOT_FLASH_PAGE_SIZE);

		if (BootW25_Read(waddr, s_page, (uint32_t)BOOT_FLASH_PAGE_SIZE) == 0u) { return 1u; }

		e = BootFlash_ErasePage(faddr);
		if (e == BOOTFLASH_OK) { e = BootFlash_WritePage(faddr, s_page); }
		if (e == BOOTFLASH_OK) { e = BootFlash_VerifyPage(faddr, s_page); }
		if (e != BOOTFLASH_OK)
		{
			Serial_SendString("FAIL inst e=");
			Serial_SendHex((uint32_t)e, 2u);
			Serial_SendString(" pg=");
			Serial_SendHex(p, 4u);
			Serial_SendString("\r\n");
			return 2u;
		}
	}

	/* ★ 提交点之前先把 image_ok 置 0（和原来的顺序一致） */
	if (Param_OnUpgrade() == 0u)
	{
		Serial_SendString("FAIL param\r\n");
		return 3u;
	}

	if (BootW25_Read(UP_STAGE_IMG_ADDR, s_page, (uint32_t)BOOT_FLASH_PAGE_SIZE) == 0u) { return 1u; }

	e = BootFlash_ErasePage((uint32_t)APP_BASE);
	if (e == BOOTFLASH_OK) { e = BootFlash_WritePage((uint32_t)APP_BASE, s_page); }
	if (e == BOOTFLASH_OK) { e = BootFlash_VerifyPage((uint32_t)APP_BASE, s_page); }
	if (e != BOOTFLASH_OK)
	{
		Serial_SendString("FAIL pg0 e=");
		Serial_SendHex((uint32_t)e, 2u);
		Serial_SendString("\r\n");
		return 4u;
	}

	if (Boot_IsAppValid() == 0u)
	{
		Serial_SendString("FAIL valid\r\n");
		return 5u;
	}

	return 0u;
}

/*==============================================================================
 *  ★ 把攒好的一页"落地"
 *
 *  ★ 两条路径：
 *    · 暂存模式（有 W25）：写进 W25 暂存区 + 回读比对，内部 Flash 不动
 *    · 老路径（没 W25）：第 0 页只暂存，其它页直接写内部 Flash
 *
 *  ★ 尾部补 0xFF（最后一页常常不满）
 *    · 补在缓冲里，让"写"永远写满一页
 *    · 而 CRC 天然不受影响（它是在收到的每个字节上边收边算的）
 *
 *  ★ 返回 1 = 这一页确认写对了；0 = 失败（已经打了 FAIL，调用者直接回 IDLE）
 *============================================================================*/
static uint8_t Up_FlushPage(void)
{
	uint16_t page = s_page_index;
	uint16_t i;
	uint8_t  e    = BOOTFLASH_OK;

	/* ★ 补齐 */
	for (i = s_page_bytes; i < (uint16_t)BOOT_FLASH_PAGE_SIZE; i++)
	{
		s_page[i] = 0xFFu;
	}

	if (s_stage != 0u)
	{
		/* ★ U3：只写 W25。s_page0 在这里当回读缓冲用（不再需要存第 0 页） */
		uint32_t waddr = UP_STAGE_IMG_ADDR + ((uint32_t)page * BOOT_FLASH_PAGE_SIZE);

		if (BootW25_Write(waddr, s_page, (uint32_t)BOOT_FLASH_PAGE_SIZE) == 0u)
		{
			Serial_SendString("FAIL stage wr\r\n");
			return 0u;
		}
		if (BootW25_Read(waddr, s_page0, (uint32_t)BOOT_FLASH_PAGE_SIZE) == 0u)
		{
			Serial_SendString("FAIL stage rd\r\n");
			return 0u;
		}
		for (i = 0u; i < (uint16_t)BOOT_FLASH_PAGE_SIZE; i++)
		{
			if (s_page0[i] != s_page[i])
			{
				Serial_SendString("FAIL stage cmp\r\n");
				return 0u;
			}
		}
	}
	else if (page == 0u)
	{
		/* ★ 第 0 页只搬走，不写
		 *   ★ 这就是老路径安全模型的支点，见 boot_upgrade.h。 */
		for (i = 0u; i < (uint16_t)BOOT_FLASH_PAGE_SIZE; i++)
		{
			s_page0[i] = s_page[i];
		}
		UP_LOG("BOOT: page0 buffered (not written yet)\r\n");
	}
	else
	{
		uint32_t addr = (uint32_t)APP_BASE + ((uint32_t)page * BOOT_FLASH_PAGE_SIZE);

		/* ★ 顺序不能换：先擦，再写，最后回读。 */
		e = BootFlash_ErasePage(addr);
		if (e == BOOTFLASH_OK) { e = BootFlash_WritePage(addr, s_page); }
		if (e == BOOTFLASH_OK) { e = BootFlash_VerifyPage(addr, s_page); }

		if (e != BOOTFLASH_OK)
		{
			/* ★ 带上错误码和页号
			 *   （错误码见 boot_flash.h 的码表，一眼就知道是擦坏了还是写坏了） */
			Serial_SendString("FAIL flash e=");
			Serial_SendHex((uint32_t)e, 2u);
			Serial_SendString(" pg=");
			Serial_SendHex((uint32_t)page, 4u);
			Serial_SendString("\r\n");
			return 0u;
		}
	}

	s_page_bytes = 0u;
	s_page_index++;
	return 1u;
}


/*==============================================================================
 *  ★ 处理 E<len> —— 声明长度 + 立刻毁掉第 0 页
 *============================================================================*/
static void Up_HandleStart(uint16_t n)
{
	uint32_t v = 0u;
	uint16_t i;
	uint8_t  e;

	if (n < 2u)
	{
		Serial_SendString("E1\r\n");            /* ★ 只有个 'E'，没数字。 */
		return;
	}

	for (i = 1u; i < n; i++)
	{
		char c = s_line[i];

		if ((c < '0') || (c > '9'))
		{
			Serial_SendString("E1\r\n");        /* ★ 有非数字。 */
			return;
		}

		/* ★ 防溢出 ——
		 *   ★ 为什么必须查：若 32 位回绕
		 *     → 可能变成一个很小，但"合法"的值
		 *     → 于是我们按它收，最后 CRC 却对不上，白折腾一轮 */
		if (v > 429496729u)
		{
			Serial_SendString("E1\r\n");
			return;
		}
		v = (v * 10u) + (uint32_t)(c - '0');
	}

	/* ★ 长度护栏（两条都不能少）
	 *   · v == 0：零长度CRC 也可能是 0
	 *     → "校验通过"，然后写个空第 0 页，跳进空区
	 *   · v > APP_SIZE：会写到 Bootloader 区，甚至越界
	 *     → 直接变砖 */
	if ((v == 0u) || (v > (uint32_t)APP_SIZE))
	{
		Serial_SendString("E1\r\n");
		return;
	}

	/* ★ 先擦第 0 页 ← 整个安全模型的"起手式"
	 *   ★ 此刻起，App 必然无效
	 *     → 之后无论发生什么（断电 / 拔线 / CRC 失败）
	 *       都不可能跳到半成品 */
	/* ★ U3（2026-11-07）：有 W25 就先落盘，内部 App 一个字节都不动
	 *   · 传输期间断电 ⇒ 旧固件照常能跑
	 *   · 只有整包校验通过之后，才去动内部 Flash
	 *   ★ 没有 W25 时退回老路径（先擦第 0 页） */
	s_stage = (BootW25_Init() != 0u) ? 1u : 0u;

	if (s_stage != 0u)
	{
		uint32_t sectors = ((uint32_t)APP_SIZE + 4095u) / 4096u;
		uint32_t k;

		(void)Up_StageHdrWrite(UP_STAGE_NONE, 0u, 0u);   /* 旧的暂存作废 */

		for (k = 0u; k < sectors; k++)
		{
			if (BootW25_EraseSector(UP_STAGE_IMG_ADDR + (k * 4096u)) == 0u)
			{
				Serial_SendString("FAIL stage erase\r\n");
				Up_Reset();
				return;
			}
		}
	}
	else
	{
		e = BootFlash_ErasePage((uint32_t)APP_BASE);
		if (e != BOOTFLASH_OK)
		{
			Serial_SendString("FAIL erase0 e=");
			Serial_SendHex((uint32_t)e, 2u);
			Serial_SendString("\r\n");
			Up_Reset();
			return;
		}
	}

	Up_Reset();
	s_expect = v;
	s_state  = UP_ST_RECV;                      /* ★ 必须在 Up_Reset() 之后设。 */

	Serial_SendString("E0\r\n");

	/* ★ 这一行必须无条件打：暂存与否是两种完全不同的安全模型，
	 *   测的时候要能一眼看出来走的哪条（UP_LOG 是编译期开关，靠不住） */
	Serial_SendString("BOOT: staging=");
	Serial_SendU32((uint32_t)s_stage);
	Serial_SendString("\r\n");

	UP_LOG("BOOT: expect len=");
	Serial_SendU32(s_expect);
	UP_LOG(" bytes; staging=");
	Serial_SendU32((uint32_t)s_stage);
	UP_LOG("\r\n");
}


/*==============================================================================
 *  ★ 处理 D<seq><hex> —— 一块数据
 *============================================================================*/
static void Up_HandleData(uint16_t n)
{
	uint16_t seq;
	uint16_t payload;
	uint16_t nbytes;
	uint16_t i;
	int8_t   hi;
	int8_t   lo;

	if (s_state != UP_ST_RECV)
	{
		/* ★ 没发 E 就直接发 D→ 明说，不装死。 */
		Up_SendFail("noE");
		return;
	}

	/* ★ 最短合法行：D + 4 位块号 + 至少 1 字节数据（2 个 hex）= 7。 */
	if (n < 7u)
	{
		Up_SendFail("dlen");
		return;
	}

	/* ---- 块号 ---- */
	seq = 0u;
	for (i = 1u; i < 5u; i++)
	{
		hi = Up_HexVal(s_line[i]);
		if (hi < 0) { Up_SendFail("seq"); return; }
		seq = (uint16_t)((seq << 4) | (uint16_t)hi);
	}

	/* ★ 块号不对 → 回 N<期望号>
	 *   ★ 为什么必须有这个：只有整体 CRC，发现丢块要等到最后
	 *   · 而丢一块之后后面的字节全都错位
	 *     → 最后一定 FAIL，但112KB 已经白传了 */
	if (seq != s_seq)
	{
		Up_SendN(s_seq);
		return;
	}

	/* ---- 数据长度 ---- */
	payload = (uint16_t)(n - 5u);
	if ((payload == 0u) || ((payload & 0x0001u) != 0u))
	{
		Up_SendFail("hlen");                    /* ★ 奇数个 hex 字符，不可能。 */
		return;
	}
	nbytes = (uint16_t)(payload >> 1);
	if (nbytes > UP_BLOCK_BYTES)
	{
		Up_SendFail("blk");
		return;
	}
	if ((s_total + (uint32_t)nbytes) > s_expect)
	{
		Up_SendFail("ovf");                     /* ★ 比声明的还多，不写。 */
		return;
	}

	/* ★ 先整体校一遍 hex，再动状态
	 *   ★ 为什么不能"边解析边存"：
	 *     · 假如第 100 个字符不是 hex
	 *       → 前面 49 个字节已经进了 s_page，CRC 也算过了
	 *       → 然后我们回 N 让 PC 重发
	 *         → 那块数据就被算了两次，整体 CRC 必错
	 *   → 所以校验和落盘必须分成两趟 */
	for (i = 0u; i < payload; i++)
	{
		if (Up_HexVal(s_line[5u + i]) < 0)
		{
			Up_SendFail("hex");
			return;
		}
	}

	/* ---- 落进页缓冲 + 更新 CRC / 计数 ---- */
	for (i = 0u; i < nbytes; i++)
	{
		hi = Up_HexVal(s_line[5u + (2u * i)]);
		lo = Up_HexVal(s_line[6u + (2u * i)]);

		s_page[s_page_bytes] = (uint8_t)(((uint8_t)hi << 4) | (uint8_t)lo);
		s_page_bytes++;
		s_total++;
		s_crc = Up_CrcUpdate(s_crc, s_page[s_page_bytes - 1u]);

		/* ★ 攒满一页就落地（8 块 = 1KB，永远正好在块边界） */
		if (s_page_bytes >= (uint16_t)BOOT_FLASH_PAGE_SIZE)
		{
			if (Up_FlushPage() == 0u)
			{
				Up_Reset();
				return;
			}
		}
	}

	/* ★ 收齐了，但最后一页可能不满→ 现在就补齐写掉
	 *   ★ 为什么要在这里写，而不是等 X 再写：
	 *     · 这样X 到来时只剩"写第 0 页"一件事
	 *     · 第 0 页的写入窗口尽可能短 */
	if ((s_total == s_expect) && (s_page_bytes != 0u))
	{
		if (Up_FlushPage() == 0u)
		{
			Up_Reset();
			return;
		}
	}

	/* ★ 写完 Flash 才回 K
	 *   ★ 为什么顺序不能反：
	 *     · 回 K = "这块好了，请发下一块"
	 *     · 而 PC 收到 K 立刻就发下一块
	 *     · 而我们擦写时要关中断，那段时间收到的字节全丢
	 *     → 所以必须先写完，再回 K（这就是"停等"的另一半） */
	Up_SendK(s_seq);
	s_seq++;
}


/*==============================================================================
 *  ★ 处理 X<crc> —— 整体校验 + 写第 0 页 + 跳
 *
 *  ★ 顺序一步都不能换：
 *    ① 长度对不对
 *    ② CRC 对不对
 *    ③ 写第 0 页（这是"让 App 生效"的唯一动作）
 *    ④ 回读第 0 页
 *    ⑤ App 有效性检查
 *    ⑥ 清升级标志
 *    ⑦ 回 PASS，等串口发完，跳
 *============================================================================*/
static void Up_HandleEnd(uint16_t n)
{
	uint16_t want = 0u;
	uint16_t i;
	uint8_t  e;
	int8_t   hv;

	if (s_state != UP_ST_RECV)
	{
		Up_SendFail("noE");
		return;
	}

	if (n != 5u)
	{
		Up_SendFail("xlen");
		return;
	}

	for (i = 1u; i < 5u; i++)
	{
		hv = Up_HexVal(s_line[i]);
		if (hv < 0) { Up_SendFail("crcfmt"); return; }
		want = (uint16_t)((want << 4) | (uint16_t)hv);
	}

	/* ★ ① 长度 */
	if (s_total != s_expect)
	{
		Up_SendFail("short");
		Up_Reset();
		return;
	}

	/* ★ ② 整体 CRC —— 最后一道闸
	 *   ★ 为什么每块已经对了还要它：
	 *     · 块号只防"丢块 / 乱序"
	 *     · 块里的位错误，只有 CRC 能发现 */
	if (s_crc != want)
	{
		Serial_SendString("FAIL crc got=");
		Serial_SendHex((uint32_t)s_crc, 4u);
		Serial_SendString(" want=");
		Serial_SendHex((uint32_t)want, 4u);
		Serial_SendString("\r\n");
		Up_Reset();
		return;
	}

	if (s_stage != 0u)
	{
		/* ★ U3：整包已经在 W25 暂存区里
		 *   · 先写"就绪"头：从现在起，下次开机可以自动重装
		 *   · 再搬进内部：搬到一半断电，头还在 ⇒ 开机自己装回去
		 *   · 搬成功后清头：暂存已消费 */
		uint8_t r;
		UpMeta_t m;

		/* ★ U5（2026-11-07）：防回滚
		 *   · 版本号在镜像里（"ATEM" 开头的 16 字节），跟着镜像走，PC 端不用改
		 *   · 比参数区记的 img_min_ver 小 ⇒ 拒绝，不写任何 Flash
		 *   · 没有元数据也拒绝：否则"剥掉元数据"就能绕过防回滚 */
		if (Up_FindMeta(s_expect, &m) == 0u)
		{
			Up_SendFail("nometa");
			Up_Reset();
			return;
		}
		if (m.version < Param_GetMinVer())
		{
			Serial_SendString("FAIL downgrade ver=");
			Serial_SendU32(m.version);
			Serial_SendString(" min=");
			Serial_SendU32(Param_GetMinVer());
			Serial_SendString("\r\n");
			Up_Reset();
			return;
		}

		if (Up_StageHdrWrite(UP_STAGE_READY, s_expect, (uint32_t)s_crc) == 0u)
		{
			Up_SendFail("stagehdr");
			Up_Reset();
			return;
		}

		r = Up_InstallFromStage(s_expect);
		if (r != 0u)
		{
			/* ★ 装失败：清头，不再自动重试（同一块 Flash 大概率还失败） */
			(void)Up_StageHdrWrite(UP_STAGE_NONE, 0u, 0u);
			Serial_SendString("FAIL install r=");
			Serial_SendHex((uint32_t)r, 2u);
			Serial_SendString("\r\n");
			Up_Reset();
			return;
		}

		/* ★ 装成功才抬高下限：装到一半失败时，旧版本还得能装回去 */
		(void)Param_SetMinVer(m.version);

		(void)Up_StageHdrWrite(UP_STAGE_NONE, 0u, 0u);
	}
	else
	{
		/* ★ 2026-11-07：先把 image_ok 置 0，再写第 0 页。
		 *   · 第 0 页 = "新镜像生效"的提交点；
		 *   · 如果先写第 0 页、后写 image_ok，中间掉电会留下
		 *     "新镜像有效 + image_ok 还是 1" ⇒ 绕过试用/回退。
		 *   · 这里失败就中止升级，不写第 0 页。 */
		if (Param_OnUpgrade() == 0u)
		{
			Up_SendFail("param");
			Up_Reset();
			return;
		}

		/* ★ ③ 写第 0 页
		 *   ★ 为什么还要再擦一次：
		 *     · 它在 E 的时候就擦过了，理论上就是 0xFF
		 *     · 但擦一个已经是 0xFF 的页是幂等的，不花钱（40ms）
		 *     · 万一中间被什么写脏了，这一步顺手保证干净
		 *       → 后面回读比对会把任何异常兜住。 */
		e = BootFlash_ErasePage((uint32_t)APP_BASE);
		if (e == BOOTFLASH_OK) { e = BootFlash_WritePage((uint32_t)APP_BASE, s_page0); }
		if (e == BOOTFLASH_OK) { e = BootFlash_VerifyPage((uint32_t)APP_BASE, s_page0); }
		if (e != BOOTFLASH_OK)
		{
			Serial_SendString("FAIL pg0 e=");
			Serial_SendHex((uint32_t)e, 2u);
			Serial_SendString("\r\n");
			Up_Reset();
			return;
		}

		/* ★ ⑤ 现在第 0 页才真正有效，用和 Bootloader 开头同一套判据再验一次
		 *   ★ 为什么不用自己"重算一遍头两个字"：
		 *     → 那会多一份实现，而两份判据迟早会不一致 */
		if (Boot_IsAppValid() == 0u)
		{
			Up_SendFail("valid");
			Up_Reset();
			return;
		}
	}

	/* ★ ⑥ 清标志 —— 到这一步才清是故意的
	 *   ★ 早清一步：中途断电，复位后标志没了，App 又无效
	 *     → 但现在"App 无效"本身就是第二条进入条件，依然安全
	 *       （这就是Q5 那个"或"条件的价值） */
	BootFlag_ClearUpgrade();

	/* ★ 2026-10-18 加（复盘 B2 的另一半）：
	 *   ★ 新固件写进去 + 校验通过 + 第 0 页有效⇒ 升级成功
	 *     ⇒ 任何还挂着的"回退请求"都该作废
	 *   ★ 不清会怎样：
	 *     · 用户之前在 App 里发过 R 但没恢复成功 ⇒ DR3 残留
	 *     · 这次升级 PASS 了、跳进新固件
	 *     · 可下次开机Boot 又看到 DR3 ⇒ 又去回退
	 *       ⇒ 刚升好的新固件被换回旧的
	 *   ⇒ 升级成功 = 回退请求作废，一起清掉。 */
	BootFlag_ClearRollback();

	/* ★ 2026-10-18 加（⑥）：新固件装好并验证通过 ⇒ IWDG 回退计数清零
	 *   ★ 语义：这算是"人重新给了它一次完整机会"
	 *   ★ 不清会怎样：以后偶发一次狗咬就直接停升级模式
	 *     而不是先尝试自愈 */
	BootFlag_ClearIwdgRetry();

	Serial_SendString("PASS\r\n");
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) { }

	Serial_SendString("BOOT: upgrade OK -> jumping to App\r\n");
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) { }

	/* ★ ⑦ 跳（不会返回） */
	Boot_JumpToApp();
}


/*==============================================================================
 *  ★ 处理 C —— 取消升级
 *
 *  ★ 为什么只在 IDLE 允许：
 *    · IDLE 时App 区还是完整的 → 跳过去是安全的
 *    · RECV 时第 0 页已经被擦了
 *      → "取消"也跳不了，只会让用户更困惑
 *============================================================================*/
static void Up_HandleCancel(void)
{
	if (s_state == UP_ST_RECV)
	{
		/* ★ 明说，不装死（而且绝不能在这里清标志或跳转） */
		Up_SendFail("busy");
		return;
	}

	UP_LOG("BOOT: cancel requested -> checking App...\r\n");

	BootFlag_ClearUpgrade();
	/* ★ 2026-11-07：参数区也要清。
	 *   · 新 Boot 的升级请求以参数区为准；
	 *   · 只清 BKP 的话，C 取消后下次开机又看到 flags=01，永远进升级模式。 */
	(void)Param_ClearFlags();
	UP_LOG("BOOT: upgrade flag cleared\r\n");

	if (Boot_IsAppValid() != 0u)
	{
		Serial_SendString("BOOT: app valid -> jumping\r\n");
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) { }
		Boot_JumpToApp();                       /* ★ 不会返回。 */
	}

	/* ★ App 无效 → 不能跳，继续等
	 *   ★ 这是安全阀：宁可不响应，也不跳到坏固件 */
	Serial_SendString("BOOT: but App is INVALID -> stay in upgrade mode\r\n");
	Serial_SendString("BOOT: (send E<size> to start an upgrade)\r\n");
}


/*==============================================================================
 *  ★ CRC 自检（见 boot_upgrade.h）
 *============================================================================*/
uint8_t BootUpgrade_SelfTest(void)
{
	static const char anchor[] = "123456789";   /* ★ 标准判据串。 */
	uint16_t crc = 0u;
	uint8_t  i;

	for (i = 0u; anchor[i] != '\0'; i++)
	{
		crc = Up_CrcUpdate(crc, (uint8_t)anchor[i]);
	}

	return (crc == 0x2189u) ? 1u : 0u;
}


/*==============================================================================
 *  ★ 空闲超时能不能直接跳回 App（2026-11-08 加）
 *
 *  ★ 三个条件全满足才允许：
 *    ① App 仍是有效的（第 0 页没被擦 ⇒ 内部 Flash 一个字节都没动过）
 *    ② 暂存区里没有"就绪"的镜像（有的话说明有个完整镜像等着装，别抢）
 *    ③ 当前在 IDLE（还没收到 E）
 *
 *  ★ 为什么第一条就够安全：
 *    · 有效判据只看第 0 页两个字；它没动，说明 App 区就是上次跑成功的那一份
 *    · 这正好对应"传输阶段断线/断电"—— U3 之后那段确实不碰内部 Flash
 *============================================================================*/
static uint8_t Up_CanBootInstead(void)
{
	UpStageHdr_t h;

	if (Boot_IsAppValid() == 0u) { return 0u; }

	/* ★ 没有 W25 就没有暂存，直接允许 */
	if (BootW25_Init() == 0u) { return 1u; }
	if (Up_StageHdrRead(&h) == 0u) { return 1u; }
	if (h.state != UP_STAGE_READY) { return 1u; }

	return 0u;                          /* 有就绪镜像 ⇒ 交给自愈那条路 */
}


/*==============================================================================
 *  ★ 主循环（永不返回，出口只有"跳 App"）
 *============================================================================*/
void BootUpgrade_Run(void)
{
	uint16_t n;
	uint8_t  r;
	uint32_t idle_waits = 0u;

	/* ★ 开工前先验契约（理由见 boot_upgrade.h）
	 *   ★ 对不上就绝对不许开始收固件
	 *     → 否则收完 112KB 才在最后一步 FAIL，白折腾 */
	if (BootUpgrade_SelfTest() == 0u)
	{
		Serial_SendString("BOOT: CRC SELFTEST FAILED -> refusing to upgrade\r\n");
		for (;;) { BOOT_FEED_IWDG(); }          /* ★ 停住，但继续喂狗。 */
	}
	Serial_SendString("BOOT: CRC selftest OK (0x2189)\r\n");

	Up_Reset();

	Serial_SendString("BOOT: upgrade mode - E<size> / D<seq><hex> / X<crc>\r\n");
	Serial_SendString("BOOT: 'C' cancels (only before any flash write)\r\n");

	for (;;)
	{
		BOOT_FEED_IWDG();

		r = Up_ReadLine(&n, (s_state == UP_ST_RECV) ? UP_RECV_POLLS : UP_IDLE_POLLS);

		if (r == 0u)
		{
			if (s_state == UP_ST_RECV)
			{
				/* ★ 收数据收到一半卡住→ 回 IDLE，让 PC 重来
				 *   ★ 为什么敢回 IDLE：第 0 页已经擦了
				 *     "App 无效"这条硬条件保证跳不出去 */
				Up_SendFail("timeout");
				Up_Reset();
			}
			else
			{
				/* ★ 心跳（让人知道"它还在等，没死"）
				 *   ★ 只在 IDLE 打—— 传输中打会干扰 PC 的行解析 */
				Serial_SendString("WAIT\r\n");
				idle_waits++;

				/* ★ 30 秒没人理就试着回 App（见 UP_IDLE_CANCEL_WAITS 的说明） */
				if (idle_waits >= UP_IDLE_CANCEL_WAITS)
				{
					idle_waits = 0u;
					if (Up_CanBootInstead() != 0u)
					{
						Serial_SendString("BOOT: idle, app still valid -> cancel & boot\r\n");
						while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) { }
						BootFlag_ClearUpgrade();
						(void)Param_ClearFlags();
						Boot_JumpToApp();        /* ★ 不会返回 */
					}
				}
			}
			continue;
		}

		if (r == 2u)
		{
			Up_SendFail("long");                /* ★ 这一行超长，已丢弃。 */
			Up_Reset();
			continue;
		}

		if (n == 0u) { continue; }              /* ★ 空行，忽略。 */

		/* ★ 调试回显，只打前几个字符
		 *   （打整行会刷屏，而且 D 行本来就有 264 个字符） */
		#if BOOT_UPGRADE_DEBUG
		{
			uint16_t k;
			Serial_SendString("RX[");
			Serial_SendU32((uint32_t)n);
			Serial_SendString("] ");
			for (k = 0u; (k < 10u) && (k < n); k++)
			{
				Serial_SendByte((uint8_t)s_line[k]);
			}
			Serial_SendString("\r\n");
		}
		#endif

		switch (s_line[0])
		{
			case 'E': case 'e': Up_HandleStart(n);  break;
			case 'D': case 'd': Up_HandleData(n);   break;
			case 'X': case 'x': Up_HandleEnd(n);    break;   /* ★ 成功时不返回。 */
			case 'C': case 'c': Up_HandleCancel();  break;

			default:
				/* ★ 未知命令 → 明说，别静默。 */
				Up_SendFail("cmd");
				break;
		}
	}
}

/*==============================================================================
 *  ★ 开机自愈：暂存区里有完整镜像、内部 App 又失效 ⇒ 直接重装
 *
 *  ★ 什么时候会走到这里：
 *    · 升级装到一半断电（第 0 页还没写）⇒ 内部 App 无效
 *    · 而 W25 里的暂存头还是"就绪" ⇒ 不用接电脑，自己装回去
 *
 *  ★ 为什么先验 CRC 再装：
 *    · 暂存区也可能被写坏（掉电/坏块），装一个坏镜像比不装更糟
 *    · 校验不过就清头，避免每次开机都白折腾 1.7 秒
 *
 *  ★ 返回值：1 = 已装好（调用者要重新判 App 是否有效）；0 = 没装
 *============================================================================*/
uint8_t Up_RecoverFromStage(void)
{
	UpStageHdr_t h;
	uint16_t     crc;

	if (BootW25_Init() == 0u) { return 0u; }
	if (Up_StageHdrRead(&h) == 0u) { return 0u; }
	if (h.state != UP_STAGE_READY) { return 0u; }

	Serial_SendString("G: stage found -> verify\r\n");

	crc = Up_StageCrc(h.size);
	if (crc != (uint16_t)h.crc)
	{
		Serial_SendString("G: stage CRC bad -> drop\r\n");
		(void)Up_StageHdrWrite(UP_STAGE_NONE, 0u, 0u);
		return 0u;
	}

	Serial_SendString("G: stage ok -> installing\r\n");

	if (Up_InstallFromStage(h.size) != 0u)
	{
		(void)Up_StageHdrWrite(UP_STAGE_NONE, 0u, 0u);
		Serial_SendString("G: stage install FAILED\r\n");
		return 0u;
	}

	(void)Up_StageHdrWrite(UP_STAGE_NONE, 0u, 0u);
	Serial_SendString("G: stage installed -> retry boot\r\n");
	return 1u;
}
