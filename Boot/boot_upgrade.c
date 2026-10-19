#include "stm32f10x.h"
#include "boot_config.h"
#include "boot_jump.h"
#include "boot_flag.h"
#include "boot_flash.h"
#include "boot_upgrade.h"
#include "Serial.h"

/*==============================================================================
 *  ★★★★ 升级协议状态机（2026-10-13 建 ✓）
 *        安全模型 / 协议 / 为什么不那样做 —— 全部写在 boot_upgrade.h ✓
 *============================================================================*/

/* ---- 调试输出开关（★ 见 boot_upgrade.h ✓）-------------------------------- */
#if BOOT_UPGRADE_DEBUG
  #define UP_LOG(s)      Serial_SendString(s)
#else
  #define UP_LOG(s)      do { } while (0)
#endif

/* ---- 超时用"空轮次数" ✗ 不用毫秒（★ 理由见下 ✓）---------------------------
 *
 *  ★★★★ 为什么【接收循环里【绝不能【塞一个粗延时】】✗✓：
 *    · ★ 115200 ✗ 8N1 → 【11.52 字节/毫秒】✓
 *    · ★★ 而【环形缓冲只有 64 字节】✗（★ `Serial.c` 里 `SER_RX_BUF_SIZE` ✓）
 *      → 【64 ÷ 11.52 ≈ 【5.6 毫秒就满】】✓✓
 *    · ★★★ 原版等待循环是【100ms 一轮**✗ → 【缓冲溢出约 18 倍 ✗ 必丢字节】✓✓
 *    → ★★★★ 【所以【必须是【紧密轮询 ✗ 一轮只做"取一个字节 + 判空"】】✓✓
 *
 *  ★★ 那超时怎么算 ✗ → 【数"空转了多少轮"】✓
 *    · ★ 一轮 ≈ 一次 `Serial_RxPop()` + 判空 ≈ **30 周期 ≈ 0.4µs** ✓
 *    · ★★ 【5,000,000 轮 ≈ 2 秒】✓
 *    · ★★★ 精度【完全不重要**✗✓：
 *      → 【超时只用来【放弃一条【卡住的半截命令】】✓
 *      → 【早 0.5 秒 / 晚 0.5 秒【都安全 ✗ 因为【失败的出口永远是"回 IDLE"】】✓✓
 *    · ★★★★ 【而且【不用 SysTick**✗：★ `stm32f10x_it.c` 里【已经有】一个
 *      `SysTick_Handler()` ✓ → 【我们再定义一次 = 【重复符号 ✗ 链接不过】】✓✓
 *      （★ 更别说【跳转前还要记得关掉它 ✗ 平白多一份状态】✓）*/
#define UP_IDLE_POLLS     5000000u    /* ★ 空闲态：约 2 秒没动静 → 打一次心跳 ✓ */
#define UP_RECV_POLLS    10000000u    /* ★★ 收数据态：约 4 秒收不满一行 → 判超时 ✓ */

/* ---- 状态 ------------------------------------------------------------------ */
typedef enum
{
	UP_ST_IDLE = 0,                   /* ★ 等 `E<len>`（★ 或 `C` ✓）*/
	UP_ST_RECV                        /* ★★ 正在收 `D` / `X` ✓ */
} UpState_t;

/* ---- 缓冲（★ 全部 static ✗ 【绝不放栈】✓ ）---------------------------------
 *  ★ 为什么【不放栈**✗✓：
 *    · ★ 两个 1KB 缓冲 = 512 words ✗
 *    · ★★ Bootloader 的栈【只有启动文件里那个 `Stack_Size`】✗（★ 通常 0x400 ✓）
 *      → 【直接【爆栈 ✗ 而且是【静默踩到别的变量】✗ 极难查】✓✓
 *  ★★ 为什么【不放堆**✗：★ 裸机【没有 malloc**✗ 也没有 FreeRTOS 的 heap ✓
 *  ★★★ RAM 【完全够**✗：★ 20KB 里 ✗ ZI 原本才 1.1KB ✓✓ */
static uint8_t  s_page0[BOOT_FLASH_PAGE_SIZE];   /* ★★★★ 第 0 页暂存 ✗ 最后才写 ✓ */
static uint8_t  s_page [BOOT_FLASH_PAGE_SIZE];   /* ★★ 当前正在攒的页 ✓ */
static char     s_line [UP_LINE_MAX];            /* ★ 一行命令缓冲 ✓ */

static UpState_t s_state;
static uint16_t  s_page_bytes;    /* ★ 当前页已攒字节数 ✓ */
static uint16_t  s_page_index;    /* ★★ 当前是第几页（★ 0 = 第 0 页 ✗ 只暂存 ✓）*/
static uint32_t  s_expect;        /* ★ E<len> 声明的总字节数 ✓ */
static uint32_t  s_total;         /* ★★ 已经收了多少字节 ✓ */
static uint16_t  s_crc;           /* ★★★ 边收边算的 CRC-16/KERMIT ✓ */
static uint16_t  s_seq;           /* ★★★★ 期望的下一块块号 ✓ */


/*==============================================================================
 *  ★★★★ 小工具
 *============================================================================*/

/* ★ hex 字符 → 4 bit ✗ 返回 -1 = 不是 hex ✓ */
static int8_t Up_HexVal(char c)
{
	if ((c >= '0') && (c <= '9')) { return (int8_t)(c - '0'); }
	if ((c >= 'A') && (c <= 'F')) { return (int8_t)(c - 'A' + 10); }
	if ((c >= 'a') && (c <= 'f')) { return (int8_t)(c - 'a' + 10); }
	return (int8_t)-1;
}

/* ★★★★ CRC-16/KERMIT【增量】更新 ✗✓
 *
 *  ★ 为什么【要"增量"版**✗：
 *    · ★ 最终 CRC 是【同一个值**✗ 但【不能在最后"把 **112KB** 读一遍再算"】✓
 *    · ★★ 【边收边算**✗ → 【收完的那一刻 CRC 已经好了】✓✓
 *
 *  ★★ 算法【和 App 的 `Hardware\Crc16.c` 【一字不差的契约】】✗✓
 *    · ★ 反射算法 ✗ 每次右移一位 ✓
 *    · ★★ 最低位为 1 就异或【反射多项式 `0x8408`】✓
 *    · ★★★ 初值 0 ✗ 无输出异或 ✓
 *    · ★★★★ 判据：`CRC("123456789") == 0x2189` ✓（★ 由 SelfTest 守着 ✓）
 *
 *  ★★★★ 【2026-10-14 起：本函数【不是 static 了】】✗✓
 *    · ★ 原因 ✗ 金副本回退（`boot_golden.c`）【也要用同一个 CRC】✓
 *    · ★★ 【绝不能再写第二份**✗✓ —— ★ 这条是【今天踩出来的教训】✓
 *      （★ 两个变体 `0x31C3` / `0x29B1` 混起来的混乱 ✗ 见 30 号 ✓）
 *    → ★★★ 【全 Bootloader【只有这一份**✗ 声明在 `boot_upgrade.h` ✓✓ */
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

/* ★ 回一条 `K<块号>` ✗ 表示"这块收好了 ✗ 而且【该写的都写完了】"✓ */
static void Up_SendK(uint16_t seq)
{
	Serial_SendString("K");
	Serial_SendHex((uint32_t)seq, 4u);
	Serial_SendString("\r\n");
}

/* ★★ 回一条 `N<块号>` ✗ 表示"块号不对 ✗ 请【从这个号】重发"✓ */
static void Up_SendN(uint16_t seq)
{
	Serial_SendString("N");
	Serial_SendHex((uint32_t)seq, 4u);
	Serial_SendString("\r\n");
}

/* ★★★ 回一条 `FAIL <原因>` ✗✓
 *   ★ 按约定：【关键路径【不许静默】】✗
 *     → 【失败【必须】带着原因说出来 ✗ 否则只能干看着】✓✓ */
static void Up_SendFail(const char *why)
{
	Serial_SendString("FAIL ");
	Serial_SendString(why);
	Serial_SendString("\r\n");
}

/* ★★★ 清空一轮传输的全部状态 ✗✓
 *   ⚠️ 【不会】去动 Flash ✗（★ 第 0 页【已经被擦掉了 ✗ 保持无效是对的 ✓）*/
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
 *  ★★★★ 读一整行（★ 紧密轮询 ✗ 无粗延时 ✓）
 *
 *  ★ 返回 ✗：
 *    · **1** = 收到了一整行（★ `*out_len` = 长度 ✗ 0 = 空行 ✓）
 *    · **0** = 超时（★ 到了 `max_empty_polls` 轮【一个字节都没来】✓）
 *    · **2** = 这一行【太长了**✗ 已经丢弃 ✗ 请重发 ✓
 *
 *  ★★ 为什么【按字节流解析 ✗ 而不是"先读满 264 字节"**✗：
 *    · ★ 我们【不知道 PC 会发多长**✗✓
 *    · ★★ 【必须【看到 `\n` 才收工 ✗ 否则【会一直等下去】】✓✓
 *    · ★★★ 而且【环形缓冲只有 64 字节**✗ → 【我们【必须【边到边取】】✓✓
 *
 *  ★★★ `\r` 【直接丢掉**✗：★ 判行尾【只认 `\n`**✓
 *    → 【这样 PC 发 `\r\n` ✗ 或者只发 `\n` ✗ 【都能跑】✓✓
 *============================================================================*/
static uint8_t Up_ReadLine(uint16_t *out_len, uint32_t max_empty_polls)
{
	uint16_t n        = 0u;
	uint8_t  overflow = 0u;
	uint32_t empty    = 0u;
	int16_t  ch;

	for (;;)
	{
		/* ★★★★ 每一轮都喂狗 ✗✓（★ 理由见 boot_config.h ✓）*/
		BOOT_FEED_IWDG();

		ch = Serial_RxPop();

		if (ch < 0)
		{
			if (++empty >= max_empty_polls)
			{
				return 0u;                      /* ★ 超时 ✓ */
			}
			continue;
		}

		empty = 0u;                             /* ★ 来了字节 ✗ 超时计数清零 ✓ */

		if ((char)ch == '\n')
		{
			s_line[n] = '\0';
			*out_len  = overflow ? 0xFFFFu : n;
			return overflow ? 2u : 1u;
		}

		if ((char)ch == '\r') { continue; }      /* ★ 丢掉 ✓ */

		if (n < (uint16_t)(UP_LINE_MAX - 1u))
		{
			s_line[n] = (char)ch;
			n++;
		}
		else
		{
			/* ★★ 太长了 ✗ 但【还得继续读到 `\n`**✗
			 *   → 【否则【下一行会被【当成这一行的尾巴】】✓✓ */
			overflow = 1u;
		}
	}
}


/*==============================================================================
 *  ★★★★ 把攒好的一页"落地"✗✓
 *
 *  ★ 规则【只有两条】✗✓：
 *    · ★ 第 0 页（`s_page_index == 0`）→ 【只搬进 `s_page0` ✗ 【一个字节都不写 Flash】】✓
 *    · ★★ 其它页 → 【擦 → 写 → 【立刻回读比对】】✓✓
 *
 *  ★★ 尾部补 `0xFF` ✗✓（★ 最后一页常常不满 ✓）
 *    · ★ 【补在缓冲里**✗ 让"写"永远写满一页 ✓
 *    · ★★ 【而 CRC 【天然不受影响】】✗✓
 *      → 【因为【它是在【收到的每一个字节】上【边收边算】的】✓
 *      → ★★★ 【补进缓冲的那些 0xFF【从来没进过 CRC】✓✓
 *        （★ 要是"最后把整页喂进 CRC"✗ 就会【和 PC 算的不一样】✗ 必 FAIL ✓）
 *
 *  ★★★ 返回 1 = 这一页【确认写对了】✓；0 = 失败（★ 已经打了 FAIL ✗ 调用者直接回 IDLE ✓）
 *============================================================================*/
static uint8_t Up_FlushPage(void)
{
	uint16_t page = s_page_index;
	uint16_t i;
	uint8_t  e    = BOOTFLASH_OK;

	/* ★ 补齐 ✗✓ */
	for (i = s_page_bytes; i < (uint16_t)BOOT_FLASH_PAGE_SIZE; i++)
	{
		s_page[i] = 0xFFu;
	}

	if (page == 0u)
	{
		/* ★★★★ 第 0 页【只搬走 ✗ 不写】✓
		 *   ★★ 【这就是整个安全模型的支点**✗ 见 boot_upgrade.h ✓ */
		for (i = 0u; i < (uint16_t)BOOT_FLASH_PAGE_SIZE; i++)
		{
			s_page0[i] = s_page[i];
		}
		UP_LOG("BOOT: page0 buffered (not written yet)\r\n");
	}
	else
	{
		uint32_t addr = (uint32_t)APP_BASE + ((uint32_t)page * BOOT_FLASH_PAGE_SIZE);

		/* ★ 顺序【不能换**✗✓：★ 先擦 ✗ 再写 ✗ 最后回读 ✓ */
		e = BootFlash_ErasePage(addr);
		if (e == BOOTFLASH_OK) { e = BootFlash_WritePage(addr, s_page); }
		if (e == BOOTFLASH_OK) { e = BootFlash_VerifyPage(addr, s_page); }

		if (e != BOOTFLASH_OK)
		{
			/* ★★★ 带上【错误码】和【页号】✗✓
			 *   （★ 错误码见 boot_flash.h 的码表 ✗ 一眼就知道是擦坏了还是写坏了 ✓）*/
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
 *  ★★★★ 处理 `E<len>` —— 声明长度 + 【立刻毁掉第 0 页】✗✓
 *============================================================================*/
static void Up_HandleStart(uint16_t n)
{
	uint32_t v = 0u;
	uint16_t i;
	uint8_t  e;

	if (n < 2u)
	{
		Serial_SendString("E1\r\n");            /* ★ 只有个 'E' ✗ 没数字 ✓ */
		return;
	}

	for (i = 1u; i < n; i++)
	{
		char c = s_line[i];

		if ((c < '0') || (c > '9'))
		{
			Serial_SendString("E1\r\n");        /* ★ 有非数字 ✓ */
			return;
		}

		/* ★★ 防溢出 ✗✓ ——
		 *   ★ 为什么【必须查**✗：★ 若 32 位【回绕**✗
		 *     → 【可能变成一个【很小 ✗ 但"合法"的值】】✓
		 *     → 【于是【我们【按它收 ✗ 最后 CRC 却对不上】✗ 白折腾一轮 ✓✓ */
		if (v > 429496729u)
		{
			Serial_SendString("E1\r\n");
			return;
		}
		v = (v * 10u) + (uint32_t)(c - '0');
	}

	/* ★★★ 长度护栏 ✗✓（★ 两条都不能少 ✓）
	 *   · ★ `v == 0` ✗：★ 零长度【CRC 也可能是 0**✗
	 *     → 【"校验通过"✗ 然后【写个空第 0 页 ✗ 跳进空区】】✓✓
	 *   · ★★ `v > APP_SIZE` ✗：★ 【会写到 Bootloader 区 ✗ 甚至越界】✓
	 *     → 【直接变砖】✓✓ */
	if ((v == 0u) || (v > (uint32_t)APP_SIZE))
	{
		Serial_SendString("E1\r\n");
		return;
	}

	/* ★★★★ 【先擦第 0 页】**✗✓ ← 整个安全模型的"起手式" ✓
	 *   ★ 此刻起 ✗ 【App 必然无效**✗✓
	 *     → 【之后【无论发生什么（★ 断电 / 拔线 / CRC 失败 ✓）
	 *       【都不可能跳到半成品】】✓✓ */
	e = BootFlash_ErasePage((uint32_t)APP_BASE);
	if (e != BOOTFLASH_OK)
	{
		Serial_SendString("FAIL erase0 e=");
		Serial_SendHex((uint32_t)e, 2u);
		Serial_SendString("\r\n");
		Up_Reset();
		return;
	}

	Up_Reset();
	s_expect = v;
	s_state  = UP_ST_RECV;                      /* ★ 必须在 Up_Reset() 之后设 ✓ */

	Serial_SendString("E0\r\n");

	UP_LOG("BOOT: expect len=");
	Serial_SendU32(s_expect);
	UP_LOG(" bytes; page0 ERASED -> App is now INVALID\r\n");
}


/*==============================================================================
 *  ★★★★ 处理 `D<seq><hex>` —— 一块数据
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
		/* ★★ 没发 `E` 就直接发 `D` ✗ → 【明说 ✗ 不装死】✓ */
		Up_SendFail("noE");
		return;
	}

	/* ★ 最短合法行 ✗：`D` + 4 位块号 + 至少 1 字节数据（2 个 hex）= 7 ✓ */
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

	/* ★★★ 块号不对 ✗ → 【回 `N<期望号>`】✗✓
	 *   ★ 为什么【必须有这个**✗：★【只有整体 CRC ✗ 发现丢块要等到【最后】】✓
	 *   · ★★ 而【丢一块之后【后面的字节【全都错位】】✗
	 *     → 【最后一定 FAIL ✗ 但【112KB 已经白传了】】✓✓ */
	if (seq != s_seq)
	{
		Up_SendN(s_seq);
		return;
	}

	/* ---- 数据长度 ---- */
	payload = (uint16_t)(n - 5u);
	if ((payload == 0u) || ((payload & 0x0001u) != 0u))
	{
		Up_SendFail("hlen");                    /* ★ 奇数个 hex 字符 ✗ 不可能 ✓ */
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
		Up_SendFail("ovf");                     /* ★★ 比声明的还多 ✗ 不写 ✓ */
		return;
	}

	/* ★★★★ 【先整体校一遍 hex ✗ 再动状态】✗✓
	 *   ★ 为什么【不能"边解析边存"**✗：
	 *     · ★ 假如【第 100 个字符不是 hex】✗
	 *       → 【前面 49 个字节【已经进了 s_page ✗ CRC 也算过了】】✓
	 *       → 【然后我们回 `N` 让 PC 重发**✗
	 *         → 【那块数据【就被算了两次】✗ 整体 CRC 必错】✓✓
	 *   → ★★ 【所以【校验和落盘【必须分成两趟】】✓✓ */
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

		/* ★ 攒满一页就落地 ✗✓（★ 8 块 = 1KB ✗ 永远【正好在块边界】✓）*/
		if (s_page_bytes >= (uint16_t)BOOT_FLASH_PAGE_SIZE)
		{
			if (Up_FlushPage() == 0u)
			{
				Up_Reset();
				return;
			}
		}
	}

	/* ★★ 收齐了 ✗ 但【最后一页可能不满**✗ → 【现在就补齐写掉】✓
	 *   ★ 为什么【要在这里写 ✗ 而不是【等 `X` 再写】**✗：
	 *     · ★ 这样【`X` 到来时【只剩"写第 0 页"一件事】】✓
	 *     · ★★ 【第 0 页的写入【窗口【尽可能短】】✓✓ */
	if ((s_total == s_expect) && (s_page_bytes != 0u))
	{
		if (Up_FlushPage() == 0u)
		{
			Up_Reset();
			return;
		}
	}

	/* ★★★★ 【写完 Flash 才回 `K`】**✗✓
	 *   ★ 为什么【顺序不能反**✗：
	 *     · ★ 回 `K` = "这块好了 ✗ 请发下一块" ✓
	 *     · ★★ 而 PC 收到 `K` 【立刻就发下一块】✗
	 *     · ★★★ 而【我们【擦写时要关中断**✗ 那段时间【收到的字节【全丢】】✓
	 *     → ★★★★ 【所以【必须先写完 ✗ 再回 K】】✓✓（★ 这就是"停等"的另一半 ✓）*/
	Up_SendK(s_seq);
	s_seq++;
}


/*==============================================================================
 *  ★★★★ 处理 `X<crc>` —— 整体校验 + 写第 0 页 + 跳
 *
 *  ★ 顺序【一步都不能换】✗✓：
 *    ① 长度对不对 ✓
 *    ② CRC 对不对 ✓
 *    ③ 写第 0 页（★ 这是"让 App 生效"的唯一动作 ✓）
 *    ④ 回读第 0 页 ✓
 *    ⑤ App 有效性检查 ✓
 *    ⑥ 清升级标志 ✓
 *    ⑦ 回 `PASS` ✗ 等串口发完 ✗ 跳 ✓
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

	/* ★ ① 长度 ✗✓ */
	if (s_total != s_expect)
	{
		Up_SendFail("short");
		Up_Reset();
		return;
	}

	/* ★★ ② 整体 CRC ✗✓ —— 【最后一道闸】✓
	 *   ★ 为什么【每块已经对了还要它**✗：
	 *     · ★ 块号【只防"丢块 / 乱序"】✓
	 *     · ★★ 【块里的【位错误】✗ 只有 CRC 能发现 ✓✓ */
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

	/* ★★★ ③ 写第 0 页 ✗✓
	 *   ★ 为什么【还要再擦一次**✗：
	 *     · ★ 它在 `E` 的时候就擦过了 ✗ 【理论上就是 0xFF】✓
	 *     · ★★ 但【擦一个已经是 0xFF 的页【是幂等的 ✗ 不花钱（40ms）】】✓
	 *     · ★★★ 【万一【中间被什么写脏了 ✗ 这一步【顺手保证干净】】✓✓
	 *       → 【后面【回读比对】会把任何异常兜住 ✓ */
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

	/* ★★★★ ⑤ 现在【第 0 页才真正有效**✗ 用【和 Bootloader 开头同一套判据】再验一次 ✓
	 *   ★ 为什么【不用自己"重算一遍头两个字"✗：
	 *     → 【那会【多一份实现 ✗ 而【两份判据【迟早会不一致】】】✓✓ */
	if (Boot_IsAppValid() == 0u)
	{
		Up_SendFail("valid");
		Up_Reset();
		return;
	}

	/* ★★★ ⑥ 清标志 ✗✓ —— 【到这一步才清是【故意的】✓
	 *   ★ 早清一步 ✗：【中途断电 ✗ 复位后【标志没了 ✗ App 又无效】】
	 *     → 【但【现在"App 无效"本身就是第二条进入条件**✗ 依然安全 ✓✓
	 *       （★ 这就是【Q5 那个"或"条件**的价值 ✓）*/
	BootFlag_ClearUpgrade();

	/* ★★★★★ 2026-10-18 加（★ 复盘 B2 的另一半 ✓）：
	 *   ★ 新固件【写进去 + 校验通过 + 第 0 页有效】⇒ 升级成功 ✓
	 *     ⇒ 【任何还挂着的"回退请求"都该作废】✗
	 *   ★★ 不清会怎样 ✗：
	 *     · 用户之前在 App 里发过 `R` 但没恢复成功 ⇒ `DR3` 残留 ✓
	 *     · 这次升级 `PASS` 了、跳进新固件 ✓
	 *     · ★★★ 可【下次开机】Boot 又看到 `DR3` ⇒ 又去回退
	 *       ⇒ 【刚升好的新固件被换回旧的】✓✓
	 *   ⇒ 升级成功 = 回退请求作废 ✓ 一起清掉 ✓ */
	BootFlag_ClearRollback();

	/* ★ 2026-10-18 加（⑥）：新固件装好并验证通过 ⇒ 【IWDG 回退计数清零】✓
	 *   ★ 语义 ✗：这算是"人重新给了它一次完整机会"✓
	 *   ★★ 不清会怎样 ✗：以后偶发一次狗咬就直接【停升级模式】✗
	 *     而不是先尝试自愈 ✓✓ */
	BootFlag_ClearIwdgRetry();

	Serial_SendString("PASS\r\n");
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) { }

	Serial_SendString("BOOT: upgrade OK -> jumping to App\r\n");
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) { }

	/* ★★★★ ⑦ 跳 ✗（★ 不会返回 ✓）*/
	Boot_JumpToApp();
}


/*==============================================================================
 *  ★★★★ 处理 `C` —— 取消升级
 *
 *  ★★ 为什么【只在 IDLE 允许**✗✓：
 *    · ★ IDLE 时【App 区还是完整的】✗ → 【跳过去是安全的】✓
 *    · ★★ RECV 时【第 0 页【已经被擦了】✗
 *      → 【"取消"也【跳不了】✗ 【只会让用户更困惑】】✓✓
 *============================================================================*/
static void Up_HandleCancel(void)
{
	if (s_state == UP_ST_RECV)
	{
		/* ★ 明说 ✗ 不装死 ✓（★ 而且【绝不能】在这里清标志或跳转 ✓）*/
		Up_SendFail("busy");
		return;
	}

	UP_LOG("BOOT: cancel requested -> checking App...\r\n");

	BootFlag_ClearUpgrade();
	UP_LOG("BOOT: upgrade flag cleared\r\n");

	if (Boot_IsAppValid() != 0u)
	{
		Serial_SendString("BOOT: app valid -> jumping\r\n");
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) { }
		Boot_JumpToApp();                       /* ★ 不会返回 ✓ */
	}

	/* ★★ App 无效 ✗ → 【不能跳**✗ 继续等 ✓
	 *   ★★★ 【这是安全阀 ✗：★ 宁可不响应 ✗ 也不跳到坏固件 ✓✓ */
	Serial_SendString("BOOT: but App is INVALID -> stay in upgrade mode\r\n");
	Serial_SendString("BOOT: (send E<size> to start an upgrade)\r\n");
}


/*==============================================================================
 *  ★★★★ CRC 自检（★ 见 boot_upgrade.h ✓）
 *============================================================================*/
uint8_t BootUpgrade_SelfTest(void)
{
	static const char anchor[] = "123456789";   /* ★ 标准判据串 ✓ */
	uint16_t crc = 0u;
	uint8_t  i;

	for (i = 0u; anchor[i] != '\0'; i++)
	{
		crc = Up_CrcUpdate(crc, (uint8_t)anchor[i]);
	}

	return (crc == 0x2189u) ? 1u : 0u;
}


/*==============================================================================
 *  ★★★★ 主循环（★ 永不返回 ✗ 出口只有"跳 App" ✓）
 *============================================================================*/
void BootUpgrade_Run(void)
{
	uint16_t n;
	uint8_t  r;

	/* ★★★★ 开工前【先验契约**✗✓（★ 理由见 boot_upgrade.h ✓）
	 *   ★★ 【对不上就【绝对不许开始收固件】】✗✓
	 *     → 【否则【收完 112KB 才在最后一步 FAIL ✗ 白折腾】✓✓ */
	if (BootUpgrade_SelfTest() == 0u)
	{
		Serial_SendString("BOOT: CRC SELFTEST FAILED -> refusing to upgrade\r\n");
		for (;;) { BOOT_FEED_IWDG(); }          /* ★ 停住 ✗ 但【继续喂狗】✓ */
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
				/* ★★ 收数据收到一半【卡住**✗ → 【回 IDLE ✗ 让 PC 重来】✓
				 *   ★ 为什么【敢回 IDLE**✗：★ 第 0 页已经擦了 ✗
				 *     【"App 无效"这条【硬条件】保证【跳不出去】】✓✓ */
				Up_SendFail("timeout");
				Up_Reset();
			}
			else
			{
				/* ★★★ 心跳 ✗✓（★ 让人知道"它还在等 ✗ 没死"✓）
				 *   ★★ 只在 IDLE 打 ✗ —— 【传输中打会【干扰 PC 的行解析】】✓✓ */
				Serial_SendString("WAIT\r\n");
			}
			continue;
		}

		if (r == 2u)
		{
			Up_SendFail("long");                /* ★ 这一行超长 ✗ 已丢弃 ✓ */
			Up_Reset();
			continue;
		}

		if (n == 0u) { continue; }              /* ★ 空行 ✗ 忽略 ✓ */

		/* ★★ 调试回显 ✗ 只打【前几个字符】✓
		 *   （★ 打整行【会刷屏 ✗ 而且 D 行本来就有 264 个字符】✓）*/
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
			case 'X': case 'x': Up_HandleEnd(n);    break;   /* ★ 成功时【不返回】✓ */
			case 'C': case 'c': Up_HandleCancel();  break;

			default:
				/* ★ 未知命令 ✗ → 【明说 ✗ 别静默】✓ */
				Up_SendFail("cmd");
				break;
		}
	}
}
