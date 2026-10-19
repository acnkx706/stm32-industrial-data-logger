#include "stm32f10x.h"
#include "boot_config.h"
#include "boot_w25.h"
#include "boot_flash.h"
#include "boot_jump.h"
#include "boot_upgrade.h"       /* ★★ 【借它的 CRC】✗ 见下面说明 ✓ */
#include "boot_golden.h"
#include "Serial.h"

/*==============================================================================
 *  ★★★★★ Bootloader 侧金副本恢复（★ 2026-10-14 建 ✗ G5 ✓）
 *    触发判据与顺序（★ 为什么这么排 ✓）见 boot_golden.h
 *
 *  ★★★★ 【CRC 只有一份**✗✓ —— ★ 这条是【今天踩出来的教训】✓
 *    · ★ Boot 里【本来就有】CRC-16/KERMIT ✗ 在 `boot_upgrade.c` 里 ✓
 *      （★ `Up_CrcUpdate` ✗ 注释写着"和 App 的 `Crc16.c` 一字不差的契约" ✓）
 *    · ★★ 【所以【绝不能再写第二份】**✗✓
 *      → 【两份一旦漂移 ✗ 现象是【"备份永远校验不过"✗ 查一天】】✓✓
 *      （★ 今天 SD 那个 CRC 变体的混乱 ✗ 就是活教材 ✓）
 *    → ★★★ 【现在把它【导出共用】✗ 本文件直接调 ✓✓
 *============================================================================*/

typedef struct {
	uint32_t magic;             /* ★ `"GOLD"` ✗ 最后写的那 4 个字节 ✓ */
	uint32_t size;              /* ★ 必须是 `APP_SIZE` ✓ */
	uint16_t crc16;             /* ★ 整段镜像的 CRC-16/KERMIT ✓ */
	uint16_t fw_tag;
	uint32_t uptime_s;          /* ★ 写备份时 App 已跑的秒数（★ 诊断 ✓）*/
	uint8_t  reserved[48];
} GoldenHdr_t;

/* ★ 1KB 页缓冲 ✗ 【只要一个】✓
 *   ★ 为什么【一个就够**✗：
 *     · ★ 校验备份时 ✗ 一页一页读进来算 CRC ✓
 *     · ★★ 搬回时 ✗ 【同一块缓冲】接着用 ✓
 *     · ★★★ 第 0 页【最后才从 W25 重新读】✗ 【不需要一直留着】✓✓ */
static uint8_t s_page[BOOT_FLASH_PAGE_SIZE];


/* ---- 复位原因（★ 读一次 ✗ 顺手清掉 ✓）------------------------------------ */
uint8_t BootGolden_ResetWasIwdg(void)
{
	uint8_t was = (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET) ? 1u : 0u;

	/* ★★★★ 【必须清】**✗✓ —— ★ 不清的话【它会一直粘着】✓
	 *   → 【下次开机又以为"刚被咬过"✗ 【于是每次都去恢复 ✗ 变成死循环】】✓✓
	 *   ★★ `RCC_ClearFlag()` 写的是 `RCC_CSR` 的 **RMVF** 位 ✓
	 *     → 【一次清掉全部复位标志 ✗ 正是我们要的】✓ */
	RCC_ClearFlag();

	return was;
}


/* ---- 备份头（★ 只读 ✗ 不动任何东西 ✓）------------------------------------ */
uint8_t BootGolden_BackupInfo(uint16_t *crc_out, uint32_t *uptime_out)
{
	GoldenHdr_t h;

	if (BootW25_Read(GOLDEN_ADDR_HDR, (uint8_t *)&h, (uint32_t)sizeof(h)) == 0u)
	{
		return 0u;
	}
	if (h.magic != GOLDEN_HDR_MAGIC)
	{
		return 0u;                                  /* ★ 头没写完 / 没写过 ✓ */
	}
	if (h.size != (uint32_t)APP_SIZE)
	{
		/* ★★ 【尺寸对不上 ✗ 说明【这份备份是【另一个分区方案】下写的】】✓
		 *   → 【宁可不恢复 ✗ 也【绝不能拿它去覆盖当前 App 区】】✓✓ */
		return 0u;
	}

	if (crc_out    != 0) { *crc_out    = h.crc16; }
	if (uptime_out != 0) { *uptime_out = h.uptime_s; }
	return 1u;
}


/* ⚠️ ★★★★★ 【2026-10-14 删掉了【App 区 CRC 比对】整段】✗✓
 *
 *  ★ 删掉的是 ✗ `BootGolden_FlashCrc()` + `BootGolden_AppCrcMatches()` ✓
 *  ★★ 【为什么删**✗✓（★ 详细理由写在 `main.c` 那边的第 ④ 条注释里 ✓）✗：
 *    · ★ 判据【"App 区 CRC vs 备份头 CRC"】和
 *      【"备份 = 上一版"】这个设计【**互相矛盾**】✓✓
 *    · ★★ 升级之后【两者本来就该不同】✗
 *      → 【留着它 = 【每次升级成功 ✗ 下次开机都被换回旧版】】✓✓
 *    · ★★★ 【而且它和判据③（IWDG）功能重叠 ✗ 删了【不损失安全性】✓】
 *  ★★★ 顺带 ✗ 省下的约 200 字节【正好填上那个薄薄的余量】✓✓ */

static uint8_t BootGolden_BackupCrc(uint16_t *out)
{
	uint16_t crc = 0u;
	uint32_t off;
	uint16_t i;

	for (off = 0u; off < (uint32_t)APP_SIZE; off += BOOT_FLASH_PAGE_SIZE)
	{
		if (BootW25_Read(GOLDEN_ADDR_IMG + off, s_page, BOOT_FLASH_PAGE_SIZE) == 0u)
		{
			return 0u;
		}
		for (i = 0u; i < (uint16_t)BOOT_FLASH_PAGE_SIZE; i++)
		{
			crc = Up_CrcUpdate(crc, s_page[i]);
		}
		BOOT_FEED_IWDG();                           /* ★ 保险 ✗ 见 boot_config.h ✓ */
	}

	*out = crc;
	return 1u;
}


/*==============================================================================
 *  ★★★★★ 真正的恢复
 *
 *  ⚠️ ★★★★ 【字符串要【省着用】**✗✓（★ 2026-10-14 实测教训 ✓）✗：
 *    · ★ 第一版【每步都写一条完整句子】✗（★ "read FAIL"/"erase FAIL"/… ✓）
 *    · ★★ 手动链一次 ✗ 结果【超出当时的 8KB 上限 **176 字节**】✗✓
 *      （★ 需要 8,368 ✗ 当时上限 8,192 ✓
 *        ★★ 后来 Boot 已扩到 **16KB** ⇒ 【这个约束已宽松】✓
 *        但"省着用"的纪律【仍然照旧】✓）
 *    · ★★★ 【而 Boot 里【没有 printf ✗ 每条字符串【都是实打实的 Flash】】✓
 *      → 【所以【合并成两条】✗ 【够定位就行 ✗ 不必成句】】✓✓
 *    · ★★★★ 【教训 ✗ 在只有 8/16KB 的地方 ✗ 日志也要【算字节】】✓✓
 *============================================================================*/

/* ★ 两个失败出口 ✗ 【就两条字符串】✓ */
static void G_FailBackup(void) { Serial_SendString("G: backup bad\r\n"); }
static void G_FailWrite(void)  { Serial_SendString("G: write FAILED\r\n"); }

uint8_t BootGolden_Restore(void)
{
	uint16_t want = 0u;
	uint16_t got  = 0u;
	uint32_t page;
	uint32_t n_pages = (uint32_t)APP_SIZE / BOOT_FLASH_PAGE_SIZE;

	/* ---- ① 备份有效吗（★ 只看头 ✗ 最便宜的一步 ✓）---- */
	if (BootGolden_BackupInfo(&want, 0) == 0u)
	{
		G_FailBackup();
		return 0u;
	}

	Serial_SendString("G: crc=");
	Serial_SendHex((uint32_t)want, 4u);
	Serial_SendString("\r\n");

	/* ---- ② 【第一遍：只读 + 算 CRC】✗ 一行都不写内部 Flash ✓ ----
	 *   ★★★ 为什么【非要有这一遍**✗✓：
	 *     · ★ 万一【W25Q64 里那份备份【本身就是坏的】】✗
	 *       （★ 掉电写坏 / 芯片老化 ✓）
	 *     · ★★ 而【我们【第一件事就是擦掉 App 第 0 页】】✗
	 *       → 【那就是【"为了救它 ✗ 先把唯一能跑的东西也毁了"】】✓✓
	 *     → ★★★ 【所以【先证明备份是完整的 ✗ 再动内部 Flash】】✓✓ */
	if (BootGolden_BackupCrc(&got) == 0u)
	{
		G_FailBackup();
		return 0u;
	}
	if (got != want)
	{
		Serial_SendString("G: crc bad got=");
		Serial_SendHex((uint32_t)got, 4u);
		Serial_SendString("\r\n");
		return 0u;
	}

	/* ---- ③ 【先毁掉第 0 页】✗ 让"半成品"在硬件上立刻无效 ✓ ---- */
	if (BootFlash_ErasePage(APP_BASE) != BOOTFLASH_OK)
	{
		G_FailWrite();
		return 0u;
	}

	Serial_SendString("G: restoring\r\n");

	/* ---- ④ 第 1 ~ 最后一页 ✗ 逐页搬 ✓ ---- */
	for (page = 1u; page < n_pages; page++)
	{
		uint32_t dst = APP_BASE + (page * BOOT_FLASH_PAGE_SIZE);

		if (BootW25_Read(GOLDEN_ADDR_IMG + (page * BOOT_FLASH_PAGE_SIZE),
		                 s_page, BOOT_FLASH_PAGE_SIZE) == 0u)
		{
			G_FailWrite();
			return 0u;
		}
		if (BootFlash_ErasePage(dst) != BOOTFLASH_OK)
		{
			G_FailWrite();
			return 0u;
		}
		if (BootFlash_WritePage(dst, s_page) != BOOTFLASH_OK)
		{
			G_FailWrite();
			return 0u;
		}
		/* ★★★ 【写完立刻回读比对】**✗ 这是唯一能证明"真的写进去了"的办法 ✓
		 *   （★ Flash 编程失败【不报错 ✗ 只是数据没进去】✓）*/
		if (BootFlash_VerifyPage(dst, s_page) != BOOTFLASH_OK)
		{
			G_FailWrite();
			return 0u;
		}

		BOOT_FEED_IWDG();
	}

	/* ---- ⑤ 【最后】写回第 0 页 ✓ ----
	 *   ★★★★ 到【这一刻】✗ App 区才第一次【完整】✓
	 *     → 【在此之前【任何时刻断电 ✗ 第 0 页都是 0xFF ✗ 检查必失败】】✓✓ */
	if (BootW25_Read(GOLDEN_ADDR_IMG, s_page, BOOT_FLASH_PAGE_SIZE) == 0u)
	{
		G_FailWrite();
		return 0u;
	}
	if (BootFlash_WritePage(APP_BASE, s_page) != BOOTFLASH_OK)
	{
		G_FailWrite();
		return 0u;
	}
	if (BootFlash_VerifyPage(APP_BASE, s_page) != BOOTFLASH_OK)
	{
		G_FailWrite();
		return 0u;
	}

	/* ---- ⑥ 复检 ✗ 用【和跳转同一个判据】✓ ---- */
	if (Boot_IsAppValid() == 0u)
	{
		Serial_SendString("G: still invalid\r\n");
		return 0u;
	}

	Serial_SendString("G: ok\r\n");
	return 1u;
}
