/**
  ******************************************************************************
  * @file    ota_stage.c
  * @brief   升级暂存区操作（W25Q64 尾部 108 KB）
  ******************************************************************************
  * ★ 做什么
  *   擦除 / 读头 / 算 CRC。不认识 UDS，也不碰内部 Flash。
  *
  * ★ 为什么擦除要拆成分步
  *   27 个 4 KB 扇区一次擦完，最坏 10.8 秒（每个 400 ms）。而 UDS 里
  *   Tester 在收到 0x78 之后只等 P2* = 5 秒。不拆开就必然超时。
  *
  * ★ W25Q64 驱动的返回值约定（看错一次，白烧一轮固件）
  *   · ReadData / WriteData / EraseSector / EraseBlock64K / WaitReady
  *     都是 **1 = 成功，0 = 失败**
  *   · 但 W25Q64_SelfTest 是反的：0 = 全过，非 0 = 第几步坏的
  *   · 本文件只用前者，所以每一处判断都写成"== 0 才算失败"
  *
  * ★ 影响什么
  *   占 256 字节静态缓冲；擦除期间 CPU 空转等 W25 的 BUSY 位。
  *
  * ★ 怎么验证
  *   31 01 FF00 之后 31 01 0202 应回"没有暂存镜像"（状态码 1）。
  ******************************************************************************
  */

#include "ota_stage.h"

#include "W25Q64.h"
#include "Crc16.h"

/* ★ 一页 256 字节：和 W25 的页大小一致，读的时候不会跨页 */
static uint8_t s_page[W25_PAGE_SIZE];

/* ★ 擦除进度：还剩几个扇区没擦 */
static uint8_t s_erase_left;

/*==============================================================================
 *  ★ 让暂存头失效
 *
 *  ★ 为什么这是擦除的第一步：
 *    擦镜像区要好几秒，中途掉电的话镜像只写了一半。如果头还写着 READY，
 *    Boot 开机就会拿这个半截镜像去装 —— 装完 App 是坏的，只能接电脑救。
 *    先把头那个扇区擦掉（magic 变 0xFF），Boot 就认不出 READY，跳过。
 *============================================================================*/
uint8_t OtaStage_InvalidateHdr(void)
{
	if (W25Q64_IsPresent() == 0u)
	{
		return 1u;
	}

	if (W25Q64_EraseSector(OTA_STAGE_HDR_ADDR) == 0u)
	{
		return 2u;
	}

	return 0u;
}

/*==============================================================================
 *  ★ 开始擦镜像区
 *============================================================================*/
uint8_t OtaStage_EraseStart(void)
{
	if (W25Q64_IsPresent() == 0u)
	{
		return 1u;
	}

	s_erase_left = (uint8_t)OTA_STAGE_SECTORS;

	return 0u;
}

/*==============================================================================
 *  ★ 擦最多 n 个扇区，返回还剩几个
 *
 *  ★ 为什么按扇区循环而不是调 64 KB 块擦：
 *    块擦一次要 150~2000 ms，粒度太粗，中间没法回 0x78。
 *    扇区擦 45~400 ms，粒度刚好够插一次"我还在忙"。
 *============================================================================*/
uint8_t OtaStage_EraseMore(uint8_t n)
{
	uint32_t addr;

	while ((n > 0u) && (s_erase_left > 0u))
	{
		/* 从高地址往低地址擦？不，从低到高，和镜像写入方向一致，
		 * 这样"已擦到哪儿"可以直接用剩余数反推，不用另存偏移 */
		addr = OTA_STAGE_IMG_ADDR +
		       ((uint32_t)(OTA_STAGE_SECTORS - s_erase_left) * OTA_STAGE_SECTOR_SIZE);

		if (W25Q64_EraseSector(addr) == 0u)
		{
			/* 擦失败就把剩余数清零，让上层立刻结束并报错 */
			s_erase_left = 0u;
			return 0u;
		}

		s_erase_left--;
		n--;
	}

	return s_erase_left;
}

/*==============================================================================
 *  ★ 读暂存头
 *============================================================================*/
uint8_t OtaStage_ReadHdr(OtaStageHdr_t *h)
{
	if (h == 0)
	{
		return 0u;
	}

	if (W25Q64_IsPresent() == 0u)
	{
		return 0u;
	}

	if (W25Q64_ReadData(OTA_STAGE_HDR_ADDR, (uint8_t *)h, (uint32_t)sizeof(*h)) == 0u)
	{
		return 0u;
	}

	return (h->magic == OTA_STAGE_HDR_MAGIC) ? 1u : 0u;
}

/*==============================================================================
 *  ★ 增量 CRC：喂一段数据，返回新的值
 *
 *  ★ 为什么要增量版：
 *    下载时数据是一块一块到的，每块 256 字节，收完就没法再回看。
 *    只能在收到的那一刻把 CRC 喂进去。整段一次算的版本只能用于"事后校验"。
 *
 *  ★ 和 Crc16_Kermit 完全等价：同样的初值 0、同样的 0x8408 反射多项式、
 *    同样的无输出异或。区别只是"分几次喂"。
 *============================================================================*/
uint16_t OtaStage_CrcUpdate(uint16_t crc, const uint8_t *d, uint32_t n)
{
	uint32_t i;
	uint8_t  b;

	for (i = 0u; i < n; i++)
	{
		crc ^= (uint16_t)d[i];
		for (b = 0u; b < 8u; b++)
		{
			crc = (crc & 0x0001u) ? (uint16_t)((crc >> 1) ^ 0x8408u)
			                      : (uint16_t)(crc >> 1);
		}
	}

	return crc;
}

/*==============================================================================
 *  ★ 算镜像前 size 字节的 CRC16/KERMIT
 *
 *  ★ 为什么要按页读：
 *    W25Q64_ReadData 一次读 108 KB 会把调用者阻塞得更久，而且中间没法
 *    插入别的事。按 256 字节一页读，节奏和写入一致，也方便以后加进度。
 *============================================================================*/
uint16_t OtaStage_CrcImage(uint32_t size)
{
	uint16_t crc = 0u;
	uint32_t off = 0u;
	uint32_t n;

	while (off < size)
	{
		n = size - off;
		if (n > (uint32_t)W25_PAGE_SIZE)
		{
			n = (uint32_t)W25_PAGE_SIZE;
		}

		if (W25Q64_ReadData(OTA_STAGE_IMG_ADDR + off, s_page, n) == 0u)
		{
			/* 读失败返回一个和任何合法 CRC 都不同的值：调用者一定判不符 */
			return 0xFFFFu;
		}

		/* 逐页累加：把上一页的结果当这一页的初值，等价于整段一次算 */
		crc = OtaStage_CrcUpdate(crc, s_page, n);

		off += n;
	}

	return crc;
}

/*==============================================================================
 *  ★ 校验暂存镜像
 *
 *  ★ 判据顺序（先便宜后贵）：
 *    ① W25 在不在位  ② 头有没有效  ③ 状态是不是 READY  ④ 才算 CRC
 *    前三步都是几十个字节的操作，最后一步要读 108 KB。
 *    顺序反过来会让"没有镜像"这种常见情况也白读 108 KB。
 *============================================================================*/
uint8_t OtaStage_CheckIntegrity(void)
{
	OtaStageHdr_t h;
	uint16_t crc;

	if (W25Q64_IsPresent() == 0u)
	{
		return OTA_STATUS_NO_W25;
	}

	if (OtaStage_ReadHdr(&h) == 0u)
	{
		return OTA_STATUS_NO_IMAGE;
	}

	if (h.state != OTA_STAGE_READY)
	{
		return OTA_STATUS_NO_IMAGE;
	}

	if ((h.size == 0u) || (h.size > (uint32_t)OTA_STAGE_IMG_MAX))
	{
		return OTA_STATUS_NO_IMAGE;
	}

	crc = OtaStage_CrcImage(h.size);

	if (crc != (uint16_t)h.crc)
	{
		return OTA_STATUS_CRC_BAD;
	}

	return OTA_STATUS_OK;
}
/*==============================================================================
 *  ★ 往暂存镜像区写一块
 *
 *  ★ 为什么这里不自己按页拆：
 *    W25Q64_WriteData 内部已经会按 256 字节页边界拆开，重复实现一遍只会多一个出错点。
 *  ★ 但目标区域必须已经擦过 —— 没擦就写等于做"旧值 AND 新值"，静默错。
 *    所以 0x36 必须排在 0x31 擦除之后，这是流程约束，不是本函数能检查的。
 *============================================================================*/
uint8_t OtaStage_WriteImage(uint32_t offset, const uint8_t *d, uint32_t len)
{
	if ((offset + len) > (uint32_t)OTA_STAGE_IMG_MAX)
	{
		return 1u;
	}

	if (W25Q64_WriteData(OTA_STAGE_IMG_ADDR + offset, d, len) == 0u)
	{
		return 2u;
	}

	return 0u;
}

/*==============================================================================
 *  ★ 写"就绪"暂存头
 *
 *  ★ 顺序（和 Boot 的 Up_StageHdrWrite 同一套思路）：
 *    ① 擦头扇区
 *    ② 写整块头，但 magic 先填 0xFFFFFFFF（等于"还没生效"）
 *    ③ 最后单独补 4 字节 magic —— 写完这一下，Boot 才认得出有就绪镜像
 *
 *  ★ 为什么把 magic 单独拿出来写：
 *    头有 64 字节，写完要几毫秒。中途断电的话，size/crc 可能只写进去一半。
 *    如果 magic 在第一步就写进去，Boot 会拿一个"magic 对、size 和 crc 都错"的头
 *    去装镜像，装完 App 是坏的。magic 最后写就没有这个窗口。
 *============================================================================*/
uint8_t OtaStage_WriteReady(uint32_t size, uint16_t crc)
{
	OtaStageHdr_t h;
	uint32_t      magic;
	uint8_t       i;

	if (W25Q64_IsPresent() == 0u)
	{
		return 1u;
	}

	if (W25Q64_EraseSector(OTA_STAGE_HDR_ADDR) == 0u)
	{
		return 2u;
	}

	h.magic = 0xFFFFFFFFu;            /* ★ 先占位，最后再补 */
	h.size  = size;
	h.crc   = (uint32_t)crc;
	h.state = OTA_STAGE_READY;

	for (i = 0u; i < 12u; i++)
	{
		h.rsv[i] = 0u;
	}

	if (W25Q64_WriteData(OTA_STAGE_HDR_ADDR, (const uint8_t *)&h, (uint32_t)sizeof(h)) == 0u)
	{
		return 3u;
	}

	magic = OTA_STAGE_HDR_MAGIC;

	if (W25Q64_WriteData(OTA_STAGE_HDR_ADDR, (const uint8_t *)&magic, 4u) == 0u)
	{
		return 4u;
	}

	return 0u;
}
