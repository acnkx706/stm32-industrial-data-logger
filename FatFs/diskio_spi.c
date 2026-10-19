/**
  ******************************************************************************
  * @file    diskio_spi.c
  * @brief   FatFs 底层盘接口 —— 把 diskio 5 函数接到 Hardware\SD_SPI.c 驱动上
  *          （执行表 T2.2）
  *
  *  职责分层（谁干什么，出了错顺着这条链上浮）：
  *    task_sd  ：业务层 —— 连续 3 批写失败 → 置错误标志、卸载、进恢复流程
  *    diskio_spi（本文件）：策略层 —— 不再叠加盲目重试！
  *        驱动内部已带 **2 次/操作**的重试（SD_SPI.c 的 `SD_OP_RETRY = 2`），
  *        这里重试只会把"拔卡"拖成十几秒的连环超时拖死 task_sd。
  *        失败即：置 STA_NOINIT + 返回 RES_ERROR（上浮）。
  *    SD_SPI   ：硬件层 —— 命令/token/busy 全套协议，操作级 3 次重试
  *
  *  ★ STA_NOINIT 的妙用：一旦写入失败就把驱动标成"未初始化"，下次
  *    f_mount(...,1) 强制挂载时 disk_initialize 会走完整 SD_Init（含 CMD0
  *    复位、281kHz 低速阶段）—— 这正是"拔卡插回自动恢复"的机制入口。
  *
  *  ★ 线程边界：本文件无锁（_FS_REENTRANT=0）。只允许 task_sd 经 FatFs
  *    调到这里，其他任务直接调 SD_xxx 也算违规（会撞正在进行的 FatFs 操作）。
  ******************************************************************************
  */
#include "diskio.h"                     /* FatFs 底层接口声明（FatFs\diskio.h） */
#include "SD_SPI.h"                     /* SD 卡驱动（Hardware\SD_SPI.h） */
#include "MyRTC.h"
/*-----------------------------------------------------------------------
/ 私有状态
/  g_disk_stat：盘状态位图（STA_NOINIT / STA_NODISK / STA_PROTECT）
/  ★ volatile：disk_status() 的结果被 FatFs 缓存判断，而本位图会被
/    disk_read/disk_write 的失败路径就地改写 —— 防止编译器把它缓存进寄存器
/-----------------------------------------------------------------------*/
static volatile DSTATUS g_disk_stat = STA_NOINIT;   /* 上电默认"未初始化" */

/*-----------------------------------------------------------------------
/ ⓪ sd_to_dres：SD_Result_t → DRESULT 的【全工程唯一翻译点】（Y3 修复）
/
/  ★ 为什么不做 1:1 数值映射（撞车表见 43 号课 §九）：
/    SD_Result_t（0~9）与 DRESULT（0~4）数值大面积重合、语义互不相干，
/    而 C 的枚举撞车没有任何编译期保护。直接 return (DRESULT)r 的后果：
/      SD_ERR_INIT_TIMEOUT(2)  → 被误读成 RES_WRPRT（写保护！）
/      SD_ERR_UNSUPPORTED(3)   → 被误读成 RES_NOTRDY（"先初始化就行"）
/      SD_ERR_READ_TIMEOUT(5+) → FatFs 字典里查无此值
/    所以这里用 switch 显式映射，把"翻译"收敛到唯一出口——将来谁想
/    "顺手简化"成 return (DRESULT)r，评审对着这一个函数就能拦住。
/
/  ★ 为什么"其余一切 → RES_ERROR"而不是逐条精分：
/    能走到这里的非 SD_OK 返回，全部发生在 SD_ReadBlocks/SD_WriteBlocks
/    内部 3 次重试之后（驱动层已救过一轮）——对 FatFs 而言结论只有一个：
/    "本次传输失败，盘不可信"。RES_ERROR 正是"盘的错"的唯一载体；
/    RES_WRPRT/RES_NOTRDY/RES_PARERR 对应的场景（写保护/未初始化/参数错）
/    都不归读/写调用点管（未初始化/参数错已在下面两道闸门提前拦截）。
/  ★ default 兜底的额外价值：将来 SD_Result_t 新增错误码，这里零改动、
/    自动落进 RES_ERROR——永不静默错映射。
/-----------------------------------------------------------------------*/
static DRESULT sd_to_dres (SD_Result_t r)
{
	switch (r)
	{
	case SD_OK:                             /* 唯一的成功入口。它与 RES_OK 数值
											   相同纯属巧合——靠的是显式 case，
											   不是数值巧合 */
		return RES_OK;
	default:                                /* 其余一切 → RES_ERROR（理由见上） */
		return RES_ERROR;
	}
}

/*-----------------------------------------------------------------------
/ ① disk_initialize：初始化物理盘（f_mount(...,1) 强制挂载 / f_open 前调用）
/  完整的 SD 上电序列（74 时钟/CMD0/CMD8/ACMD41/…/切 9MHz）都在 SD_Init 里，
/  本函数只管"成败映射到 STA_NOINIT 位"。
/-----------------------------------------------------------------------*/
DSTATUS disk_initialize (BYTE pdrv)
{
	if (pdrv)               /* 单卡系统只认 0 号物理盘，其余一律不认 */
	{
		return STA_NOINIT;
	}

	if (SD_Init() == SD_OK)
	{
		g_disk_stat &= (DSTATUS)~STA_NOINIT;    /* 初始化成功：清"未初始化"位 */
	}
	else
	{
		g_disk_stat |= STA_NOINIT;              /* 失败：维持"未初始化"，
												   FatFs 挂载会返回 FR_NOT_READY */
	}
	return g_disk_stat;
}

/*-----------------------------------------------------------------------
/ ② disk_status：返回当前盘状态（FatFs 每次卷访问前会查 STA_NOINIT）
/  没有卡检测脚（CD 引脚），"拔卡"只能靠读/写操作失败来发现 ——
/  发现后由失败路径把 STA_NOINIT 置回来，形成闭环。
/-----------------------------------------------------------------------*/
DSTATUS disk_status (BYTE pdrv)
{
	return pdrv ? STA_NOINIT : g_disk_stat;
}

/*-----------------------------------------------------------------------
/ ③ disk_read：读 1~N 个扇区（每扇区 512B，与 _MIN_SS/_MAX_SS 一致）
/-----------------------------------------------------------------------*/
DRESULT disk_read (BYTE pdrv, BYTE* buff, DWORD sector, UINT count)
{
	SD_Result_t r;

	(void)pdrv;                             /* 单盘，驱动号无意义 */

	if (count == 0)
	{
		return RES_PARERR;
	}
	if (g_disk_stat & STA_NOINIT)           /* 盘没初始化好：直接拒绝 */
	{
		return RES_NOTRDY;
	}

	r = SD_ReadBlocks(buff, (uint32_t)sector, (uint32_t)count);
	if (r != SD_OK)
	{
		/* 读都读不出来 = 卡已不可信（拔卡/坏块大面积扩散）：
		 * 置 NOINIT 强制下次挂载走完整重初始化，错误上浮给 FatFs */
		g_disk_stat |= STA_NOINIT;
	}
	/* ★ Y3 修复：不再手写 return RES_ERROR（散弹式翻译），统一走唯一翻译点
	 *   sd_to_dres() —— "翻译必须经过唯一出口"，杜绝数值撞车静默错映射 */
	return sd_to_dres(r);
}

/*-----------------------------------------------------------------------
/ ④ disk_write：写 1~N 个扇区
/
/  写失败重试与错误上浮策略（执行表 T2.1/T2.6③ 的实现落点）：
/    第 1 层  SD_SPI 内部：整操作 3 次重试 —— 对付瞬时干扰/CRC 偶发错；
/             拔卡时卡不响应，3 次都是毫秒级快速失败，不会拖时间。
/    第 2 层  本函数：【不重试】。失败 → STA_NOINIT + RES_ERROR。
/             理由：能被 3 次重试救回的已经在下层救完了；到这里还失败
/             说明卡已离场/临终，盲目重试只会拖死 task_sd（执行表原话）。
/    第 3 层  task_sd：连续 3 批写失败 → 置 g_sd_error 丢批 → 卸载 →
/             周期性重试挂载（disk_initialize 全序列重跑）→ 插回自动恢复。
/-----------------------------------------------------------------------*/
DRESULT disk_write (BYTE pdrv, const BYTE* buff, DWORD sector, UINT count)
{
	SD_Result_t r;

	(void)pdrv;

	if (count == 0)
	{
		return RES_PARERR;
	}
	if (g_disk_stat & STA_NOINIT)
	{
		return RES_NOTRDY;
	}

	r = SD_WriteBlocks(buff, (uint32_t)sector, (uint32_t)count);
	if (r != SD_OK)
	{
		g_disk_stat |= STA_NOINIT;          /* 卡已不可信：强制下次重新初始化 */
	}
	/* ★ Y3 修复：同 disk_read，翻译统一走 sd_to_dres() 唯一出口
	 *   （上浮：失败时 f_write 会拿到 FR_DISK_ERR） */
	return sd_to_dres(r);
}

/*-----------------------------------------------------------------------
/ ⑤ disk_ioctl：杂项控制。ffconf 裁剪后 FatFs 只会调 CTRL_SYNC：
/     - CTRL_SYNC       ：f_sync/f_close 时要求底层把缓存冲掉 —— 我们的写是
/                         同步完成的（SD_SPI 返回时数据已在卡里烧好），直接 OK
/     - GET_SECTOR_COUNT：_USE_MKFS=0 才需要 → 已裁，不实现
/     - GET_SECTOR_SIZE ：_MAX_SS==_MIN_SS 才需要 → 相等，不实现
/     - GET_BLOCK_SIZE  ：_USE_MKFS=0 才需要 → 已裁，不实现
/     - CTRL_TRIM       ：_USE_TRIM=0 → 不实现
/  调试用的 MMC_GET_TYPE 顺手实现（返回 SD_CT_xxx，验收时串口报卡型方便）
/-----------------------------------------------------------------------*/
DRESULT disk_ioctl (BYTE pdrv, BYTE cmd, void* buff)
{
	(void)pdrv;

	switch (cmd)
	{
	case CTRL_SYNC:                         /* 无写缓存需要冲刷，恒就绪 */
		return RES_OK;

	case MMC_GET_TYPE:                      /* 非 essentials：调试用 */
		*(BYTE*)buff = SD_GetCardType();
		return RES_OK;

	case GET_SECTOR_COUNT:                  /* 以下均为已裁功能，明确拒绝 */
	case GET_SECTOR_SIZE:
	case GET_BLOCK_SIZE:
	case CTRL_TRIM:
	default:
		return RES_PARERR;
	}
}

/*-----------------------------------------------------------------------
/ get_fattime：FatFs 写目录项/文件时间戳时调用（_FS_NORTC=0 启用）
/
/  ★ 阶段 2：没有 RTC，返回固定值 2026-01-01 00:00:00 ——
/    文件时间不真实但合法（年份 ≥1980 才被 FAT 接受，给了 1970/0 会拒写）。
/  ★ 阶段 3（执行表 T3.1）：接上 RTC 后把返回值换成真实时间即可，
/    本函数是唯一落点，ffconf.h / ff.c 都不用再动。
/
/  FAT 时间位布局：bit31:25=年-1980  bit24:21=月  bit20:16=日  bit15:11=时
/                  bit10:5 =分      bit4:0 =秒/2
/-----------------------------------------------------------------------*/
DWORD get_fattime (void)
{
RTC_DateTime_t dt;

	MyRTC_GetDateTime(&dt);
	if (dt.year < 1980u) { return 0u; }              /* ★ FAT 起点是 1980 ✓ */

	return ((DWORD)(dt.year - 1980u) << 25)
	     | ((DWORD)dt.month  << 21)
	     | ((DWORD)dt.day    << 16)
	     | ((DWORD)dt.hour   << 11)
	     | ((DWORD)dt.minute << 5)
	     | ((DWORD)(dt.second / 2u));
}
