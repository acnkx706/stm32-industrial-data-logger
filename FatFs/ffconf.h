/*---------------------------------------------------------------------------
/  FatFs R0.12c 配置文件 —— 按《28-数据记录仪项目-执行表》T2.2 裁剪表定制
/-----------------------------------------------------------------------------
/  裁剪目标（阶段2 资源预算，执行表第八节）：
/    Flash：FatFs 裁剪后 7~9KB（应用区 48KB 总账）；RAM：FATFS+FIL ≈ 660B
/  裁剪原则：
/    只留"追加写 CSV 日志"这一条数据通路：f_open/f_write/f_lseek/f_sync/
/    f_close/f_read，其余函数全部裁掉。每一项都标了【改了什么/为什么】，
/    与官方 R0.12c 默认值的差异集中在本文件，ff.c/ff.h 一行未改。
/  ★ 完整性校验：_FFCONF 必须等于 ff.h 里的 _FATFS（68300），版本不符
/    会在编译期直接报错，所以 ff.c/ff.h 绝不能悄悄换成 R0.15（执行表 T2.2）
/---------------------------------------------------------------------------*/
#ifndef _FFCONF
#define _FFCONF 68300   /* 版本号：必须与 ff.h 的 _FATFS 一致（R0.12c） */

/*---------------------------------------------------------------------------
/ 功能开关（Function Configurations）
/---------------------------------------------------------------------------*/

/* 【改】1 → 0：读写都开。
 * 记录仪要往卡里写 CSV，只读模式会把 f_write/f_sync 一起裁掉 */
#define _FS_READONLY    0

/* 【改】0 → 2：最小化裁剪。
 * =1 裁掉 f_stat/f_getfree/f_unlink/f_mkdir/f_truncate/f_rename（本项目不用）；
 * =2 再裁掉 f_opendir/f_readdir/f_closedir（不做文件浏览）。
 * ★ f_lseek 在 MINIMIZE=3 才被裁，=2 保留 —— 追加写全靠它（T2.5） */
#define _FS_MINIMIZE    2

/* 【改】2 → 0：裁掉 f_gets/f_putc/f_puts/f_printf 字符串族。
 * 写行格式在 task_sd 里自己拼（还能控制分隔符与缓冲），不依赖 FatFs 格式化 */
#define _USE_STRFUNC    0

/* 0：f_findfirst/f_findnext 本来就没开（官方默认 0，未动） */
#define _USE_FIND       0

/* 【改】1 → 0：裁掉 f_mkfs 格式化 —— 执行表单项最大头，省 4~5KB Flash。
 * 约定：卡一律先在 PC 上格成 FAT32（执行表 4.1 采购要求），板子只写不格 */
#define _USE_MKFS       0

/* 【改】1 → 0：关快速 seek（维护 File 宗 map 表的代码也不进来了）。
 * 我们只有"追加到文件尾"一种 seek 模式，用不着 */
#define _USE_FASTSEEK   0

/* 0：f_expand 预分配簇链，未动（不用） */
#define _USE_EXPAND     0

/* 0：f_chmod/f_utime 属性/时间修改，未动（不用；且时间戳是假的，改了也没意义） */
#define _USE_CHMOD      0

/* 0：卷标函数，未动（不用） */
#define _USE_LABEL      0

/* 0：f_forward 流转发，未动（不用） */
#define _USE_FORWARD    0

/*---------------------------------------------------------------------------
/ 地区与命名空间（Locale and Namespace）
/---------------------------------------------------------------------------*/

/* 【改】850 → 1：ASCII 码页。
 * 文件名只用 "LOG0001.CSV" 这类 8.3 大写形式；LFN=0 时码页只允许 1。
 * CSV 内容是纯 ASCII 数字字段，不涉及中文（中文进不了 8.3 文件名） */
#define _CODE_PAGE      1

/* 【改】1 → 0：关长文件名 —— 省 512B 以上 RAM（执行表 RAM 总账卡得很紧）。
 * 代价：路径必须是 8.3 格式，"LOG%04u.CSV" 恰好合规 */
#define _USE_LFN        0

/* _MAX_LFN 在 LFN=0 时无效果，占位保留官方默认 */
#define _MAX_LFN        255

/* 0：API 用 ANSI/OEM 字符串（char），未动（LFN=0 时也只能是 0） */
#define _LFN_UNICODE    0

/* 3：仅在 _LFN_UNICODE=1 时生效，占位保留官方默认 */
#define _STRF_ENCODE    3

/* 0：关相对路径（f_chdir/f_getcwd 裁掉）。永远操作 "0:" 根目录 */
#define _FS_RPATH       0

/*---------------------------------------------------------------------------
/ 卷与分区（Drive/Volume）
/---------------------------------------------------------------------------*/

/* 1：只有一张 SD 卡（单卷），未动 */
#define _VOLUMES        1

/* 0：盘符用数字 "0:"，不用字符串卷标，未动 */
#define _STR_VOLUME_ID  0

#define _VOLUME_STRS    "RAM","NAND","CF","SD1","SD2","USB1","USB2","USB3"  /* 占位，STR_VOLUME_ID=0 时不用 */

/* 0：单分区模式 —— 卡只有 1 个 FAT32 分区，f_fdisk 也不存在（MKFS=0） */
#define _MULTI_PARTITION 0

/* 512 / 512：固定 512B 扇区。SD 卡 sector 就是 512B，两边一致则
 * disk_ioctl 的 GET_SECTOR_SIZE 都不用实现 */
#define _MIN_SS         512
#define _MAX_SS         512

/* 0：ATA-TRIM，未动（SD over SPI 不支持 TRIM） */
#define _USE_TRIM       0

/* 0：允许用 FSINFO 里的空闲簇计数（只影响 f_getfree，已被裁；占位） */
#define _FS_NOFSINFO    0

/*---------------------------------------------------------------------------
/ 系统配置（System Configurations）
/---------------------------------------------------------------------------*/

/* 【改】0 → 1：精简模式 —— FIL 里去掉 512B 私有扇区缓冲，全卷共用 FATFS 的
 * win[] 一块。FIL 从 ~550B 缩到 ~40B，正是执行表 RAM 总账里
 * "FATFS+FIL(TINY=1) ≈ 660B" 的依据。代价：连续读/写经过公共缓冲多拷一次，
 * 本项目每秒才几 KB 流量，完全无感 */
#define _FS_TINY        1

/* 0：不支持 exFAT ——【必须】（执行表采购要求卡 ≤32GB 格 FAT32）。
 * 注意 R0.12c 里开 exFAT 会破坏 C89 兼容，本工程 AC5/C90 更不能开 */
#define _FS_EXFAT       0

/* 【按执行表】0：启用时间戳函数 —— ff.c 会调用 get_fattime()
 * （实现在 FatFs\diskio_spi.c）。阶段 2 返回固定值 2026-01-01，
 * 阶段 3 接上 RTC 后原地换成真实时间即可，ffconf 不用再动。
 * （若选 _FS_NORTC=1 则不调 get_fattime、用下面三个常量拼时间 —— 两种做法等价，
 *  选 0 是为了阶段 3 只改 diskio_spi.c 一个文件） */
#define _FS_NORTC       0
#define _NORTC_MON      1       /* 以下三项在 _FS_NORTC=0 时不生效，占位对齐 diskio_spi.c 的固定值 */
#define _NORTC_MDAY     1
#define _NORTC_YEAR     2026

/* 0：关文件锁。锁表要额外 RAM；全工程只有 task_sd 一个任务碰 FatFs
 * （执行表 T2.3 铁律），不存在并发打开，锁没有意义 */
#define _FS_LOCK        0

/* 【按执行表】0：关重入（线程安全）。
 * ★ 这是"FatFs 调用只允许出现在 task_sd"这条铁律的配置面前提：
 *   没有互斥保护，谁都不能在别的任务里调 f_xxx。
 * 开 1 需要实现 ff_cre_syncobj 等 4 个 OS 钩子，本项目不需要 */
#define _FS_REENTRANT   0

#define _FS_TIMEOUT     1000    /* REENTRANT=0 时无效果，占位保留 */
#define _SYNC_t         int     /* REENTRANT=0 时无效果，占位保留 */

#endif /* _FFCONF */
