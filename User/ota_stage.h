#ifndef __OTA_STAGE_H
#define __OTA_STAGE_H

/*==============================================================================
 *  升级暂存区操作（W25Q64 尾部 108 KB）
 *
 *  ★ 做什么
 *    在 W25Q64 里维护一块"新固件暂存区"，给 UDS 的 31/34/36/37 四个服务用。
 *    本模块只碰 W25Q64，不认识 UDS，也不碰内部 Flash。
 *
 *  ★ 为什么单独一层
 *    UDS 那层负责"报文怎么来回"，这层负责"字节怎么落到外部 Flash"。
 *    分开之后，写坏 Flash 的可能只在这一层，排查范围小一半。
 *
 *  ★ 影响什么
 *    占 256 字节静态缓冲（一页）。擦除时 CPU 被阻塞（W25 的擦除要等 BUSY），
 *    最长一次擦 4 KB 约 45~400 ms，调用者必须自己安排"先回话再干活"。
 *
 *  ★ 怎么验证
 *    31 01 FF00 擦完之后，用 31 01 0202 查应当回"没有暂存镜像"；
 *    写进一份合法镜像后应当回 CRC 一致。
 *============================================================================*/

#include <stdint.h>

/* ---- 地址与容量：必须和 Boot\boot_config.h 一字不差 --------------------- */

/* ★ 权威布局图在 User\golden.c 的注释里，这里是它的引用
 *   · 0x7E0000 4 KB 暂存头（单独一个扇区，坏了不影响镜像）
 *   · 0x7E1000 108 KB 暂存镜像（27 个 4 KB 扇区） */
#define OTA_STAGE_HDR_ADDR    0x7E0000u
#define OTA_STAGE_IMG_ADDR    0x7E1000u
#define OTA_STAGE_HDR_MAGIC   0x31475453u      /* "STG1" 小端 */
#define OTA_STAGE_IMG_MAX     (108u * 1024u)
#define OTA_STAGE_SECTOR_SIZE 4096u
#define OTA_STAGE_SECTORS     (OTA_STAGE_IMG_MAX / OTA_STAGE_SECTOR_SIZE)  /* 27 */

/* ★ 暂存头状态：只有 READY 才会被 Boot 拿去装 */
#define OTA_STAGE_NONE        0u
#define OTA_STAGE_READY       1u

/* ★ 结构必须和 Boot 的 UpStageHdr_t 完全一致（字段顺序、类型、总长 64 字节）
 *   ⚠️ 两边不一致的后果：Boot 读到 size/crc 全错，会判"镜像无效"然后跳过升级，
 *      而现象只是"刷完还是老版本"，极难查 */
typedef struct
{
	uint32_t magic;
	uint32_t size;
	uint32_t crc;
	uint32_t state;
	uint32_t rsv[12];
} OtaStageHdr_t;

/* ---- 31 服务的例程状态码（回给 Tester 的那个字节） ---------------------- */

#define OTA_STATUS_OK          0u   /* 成功 */
#define OTA_STATUS_NO_IMAGE    1u   /* 没有可用的暂存镜像 */
#define OTA_STATUS_CRC_BAD     2u   /* 镜像 CRC 和头里记的不一致 */
#define OTA_STATUS_NO_W25      3u   /* W25Q64 不在位 */

/* ---- 接口 --------------------------------------------------------------- */

/* ★ 擦掉暂存头那个扇区（让旧镜像立刻失效）
 * · 0 = 成功，非 0 = 失败
 * · 顺序很重要：先让头失效，再擦镜像区。反过来的话，擦到一半掉电，
 *   头还是 READY，Boot 就会去装一个半截镜像 */
uint8_t OtaStage_InvalidateHdr(void);

/* ★ 开始擦镜像区：检查 W25 在位并把剩余扇区数记进内部状态
 * · 0 = 可以开始，非 0 = 失败（W25 不在位） */
uint8_t OtaStage_EraseStart(void);

/* ★ 擦最多 n 个扇区，返回**还剩几个**（0 = 擦完了）
 * · 为什么拆成"分步"而不是一次擦完：
 *   一次擦完最长要 10.8 秒（27 × 400 ms），而 UDS 的 P2* 只有 5 秒。
 *   拆开之后每擦一批就能回一次 0x78，Tester 那边永远不超时。 */
uint8_t OtaStage_EraseMore(uint8_t n);

/* ★ 读暂存头
 * · 返回 1 = magic 对得上，0 = 头无效（没写过 / 被擦过 / 读失败） */
uint8_t OtaStage_ReadHdr(OtaStageHdr_t *h);

/* ★ 校验暂存镜像的完整性
 * · 返回 OTA_STATUS_xxx
 * · 内部按 256 字节一页读，108 KB 约 220 ms，调用者最好先回一句 0x78 */
uint8_t OtaStage_CheckIntegrity(void);

/* ★ 算镜像前 size 字节的 CRC16/KERMIT（和 PC 的 X 命令同一算法） */
uint16_t OtaStage_CrcImage(uint32_t size);

/* ★ 增量 CRC：喂一段数据，返回新的 CRC 值
 * · 下载时每收一块就喂一次，和 PC 那边逐块累加算出来的比
 * · 初值传 0，和 Crc16_Kermit 的初值一致 */
uint16_t OtaStage_CrcUpdate(uint16_t crc, const uint8_t *d, uint32_t n);

/* ★ 往暂存镜像区写一块（目标区域必须已经被 0x31 擦过）
 * · offset 从 0 开始，len 建议 256（一页），返回 0 = 成功 */
uint8_t OtaStage_WriteImage(uint32_t offset, const uint8_t *d, uint32_t len);

/* ★ 写"就绪"暂存头：从这一刻起，Boot 开机就会把这份镜像装进内部 Flash
 * · 顺序：先擦头扇区 → 写整块（magic 填 0xFF）→ 最后单独补 magic
 * · 为什么 magic 最后写：中途断电的话头无效，Boot 当没暂存，
 *   而不是去装一个 size/crc 都还没写完的半截头
 * · 返回 0 = 成功 */
uint8_t OtaStage_WriteReady(uint32_t size, uint16_t crc);

#endif /* __OTA_STAGE_H */
