#ifndef __SD_SPI_H
#define __SD_SPI_H

#include "stm32f10x.h"

/*==============================================================================
 * SD 卡 SPI 模式驱动（硬件 SPI1）—— 对应执行表 T2.1
 *   引脚（执行表第二节引脚总表）：
 *     PA4 = CS   （软件控制推挽，空闲拉高）
 *     PA5 = SCK  （SPI1，复用推挽）
 *     PA6 = MISO （上拉输入！卡识别期 DO 高阻，必须靠内部上拉读出确定的 0xFF）
 *     PA7 = MOSI （SPI1，复用推挽）
 *   速率：上电 281kHz（72M/256，卡上电只保证 ≤400kHz）→ 初始化成功切 9MHz（72M/8）
 *   依赖：system\DWT_Delay.c（busy 等待/节拍全部走 DWT，绝不碰 SysTick 版 Delay）
 *============================================================================*/

/* ---- 卡类型（CMD58 读 OCR 的 CCS 位判定，disk_ioctl(MMC_GET_TYPE) 会用到）---- */
#define SD_CT_NONE      0x00    /* 未初始化/无卡 */
#define SD_CT_V1        0x01    /* SD v1.x（字节寻址，≤2GB） */
#define SD_CT_V2        0x02    /* SD v2.x 标准容量（字节寻址） */
#define SD_CT_SDHC      0x04    /* SDHC/SDXC（块寻址，命令参数直接给块号） */

/* ---- 错误码 ----
 * ★ 设计原则：错误一路上浮 —— 驱动层 3 次重试救不回来 → diskio 层置 STA_NOINIT
 *   返回 RES_ERROR → task_sd 判定连续 3 批失败 → 置 g_sd_error 进"拔卡恢复"流程 */
typedef enum
{
	SD_OK = 0,              /* 成功 */
	SD_ERR_NO_CARD,         /* CMD0 无响应：没插卡/接触不良/CS·SCK·MOSI 接线错 */
	SD_ERR_INIT_TIMEOUT,    /* ACMD41 初始化超时(>1s)：卡坏/供电不足/买成 64G+ exFAT 卡 */
	SD_ERR_UNSUPPORTED,     /* 卡不合作（MMC/非标卡，本驱动不支持 CMD1） */
	SD_ERR_CMD_FAILED,      /* 命令 R1 带错误位（非法命令/地址错等） */
	SD_ERR_READ_TIMEOUT,    /* 读数据：等起始 token 超时（卡内部 NAC 最长 100ms 已计余量） */
	SD_ERR_READ_CRC,        /* 读数据：CRC 校验错（本驱动关 CRC 后不会出现，留作扩展） */
	SD_ERR_WRITE_RESPONSE,  /* 写数据：卡拒收（数据响应 token 不是"已接受"） */
	SD_ERR_WRITE_TIMEOUT,   /* 写数据：卡 busy 超时（>500ms，规范 NAC 250ms 已留一倍余量） */
	SD_ERR_PARAM            /* 参数非法（块数 0、缓冲未对齐等） */
} SD_Result_t;

/* ---- 对外接口 ---- */
SD_Result_t SD_Init(void);                                                  /* 上电初始化（可反复调，用于拔卡重插恢复） */
SD_Result_t SD_ReadBlocks (uint8_t       *buf, uint32_t sector, uint32_t count);  /* 读 1~N 块（每块 512B） */
SD_Result_t SD_WriteBlocks(const uint8_t *buf, uint32_t sector, uint32_t count);  /* 写 1~N 块（每块 512B） */
uint8_t     SD_GetCardType(void);                                           /* 返回 SD_CT_xxx，调试/ioctl 用 */
const char *SD_ErrorStr(SD_Result_t err);                                   /* 错误码 → 可读字符串（串口报错用） */
/* ---- ★ 硬件排查探针（开关在 User\ProbeConfig.h ✓ 关掉时不编译、也不声明 ✓）----
 * ★ 声明也必须跟着开关走 ✗ —— 否则关了之后头文件还声明着，
 *   调用方若不慎引用就会链接报未定义 ✓（虽然只是声明不占 Flash，但语义上要一致 ✓）*/
#include "ProbeConfig.h"

#if PROBE_SD_LOOPBACK
void SD_LoopbackTest(void);
#endif
#if PROBE_SD_PINFINDER
void SD_PinFinder(void);
#endif
#if PROBE_SD_MISO
void SD_MisoProbe(void);
#endif
#if PROBE_SD_BITBANG
void SD_BitBangProbe(void);
#endif
#if PROBE_SD_VERBOSE
void SD_VerboseInit(void);
#endif
#if PROBE_SD_FAKECARD
void SD_FakeCardProbe(void);
#endif
#endif
