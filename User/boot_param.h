#ifndef __BOOT_PARAM_H
#define __BOOT_PARAM_H

#include <stdint.h>

/*==============================================================================
 *  ★ 4KB 参数区（U1/U2，2026-11-07）
 *
 *  ★ 为什么单独一层：
 *    · 升级请求/回退请求/boot_count/image_ok 需要掉电一致；
 *    · BKP 寄存器会随电池丢失，且不能双份+CRC；
 *    · 内部 flash 末尾 4KB 做 A/B 双槽，valid 最后写。
 *
 *  ★ 地址（Boot 与 App 必须一字不差）：
 *    0x0801F000  A 槽（1KB）
 *    0x0801F400  B 槽（1KB）
 *    0x0801F800  预留
 *    0x0801FC00  预留
 *
 *  ★ 记录 52 字节：
 *    magic/seq/flags/boot_count/image_ok/iwdg_retry/version/rsv[5]/crc/valid
 *
 *  ★ 写序：擦非活动槽 → 写 1KB（valid=0xFFFF）→ 回读
 *          → 最后写半字 0xFF01 → 回读
 *  ★ 读序：两槽都验 valid==0xFF01 + CRC，取 seq 大的合法槽
 *  ★ 正常启动 image_ok=1 时只读不写，避免 10k 擦写寿命被 boot_count 磨掉。
 *============================================================================*/

#define PARAM_BASE       0x0801F000u
#define PARAM_SIZE       (4u * 1024u)
#define PARAM_SLOT_SIZE  1024u
#define PARAM_SLOT_A     PARAM_BASE
#define PARAM_SLOT_B     (PARAM_BASE + PARAM_SLOT_SIZE)

#define PARAM_MAGIC      0x50415241u   /* ★ "PARA" */
#define PARAM_VERSION    1u

/* ★ flags 位 */
#define PARAM_FLAG_UPGRADE    0x00000001u
#define PARAM_FLAG_ROLLBACK   0x00000002u

/* ★ valid 半字：高字节保持 0xFF，低字节 0x01
 *   · 擦除态 0xFFFF
 *   · 写 0xFF01 ⇒ 只把低字节 0xFF 写成 0x01 */
#define PARAM_ERASED     0xFFFFu
#define PARAM_VALID      0xFF01u

/* ★ 试用次数上限：image_ok=0 时连续启动超过这个数 ⇒ 回退 */
#define PARAM_BOOT_TRIAL_MAX  3u

typedef struct
{
	uint32_t magic;
	uint32_t seq;
	uint32_t flags;
	uint32_t boot_count;
	uint32_t image_ok;
	uint32_t iwdg_retry;
	uint32_t version;
	uint32_t img_min_ver;   /* ★ 2026-11-07（U5）：允许安装的最低镜像版本，防回滚 */
	uint32_t rsv[4];
	uint16_t crc;
	uint16_t valid;
} ParamRec_t;

/* ★ 2026-11-07（U5）：防回滚计数器
 *   · 升级前读它：新镜像 version 比它小 ⇒ 拒绝
 *   · 升级成功后取 max(旧值, 新版本)
 *   ⚠️ 金副本恢复不走这个判断 —— 那是"救回来"，不是"安装更新"，
 *      拦住它只会把板子锁死在升级模式。 */
uint32_t Param_GetMinVer(void);
uint8_t  Param_SetMinVer(uint32_t ver);

/* ★ 填默认值：magic/version，image_ok=1（迁移时把当前 App 当好的） */
void    Param_Default(ParamRec_t *r);

/* ★ 读：1 = 找到合法槽；0 = 两槽都无效（out 被填默认值） */
uint8_t Param_Load(ParamRec_t *out);

/* ★ 写：1 = 成功；0 = 失败。内部 seq+1，写非活动槽，valid 最后写。 */
uint8_t Param_Save(ParamRec_t *in);

/* ★ 读或默认：1 = 读到合法记录；0 = 用了默认值 */
uint8_t Param_Get(ParamRec_t *out);

/* ★ 置/清一个 flag 位，返回 1 = 成功 */
uint8_t Param_SetFlag(uint32_t flag, uint8_t on);
uint8_t Param_ClearFlags(void);

/* ★ Boot 用：新镜像已提交（升级），等 App 确认
 *   image_ok=0、boot_count=0、flags=0、iwdg_retry=0 */
uint8_t Param_OnUpgrade(void);

/* ★ Boot 用：金副本恢复完成，等 App 确认
 *   image_ok=0、boot_count=0、flags=0；保留 iwdg_retry（恢复次数上限） */
uint8_t Param_OnNewImage(void);

/* ★ App 用：POST 通过，确认本次镜像
 *   image_ok=1、boot_count=0、清 flags */
uint8_t Param_ConfirmImage(void);

#endif
