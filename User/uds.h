#ifndef __UDS_H
#define __UDS_H

/*==============================================================================
 *  UDS 诊断服务层（ISO 14229 子集）—— CAN-OTA 的业务层
 *
 *  ★ 做什么
 *    在 ISO-TP 之上实现刷写需要的 10 个服务。收到一条完整请求报文就分派给
 *    对应服务，回一条肯定响应或否定响应。
 *
 *  ★ 为什么只做子集
 *    完整的 UDS 有二十多个服务，动手前先按"刷固件到底用哪几个"筛过：
 *    · 会话/保活/复位/安全访问：进编程会话的前置条件
 *    · 让总线安静（28）、关故障记录（85）：刷写期间的现场卫生
 *    · 擦除/校验（31）、请求下载（34）、传数据（36）、退出（37）：真正写 Flash 的
 *    故障码读写（19/14/22/2E）对这块板没有意义，不做。
 *
 *  ★ 影响什么
 *    本模块自己不碰 Flash，只做"报文进、报文出"。写暂存区在下一层。
 *    收到请求时会停掉周期报文（28 服务），对方 5 秒不发 3E 就自动恢复。
 *
 *  ★ 怎么验证
 *    PC 发 3E 00 应回 7E 00；发 10 03 应回 50 03 00 32 13 88；
 *    发一个没实现的服务应回 7F <SID> 11。详见 77 号文档 §7。
 *============================================================================*/

#include "can_tx.h"          /* ★ 拿 CAN_EN */

#if (CAN_EN != 0)

#include <stdint.h>

/* ---- 服务号 ------------------------------------------------------------- */

#define UDS_SID_SESSION        0x10u    /* 诊断会话控制 */
#define UDS_SID_ECU_RESET      0x11u    /* ECU 复位 */
#define UDS_SID_SECURITY       0x27u    /* 安全访问（种子-密钥） */
#define UDS_SID_COMM_CTRL      0x28u    /* 通信控制（停/开周期报文） */
#define UDS_SID_ROUTINE        0x31u    /* 例程控制（擦除 / 校验） */
#define UDS_SID_REQ_DOWNLOAD   0x34u    /* 请求下载（协商块长） */
#define UDS_SID_TRANSFER       0x36u    /* 传输数据 */
#define UDS_SID_TRANSFER_EXIT  0x37u    /* 退出传输 */
#define UDS_SID_TESTER_PRESENT 0x3Eu    /* 保活 */
#define UDS_SID_CONTROL_DTC    0x85u    /* 故障记录开关 */
#define UDS_SID_READ_DID       0x22u    /* 按标识符读数据（2026-11-24 加） */
#define UDS_DID_ISOTP_STAT     0xF1A0u  /* 上一条 ISO-TP 包的长度与 CRC16，给回归用 */

/* ---- 否定响应码（只列这套实现用得到的） --------------------------------- */

#define UDS_NRC_SERVICE_NOT_SUPPORTED   0x11u   /* 不认识这个服务 */
#define UDS_NRC_SUBFUNC_NOT_SUPPORTED   0x12u   /* 认识服务，不认这个子功能 */
#define UDS_NRC_INCORRECT_LENGTH        0x13u   /* 报文长度不对 */
#define UDS_NRC_CONDITIONS_NOT_CORRECT  0x22u   /* 当前条件不满足 */
#define UDS_NRC_REQUEST_SEQUENCE_ERROR  0x24u   /* 调用顺序不对 */
#define UDS_NRC_REQUEST_OUT_OF_RANGE    0x31u   /* 参数越界 */
#define UDS_NRC_SECURITY_DENIED         0x33u   /* 没通过安全访问 */
#define UDS_NRC_INVALID_KEY             0x35u   /* 密钥错 */
#define UDS_NRC_EXCEED_ATTEMPTS         0x36u   /* 错太多次，暂时锁了 */
#define UDS_NRC_WRONG_BLOCK_SEQ         0x73u   /* 块序号不对 */
#define UDS_NRC_RESPONSE_PENDING        0x78u   /* 我在忙，等我 */
#define UDS_NRC_SERVICE_NOT_IN_SESSION  0x7Fu   /* 当前会话不允许这个服务 */

/* ---- 会话类型 ----------------------------------------------------------- */

#define UDS_SESSION_DEFAULT     0x01u   /* 默认会话：日常运行 */
#define UDS_SESSION_PROGRAM     0x02u   /* 编程会话：刷写专用 */
#define UDS_SESSION_EXTENDED    0x03u   /* 扩展会话：预编程/后编程 */

/* ---- 时间参数（响应里要报给 Tester，所以必须和实际行为一致） ------------- */

#define UDS_P2_MS      50u      /* 收到请求后多久给响应 */
#define UDS_P2STAR_MS  5000u    /* 回了 0x78 之后允许多久 */
#define UDS_S3_MS      5000u    /* 多久没收到 3E 就退回默认会话 */
#define UDS_LOCK_MS    600000u  /* 密钥连错 3 次锁多久（10 分钟） */

/* ---- 接口 --------------------------------------------------------------- */

/* ★ 初始化：会话回默认、计数清零。必须在 IsoTp_Init() 之后调用 */
void Uds_Init(void);

/* ★ 任务侧轮询
 * · 干两件事：① 从 ISO-TP 取一包请求并分派 ② 检查 S3 超时
 * · 必须按 ISOTP_POLL_MS 的周期调用 */
void Uds_Poll(void);

/* ★ 当前会话，给上层（比如 0x34 判断能不能收数据）用 */
uint8_t Uds_GetSession(void);

/* ★ 周期报文是否被 0x28 关掉了：1 = 关（总线安静），0 = 正常发 */
uint8_t Uds_CommDisabled(void);

/* ★ 故障记录是否被 0x85 关掉了：1 = 关（别写假故障），0 = 正常记 */
uint8_t Uds_DtcDisabled(void);

/* ★ 安全访问通过了吗
 * · 1 = 已解锁，危险动作（擦除 / 传数据 / 复位）放行
 * · 0x11 复位现在要求它；下周的 31/34/36/37 也用它做门槛 */
uint8_t Uds_IsUnlocked(void);

/* ---- 统计量 ------------------------------------------------------------- */

extern volatile uint32_t g_uds_req_cnt;    /* 收到的请求数 */
extern volatile uint32_t g_uds_nrc_cnt;    /* 回过的否定响应数 */
extern volatile uint32_t g_uds_s3_timeout; /* 因超时退回默认会话的次数 */

#endif /* CAN_EN */

#endif /* __UDS_H */
