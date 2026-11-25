#ifndef __I2C_LINK_H
#define __I2C_LINK_H

/*==============================================================================
 *  I2C 板间链路（F103 主机 -> F407 从机）
 *
 *  ★ 编译开关
 *    I2C_EN = 0：整个模块不参与编译，产物与 ver79 逐字节一致
 *    I2C_EN = 1：启用 I2C1 主机，PB8=SCL / PB9=SDA，50 kHz
 *
 *  ★ 为什么用 I2C 而不是 CAN
 *    CAN 的 TX 是推挽输出，不能并联，ACK 位要靠收发器做线与才能回环。
 *    I2C 本身就是开漏 + 上拉的总线，天然支持线与，两根线直连就能通。
 *
 *  ★ 为什么速率只有 50 kHz
 *    F103 的 I2C 引脚在开漏复用模式下用不了内部上拉，
 *    整条总线的上拉只能靠 F407 那边的内部 40k。
 *    40k 上拉边沿慢，压到 50 kHz 才留得出余量。
 *============================================================================*/

#define I2C_EN   0

#if (I2C_EN != 0)

#include <stdint.h>

#define I2C_LINK_ADDR7     0x50u    /* 7 位从机地址 */
#define I2C_LINK_FRAME     8u       /* 每帧字节数 */

/* ★ 初始化：需要开 AFIO 时钟做 I2C1 重映射 */
void    I2C_Link_Init(void);

/* ★ 发一帧
 * · 返回 1 = 从机应答了，0 = 没应答或超时
 * · 阻塞时间约 1.6 ms（8 字节 @ 50 kHz），不要放在高优先级任务里太久
 */
uint8_t I2C_Link_Send(const uint8_t *data, uint8_t len);

/* ★ 统计量 */
extern volatile uint32_t g_i2c_tx_ok;
extern volatile uint32_t g_i2c_tx_fail;

#endif /* I2C_EN */

#endif /* __I2C_LINK_H */
