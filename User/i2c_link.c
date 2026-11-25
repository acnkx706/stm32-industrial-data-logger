/**
  ******************************************************************************
  * @file    i2c_link.c
  * @brief   I2C 板间链路（F103 主机，PB8=SCL / PB9=SDA，50 kHz）
  ******************************************************************************
  * ★ 用了哪些外设
  * · I2C1 重映射到 PB8/PB9（默认在 PB6/PB7，那两个脚被别的外设占了）
  * · 重映射必须开 AFIO 时钟，漏了 GPIO_PinRemapConfig 会静默失败
  *
  * ★ 为什么是开漏
  * · I2C 是开漏总线，两个节点共拉一条线，谁拉低都能被所有人看到
  * · 如果配成推挽，两个节点一个拉高一个拉低就会短路
  *
  * ★ 影响什么
  * · PB8/PB9 配成 I2C 后不能再给 CAN 用
  * · 发一帧阻塞约 1.6 ms，放在 task_PROC 里会被 ACQ 抢占，不影响采集节拍
  *
  * ★ 怎么验证
  * · g_i2c_tx_ok 每秒 +10，g_i2c_tx_fail 保持 0
  * · F407 那边 RX 计数同步增长
  ******************************************************************************
  */

#include "i2c_link.h"

#if (I2C_EN != 0)

#include "stm32f10x.h"

volatile uint32_t g_i2c_tx_ok   = 0u;
volatile uint32_t g_i2c_tx_fail = 0u;

#define I2C_LINK_TIMEOUT   20000u   /* 忙等上限，防止总线被拉死时卡住 */

/* ★ 初始化
 * · 先开时钟、重映射、配开漏引脚，再配 I2C 参数
 * · 主机模式下 OwnAddress1 随便填，用不到
 */
void I2C_Link_Init(void)
{
    GPIO_InitTypeDef  gpio;
    I2C_InitTypeDef   i2c;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    /* ★ I2C1 重映射到 PB8(SCL) / PB9(SDA) */
    GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);

    gpio.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9;
    gpio.GPIO_Mode  = GPIO_Mode_AF_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    i2c.I2C_Mode                = I2C_Mode_I2C;
    i2c.I2C_DutyCycle           = I2C_DutyCycle_2;
    i2c.I2C_OwnAddress1         = 0x30u;
    i2c.I2C_Ack                 = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i2c.I2C_ClockSpeed          = 50000u;   /* 上拉只有 40k，压到 50 kHz */

    I2C_Init(I2C1, &i2c);
    I2C_Cmd(I2C1, ENABLE);
}

/* ★ 发一帧
 * · 标准的主机发送时序：START -> 地址(写) -> 数据... -> STOP
 * · 每一步都带超时，从机不应答就立刻放弃，绝不死等
 */
uint8_t I2C_Link_Send(const uint8_t *data, uint8_t len)
{
    uint32_t timeout;
    uint8_t  i;

    if (len > I2C_LINK_FRAME)
    {
        len = I2C_LINK_FRAME;
    }

    /* ① START */
    I2C_GenerateSTART(I2C1, ENABLE);
    timeout = I2C_LINK_TIMEOUT;
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) == ERROR)
    {
        if (--timeout == 0u)
        {
            g_i2c_tx_fail++;
            return 0u;
        }
    }

    /* ② 从机地址 + 写方向（8 位形式：7 位地址左移一位） */
    I2C_Send7bitAddress(I2C1, (uint8_t)(I2C_LINK_ADDR7 << 1), I2C_Direction_Transmitter);
    timeout = I2C_LINK_TIMEOUT;
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == ERROR)
    {
        if (--timeout == 0u)
        {
            I2C_GenerateSTOP(I2C1, ENABLE);
            g_i2c_tx_fail++;
            return 0u;
        }
    }

    /* ③ 数据 */
    for (i = 0u; i < len; i++)
    {
        I2C_SendData(I2C1, data[i]);
        timeout = I2C_LINK_TIMEOUT;
        while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING) == ERROR)
        {
            if (--timeout == 0u)
            {
                I2C_GenerateSTOP(I2C1, ENABLE);
                g_i2c_tx_fail++;
                return 0u;
            }
        }
    }

    /* ④ 等最后一个字节真正发完再 STOP */
    timeout = I2C_LINK_TIMEOUT;
    while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == ERROR)
    {
        if (--timeout == 0u)
        {
            break;
        }
    }

    I2C_GenerateSTOP(I2C1, ENABLE);

    g_i2c_tx_ok++;
    return 1u;
}

#endif /* I2C_EN */
