#ifndef __DS18B20_H
#define __DS18B20_H

#include "stm32f10x.h"

/*==============================================================================
 * DS18B20 单总线温度传感器（PB14 = DQ）—— 对应执行表 T3.2
 *
 *  ★ 上拉策略：暂不接外部上拉。驱动用「IPU 输入 / 推挽输出」动态切换来造出上拉：
 *      - 释放总线时配 GPIO_Mode_IPU（内部上拉 ~40k 把线拉高）
 *      - 拉低总线时配 GPIO_Mode_Out_PP 并输出 0
 *    短杜邦线（<30cm）下 40k 上拉的上升时间约几微秒，小于 15us 采样窗，够用。
 *    ⚠️ 若导线较长或读数不稳（时好时坏 / CRC 错），在 DQ 与 3.3V 之间加 4.7kΩ。
 *
 *  ★ 临界区：每个字节的传输（~**630us** ✗ 原写 560us ✓）包一次 taskENTER_CRITICAL/EXIT
 *    （见 30 号 §16.2：位操作协议不能整段包临界区，否则 tick 连丢几毫秒）。
 *    → 本驱动依赖 FreeRTOS 的临界区宏；调度器启动前调用也安全。
 *
 *  ★ 两段式（不阻塞）：本文件只提供「启动转换」和「读取结果」两个动作，
 *    中间的 750ms 等待由调用者 vTaskDelay 让出 CPU（见 task_sensor）。
 *============================================================================*/

#define DS18B20_GPIO      GPIOB
#define DS18B20_PIN       GPIO_Pin_14

#define DS18B20_INVALID   (-32768)      /* 读取失败时的标记值 */

uint8_t DS18B20_Init(void);             /* 配时钟 + 复位检测：1 = 器件在线，0 = 无应答 */
void    DS18B20_Start(void);            /* SkipROM + 0x44：启动一次转换（立即返回） */
uint8_t DS18B20_Read(int16_t *raw);     /* SkipROM + 0xBE：读 9 字节并校验 CRC8；1 = 成功 */
int16_t DS18B20_ToX100(int16_t raw);    /* 原始值 -> 摄氏 x100 整数（如 2512 = 25.12C） */

/* 临时调试用（CRC 失败时定位是哪几个字节错，测好可删）：
 * 只做 复位+SkipROM+ReadScratchpad，把 9 字节原样给出，不做 CRC 校验。
 * 返回 1 = 存在脉冲检测到；sp9[0..1]=温度低/高字节，sp[8]=器件算的 CRC。 */
uint8_t DS18B20_ReadScratchpad(uint8_t *sp9);

/* 临时调试用（位级采样探针，测好可删）：
 * 读 8 位，每一位在下降沿后约 4/6/8/10/12/14/16us 各采一次，组成 7 位掩码
 * （bit0=最早那点，1=高电平）。全 1 说明器件没驱动总线。 */
void DS18B20_BitProbe(uint8_t *m8, uint8_t *present);

/* 临时调试用（测好可删）：读 ROM（0x33）。真 DS18B20 应回 8 字节且 rom[0]=0x28、rom[7]=CRC。
 * 用来分辨"写命令通不通"与"芯片到底是什么"。返回 1 = 存在脉冲 OK。 */
uint8_t DS18B20_ReadRom(uint8_t *rom8);

/* 临时调试用（测好可删）：量存在脉冲宽度（us）。0=无应答；0xFFFF=线卡死；真器件 60~240us */
uint16_t DS18B20_MeasurePresence(void);

/* 临时静态测试（测完删）：low=1 拉低 DQ / low=0 释放；函数不返回，用万用表量 PB14 */
void DS18B20_PinTest(uint8_t low);
/* 临时静态测试（测完删）：死循环发复位脉冲；万用表量 PB14 平均电压应明显下降 */
void DS18B20_ResetLoop(void);

/* 临时调试用（测好可删）：量时隙实际宽度（DWT 周期，72MHz 下 1us=72）
 * pull_ovh=GPIO 重配耗时 / lo6us=DWT_Delay_us(6) / one_ms=DWT_Delay_us(1000) 应≈72000 / slot70us
 * rd_release=读时隙"下降沿->释放" / rd_sample=读时隙"下降沿->采样"（必须 <1080 = 15us）*/
void DS18B20_TimingCheck(uint32_t *pull_ovh, uint32_t *lo6us, uint32_t *one_ms, uint32_t *slot70us,
                         uint32_t *rd_release, uint32_t *rd_sample);

#endif
