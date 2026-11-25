#ifndef __DHT22_H
#define __DHT22_H

#include "stm32f10x.h"

/*==============================================================================
 * DHT22 / AM2302 温湿度（PB15 = DATA）—— 对应执行表 阶段3 新增
 *
 *  ★ 与 DS18B20 的关键区别（决定了本驱动更简单）：
 *    · DHT22 模块自带 10k 上拉+ 去耦电容（R=103，C=100nF）
 *      → 所以固定用开漏输出 Out_OD即可
 *      → 不像 DS18B20 那样需要 IPU/Out_PP 动态切换（那会带来 4.26us 的重配开销）
 *    · 协议不同：DHT22 是私有单总线，不是 1-Wire
 *      时序完全不同，且位间隙没有容错空间 → 整帧必须包一次临界区
 *
 *  ★ 时间约束（调用者必须遵守）：
 *    · 采样间隔 ≥2 秒（器件要求，快了会读到旧值或失败）
 *    · 一帧 40 位 × ~120us ≈ 4.8ms全部在临界区内
 *      → 配合 DS18B20 的 7ms/s，任务侧按2 秒一次调用，平均占用 <0.3%
 *    · 上电后器件需约 1 秒稳定 → DHT22_Init 里已延时
 *
 *  ★ 无效值约定：读取失败时调用者应填 0xFFFF（与记录格式一致）
 *    本驱动不直接填，因为它不知道你要写进哪个字段
 *============================================================================*/

#define DHT22_GPIO      GPIOB
#define DHT22_PIN       GPIO_Pin_15
/* 读取一次：成功返回 1
 *   humi_x10 —— 湿度 ×10（如 653 = 65.3% RH）
 *   temp_x10 —— 温度 ×10（如 245 = 24.5°C，负温为负值）
 * ★ 失败时不改动两个输出，由调用者决定填什么（建议 0xFFFF）。 */
uint8_t DHT22_Read(uint16_t *humi_x10, int16_t *temp_x10);

/* 初始化：配引脚（开漏输出 + 上拉输入复用同一寄存器配置）+ 上电稳定延时
 * ★ 只需要调一次（系统初始化时）；不需要每次读之前调。 */
void DHT22_Init(void);

void    DHT22_BootTest(void);       /* ★ 2026-10-24 从 main.c 搬来：开机自检读一次。 */

/* ★ 2026-11-04 加：失败原因记账（查"约 15% 偶发失败"用，查完可删）
 *   reason 码值见 DHT22.c 顶部那段说明；ok/fail 是累计次数 */
extern volatile uint8_t  g_dht_reason;
extern volatile uint16_t g_dht_ok;
extern volatile uint16_t g_dht_fail;

/* ★ 2026-11-04 加：reason=2 那次的加长观察结果（查责任方用）
 *   extra_us = 超时之后又等了多少微秒线才变高（65000 = 一直没高）
 *   line_high = 最后读到的电平 */
extern volatile uint16_t g_dht_extra_us;
extern volatile uint8_t  g_dht_line_high;

/* ★ 2026-11-24 加：位宽体检（成功也记，不用等失败）
 *
 * ★ 为什么：3~6% 的偶发失败可遇不可求，等一次要几十分钟。
 *   而"判决余量还剩多少"在成功读取时就能测出来 ——
 *   余量最薄的那一项，就是将来失败的地方，不用等它坏。
 *
 * ★ 影响什么：正常读多 40 次 CYCCNT 差值；一帧的关中断总时长不变
 *
 * ★ 怎么验证：串口发 H，下一次读取（≤3 秒）后打出 40 位宽度与应答时序 */
#define DHT22_BITS          40u   /* 湿度16 + 温度16 + 校验8；DHT22.c 的 DHT_BITS 由它派生 */

extern volatile uint16_t g_dht_width[DHT22_BITS];   /* 每位高电平宽度 us */
extern volatile uint8_t  g_dht_bits[DHT22_BITS];    /* 每位解出来的 0/1 */
extern volatile uint16_t g_dht_ack_delay_us;        /* 释放 -> 器件拉低 us */
extern volatile uint16_t g_dht_ack_low_us;          /* 应答低电平宽度 us */
extern volatile uint16_t g_dht_ack_high_us;         /* 应答高电平宽度 us */
extern volatile uint16_t g_dht_rc[7];               /* reason 0..6 各出现几次 */
extern volatile uint8_t  g_dht_diag_ready;          /* 1 = 有完整帧可供体检 */
extern volatile uint16_t g_dht_last_humi;           /* 上次成功解出的湿度 ×10 */
extern volatile int16_t  g_dht_last_temp;           /* 上次成功解出的温度 ×10 */

/* 打印上一次读的位宽明细与余量。只在任务里调，临界区内禁止（它会打印）。 */
void DHT22_DumpDiag(void);

extern volatile uint16_t g_dht_reinit_cnt;   /* 连续失败后重配引脚了几次 */
extern volatile uint16_t g_dht_humi_ffff;    /* 帧校验通过但湿度字段是 0xFFFF 的次数 */
#endif
