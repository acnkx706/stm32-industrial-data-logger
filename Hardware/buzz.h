#ifndef __buzz_H
#define __buzz_H

void Buzz_Init(void);
void Buzz_ON(void);
void Buzz_Turn(void);
void Buzz_OFF(void);

/*==============================================================================
 *  ★ 蜂鸣器服务层（2026-10-13 加）
 *
 *  ★ 为什么要有这一层：
 *    · 原来只有 Buzz_ON/OFF→ 直接操作 GPIO
 *    · 于是"什么时候响、响多久"会散落到各个任务里
 *      → 想改个节奏要满工程找
 *    · 所以做成两层：
 *      · 请求层：任何任务只调 Buzz_Request(事件)，不碰 GPIO
 *      · 节奏层：一个任务（task_DISP）调 Buzz_Poll()
 *    → 好处：加新事件只需加一个宏，不用改节奏逻辑
 *
 *  ★ 硬件事实（写在代码里，免得以后忘）：
 *    · PB12，/ 低电平触发（Buzz_ON = ResetBits）
 *    · 是有源蜂鸣器（给电平就响，不需要 PWM）
 *
 *  ★ 为什么不直接在任务里 Buzz_ON()：
 *    · 那样没人负责关掉它 → 一直响
 *    · 而且多个任务同时想响，会互相打架
 *============================================================================*/

/* ★ 事件定义（位标志，可以叠加） */
#define BUZZ_EV_SD_ERR   0x01u   /* ★ SD 故障：持续间歇响（直到 Clear） */
#define BUZZ_EV_KEY      0x02u   /* ★ 按键反馈：短促一声（自动结束） */
#define BUZZ_EV_LONG     0x04u   /* ★ 长响一声（2026-10-13 加
                                  *   用途：安全关机完成的确认音
                                  *   ★ 为什么比按键音长：
                                  *     要让人在"可以拔电了"这件事上不犹豫） */

/* ★ 2026-11-15 加：两种告警音，和 SD 故障音区分开
 *
 * · DATA_LOSS = 数据正在丢失（缓存溢出 / 积压过多 / 卡不在）
 *   节奏：两短一停，周期约 2.4 秒
 *   依据：2026-11-13 至 11-15 拔卡两天，缓存积压 3419 批、溢出 68 次，
 *         现场没有任何提示，事后才发现丢了两天数据
 *
 * · SENSOR = 传感器故障（读数非法 / 卡死 / 长期无回波）
 *   节奏：一短一长停，周期约 1.6 秒，比 DATA_LOSS 缓
 *   依据：2026-10-29 至 11-04 DHT22 连续 7 天读失败，CSV 里全是 65535 */
#define BUZZ_EV_DATA_LOSS 0x08u
#define BUZZ_EV_SENSOR    0x10u

void    Buzz_Request(uint8_t ev);    /* ★ 请求响，任意任务可调。 */
void    Buzz_Clear(uint8_t ev);      /* ★ 解除持续性事件（如 SD 恢复） */
void    Buzz_Poll(void);             /* ★ 由 task_DISP 每帧调用，产生节奏。 */
void    Buzz_Silence(uint8_t on);    /* ★ 静音开关，1=静音。 */
uint8_t Buzz_IsSilent(void);

/* ★ 2026-11-10 加：静音状态持久化（存 BKP_DR4，有电池 ⇒ 断电也记得）
 *   为什么：M1/M0 原来只有 RAM 标志，复位/重烧就失效，台面调试时反复响。
 *   用法：开机 Buzz_Init() 之后调 Buzz_Restore()；M1/M0 用 Buzz_SilencePersist()。 */
void    Buzz_Restore(void);
void    Buzz_SilencePersist(uint8_t on);

#endif
