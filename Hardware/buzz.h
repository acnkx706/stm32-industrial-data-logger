#ifndef __buzz_H
#define __buzz_H

void Buzz_Init(void);
void Buzz_ON(void);
void Buzz_Turn(void);
void Buzz_OFF(void);

/*==============================================================================
 *  ★★★★ 蜂鸣器服务层（2026-10-13 加 ✓）
 *
 *  ★ 为什么要有这一层 ✗：
 *    · ★ 原来只有 `Buzz_ON/OFF` ✗ → 【直接操作 GPIO】✓
 *    · ★★ 于是【"什么时候响、响多久"会散落到各个任务里】✗✓
 *      → 【想改个节奏要满工程找】✓
 *    · ★★★ 所以做成【两层】✗✓：
 *      · **请求层**：★ 任何任务只调 `Buzz_Request(事件)` ✗ 不碰 GPIO ✓
 *      · **节奏层**：★ 一个任务（★ task_DISP ✓）调 `Buzz_Poll()` ✓
 *    → ★★★★ 好处：★【加新事件只需加一个宏 ✗ 不用改节奏逻辑】✓✓
 *
 *  ★★ 硬件事实（★ 写在代码里 ✗ 免得以后忘 ✓）：
 *    · ★ PB12 ✗ / 【低电平触发】✓（★ Buzz_ON = ResetBits ✓）
 *    · ★★ 是【有源】蜂鸣器 ✓（★ 给电平就响 ✗ 不需要 PWM ✓）
 *
 *  ★★★ 为什么【不直接在任务里 Buzz_ON()**✗：
 *    · ★ 那样【没人负责关掉它】✗ → 【一直响】✓
 *    · ★★ 而且【多个任务同时想响 ✗ 会互相打架】✓✓
 *============================================================================*/

/* ★ 事件定义（★ 位标志 ✗ 可以叠加 ✓）*/
#define BUZZ_EV_SD_ERR   0x01u   /* ★ SD 故障：持续间歇响（★ 直到 Clear ✓）*/
#define BUZZ_EV_KEY      0x02u   /* ★ 按键反馈：短促一声（★ 自动结束 ✓）*/

void    Buzz_Request(uint8_t ev);    /* ★ 请求响 ✗ 任意任务可调 ✓ */
void    Buzz_Clear(uint8_t ev);      /* ★ 解除【持续性】事件 ✓（如 SD 恢复 ✓）*/
void    Buzz_Poll(void);             /* ★ 由 task_DISP 每帧调用 ✗ 产生节奏 ✓ */
void    Buzz_Silence(uint8_t on);    /* ★ 静音开关 ✗ 1=静音 ✓ */
uint8_t Buzz_IsSilent(void);

#endif
