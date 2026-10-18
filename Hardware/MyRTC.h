#ifndef __MYRTC_H
#define __MYRTC_H

#include "stm32f10x.h"

/*==============================================================================
 *  MyRTC —— RTC 走时 + 日期换算（阶段 3）
 *
 *  ★ 命名跟 MyI2C 一致（"自己写的模块"统一前缀 ✓）
 *    —— 同时避开 SPL 自带的 stm32f10x_rtc.c ✗（Windows 文件系统大小写不敏感 ✓）
 *
 *  ★ 硬件前提（已实测确认 ✓）：
 *    · 板载 32.768kHz 晶振起振正常 ✓（RCC_BDCR = 0x00000003：LSEON=1 / LSERDY=1 /
 *      LSEBYP=0 → ★ 用的是【真晶振】✗ 不是内部旁路 ✓）
 *    · 分频 32768 → 1Hz（RTC_SetPrescaler(32767) ✓）
 *    · ★ 走时已实测：counter 每秒严格 +1，10 秒涨 10 ✓✓
 *
 *  ★ 时间基准 = 【编译时刻】（`__DATE__` / `__TIME__` ✓）
 *    · 优点：零硬件、零上位机 ✓ 立刻有个"日期"✓
 *    ★★★★★ 2026-10-17 实测推翻了三件事 ✗✓：
 *      · ★★ 它【不是"这次构建"的时刻】✗ ——
 *        它是【`MyRTC.c` 这个 .o 最后一次被编译的时刻】✓
 *        ⇒ 增量编译会让它【停在很久以前】✗ 而且【你不会被通知】✓✓
 *        （★ 实测：嵌入 10-16 16:35:55 ✗ 而真构建时刻 10-17 01:10:54
 *          ⇒ 【旧了 8 小时 35 分】✓）
 *      · ★★ 而且它被当成【UTC】用 ✗ ⇒ 比真 UTC【大 8 小时】✓
 *      · ★★★ 净效果：上电后时间可能【偏 22 小时】✗
 *        而开机那个"自检"【照样 PASS】✗（★ 见 main.c 里的说明 ✓✓）
 *    ⚠️ 结论：★ 它只能当【兜底】✗ 【绝不能当"当前时间"用】✓✓
 *       正式产品应改成【显式版本常量】或【干脆不设 ✗ 只等对时】✓
 *
 *  ★ 掉电保时：★ 需要 VBAT 接 CR2032 ✓
 *    · 没装电池 → 断电后时间丢 ✗（但【运行期间】完全正常 ✓）
 *    · 备份域靠 BKP 寄存器里的【魔数】判断"是否已配置过"✓
 *      → 有电池时，复位/断电都不会把时间清零 ✓✓
 * ============================================================================*/

typedef struct
{
	uint16_t year;          /* 例如 2026 ✓ */
	uint8_t  month;         /* 1~12 ✓ */
	uint8_t  day;           /* 1~31 ✓ */
	uint8_t  hour;          /* 0~23 ✓ */
	uint8_t  minute;        /* 0~59 ✓ */
	uint8_t  second;        /* 0~59 ✓ */
} RTC_DateTime_t;

/* MyRTC_Init 返回值 */
#define RTC_OK               0u
#define RTC_ERR_NO_LSE       1u     /* ★ LSE 没起振（时钟源问题 ✓ 与 RTC 逻辑无关 ✓）*/

/* 初始化（含 LSE 检查 + 分频 + 魔数保护 + 编译时刻设初值 ✓）
 * 返回：RTC_OK / RTC_ERR_NO_LSE ✓
 * ★ 可重复调用：已配置过（魔数命中）就只做 LSE 检查 + 同步，不动计数器 ✓ */
uint8_t MyRTC_Init(void);

/* 读当前 Unix 时间戳（秒 ✓）—— ★ 记录里的 rtc 字段就用它 ✓ */
uint32_t MyRTC_GetUnix(void);

/* Unix 时间戳 → 年月日时分秒 ✓（显示/调试用 ✓）*/
void MyRTC_GetDateTime(RTC_DateTime_t *dt);

/* 设置时间（★ 留给以后上位机/串口对时 ✓）*/
void MyRTC_SetUnix(uint32_t ts);

/* ★★★ RTC 是否曾【被正确初始化过】✗（★ 备份寄存器魔数判据 ✓）
 *   ★ 0 = 没配过 ✗ → 【十有八九是 LSE 没起振】✓
 *     → ★★ 这时【写计数器毫无意义 ✗ 串口对时前必须先查它】✓✓ */
uint8_t MyRTC_IsConfigured(void);

/* ★★★★★ RTC 的值【可不可信】✗✓（★ 2026-10-17 新增 ✓）
 *   ★ 判据 = 备份寄存器 `BKP_DR5` 上的魔数 ✓
 *     · ★ 只有【串口对时成功】（`MyRTC_SetUnix`）才会置它 ✓
 *     · ★★ 没装 VBAT ⇒ 一断电就丢 ⇒ 回到"不可信"✓✓
 *   ★★★★ 为什么【不能】用 `MyRTC_IsConfigured()`（`BKP_DR1`）代替 ✗✓：
 *     · `DR1` 只说明"RTC 被【初始化】过" ✗
 *     · 而初始化用的可能正是【编译时刻】—— 一个可能很旧、且被当 UTC 用的值 ✓
 *     ⇒ ★★【"配过" ≠ "对过"】✗ 这两件事必须分开记 ✓✓
 *   ★ 用途：★ 数据时间戳的"信用标签" ✓
 *     · 0 ⇒ 串口报警 + OLED 显示 `T?`（★ 见 main.c / task_DISP ✓）*/
uint8_t MyRTC_IsTrusted(void);

/* ★ 纯函数（不碰硬件 ✓ 单独拿出来方便自检 ✓）
 *   YmdToUnix / UnixToYmd 互为反函数 ✓ */
uint32_t MyRTC_YmdToUnix(uint16_t y, uint8_t mo, uint8_t d,
                         uint8_t hh, uint8_t mm, uint8_t ss);
void     MyRTC_UnixToYmd(uint32_t ts, RTC_DateTime_t *dt);

#endif
