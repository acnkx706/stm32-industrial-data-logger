#ifndef __MYRTC_H
#define __MYRTC_H

#include "stm32f10x.h"

/*==============================================================================
 *  MyRTC —— RTC 走时 + 日期换算（阶段 3）
 *
 *  ★ 命名跟 MyI2C 一致（"自己写的模块"统一前缀）
 *    —— 同时避开 SPL 自带的 stm32f10x_rtc.c（Windows 文件系统大小写不敏感）
 *
 *  ★ 硬件前提（已实测确认）：
 *    · 板载 32.768kHz 晶振起振正常（RCC_BDCR = 0x00000003：LSEON=1 / LSERDY=1 /
 *      LSEBYP=0 → 用的是真晶振，不是内部旁路）
 *    · 分频 32768 → 1Hz（RTC_SetPrescaler(32767)）
 *    · 走时已实测：counter 每秒严格 +1，10 秒涨 10
 *
 *  ★ 时间基准 = 编译时刻（__DATE__ / __TIME__）
 *    · 优点：零硬件、零上位机，立刻有个"日期"
 *    ★ 2026-11-04 实测：本板已装 CR2032 ⇒ 这条路径【平时不跑】，
 *      只在"首次上电 / 换电池 / 电池耗尽"导致备份域清零时才走一次
 *    ★ 2026-10-17 实测推翻了三件事：
 *      · 它不是"这次构建"的时刻——
 *        它是MyRTC.c 这个 .o 最后一次被编译的时刻
 *        ⇒ 增量编译会让它停在很久以前，而且你不会被通知
 *        （实测：嵌入 10-16 16:35:55，而真构建时刻 10-17 01:10:54
 *          ⇒ 旧了 8 小时 35 分）
 *      · 而且它被当成UTC用，⇒ 比真 UTC大 8 小时
 *      · 净效果：上电后时间可能偏几个小时
 *        ⚠️ 2026-10-19 修正口径：原写"偏 22 小时"
 *          · 而本节自己列的两项是"旧了 8h35m"与"比 UTC 大 8h"
 *            ★ 两者方向相反⇒ 两种算法都得不到 22h
 *          · 真正被实测记录到的是 main.c 里那行
 *            real error measured: 22h35m（一次现场事故）
 *            ⇒ 别把"某次的 22h35m"当成"必然的净偏差"
 *          ★ 结论不变，它只能当兜底
 *        而开机那个"自检"照样 PASS（见 main.c 里的说明）
 *    ⚠️ 结论：它只能当兜底，绝不能当"当前时间"用
 *       正式产品应改成显式版本常量或干脆不设，只等对时
 *
 *  ★ 掉电保时：VBAT 接 CR2032（本板已装，2026-11-04 实测确认）
 *    · 有电池 → 断电后 RTC 计数器继续走、BKP 寄存器全部保留
 *      ⇒ MyRTC_Init 走"already configured"分支，不会再用编译时刻重设
 *    · 没电池（换电池 / 电池耗尽）→ 备份域清零 → 下次上电才用编译时刻
 *    · 电池只保"供电"，不保"晶振"：LSE 起不来时，有电池照样不走时
 * ============================================================================*/

typedef struct
{
	uint16_t year;          /* 例如 2026。 */
	uint8_t  month;         /* 1~12。 */
	uint8_t  day;           /* 1~31。 */
	uint8_t  hour;          /* 0~23。 */
	uint8_t  minute;        /* 0~59。 */
	uint8_t  second;        /* 0~59。 */
} RTC_DateTime_t;

/* MyRTC_Init 返回值 */
#define RTC_OK               0u
#define RTC_ERR_NO_LSE       1u     /* ★ LSE 没起振（时钟源问题，与 RTC 逻辑无关） */

/* 初始化（含 LSE 检查 + 分频 + 魔数保护 + 编译时刻设初值）
 * 返回：RTC_OK / RTC_ERR_NO_LSE
 * ★ 可重复调用：已配置过（魔数命中）就只做 LSE 检查 + 同步，不动计数器。 */
uint8_t MyRTC_Init(void);

/* 读当前 Unix 时间戳（秒）—— 记录里的 rtc 字段就用它。 */
uint32_t MyRTC_GetUnix(void);

/* Unix 时间戳 → 年月日时分秒（显示/调试用） */
void MyRTC_GetDateTime(RTC_DateTime_t *dt);

/*==============================================================================
 *  ★ 2026-10-21 补声明：任意时间戳 → 年月日时分秒
 *
 *  ★ 为什么需要：这个函数一直存在（MyRTC.c 里是非 static）
 *    但头文件里没有声明，⇒ 别的文件调不到
 *    （这正是审计点过的那类"头/源接口不一致"
 *      —— 定义在、声明无 ⇒ 谁都用不了，编译器也不报）
 *
 *  ★ 用途：显示本地时间
 *    · RTC 和 CSV 里存的都是 UTC（数据必须无歧义，老规矩）
 *    · 而屏上给人看 ⇒ 用 MyRTC_UnixToYmd(MyRTC_GetUnix() + 8*3600, &dt)
 *    · 为什么不改 RTC 本身：那样 CSV 的 rtc 列就变成"本地时间戳"，
 *      换时区/换季节就全乱
 *============================================================================*/
void MyRTC_UnixToYmd(uint32_t ts, RTC_DateTime_t *dt);
/* 时区偏移（小时）：本地时间 = UTC + 这个值。
 * 只用于"给人看"的日期（文件命名、换文件判据、屏显），
 * 绝对不用于 CSV 的 rtc 列 —— 那列必须是无歧义的 UTC。 */
#define MYRTC_TZ_OFFSET_HOURS  8u
#define MYRTC_TZ_OFFSET_SEC    (MYRTC_TZ_OFFSET_HOURS * 3600u)

/* 读当前本地年月日时分秒（= UTC + MYRTC_TZ_OFFSET_SEC）。
 * 与 MyRTC_GetDateTime() 的区别：那个返回 UTC，这个返回本地。 */
void MyRTC_GetLocalDateTime(RTC_DateTime_t *dt);

/* 设置时间（留给以后上位机/串口对时） */
void MyRTC_SetUnix(uint32_t ts);

/* ★ RTC 是否曾被正确初始化过（备份寄存器魔数判据）
 *   ★ 0 = 没配过 → 十有八九是 LSE 没起振
 *     → 这时写计数器毫无意义，串口对时前必须先查它 */
uint8_t MyRTC_IsConfigured(void);

/* ★ RTC 的值可不可信（2026-10-17 新增）
 *   ★ 判据 = 备份寄存器 BKP_DR5 上的魔数
 *     · 只有串口对时成功（MyRTC_SetUnix）才会置它
 *     · 没装 VBAT 时一断电就丢；本板已装 ⇒ 断电也保留
 *       （换电池 / 电池耗尽 / 备份域被复位时才丢）
 *   ★ 为什么不能用 MyRTC_IsConfigured()（BKP_DR1）代替：
 *     · DR1 只说明"RTC 被初始化过"
 *     · 而初始化用的可能正是编译时刻—— 一个可能很旧、且被当 UTC 用的值
 *     ⇒ "配过" ≠ "对过"，这两件事必须分开记
 *   ★ 用途：数据时间戳的"信用标签"
 *     · 0 ⇒ 串口报警 + OLED 显示 T?（见 main.c / task_DISP） */
uint8_t MyRTC_IsTrusted(void);

/* ★ 纯函数（不碰硬件，单独拿出来方便自检）
 *   YmdToUnix / UnixToYmd 互为反函数。 */
uint32_t MyRTC_YmdToUnix(uint16_t y, uint8_t mo, uint8_t d,
                         uint8_t hh, uint8_t mm, uint8_t ss);
void     MyRTC_UnixToYmd(uint32_t ts, RTC_DateTime_t *dt);

#endif
