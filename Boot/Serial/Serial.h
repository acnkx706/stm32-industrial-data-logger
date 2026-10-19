#ifndef __Serial_H
#define __Serial_H

/*==============================================================================
 *  ★★★★★ Bootloader 专用串口（2026-10-13 【有意与 App 分叉】✓）
 *
 *  ⚠️ ★★★★ 重要 ✗：★ 这份 `Serial.h` / `Serial.c` 【只给 Bootloader 用】✗✓
 *    · ★ App 用的是 `Hardware\Serial.c` ✗【那一份【保留 `Serial_Printf`】】✓
 *    · ★★ 【两份【故意不同】✗✓ —— ★【不是 bug ✗ 别去"修"】✓✓
 *
 *  ★ 为什么分叉 ✗（★ 实测数据 ✓）：
 *    · ★ Bootloader 加 `Serial_Printf`（★ `vsnprintf` ✓）✗
 *      → Code 从 **2,564 B** 涨到 **6,228 B** ✗✓!!
 *      · ★★ 【多出来的 3,664 B 里 ✗ 还有 970 B 是【浮点库】✗
 *        （★ 因为 `vsnprintf` 支持 `%f` ✗ 而我们【完全不用】✓）
 *    · ★★★ 而【16KB 的 Bootloader（★ 扩过 ✗ 原 8KB）现在余量约 8KB】**✗
 *      当年 8KB 时【只剩 1.8KB】⇒ 那才是"塞不下"的由来 ✓✓
 *    · ★★★★ App 那边【没这个压力 ✗ 而且【要打印带小数点的传感器值】✗
 *      → 【保留 printf 更省事】✓✓
 *
 *  ★★ 所以 Bootloader 这边【只留两种输出】✗：
 *    · `Serial_SendString()` ✗ —— ★【固定字符串 ✗ 11/12 处调用都是它】✓
 *    · `Serial_SendHex()`    ✗ —— ★【替代 `%X` / `%02X` / `%08X`】✓
 *    · `Serial_SendU32()`    ✗ —— ★【替代 `%u`】✓✓
 *============================================================================*/

void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SenaArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(const char *String);

/* ★★★★ 极简数字输出（★ 替代 printf ✓ 理由见上面 + Serial.c ✓）*/
void Serial_SendHex(uint32_t v, uint8_t digits);   /* ★ 十六进制 ✗ 指定位数 ✓ */
void Serial_SendU32(uint32_t v);                   /* ★ 无符号十进制 ✓ */

/* ★★★★ 串口接收（2026-10-12 加 ✓）—— 为"PC 下发命令/固件"服务 ✓
 *   ★ Serial_RxPop() ：★ 取一个字节 ✗ 返回 -1 表示【当前没有数据】✓
 *     （★ 用 int16_t 是因为【0x00 是合法字节 ✗ 不能拿 0 当"空"】✓✓）
 *   ★★ Serial_RxCount()：★ 还剩几个待取 ✓（调试观察用 ✓）*/
int16_t  Serial_RxPop(void);
uint16_t Serial_RxCount(void);

#endif

