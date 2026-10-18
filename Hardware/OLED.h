#ifndef __OLED_H
#define __OLED_H

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_WriteDataN(const uint8_t *p, uint16_t n);

void OLED_SelfCheck(void);        /* ★★★ 2026-10-14 加：开机自检（一行输出 ✓）*/

/* ★★★★ 2026-10-18 加：显示开关（★ 低功耗改造 序 2 ✓）
 *   · `OLED_DisplayOff()` 会【连电荷泵一起关】✗ 才是真省电 ✓
 *     （★ 只发 0xAE 关显示 ✗ 电荷泵照跑 ⇒ 省不到一半 ✓）
 *   · ★★ 关掉再开回来之后，面板会显示【旧内容】✗ 调用者必须整屏重画 ✓ */
void    OLED_DisplayOff(void);
void    OLED_DisplayOn(void);
uint8_t OLED_IsOn(void);
#endif
