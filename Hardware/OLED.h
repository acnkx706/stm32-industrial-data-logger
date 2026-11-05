#ifndef __OLED_H
#define __OLED_H

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
/* ★ 2026-10-23 删：ShowHexNum / ShowBinNum 声明（零调用，见 OLED.c） */
void OLED_WriteDataN(const uint8_t *p, uint16_t n);

void OLED_SelfCheck(void);        /* ★ 2026-10-14 加：开机自检（一行输出） */

/* ★ 2026-10-18 加：显示开关（低功耗改造 序 2）
 *   · OLED_DisplayOff() 会连电荷泵一起关，才是真省电
 *     （只发 0xAE 关显示，电荷泵照跑 ⇒ 省不到一半）
 *   · 关掉再开回来之后，面板会显示旧内容，调用者必须整屏重画。 */
void    OLED_DisplayOff(void);
void    OLED_DisplayOn(void);
uint8_t OLED_IsOn(void);

/* ★ 2026-11-04 加：面板点亮测试（查"总线 ACK 全 1 但屏黑"）
 *   · 发 0xA5 强制全屏点亮 3 秒，再发 0xA4 回到显存
 *   · 白屏出现 = 面板与电荷泵是好的，问题在数据通道 */
void    OLED_PanelTest(void);

/* ★ 2026-11-04 加：非阻塞版全显/回显存（on=1 全白，on=0 回显存） */
void    OLED_PanelForce(uint8_t on);

/* ★ 2026-11-04 加：SCL/SDA 1Hz 慢速拉低（给万用表看，判引脚能不能驱动） */
void    OLED_PinSlowToggle(uint8_t low);

/* ★ 2026-11-04 加：全屏写 + 逐字节数应答（判"雪花是总线丢还是面板坏"） */
void    OLED_BusStress(void);
#endif
