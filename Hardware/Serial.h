#ifndef __Serial_H
#define __Serial_H
#include <stdio.h>
void Serial_SendByte(uint8_t Byte);
void Serial_Init(void);
void Serial_SenaArray(uint8_t *Array,uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number,uint8_t Length);
uint32_t Serial_Pow(uint32_t X,uint32_t Y);
void Serial_Printf(char*format,...);

/* ★★★★ 串口接收（2026-10-12 加 ✓）—— 为"PC 连上就自动对时"服务 ✓
 *   ★ Serial_RxPop() ：★ 取一个字节 ✗ 返回 -1 表示【当前没有数据】✓
 *     （★ 用 int16_t 是因为【0x00 是合法字节 ✗ 不能拿 0 当"空"】✓✓）
 *   ★★ Serial_RxCount()：★ 还剩几个待取 ✓（调试观察用 ✓）*/
int16_t  Serial_RxPop(void);
uint16_t Serial_RxCount(void);

#endif
