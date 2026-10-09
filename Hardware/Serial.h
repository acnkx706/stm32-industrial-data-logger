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
#endif
