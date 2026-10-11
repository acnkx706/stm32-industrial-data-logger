#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>

void Serial_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin= GPIO_Pin_9 ;
	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate=115200;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode=USART_Mode_Tx;
	USART_InitStructure.USART_Parity=USART_Parity_No;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_Init(USART1,&USART_InitStructure);
	
	USART_Cmd(USART1,ENABLE);
	
}


void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART1,Byte);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
	
}

void Serial_SenaArray(uint8_t *Array,uint16_t Length)
{
	uint16_t i;
	for(i=0;i<Length;i++)
	{
		Serial_SendByte(Array[i]);
	}
}

void Serial_SendString(char *String)
{
	uint16_t i;                    /* ★ uint8_t → uint16_t（2026-10-11 独立审查 F19 修）
	                               *   原写法 i 是 uint8_t ✗ —— 字符串 ≥256 字节时
	                               *   i 走到 255 后 i++ 回绕成 0 ✗ → 而 String[0] 不是 '\0'
	                               *   → ★【循环永不退出】✗ 整机硬死机 + 串口狂吐同一段 ✓
	                               *   引爆点：freertos_demo.c 里
	                               *     Serial_SendString(s_task_list_buf);
	                               *   而 s_task_list_buf 是 static char[512] ✗
	                               *   现状：任务表约 190 字节 ✓ 侥幸安全 ✗
	                               *         加一个任务 / 栈水位变 5 位数 → 立刻爆 ✓ */
	for(i=0;String[i]!='\0';i++)
	{
		Serial_SendByte(String[i]);
	}
}


uint32_t Serial_Pow(uint32_t X,uint32_t Y)
{
	uint32_t Result=1;
	while(Y--)
	{
		Result*=X;
		
	}
	return Result;
}
void Serial_SendNumber(uint32_t Number,uint8_t Length)
{
	uint8_t i;
	for(i=0;i<Length;i++)
	{
		Serial_SendByte(Number/Serial_Pow(10,Length-i-1)%10+'0');
	}
}

int fputc(int ch,FILE *f)
{
	Serial_SendByte(ch);
	return ch;
}

void Serial_Printf(char*format,...)
{
	/* ★ 第 6 课修复(6)：200 → 256。账：REC 行最坏 220 字节
	 *   （4 个 uint32 最坏各 10 位 + 8 个 int16 最坏各 6 位"-32768"
	 *   + 4 个 uint16 最坏 5 位 + 15 个逗号 + 前缀"REC,"与字段名括号 + "\r\n"）
	 *   —— 200 的缓冲会被 vsnprintf 静默截断成 199 字节，CRC 列永远看不全 */
	char String[256];
	va_list arg;
	va_start(arg,format);
	vsnprintf(String,sizeof(String),format,arg);
	va_end(arg);
	Serial_SendString(String);
}
