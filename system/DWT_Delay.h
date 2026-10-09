#ifndef __DWT_DELAY_H
#define __DWT_DELAY_H

#include "stm32f10x.h"

/*==============================================================================
 *  DWT 寄存器兼容 shim（★ 为什么写在头文件里）
 *  本工程 Start\core_cm3.h 是 ST 标准库 V3.5 附带的老版 CMSIS（v1.30），
 *  里面【没有】DWT 外设结构体（CoreDebug 有、DWT 没有）；新版 CMSIS 才有。
 *  不改现有文件的对策：按 Cortex-M3 手册地址 0xE0001000 本地补一个。
 *  ★ 用 #ifndef 保护：将来换新 CMSIS（自带 DWT 定义）时这段自动失效，不冲突。
 *============================================================================*/
#ifndef DWT

typedef struct
{
	volatile uint32_t CTRL;     /* 0x00 控制寄存器（bit0 = CYCCNTENA 计数器使能） */
	volatile uint32_t CYCCNT;   /* 0x04 周期计数器（自由递增，32 位回绕） */
} DWT_Delay_Regs_t;

#define DWT                     ((DWT_Delay_Regs_t *)0xE0001000ul)
#define DWT_CTRL_CYCCNTENA_Msk  (1ul << 0)

#endif /* DWT */

void DWT_Delay_Init(void);
void DWT_Delay_us(uint32_t us);
void DWT_Delay_ms(uint32_t ms);

#endif
