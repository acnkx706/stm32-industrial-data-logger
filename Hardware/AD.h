#ifndef __AD_H
#define __AD_H

/* 两个通道的原始值（12 位，0~4095）：由 DMA 在后台连续刷新，任务里直接读、零阻塞
 *   AD_Value[0] = PA1 = ADC1_IN1 = 光敏 AO（模拟量）
 *   AD_Value[1] = PB0 = ADC1_IN8 = 电池分压（20k+10k → 实际电压 = 读数 × VDDA/4096 × 3） */
extern volatile uint16_t AD_Value[2];
void AD_TestVdd();  
void AD_Init(void);

#endif