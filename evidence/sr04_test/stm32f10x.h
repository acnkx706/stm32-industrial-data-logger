/* stm32f10x.h -- 主机侧桩头文件（只为编译 HC_SR04.c 做单元测试）
 *
 * 做了什么：提供 HC_SR04.c 用到的全部类型/宏/函数的最小替身
 * 为什么：真机头文件依赖 ARM 内联汇编和寄存器地址，x86 上编不过
 * 影响什么：只在这个测试目录里生效，不进 Keil 工程
 * 怎么验证：gcc 能编译通过且 HC_Poll 的行为与真机一致
 */
#ifndef STUB_STM32F10X_H
#define STUB_STM32F10X_H

#include <stdint.h>

#define ENABLE  1
#define DISABLE 0
#define RESET   0
#define SET     1

/* ---- 外设：只需要"不相等"的哑指针 ---- */
#define GPIOB   ((void *)0x40010C00u)
#define TIM4    ((void *)0x40000800u)
#define DWT     (&g_stub_dwt)

typedef struct { volatile uint32_t CYCCNT; } DWT_TypeDef;
extern DWT_TypeDef g_stub_dwt;
extern uint32_t   SystemCoreClock;

/* ---- 结构体（字段名必须与真头文件一致，否则 . 访问编不过） ---- */
typedef struct { uint16_t GPIO_Pin; uint8_t GPIO_Mode; uint8_t GPIO_Speed; } GPIO_InitTypeDef;
typedef struct { uint16_t TIM_Prescaler; uint16_t TIM_CounterMode; uint32_t TIM_Period;
                 uint16_t TIM_ClockDivision; uint8_t TIM_RepetitionCounter; } TIM_TimeBaseInitTypeDef;
typedef struct { uint16_t TIM_Channel; uint16_t TIM_ICPolarity; uint16_t TIM_ICSelection;
                 uint16_t TIM_ICPrescaler; uint16_t TIM_ICFilter; } TIM_ICInitTypeDef;
typedef struct { uint8_t NVIC_IRQChannel; uint8_t NVIC_IRQChannelPreemptionPriority;
                 uint8_t NVIC_IRQChannelSubPriority; uint8_t NVIC_IRQChannelCmd; } NVIC_InitTypeDef;

/* ---- 常量 ---- */
#define GPIO_Pin_5                 ((uint16_t)0x0020)
#define GPIO_Pin_6                 ((uint16_t)0x0040)
#define GPIO_Mode_Out_PP           ((uint8_t)0x10)
#define GPIO_Mode_IPD              ((uint8_t)0x28)
#define GPIO_Speed_50MHz           ((uint8_t)0x03)

#define RCC_APB2Periph_GPIOB       ((uint32_t)0x00000008)
#define RCC_APB1Periph_TIM4        ((uint32_t)0x00000004)

#define TIM_Channel_1              ((uint16_t)0x0000)
#define TIM_ICPolarity_Rising      ((uint16_t)0x0000)
#define TIM_ICPolarity_Falling     ((uint16_t)0x0002)
#define TIM_ICSelection_DirectTI   ((uint16_t)0x0001)
#define TIM_ICPSC_DIV1             ((uint16_t)0x0000)
#define TIM_IT_CC1                 ((uint16_t)0x0002)
#define TIM_CounterMode_Up         ((uint16_t)0x0000)
#define TIM_CKD_DIV1               ((uint16_t)0x0000)

#define TIM4_IRQn                  30

/* ---- 测试可控量 ---- */
extern uint16_t g_stub_capture;   /* TIM_GetCapture1 的返回值 */
extern uint8_t  g_stub_it;        /* TIM_GetITStatus 的返回值 */
extern uint16_t g_stub_polarity;  /* 最近一次设置的捕获极性（供断言） */

/* ---- 函数替身 ---- */
static void RCC_APB2PeriphClockCmd(uint32_t p, int e) { (void)p; (void)e; }
static void RCC_APB1PeriphClockCmd(uint32_t p, int e) { (void)p; (void)e; }
static void GPIO_Init(void *p, GPIO_InitTypeDef *g) { (void)p; (void)g; }
static void GPIO_SetBits(void *p, uint16_t pin) { (void)p; (void)pin; }
static void GPIO_ResetBits(void *p, uint16_t pin) { (void)p; (void)pin; }
static uint8_t GPIO_ReadInputDataBit(void *p, uint16_t pin) { (void)p; (void)pin; return 0u; }
static void TIM_TimeBaseInit(void *t, TIM_TimeBaseInitTypeDef *tb) { (void)t; (void)tb; }
static void TIM_ICInit(void *t, TIM_ICInitTypeDef *ic) { (void)t; g_stub_polarity = ic->TIM_ICPolarity; }
static void TIM_ClearITPendingBit(void *t, uint16_t it) { (void)t; (void)it; }
static void TIM_ITConfig(void *t, uint16_t it, int e) { (void)t; (void)it; (void)e; }
static void TIM_Cmd(void *t, int e) { (void)t; (void)e; }
static uint8_t TIM_GetITStatus(void *t, uint16_t it) { (void)t; (void)it; return g_stub_it; }
static uint16_t TIM_GetCapture1(void *t) { (void)t; return g_stub_capture; }
static void NVIC_Init(NVIC_InitTypeDef *n) { (void)n; }

#endif
