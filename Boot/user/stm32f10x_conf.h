/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_conf.h 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Library configuration file.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_CONF_H
#define __STM32F10x_CONF_H

/* Includes ------------------------------------------------------------------*/
/* Uncomment/Comment the line below to enable/disable peripheral header file inclusion */

/* ★★★★ 2026-10-13 裁剪（★ Bootloader 专用 ✓）dsh 改
 *
 *  ★ 为什么必须裁 ✗：
 *    · ★ 这个文件【把【所有外设模块的头文件【全部 include 进来】】✗✓
 *    · ★★ 而【Bootloader 只保留了 5 个标准库 .c ✗
 *      对应的 .h 也只有那 5 个】✓
 *    → ★★★ 【不裁 ✗ 就会报 `cannot open stm32f10x_adc.h`】✓✓
 *
 *  ★★ 保留原则 ✗：★【只留【实际要用到的】✗ 让依赖【显式可见】】✓
 *    · ★ flash ✗（★ 擦写 App 区 —— Bootloader 的核心 ✓）
 *    · ★★ gpio  ✗（★ 按键 / LED / 片选 ✓）
 *    · ★★★ rcc   ✗（★ 时钟使能 ✓）
 *    · ★★★★ usart ✗（★ 串口日志 + 以后的升级通道 ✓）
 *    · ★★★★★ misc  ✗（★ NVIC / SysTick ✓，在下面一行 ✗ 不在此列 ✓）
 *
 *  ★★★ 以后要用别的模块 ✗ → ★【在这里取消注释 ✗ 并把对应 .c 加进工程】✓✓
 *
 *  ★★★★ 注意 ✗：★ 原来的 ST 模板是【全部打开的】✗ ——
 *    那是给"什么都可能用到"的工程用的 ✓ 我们要按需裁剪 ✓✓ */

/*#include "stm32f10x_adc.h"*/          /* ★ 不用（★ App 的 AD 驱动才需要 ✓）*/
#include "stm32f10x_bkp.h"              /* ★★★【要】✗ 升级请求标志存这里（BKP_DR2）✓✓
                                          *   ★★ 它【不占 Flash ✗ 只用一个备份寄存器】✓
                                          *   ★★★ 2026-10-13 打开（★ 第二步用 ✓）*/
/*#include "stm32f10x_can.h"*/          /* ★ 不用 ✓ */
/*#include "stm32f10x_cec.h"*/          /* ★ 不用 ✓ */
/*#include "stm32f10x_crc.h"*/          /* ★ 不用（★ 我们用软件 CRC-16 ✓）*/
/*#include "stm32f10x_dac.h"*/          /* ★ 不用 ✓ */
/*#include "stm32f10x_dbgmcu.h"*/       /* ★ 不用 ✓ */
/*#include "stm32f10x_dma.h"*/          /* ★ 不用 ✓ */
/*#include "stm32f10x_exti.h"*/         /* ★ 不用 ✓ */
#include "stm32f10x_flash.h"            /* ★★★【要】✗ 擦写 App 区 ✓✓ */
/*#include "stm32f10x_fsmc.h"*/         /* ★ 不用（★ C8T6 没有 FSMC ✓）*/
#include "stm32f10x_gpio.h"             /* ★★【要】✗ 按键 / 片选 ✓ */
/*#include "stm32f10x_i2c.h"*/          /* ★ 不用 ✓ */
/*#include "stm32f10x_iwdg.h"*/         /* ★ 不用（★ 裸机 Bootloader 跑几秒就跳 ✗ 不需要 ✓）*/
#include "stm32f10x_pwr.h"              /* ★★★【要】✗ 解锁备份域必须调
                                          *   `PWR_BackupAccessCmd(ENABLE)` ✓
                                          *   ★★ 没有它 ✗【BKP 寄存器【读不出也写不进】】✓✓
                                          *   ★ 2026-10-13 打开（★ 第二步用 ✓）*/
#include "stm32f10x_rcc.h"              /* ★★★【要】✗ 时钟 ✓ */
/*#include "stm32f10x_rtc.h"*/          /* ★ 不用 ✓ */
/*#include "stm32f10x_sdio.h"*/         /* ★ 不用（★ 我们用 SPI 模式 ✗ 不用 SDIO ✓）*/
/*#include "stm32f10x_spi.h"*/          /* ★ 不用（★ App 的 SD 驱动才需要 ✓）
                                          *   ★★ 以后做"SD 卡升级"时要打开 ✓ */
#include "stm32f10x_usart.h"            /* ★★★★【要】✗ 串口 ✓ */
/*#include "stm32f10x_wwdg.h"*/         /* ★ 不用 ✓ */
#include "misc.h" /* High level functions for NVIC and SysTick (add-on to CMSIS functions) */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Uncomment the line below to expanse the "assert_param" macro in the 
   Standard Peripheral Library drivers code */
/* #define USE_FULL_ASSERT    1 */

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT

/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function which reports 
  *         the name of the source file and the source line number of the call 
  *         that failed. If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */

#endif /* __STM32F10x_CONF_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
