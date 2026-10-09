/**
  ******************************************************************************
  * @file    DWT_Delay.c
  * @brief   DWT 微秒/毫秒延时（FreeRTOS 安全版）
  *
  *  ★★ 为什么必须有这个模块：system\Delay.c 是雷 ★★
  *  裸机的 Delay_us 用 SysTick 计数实现，且每次调用都会【重新配置并关闭 SysTick】。
  *  FreeRTOS 的心跳恰恰也压在 SysTick 上 —— 调度器启动后任何代码调一次
  *  Delay_us/ms/s，等于把 FreeRTOS 的时基拆了：所有 vTaskDelay 永远不醒、
  *  时间片轮转失效（用户已踩过此坑，记录在 skill 的 traps 里）。
  *
  *  DWT（Data Watchpoint and Trace）是 Cortex-M3 内核自带的调试计数器，
  *  与 SysTick 完全独立：不占中断、不占外设、不碰 RTOS 的任何资源。
  *  它随 CPU 主频自由递增（72MHz 下每秒 +72000000），用【无符号减法】测流逝
  *  周期即可实现精确忙等。
  *
  *  使用规则（对齐 skill 规范）：
  *    - 调度器启动前：可用 Delay.c，也可用本模块（均可）
  *    - 调度器启动后：只能用本模块 + vTaskDelay，【禁止】Delay.c
  *    - 本延时是【忙等】：空转期间 CPU 不让出，只适合微秒~毫秒级的短等待
  *      （SPI 时序、单总线时隙）。长延时一律 vTaskDelay，别拿它睡大觉。
  *
  *  典型用户：Hardware\SD_SPI.c（SD 卡 busy 等待/初始化节拍）、
  *           Hardware\MyI2C.c（软件 I2C 时隙）—— 都是"短、必须原地等"的场景
  ******************************************************************************
  */
#include "stm32f10x.h"                  /* Device header（内含 core_cm3.h：DWT/CoreDebug 寄存器） */
#include "DWT_Delay.h"

/**
  * @brief  DWT 延时初始化：打开周期计数器
  *         只需调用一次（重复调用无害）；SD_SPI\MyI2C 的 Init 内部已各自调用，
  *         用户一般不需要手动调。
  * @param  无
  * @retval 无
  */
void DWT_Delay_Init(void)
{
	/* DEMCR 的 TRCENA：DWT 属于调试跟踪部件，这一位是它的总电源开关
	 * （复位后默认关闭，不打开的话 DWT->CTRL/CYCCNT 写入无效） */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

	DWT->CYCCNT = 0;                        /* 周期计数值清零 */
	DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;   /* CYCCNTENA：启动周期计数器 */
}

/**
  * @brief  微秒级忙等延时
  * @param  us : 延时的微秒数（单次上限约 59.6s：2^32/72/1e6，超过会绕圈）
  * @retval 无
  * @note   为什么用"无符号减法"判断：CYCCNT 是 32 位自由计数器，约 59.6s 就
  *         溢出回绕；无符号减法（当前-起点）在回绕时结果依然正确，
  *         比判断"是否溢出"再分支要简洁且无漏洞。
  */
void DWT_Delay_us(uint32_t us)
{
	uint32_t start;

	/* 自愈检查：万一调用者忘了 Init，计数器没在跑，下面会死等 —— 这里补救一次 */
	if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0 ||
		(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0)
	{
		DWT_Delay_Init();
	}

	start = DWT->CYCCNT;
	while ((DWT->CYCCNT - start) < us * 72u);   /* 72MHz 主频：1µs = 72 个周期 */
}

/**
  * @brief  毫秒级忙等延时
  * @param  ms : 延时的毫秒数
  * @retval 无
  * @note   拆成 1ms 片循环而不是一次算 ms*72000：避免大参数时 us*72 的
  *         32 位溢出（一次最多 59.6s 的限制就不存在了）。
  *         ★ 提醒：超过 10ms 的等待请改用 vTaskDelay，让出 CPU 给别的任务。
  */
void DWT_Delay_ms(uint32_t ms)
{
	while (ms--)
	{
		DWT_Delay_us(1000);
	}
}
