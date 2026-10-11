#include "stm32f10x.h"  
#include "AD.h"
#include "Serial.h"

volatile uint16_t AD_Value[2];

void AD_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef  ADC_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); 
	
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;      /* PB0 = ADC1_IN8 = 电池分压 */
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 2, ADC_SampleTime_55Cycles5);
	
	ADC_InitStructure.ADC_Mode               = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode       = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign          = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel       = 2;
	ADC_Init(ADC1, &ADC_InitStructure);
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)&AD_Value;
	DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize         = 2;
	DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	
	DMA_Cmd(DMA1_Channel1, ENABLE); 
	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE); 
	
	ADC_ResetCalibration(ADC1);
	while (ADC_GetResetCalibrationStatus(ADC1) == SET);
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1) == SET);
	
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	
	
}

/* 临时：用内部 VREFINT(1.20V) 反推 VDD —— 独立于万用表的板子电压验证。
 * 注意：本函数会重配 ADC，所以【必须放在 AD_Init() 之前】调用，让 AD_Init 之后重新配好。 */
void AD_TestVdd(void)
{
	ADC_InitTypeDef adc;
	uint16_t raw, cal;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);               /* 72/6 = 12MHz (<=14MHz) */

	ADC_DeInit(ADC1);
	adc.ADC_Mode               = ADC_Mode_Independent;
	adc.ADC_ScanConvMode       = DISABLE;
	adc.ADC_ContinuousConvMode = DISABLE;
	adc.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None;
	adc.ADC_DataAlign          = ADC_DataAlign_Right;
	adc.ADC_NbrOfChannel       = 1;
	ADC_Init(ADC1, &adc);

	ADC_TempSensorVrefintCmd(ENABLE);               /* 打开内部通道 16/17 */
	ADC_Cmd(ADC1, ENABLE);
	ADC_ResetCalibration(ADC1); while (ADC_GetResetCalibrationStatus(ADC1) == SET);
	ADC_StartCalibration(ADC1); while (ADC_GetCalibrationStatus(ADC1) == SET);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 1, ADC_SampleTime_239Cycles5);
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	raw = ADC_GetConversionValue(ADC1);

	cal = *(uint16_t *)0x1FFFF7BA;                  /* VREFINT_CAL（VDD=3.3V 时的原始值）*/

	/* ★ 2026-10-05 修正：CAL 非法时 `3300*cal/raw` 会吐出"VDD=12784 mV"这种荒唐值 ✗
	 *   （本板 cal=8465 ✓ 克隆芯片的 CAL 是垃圾 ✓）—— 那行会把排查方向带偏 ✓
	 *   现在：CAL 合法（1..4095）才给绝对值；非法时只给 raw + 方向判据（不依赖 CAL ✓）
	 * ★★ 严格遵守项目铁律：**字符串字面量里绝对不许出现中文** ✗
	 *   （中文会打成一堆乱码，还会"吃掉下一行的首字符" ✓ —— 2026-10-05 我在这里犯过一次 ✗
	 *    中文只能写在注释里 ✓ 注释是无害的 ✓）*/
	if ((cal >= 1u) && (cal <= 4095u))
	{
		Serial_Printf("VREFINT raw=%u cal=%u -> VDD~%u mV (CAL valid, trustworthy)\r\n",
		              (unsigned)raw, (unsigned)cal,
		              (unsigned)(3300UL * (uint32_t)cal / (uint32_t)raw));
	}
	else
	{
		Serial_Printf("VREFINT raw=%u cal=%u (CAL invalid -> use direction only)\r\n",
		              (unsigned)raw, (unsigned)cal);
		Serial_Printf("  raw up = VDD down: 1489=3.3V ok / 1800=2.73V / 2185=2.25V brownout!\r\n");
	}
}

/*==============================================================================
 *  读内部基准 VREFINT（通道 17）—— 用【注入通道组】读，对常规组零影响 ✓
 *  ★ 为什么必须用注入组（而不能把通道 17 配成常规通道 ✗）：
 *    本工程的 ADC 是「常规组 = 通道1(PA1 光敏) + 通道8(PB0 电池) + 连续转换
 *    + DMA 循环写 AD_Value[]」。一旦改动常规组的通道序列 ✗
 *      -> DMA 会把 VREFINT 的转换值写进 AD_Value ✗
 *      -> 光敏/电池读数被【静默污染】✓ 而且不容易发现 ✗
 *    注入组有独立的通道配置和独立数据寄存器(JDR1) ✓ 读它不碰常规组 ✓✓
 *  ★ 典型用途：在别处（如 SD 驱动的轮询里）监测 VDD 有没有掉 ✓
 *    判据：raw 越大 = VDD 越低（1489≈3.3V / 1800≈2.73V / 2185≈2.25V ✓）
 *  ★ 注意：克隆芯片的 VREFINT_CAL 非法（本板 8465 ✗）-> 只能用 raw 判方向 ✓
 *============================================================================*/
uint16_t AD_ReadVrefint(void)
{
	uint16_t v;
	uint32_t t;

	ADC_TempSensorVrefintCmd(ENABLE);                     /* 使能内部通道（幂等 ✓）*/

	/* 注入组：软件触发、只放 1 个通道（通道 17）✓ */
	ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_None);
	ADC_ExternalTrigInjectedConvCmd(ADC1, ENABLE);        /* 允许软件触发注入组 */
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_17, 1, ADC_SampleTime_239Cycles5);

	ADC_SoftwareStartInjectedConvCmd(ADC1, ENABLE);       /* 触发一次 */

	/* ★ 加超时：绝不允许在这里死等 ✗（本函数的调用点可能在启动流程里，
	 *   一旦死等，整机上电卡住且串口一行都不打 —— 比读不到电压糟糕得多 ✓）*/
	for (t = 0; t < 200000u; t++)
	{
		if (ADC_GetFlagStatus(ADC1, ADC_FLAG_JEOC) != RESET)
		{
			break;
		}
	}

	if (ADC_GetFlagStatus(ADC1, ADC_FLAG_JEOC) != RESET)
	{
		ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);
		v = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
		return v;
	}

	return 0;                                             /* ★ 0 = 读取失败（不是"VDD=无穷大" ✗）*/
}
