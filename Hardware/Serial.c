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

	/* ★★★★ PA10 = USART1_RX，2026-10-12 加 ✓
	 *   ★ 为什么用【上拉输入 IPU】✗：
	 *     · USART 空闲时线是【高电平】✓
	 *     · ★★ 如果配成【浮空 ✗ 而 CH340 又没接】→ 引脚悬空 ✓
	 *       → 【随机抖动会被当成【起始位】✗ → 疯狂进中断】✓✓
	 *     · ★★★ IPU 保证【没接时也是高 ✗ 不产生假起始位】✓
	 *   ★★ 原来的配置只有 Tx ✗ —— 所以【单片机根本收不到任何东西】✓ */
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin= GPIO_Pin_10 ;
	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate=115200;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode=USART_Mode_Tx | USART_Mode_Rx;   /* ★ 加 Rx ✓ */
	USART_InitStructure.USART_Parity=USART_Parity_No;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_Init(USART1,&USART_InitStructure);

	/* ★★ 只开 RXNE ✗ —— 【不开 ERR 类中断】✓
	 *   理由：★ 线路噪声会置 FE/NE/ORE ✗ 若开了【会被噪声刷屏】✓
	 *   ★★ 而 ORE 不用管 ✗：我们的 ISR 每次读 DR ✓
	 *     → 【读 DR 会顺带清掉 ORE ✗ 不会卡死】✓✓ */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	/* ★★★ NVIC（★ 组别由 main 统一设成 Group_4 ✗ 驱动里【不许再配】**✓）
	 *   · ★ 抢占优先级 6 ✗ → ★★ 【落在 RTOS 管理区（5~15）】✓
	 *     → 可以被 taskENTER_CRITICAL 屏蔽 ✓
	 *   · ★★ 而我们的 ISR【不调用任何 FreeRTOS API】✗（只写环形缓冲 ✓）
	 *     → 【所以不需要 FromISR 家族**✗ 也不用 portYIELD_FROM_ISR ✓✓
	 *   · ★★★ 子优先级 0 ✓ */
	{
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_Init(&NVIC_InitStructure);
	}

	USART_Cmd(USART1,ENABLE);
	
}


/*==============================================================================
 *  ★★★★ 串口接收环形缓冲（2026-10-12 加 ✓）
 *        用途：★ 让 PC 能把"当前时间"发给单片机 ✗ 实现【连上就对时】✓
 *
 *  ★ 为什么用【环形缓冲 + 裸 ISR】✗：
 *    · ★ ISR 里【不碰任何 FreeRTOS API】✗ → 最短、最安全 ✓
 *      （★ 队列的 FromISR 也能用 ✗ 但【这里没必要 ✗ 杀鸡用牛刀】✓）
 *    · ★★ 生产者在【中断上下文】✗ 消费者在【任务上下文】✓
 *      → 【单生产者单消费者 ✗ 天然无锁】✗✓!!
 *      · head 只被 ISR 改 ✓ tail 只被任务改 ✓
 *      · ★★★ 只要 head/tail 是【volatile ✗ 且各改各的】✗
 *        → 【不需要临界区】✓✓
 *
 *  ★★ 缓冲区大小【必须是 2 的幂】✗ —— 这样【取模可以用位与 & ✗ 比 % 快】✓
 *  ★★★ 满了怎么办 ✗：★【丢弃新字节 ✗ 不覆盖未读数据】✗
 *    → 【宁可丢 ✗ 不能让 PC 的命令被撕成半截】✓✓
 *============================================================================*/
#define SER_RX_BUF_SIZE   64u
#define SER_RX_MASK       (SER_RX_BUF_SIZE - 1u)

static volatile uint8_t  s_rx_buf[SER_RX_BUF_SIZE];
static volatile uint16_t s_rx_head = 0u;      /* ★ ISR 写 ✓ */
static volatile uint16_t s_rx_tail = 0u;      /* ★ 任务读 ✓ */

/* 取一个已收到的字节
 * 返回 0~255 = 数据 ✓；-1 = 【当前没有数据】✓
 * ★★ 为什么用 int16_t 而不是 uint8_t ✗：
 *   【0x00 是合法的字节值】✗✓ —— 用 0 表示"没有数据"会【把真数据当成空】✓ */
int16_t Serial_RxPop(void)
{
	int16_t v = -1;

	if (s_rx_head != s_rx_tail)
	{
		v = (int16_t)s_rx_buf[s_rx_tail];
		s_rx_tail = (uint16_t)((s_rx_tail + 1u) & SER_RX_MASK);
	}
	return v;
}

/* ★ 当前缓冲里还有多少字节待取（★ 只为调试观察 ✗ 不必精确 ✓）*/
uint16_t Serial_RxCount(void)
{
	return (uint16_t)((s_rx_head - s_rx_tail) & SER_RX_MASK);
}

/* ★★★ USART1 中断服务函数
 *   ★ 名字【必须一字不差】✗（★ 在 startup 文件的向量表里 ✓）
 *     写错【不报错 ✗ 只是永远不被调用】✓✓
 *   ★★ ISR 三条规矩（★ 本函数全部满足 ✓）：
 *     · 只做【最少的事】✗（读一个字节 + 存进缓冲 ✓）
 *     · 不调用任何 FreeRTOS API ✓
 *     · 不阻塞、不死等 ✓ */
void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		uint8_t  b    = (uint8_t)USART_ReceiveData(USART1);
		uint16_t next = (uint16_t)((s_rx_head + 1u) & SER_RX_MASK);

		if (next != s_rx_tail)          /* ★ 没满才存 ✗ 满则丢弃 ✓ */
		{
			s_rx_buf[s_rx_head] = b;
			s_rx_head = next;
		}
		/* ★ 读 DR 已经把 RXNE 清了 ✗ 不需要再手动清标志 ✓ */
	}
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
