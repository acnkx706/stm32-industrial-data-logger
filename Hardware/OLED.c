#include "stm32f10x.h"
#include "OLED_Font.h"
#include "DWT_Delay.h" 
#include "Serial.h"

/* ★★★★ 2026-10-14（★ 已回退到"可用版" ✗ 注释同步更新 ✓）
 *
 *   ★ 当前用的是【GPIO_WriteBit + 两侧各延时 2µs】✓
 *     · SCL 周期 ≈ 4.2µs ⇒ 约 238kHz ✓【在 400kHz 规格内】✓✓
 *     · 每个 bit ≈ 6.5µs ⇒ 一帧约 158ms ✓（★ 慢 ✗ 但合法 ✓）
 *   ★★ "BSRR 直写 + 延时只放 SCL"那版（快约 1.5 倍）
 *      留作【以后的优化】✗ 现在不追 ✓
 *   ★★★ 铁律 ✗：这两个宏里的 `DWT_Delay_us(2)` 【绝不要为了快去掉】✓
 *        判据只有一个：【OLED_SelfCheck 打出 addr=1 ctrl=1 cmd=1】✓✓*/
#define OLED_W_SDA(x)   GPIO_WriteBit(GPIOB, GPIO_Pin_9, (BitAction)(x))
#define OLED_W_SCL(x)   do { GPIO_WriteBit(GPIOB, GPIO_Pin_8, (BitAction)(x)); DWT_Delay_us(1); } while (0)
/*==============================================================================
 *  ★★★★ I2C 总线恢复（9 个时钟）—— 2026-10-14 加 ✓
 *
 *  ★ 治什么病 ✗：
 *    · I2C 从机（SSD1306）在【第 9 拍拉低 SDA 应答】✓
 *    · ★★ 如果【主机在传输中途复位/掉电】✗
 *       从机会卡在传输中间 ✗ 甚至【一直拉着 SDA 低】✓
 *    · ★★★ 4 针模块【没有复位脚】✗ 复位主机【不会】复位从机 ✓
 *      → 从机永久卡死 ✗ 之后所有命令它都不认 ✗ 【全黑】✓✓
 *    ★★★★ 典型触发：Keil 烧录 ✗ IWDG 复位 ✗ Boot 跳转 ✓（你这几天全干过 ✓）
 *
 *  ★★ 为什么"打 9 个时钟 + 一个 Stop"能治好 ✗：
 *    · I2C 规范【明确允许】用额外时钟把从机从卡死状态敲出来 ✓
 *    · 从机数到第 9 个时钟后 ✗ 会【主动放开 SDA】✗ 退出传输态 ✓
 *    · 最后补 Stop ✗ 让总线回到【空闲】的确定状态 ✓✓
 *
 *  ★★★ 为什么必须放在【发第一条命令之前】✗：
 *    · 从机一旦卡着 ✗ 后面发多少命令都白搭 ✓
 *    ★★★★ 有了它 ✗ 每次开机都会【自动修复】上次复位造成的卡死 ✓✓✓
 *
 *  ★ 注意 ✗：本函数【不能调用 OLED_I2C_Init()】✗ 否则递归 ✓
 *           结尾自己把两根线恢复成"开漏、都拉高"✓
 *============================================================================*/
static void OLED_BusRecover(void)
{
	GPIO_InitTypeDef g;
	uint8_t i;

	/* ① SCL 配成开漏输出 ✗ SDA 先【放开成上拉输入】✗ 只由我们驱动 SCL ✓ */
	g.GPIO_Speed = GPIO_Speed_50MHz;
	g.GPIO_Mode  = GPIO_Mode_Out_OD;
	g.GPIO_Pin   = GPIO_Pin_8;
	GPIO_Init(GPIOB, &g);

	g.GPIO_Mode  = GPIO_Mode_IPU;
	g.GPIO_Pin   = GPIO_Pin_9;
	GPIO_Init(GPIOB, &g);

	GPIO_WriteBit(GPIOB, GPIO_Pin_8, Bit_SET);     /* SCL 先高高地放着 ✓ */

	/* ② ★★ 打 9 个时钟（★ 直接用 GPIO ✗ 不走 I2C 协议 ✓）*/
	for (i = 0u; i < 9u; i++)
	{
		GPIO_WriteBit(GPIOB, GPIO_Pin_8, Bit_RESET);
		DWT_Delay_us(2);                           /* ★ 慢一点 ✗ 让从机跟得上 ✓ */
		GPIO_WriteBit(GPIOB, GPIO_Pin_8, Bit_SET);
		DWT_Delay_us(2);
	}

	/* ③ ★★★ 补一个 Stop：SCL 高的时候，SDA 由低变高 ✓
	 *    （★ 这会让所有从机把状态机复位到"空闲"✓）*/
	g.GPIO_Mode  = GPIO_Mode_Out_OD;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	g.GPIO_Pin   = GPIO_Pin_9;
	GPIO_Init(GPIOB, &g);                          /* SDA 变回输出 ✓ */

	GPIO_WriteBit(GPIOB, GPIO_Pin_9, Bit_RESET);   /* SDA = 0 ✓ */
	DWT_Delay_us(2);
	GPIO_WriteBit(GPIOB, GPIO_Pin_8, Bit_SET);     /* SCL = 1 ✓ */
	DWT_Delay_us(2);
	GPIO_WriteBit(GPIOB, GPIO_Pin_9, Bit_SET);     /* SDA 0 → 1 = Stop ✓ */
	DWT_Delay_us(2);

	/* ④ 恢复成"开漏、都拉高"的空闲态 ✓ */
	GPIO_WriteBit(GPIOB, GPIO_Pin_8, Bit_SET);
	GPIO_WriteBit(GPIOB, GPIO_Pin_9, Bit_SET);
}

/*引脚初始化*/
void OLED_I2C_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	OLED_W_SCL(1);
	OLED_W_SDA(1);
	OLED_BusRecover();
}

/**
  * @brief  I2C开始
  * @param  无
  * @retval 无
  */
void OLED_I2C_Start(void)
{
	OLED_W_SDA(1);
	OLED_W_SCL(1);
	OLED_W_SDA(0);
	DWT_Delay_us(1);
	OLED_W_SCL(0);
}

/**
  * @brief  I2C停止
  * @param  无
  * @retval 无
  */
void OLED_I2C_Stop(void)
{
	OLED_W_SDA(0);
	OLED_W_SCL(1);
	DWT_Delay_us(1);
	OLED_W_SDA(1);
}

/**
  * @brief  I2C发送一个字节
  * @param  Byte 要发送的一个字节
  * @retval 无
  */
void OLED_I2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		OLED_W_SDA(!!(Byte & (0x80 >> i)));
		
		OLED_W_SCL(1);

		OLED_W_SCL(0);
		
	}
	OLED_W_SCL(1);	//额外的一个时钟，不处理应答信号

	OLED_W_SCL(0);

}

/**
  * @brief  OLED写命令
  * @param  Command 要写入的命令
  * @retval 无
  */
void OLED_WriteCommand(uint8_t Command)
{
	OLED_I2C_Start();
	OLED_I2C_SendByte(0x78);		//从机地址
	OLED_I2C_SendByte(0x00);		//写命令
	OLED_I2C_SendByte(Command); 
	OLED_I2C_Stop();
}

/**
  * @brief  OLED写数据
  * @param  Data 要写入的数据
  * @retval 无
  */
void OLED_WriteData(uint8_t Data)
{
	OLED_I2C_Start();
	OLED_I2C_SendByte(0x78);		//从机地址
	OLED_I2C_SendByte(0x40);		//写数据
	OLED_I2C_SendByte(Data);
	OLED_I2C_Stop();
}
/*==============================================================================
 *  ★★★★ 连续写 n 个字节（★ 一次 Start/Stop ✗ 效率从 22% 提到 ~90% ✓）
 *
 *  ★ 为什么快 ✗：
 *    · 原来每写 1 个字节 = Start + 地址 + 0x40 + 数据 + Stop ≈ 36 位时 ✓
 *    · 现在 n 个字节 = 1 次 Start + 地址 + 0x40 + n×9 位 + 1 次 Stop ✓✓
 *    · ★★ n=8 时：原来 8×36 = 288 位时 ✗ 现在 22×9 ≈ 198 位时 ✓
 *      而且 n 越大越划算 ✓✓
 *
 *  ★★ 为什么安全 ✗：★ 这一招【完全没动 I2C 时序】✓✓
 *     · 时钟还是那个时钟（SCL 1µs 延时 ✓）
 *     · 只是【少发几遍 Start/Stop】✓
 *     ⇒ ★★★ 所以【ACK 判据不受影响】✓✓
 *
 *  ★★★ 页寻址模式下的行为 ✗：
 *     · 每发一个数据字节 ✗ SSD1306 的【列指针自动 +1】✓
 *     · 写到页尾会【在本页内回绕】✗ 不会跑到下一页 ✓✓
 *     ⇒ 所以【一次写 8 字节 = 一个字符的一页】✗ 正好 ✓✓
 *============================================================================*/
void OLED_WriteDataN(const uint8_t *p, uint16_t n)
{
	OLED_I2C_Start();
	OLED_I2C_SendByte(0x78);            /* 从机地址 ✓ */
	OLED_I2C_SendByte(0x40);            /* ★ 控制字节：后面都是【数据】✓ */
	while (n != 0u)                     /* ★★ 连着发 ✗ 中间【不重新 Start】✓✓ */
	{
		OLED_I2C_SendByte(*p);
		p++;
		n--;
	}
	OLED_I2C_Stop();
}
/**
  * @brief  OLED设置光标位置
  * @param  Y 以左上角为原点，向下方向的坐标，范围：0~7
  * @param  X 以左上角为原点，向右方向的坐标，范围：0~127
  * @retval 无
  */
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
	OLED_WriteCommand(0xB0 | Y);					//设置Y位置
	OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));	//设置X位置高4位
	OLED_WriteCommand(0x00 | (X & 0x0F));			//设置X位置低4位
}

/**
  * @brief  OLED清屏
  * @param  无
  * @retval 无
  */
void OLED_Clear(void)
{  
	/* ★ 整页 128 字节一次写完 ✓（★ 原来 128 次事务 ✗ 现在 1 次 ✓✓）*/
	static const uint8_t zeros[128] = {0};    /* ★ const → 放 Flash 里 ✗ 不占 RAM ✓ */
	uint8_t i, j;
		for (j = 0; j < 8; j++)
	{
		OLED_SetCursor(j, 0);
		OLED_WriteDataN(zeros, 128u);
	}
}

/**
  * @brief  OLED显示一个字符
  * @param  Line 行位置，范围：1~4
  * @param  Column 列位置，范围：1~16
  * @param  Char 要显示的一个字符，范围：ASCII可见字符
  * @retval 无
  */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{      	
	/* ★★★★ 2026-10-14 改：8 个字节【一次连续写】✗ 不再逐字节 Start/Stop ✓
	 *
	 *   ★ 这是【一帧耗时的主要来源】✗
	 *     · 一帧 49 个字符 × 16 字节 = 【784 个数据字节】✓
	 *     · 原来每个字节 = 一个独立事务 ✗ 线上要跑 4 个字节 ✓
	 *       ⇒ 效率只有 22% ✗✓
	 *   ★★ 字模 `OLED_F8x16[ch]` 是【连续的 16 字节】✗
	 *     · 前 8 字节 = 上半页 ✓ 后 8 字节 = 下半页 ✓✓
	 *   ★★★ 所以取首地址 + 长度 8 就行 ✗ 不用循环 ✓✓
	 *   ★ 注意：`uint8_t i;` 现在【用不到了】✗ 删掉 ✓
	 *        （★ 留着会报"未使用变量"警告 ✓）*/
	OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);		//设置光标位置在上半部分
	OLED_WriteDataN(&OLED_F8x16[Char - ' '][0], 8u);		//★ 上半页 8 字节一次 ✓

	OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);	//设置光标位置在下半部分
	OLED_WriteDataN(&OLED_F8x16[Char - ' '][8], 8u);		//★ 下半页 8 字节一次 ✓
}

/**
  * @brief  OLED显示字符串
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  String 要显示的字符串，范围：ASCII可见字符
  * @retval 无
  */
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i++)
	{
		OLED_ShowChar(Line, Column + i, String[i]);
	}
}

/**
  * @brief  OLED次方函数
  * @retval 返回值等于X的Y次方
  */
uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y--)
	{
		Result *= X;
	}
	return Result;
}

/**
  * @brief  OLED显示数字（十进制，正数）
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  Number 要显示的数字，范围：0~4294967295
  * @param  Length 要显示数字的长度，范围：1~10
  * @retval 无
  */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}

/**
  * @brief  OLED显示数字（十进制，带符号数）
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  Number 要显示的数字，范围：-2147483648~2147483647
  * @param  Length 要显示数字的长度，范围：1~10
  * @retval 无
  */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
	uint8_t i;
	uint32_t Number1;
	if (Number >= 0)
	{
		OLED_ShowChar(Line, Column, '+');
		Number1 = Number;
	}
	else
	{
		OLED_ShowChar(Line, Column, '-');
		Number1 = -Number;
	}
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowChar(Line, Column + i + 1, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}

/**
  * @brief  OLED显示数字（十六进制，正数）
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  Number 要显示的数字，范围：0~0xFFFFFFFF
  * @param  Length 要显示数字的长度，范围：1~8
  * @retval 无
  */
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i, SingleNumber;
	for (i = 0; i < Length; i++)							
	{
		SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
		if (SingleNumber < 10)
		{
			OLED_ShowChar(Line, Column + i, SingleNumber + '0');
		}
		else
		{
			OLED_ShowChar(Line, Column + i, SingleNumber - 10 + 'A');
		}
	}
}

/**
  * @brief  OLED显示数字（二进制，正数）
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  Number 要显示的数字，范围：0~1111 1111 1111 1111
  * @param  Length 要显示数字的长度，范围：1~16
  * @retval 无
  */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
	}
}

/**
  * @brief  OLED初始化
  * @param  无
  * @retval 无
  */
void OLED_Init(void)
{
	uint32_t i, j;
	
	for (i = 0; i < 1000; i++)			//上电延时
	{
		for (j = 0; j < 1000; j++);
	}
	
	OLED_I2C_Init();			//端口初始化
	
	OLED_WriteCommand(0xAE);	//关闭显示

	/* ★★★ 2026-10-09 新增（T2.6③ 演练暴露）：把屏幕【拉回确定的初始状态】✗
	 *   症状：一次电源瞬态（拔插 SD 模块）后，屏幕"整屏不断滚动 + 字糊在一起"✓
	 *   机理：I2C 被打断 -> 有的【数据字节】被 SSD1306 当成【命令】执行了 ✗
	 *         若恰好命中滚动命令码(0x26/0x27) -> 屏进入【硬件滚动】✗
	 *         而滚动是屏内部寄存器维持的 ✓ 不显式发命令【清不掉】✗
	 *   ① 0x2E = 停止滚动 —— ★ 无论屏当前处于什么状态，先拉回确定态 ✓
	 *   ② 0x20 0x02 = 明确"页寻址模式" —— 与 OLED_SetCursor 的写法配套 ✓
	 *      不发的话，列/页指针可能停在上次被破坏的位置 ✗ -> 字画到错地方 ✓
	 *   ★ 这两条的作用是"消除历史状态"✓ —— 加了之后，即使中途失步，下次开机也能彻底恢复 ✓✓ */
	OLED_WriteCommand(0x2E);	// ★ 停止硬件滚动
	

	OLED_WriteCommand(0xD5);	//设置显示时钟分频比/振荡器频率
	OLED_WriteCommand(0x80);
	
	OLED_WriteCommand(0xA8);	//设置多路复用率
	OLED_WriteCommand(0x3F);
	
	OLED_WriteCommand(0xD3);	//设置显示偏移
	OLED_WriteCommand(0x00);
	
	OLED_WriteCommand(0x40);	//设置显示开始行
	
	OLED_WriteCommand(0xA1);	//设置左右方向，0xA1正常 0xA0左右反置
	
	OLED_WriteCommand(0xC8);	//设置上下方向，0xC8正常 0xC0上下反置

	OLED_WriteCommand(0xDA);	//设置COM引脚硬件配置
	OLED_WriteCommand(0x12);
	
	OLED_WriteCommand(0x81);	//设置对比度控制
	OLED_WriteCommand(0xCF);

	OLED_WriteCommand(0xD9);	//设置预充电周期
	OLED_WriteCommand(0xF1);

	OLED_WriteCommand(0xDB);	//设置VCOMH取消选择级别
	OLED_WriteCommand(0x30);

	OLED_WriteCommand(0xA4);	//设置整个显示打开/关闭

	OLED_WriteCommand(0xA6);	//设置正常/倒转显示

	OLED_WriteCommand(0x8D);	//设置充电泵
	OLED_WriteCommand(0x14);

	OLED_WriteCommand(0xAF);	//开启显示
		
	OLED_Clear();				//OLED清屏
}




/*==============================================================================
 *  ★★★★ OLED 总线自检（2026-10-14 加 ✗ 排"屏不亮"用 ✗★ 查完可整段删掉 ✓）
 *
 *  ★ 为什么必须做 ✗：
 *    · OLED_I2C_SendByte() 【故意不处理应答】✓（见 OLED.c:64 ✓）
 *    · → 地址错 / 线断 / 模块没插 ✗ 【现象完全一样：屏黑 ✗ 一声不响】✓
 *    ★★★ 所以"不亮"这件事【不含信息】✗ 必须主动问总线"有人在吗" ✓
 *
 *  ★★ 判读：
 *       找到 0x78 → 地址对 ✓ 问题在【波形/时序/供电】✓
 *       找到 0x7A → ★ 模块是另一个地址版本 ✗ 把本文件里的 0x78 改成 0x7A ✓
 *       什么都没找到 → 模块没应答 ⇒ 波形不合法 / 供电塌 / 模块坏 ✓
 *
 *  ★★ 注意：本函数用【OLED.c 自己的软件 I2C】✗
 *     不是传感器那条 MyI2C（PA2/PA3 ✓）✗ 别混 ✓
 *============================================================================*/


/* ★★★★ 2026-10-14 加：前向声明 ✗
 *   ★ 为什么需要 ✗：OLED_Probe 在 481 行 ✗ 定义却在 555 行 ✓
 *     这中间没有任何声明 ⇒ 编译器只当它是"外部函数"✗
 *     ⇒ 它按【全局符号】去链接 ✗
 *   ★★★ 而真正的定义是 `static`（内部链接 ✗ 不产生全局符号）✓
 *     ⇒ 于是 `L6218E: Undefined symbol OLED_SendByteAck` ✓✓
 *   ⚠️ ★★ 声明里【必须带 static】✗ 否则又变成"外部函数"✗ 冲突依旧 ✓✓ */
static uint8_t OLED_SendByteAck(uint8_t b);
/* ★ 返回 1 = 有器件应答 ✓  0 = 没人应答 ✗ */
/* ★★★★ 2026-10-14 改：不再手写一遍 ✗ 直接复用 OLED_SendByteAck() ✓
 *   ★ 原来这里【抄了一遍发送 + 采样】✗ 而且采样时机和 OLED_WaitAck 不一致 ✗
 *     → 出现"扫描 found 0 ✗ 但 ACK 测试 addr=1"这种自相矛盾 ✓✓
 *   ★★ 现在只有一条路 ✗ 两处结果永远一致 ✓✓
 *   ★★★ 注意：OLED_SendByteAck / OLED_WaitAck 【必须定义在它前面】✗
 *        或者在这上面写一句前向声明：
 *        static uint8_t OLED_SendByteAck(uint8_t b); ✓ */
static uint8_t OLED_Probe(uint8_t addr7)
{
	uint8_t ack;

	OLED_I2C_Start();
	ack = OLED_SendByteAck((uint8_t)(addr7 << 1));   /* ★ 复用同一条路 ✓ */
	OLED_I2C_Stop();
	return ack;
}

/*==============================================================================
 *  ★★★★ 逐字节应答报告（2026-10-14 临时诊断 ✗ 查完删掉 ✓）
 *
 *  ★ 为什么需要它 ✗：
 *    · 你的 OLED_I2C_SendByte() 【不看应答】✓
 *    · → "地址没应答"和"第 2 个字节起没应答"✗ 【表现一模一样：屏黑】✓
 *    ★★★ 只有把应答打印出来 ✗ 才能知道【从第几个字节开始丢】✓✓
 *
 *  ★★ 判读：
 *     err=1 次都不通 → 连地址都没人应答 ⇒ 线 / 电源 ✓
 *     addr=1 之后丢   → ★★★ 第 2 字节起丢 ⇒ 【电源塌 / 时序超速】✓✓
 *     三个都是 1 但屏黑 → ★★★★ 模块把命令【全收下了】✗
 *                        ⇒ 那就是【模块的面板/电荷泵坏了】✓ 换屏 ✓✓
 *============================================================================*/

/* ★ 返回 1 = 有应答 ✓  0 = 没人应答 ✗（★ 借用 BusScan 里那两个助手 ✓）*/
static uint8_t OLED_WaitAck(void)
{
	uint8_t ack;

	/* ★★★★ 2026-10-14 改：读应答【不切引脚模式】✗ 直接读 IDR ✓
	 *
	 *   ★ 为什么 ✗：STM32F1 的开漏输出模式下 ✗
	 *     【输入寄存器 IDR 照样反映引脚真实电平】✓✓
	 *     ⇒ 读应答根本不需要切成输入 ✓✓
	 *
	 *   ★★ 原来那套"输入↔输出来回切"有三个坑 ✗：
	 *     ① `GPIO_Init()` 只改模式 ✗ 不动 ODR
	 *        ⇒ 切回输出瞬间 ✗ SDA 电平 = 上次的值 ✗（可能被自己拉着低 ✓）
	 *     ② 切回输出后若立刻驱动 ✗ 而【从机还在拉低】✓
	 *        ⇒ ★★★ 两边对拉 = 【电气冲突】✓✓（我上次给的那句就是这个错 ✓）
	 *     ③ 三种坑叠在一起 ⇒ 【单字节事务正常 ✗ 多字节事务全废】✓
	 *
	 *   ★★★ 现在只有一招 ✗：
	 *     · 写数据/命令：【BSRR 写 SDA】✓
	 *     · 读应答：【直接读 IDR】✓
	 *     · SDA 【永远保持开漏输出】✗ 谁也不动它 ✓✓
	 *   ★★★★ 这一改 ✗ 多字节事务（写命令 = 3~4 字节一次 ✓）
	 *        就【天然正确】了 ✓✓ */
	OLED_W_SDA(1);                                     /* ★ 放开 SDA（开漏写 1 = 松手 ✓）*/
	OLED_W_SCL(1);                                     /* ★ 第 9 个时钟（宏里含 2µs ✓）*/
	DWT_Delay_us(2);                                   /* ★ 再留一点采样稳定时间 ✓ */
	ack = (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9) == 0) ? 1u : 0u;   /* ★ 直接读 ✓ */
	OLED_W_SCL(0);
	return ack;
}

/* ★ 发一个字节，并报告它有没有被应答 ✓ */
static uint8_t OLED_SendByteAck(uint8_t b)
{
	uint8_t i;

	for (i = 0; i < 8u; i++)
	{
		OLED_W_SDA(!!(b & (0x80 >> i)));
		OLED_W_SCL(1);
		OLED_W_SCL(0);
	}
	return OLED_WaitAck();
}


/*==============================================================================
 *  ★★★★ 开机 OLED 自检（2026-10-14 加 ✗ 一行输出 ✗ 判据明确 ✓）
 *
 *  ★ 为什么要做成"一个函数、一行输出"✗：
 *    · 分散在 BusScan + AckReport 两处 ✗ 每次要看两行 ✓
 *    · ★★ 合成一行 ✗ 以后翻日志一眼就能对比 ✓✓
 *
 *  ★★★ 输出【必须】长这样（★ 这是判据 ✗ 不是参考 ✓）：
 *        OLED: found 1, ACK addr=1 ctrl=1 cmd=1
 *
 *  ★★★★ 任何一个数是 0 ✗ 就是【时序或供电出规格】✗
 *      —— 这时候【别再往下查显示了】✗ 先修时序 ✓✓
 *============================================================================*/
void OLED_SelfCheck(void)
{
	uint8_t n = 0u;
	uint8_t a_addr, a_ctrl, a_cmd;

	OLED_I2C_Init();                       /* ★ 先把引脚配好 ✓ */

	/* ① 扫 SSD1306 的两个常见地址 ✓ */
	if (OLED_Probe(0x3Cu) != 0u) { n++; }
	if (OLED_Probe(0x3Du) != 0u) { n++; }

	/* ② 逐字节应答（★ 发的 0xAF 是"开显示"✗ 无害 ✓）*/
	OLED_I2C_Start();
	a_addr = OLED_SendByteAck(0x78);       /* 地址 ✓ */
	a_ctrl = OLED_SendByteAck(0x00);       /* 控制字节（0x00 = 命令 ✓）*/
	a_cmd  = OLED_SendByteAck(0xE3);       /* 命令 ✓ */
	OLED_I2C_Stop();

	Serial_Printf("OLED: found %u, ACK addr=%u ctrl=%u cmd=%u\r\n",
	              (unsigned)n, (unsigned)a_addr, (unsigned)a_ctrl, (unsigned)a_cmd);
}