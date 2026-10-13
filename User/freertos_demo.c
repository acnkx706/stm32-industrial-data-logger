/**
  ******************************************************************************
  * @file    freertos_demo.c
  * @brief   FreeRTOS 静态方式创建与删除任务
  
  ******************************************************************************
  */
#include "freertos_demo.h"
#include "stm32f10x.h"
#include <stdio.h>                      /* sprintf：PROC 拼 CSV 行（Y8+⑤） */
#include <string.h>                     /* memcpy：CSV 行拷进乒乓缓冲 */
#include "OLED.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Serial.h"
#include "queue.h"
#include "semphr.h"  
#include "Key.h"
#include "MPU6050.h"
#include "DWT_Delay.h"
#include "AD.h"
#include "DS18B20.h"
#include "DHT22.h"                 /* ★ DHT22 温湿度（PB15 ✓ 模块自带 10k 上拉 ✓）*/
#include "buzz.h"                  /* ★ 蜂鸣器服务（PB12 ✓ 低电平触发 ✓）*/
#include "BMP280.h"                /* ★ BMP280 气压（与 MPU6050 共用 PA2/PA3 软件 I2C ✓）*/
#include "task_sd.h"                    /* ★ R1 集成：SD 交接接口/乒乓缓冲/task_SD 本体 */
#include "MyRTC.h" 
#include "boot_flag.h"             /* ★ 升级请求标志（★ 和 Boot\boot_flag.h 逐字节一致 ✓）*/
#include "Crc16.h"
/*==============================================================================
 *  数据记录仪 —— 应用层（任务总装）
 *      数据流： task_ACQ --Record_t--> [xQueueRec] --> task_PROC
 *              --拼CSV行+乒乓缓冲(SD_BufAcquire/SD_LogEnqueue)--> task_SD --> FatFs --> SD 卡
 *      任务表： ACQ 6/256/100ms   PROC 2/256/队列   KEY 2/128/20ms   DISP 1/256/200ms   SD 1/256/队列
 *  ★ 每个【为什么】（优先级/栈大小/快照为何要进临界区/踩过哪些坑）都在：
 *      stm32\学习笔记\30-项目设计说明-为什么这么写.md
 *============================================================================*/
/******************************************************************************
 *  任务配置（静态方式：句柄 + 栈数组 + TCB 三样都要自己定义）
 ******************************************************************************/

/* start_task：装配工 —— 建完其它任务就自删（理由见 30 号文档 §一） */
/* START_TASK 任务配置 */
#define START_TASK_PRIO         1
#define START_TASK_STACK_SIZE   128
TaskHandle_t    start_task_handler;
void start_task(void *pvParameters);

/* task_ACQ：100ms 采集 → 组装记录 → 入队
 * ★ 本任务【绝不阻塞】：入队超时给 0；周期靠 vTaskDelayUntil 保证不漂移 */
/* TASK_ACQ任务配置：100ms节拍器*/
#define TASK_ACQ_PRIO        6
#define TASK_ACQ_STACK_SIZE        256
TaskHandle_t    task_acq_handler;
void task_ACQ(void *pvParameters);

/* task_PROC：出队 → 拼 CSV 行 → 填乒乓缓冲交 task_SD（Y8+⑤ 改造）
 * ★ 仍是【唯一】串口打印者；栈 256 字的依据是 Serial_Printf 调用链 352 字节 */
/* TASK_PROC 任务配置：出队+拼批落卡+串口回显*/
#define TASK_PROC_PRIO              2
#define TASK_PROC_STACK_SIZE        256
TaskHandle_t    task_proc_handler;
void task_PROC(void *pvParameters);

/* task_KEY：20ms 扫键（扫描周期本身就是消抖基准）
 * ★ 本任务【禁止 Serial_Printf】：栈 128 字只剩约 96 字节余量，打印甩给 PROC */
#define TASK_KEY_PRIO               2
#define TASK_KEY_STACK_SIZE         128
TaskHandle_t    task_key_handler;
void task_KEY(void *pvParameters);

/* task_DISP：OLED 显示（200ms 刷一次）
 * ★ 本任务是 OLED 的【唯一所有者】，别的任务一律不许碰屏 */
#define TASK_DISP_PRIO              1
#define TASK_DISP_STACK_SIZE        256
TaskHandle_t    task_disp_handler;
void task_DISP(void *pvParameters);

/* task_SD：SD 卡落盘任务（乒乓缓冲的接收方，task_sd.c 实现本体）
 * ★ 优先级/栈大小用 task_sd.h 的定版宏（1 / 256 字），别在这里手抄数字 ——
 *   两处漂移是事故温床（改了宏忘了改这里，或反之） */
TaskHandle_t    task_sd_handler;
void task_SD(void *pvParameters);

/* task_SENSOR：DS18B20 温度，1 秒一拍
 * ★ 位操作协议 → 放最低优先级，且与 ACQ 的 100ms 网格【错开相位】（30 号 §16.2）
 * ★ 栈 128 字够：只做位操作 + 一次赋值，【不打印】 */
#define TASK_SENSOR_PRIO            1
#define TASK_SENSOR_STACK_SIZE      128
TaskHandle_t    task_sensor_handler;
void task_sensor(void *pvParameters);


/* Record_t：一条记录，定长 40 字节（队列是值拷贝，定长最省事）
 * ★ 布局不要随便改；改字段要同步改 SD 卡格式 和 下面的编译期断言 */
typedef struct {
    uint32_t seq;          /* 序号：从 1 开始，只有 task_acq 会自增 */
    uint32_t tick;         /* 时间戳：产生这一条时的 tick（阶段3 换成 RTC 时间）*/
	uint32_t press;     /* ×10 Pa（BMP580；未接时恒 0）*/
	uint32_t rtc;       /* 秒（阶段3 填；现在恒 0）*/
    int16_t  ax, ay, az;   /* 加速度原值（阶段2 换 MPU6050） */
    int16_t  gx, gy, gz;   /* 角速度原值 */
	int16_t  temp_ds;   /* ★ ×100 °C（DS18B20）*/
    int16_t  temp_chip;    /* 芯片内部温度 ×100 */
	uint16_t humi;      /* ×10 %RH（DHT22；未接时约定 0xFFFF）*/
    uint16_t light;        /* 光敏：PB13 是数字量，只能判亮/暗，不是 ADC */
	uint16_t vbat; 
	uint16_t crc;       /* CRC16（阶段3 填；现在恒 0）*/
} Record_t;

/* 尺寸断言：布局一变就【编译期】报错（不占 Flash/RAM） */
typedef char assert_record_size[( sizeof(Record_t) == 40 ) ? 1 : -1];


/* 队列长度：8 是初值、不是设计值 —— 阶段2 写完 SD 卡后按实测 QMAX 重新定容
 * （当前实测 QMAX=1；长度还有【吸收抖动】的作用，别缩到 1~2） */
#define REC_QUEUE_LEN     8

/* 队列句柄：本质是指针；创建失败返回 NULL，必须检查 */
QueueHandle_t xQueueRec; 

/* g_stat：跨任务统计。★ 整体拷贝快照(snap = g_stat)必须进临界区；
 * q_max_depth 在【ACQ 入队成功后】测（在 PROC 出队后测会少 1 条） */
typedef struct {
    uint32_t acq_count;    /* 采集总条数 */
    uint32_t proc_count;   /* 处理总条数 */
    uint32_t queue_full;   /* 队列满的次数 ★ 只要 >0 就说明缓冲不够、真丢过数据 */
    uint32_t q_max_depth;  /* 队列历史最大积压 ★ 决定队列该开多大的依据 */
} Stat_t;
Stat_t g_stat;
volatile uint16_t g_i2c_us_last = 0;
volatile uint16_t g_i2c_us_max  = 0;
/* ★★ I2C 总线互斥量（2026-10-09 加）—— MPU6050(ACQ,prio6) 与 BMP280(sensor,prio1)
 *    共用同一条软件 I2C（PA2/PA3）✗ 必须串行化 ✓
 *    ★ 必须用 Mutex ✗ 不能用二值信号量 ✓：
 *      Mutex 有【优先级继承】✓ —— ACQ(prio6) 等 sensor(prio1) 手里的锁时，
 *      sensor 会被临时提升到 6 ✓ 尽快跑完还锁 ✓
 *      （二值信号量没有继承 ✗ → 优先级反转 ✗ → ACQ 的 100ms 节拍会被拖爆 ✓）*/
static SemaphoreHandle_t s_i2c_lock = NULL;

/* DS18B20 温度 ×100 °C（例 2400 = 24.00C）；读失败 = DS18B20_INVALID(-32768) */
volatile int16_t g_temp_x100 = DS18B20_INVALID;

/* ★★ BMP280 气压 ×10 Pa —— 契约单位（`30` 号 §15.2 ✓，1013.25hPa = 1013250 ✓）
 *    读失败 / 未接 → 0 ✓（符合"未接字段填约定值"的规矩 ✓）
 *    ★ 为什么是 volatile：task_sensor(prio1) 写、task_PROC(prio2) 读 ✓
 *      —— 跨任务可见性必须靠它 ✓（不加会被编译器缓存进寄存器 ✗）*/
volatile int32_t g_press_x10pa = 0;
/* BMP280 自己的温度 ×100 °C —— ★★ 目前【只写不读】✗（2026-10-11 独立审查 F18 指出）
 *   ★ 处置：保留并标注"预留"✗ —— 而不是删掉 ✓
 *     理由：它是个【有用的交叉校验量】✗（BMP280 的温度 vs DS18B20 的温度 ✓
 *           本次实测两者差 0.87°C ✓ 在 ±1°C 内 ✓✓）
 *   ★ 记录格式里【没有它的位置】✗（temp_chip 那个字段已经被 MPU6050 的片内温度占了 ✓）
 *     → 所以只能先读出来放着 ✓ 将来做"传感器互检/故障诊断"时直接用 ✓✓
 *   ★ 当前唯一的实际用途：配合 g_press_x10pa 一起写 ✓ 便于以后排查"气压读数异常"时
 *     判断是"传感器坏了"还是"温度补偿出错"（气压公式依赖温度 ✓）✓ */
volatile int16_t g_bmp_temp_x100 = 0;

/* ★★ DHT22 湿度 ×10 %RH（2026-10-11 加 ✓）—— 例 653 = 65.3% RH
 *   · 读失败 / 未接 → 【0xFFFF】✗ ✓（★ 记录格式的"无效"约定 ✓）
 *   · ★ 为什么是 volatile：task_sensor(prio1) 写、task_ACQ(prio6) 读 ✓
 *   ★★ 记录里 humi 字段【早就留好了位置】✗（16 列里的第 13 列 ✓）
 *      以前恒 0xFFFF ✗ → 现在开始填真值 ✓✓ */
volatile uint16_t g_humi_x10 = 0xFFFF;

/* ★★ DHT22 自己的温度 ×10 °C —— ★【只写不读】✗ 和 g_bmp_temp_x100 同性质
 *   ★ 处置：保留并标注"预留"✗ 而不是删掉 ✓
 *   · 理由：它是个【交叉校验量】✗ —— DHT22 温度 vs DS18B20(DHT22 自身会发热 ✓)
 *     ★★ 用途：将来做"传感器互检"时，它能帮着分辨
 *       "是 DS18B20 坏了"还是"环境真的变了" ✓✓
 *   ★ 记录格式里【没有它的位置】✗（temp_ds 是 DS18B20 的、temp_chip 是 MPU 的 ✓）*/
volatile int16_t g_dht_temp_x10 = 0;

/* ---- 按键相关的两个任务间信息通道（状态类/事件类分流，执行表 T1.3 设计） ---- */

/* g_pause 必须 volatile：ACQ 反复读它，不加会被编译器缓存进寄存器 */
/* KEY1 → 暂停标志：这是"状态"不是"事件"，用 volatile 标志让 ACQ 每拍自己读，
 * 不发队列不发通知——状态随时可查，不占任何内核对象 */
volatile uint8_t g_pause = 0;       /* 0=采集运行  1=采集暂停 */

/* KEY2 → 打印统计：这是"一次性事件"，用任务通知（比队列/信号量都轻）。
 * PROC 收到后自己打印（KEY 任务栈小，打印不在它那儿做） */
/* 通知配对：eSetBits 发 <-> xTaskNotifyWait 收（不能配 ulTaskNotifyTake） */
#define EVENTBIT_KEY2       (1UL << 0)

/* 诊断用缓冲区：★ 必须放全局，不能放任务栈上
 *   · 任务状态表格式化后 200+ 字节；TaskStatus_t 数组 8×约24 字节
 *   · PROC 的栈只有 1KB，这些放栈上就爆了 */
static char         s_task_list_buf[512];
static TaskStatus_t s_task_status[8]; 
/******************************************************************************
 *  FreeRTOS 例程入口
 ******************************************************************************/
 
 
/* freertos_demo：内核对象必须在 vTaskStartScheduler() 之前创建，NULL 要查 */
void freertos_demo(void)
{
    xQueueRec = xQueueCreate(REC_QUEUE_LEN, sizeof(Record_t));
	if (xQueueRec == NULL)
	{
		Serial_Printf("!!! xQueueCreate FAILED (out of heap) !!!\r\n");
		return; 
	}
		/* ★ 开机预读一次 BMP280（2026-10-09）
	 *   为什么要这里读：task_sensor 有 500ms 相位错开 + 750ms DS18B20 转换 ✓
	 *   → 它第一次读要等到 ~1.25 秒之后 ✗ → 前面 ~12 条记录 press=0 ✗
	 *   在这里先读一次 → ACQ 第一条记录就有真实气压 ✓✓
	 *   ★ 此时【调度器还没启动】✓ 没有并发 → 不需要拿互斥量 ✓✓ */
	{
		int32_t p_x10, t_c100;
		if (BMP280_Read(&p_x10, &t_c100) == 0u)
		{
			g_press_x10pa   = p_x10;
			g_bmp_temp_x100 = (int16_t)t_c100;
		}
	}
	/* ★ 新增：I2C 互斥量（同样必须在 vTaskStartScheduler() 之前 ✓）*/
	s_i2c_lock = xSemaphoreCreateMutex();
	
	if (s_i2c_lock == NULL)
	{
		Serial_Printf("!!! xSemaphoreCreateMutex FAILED (out of heap) !!!\r\n");
		return;
	}

	Serial_Printf("queue ok: item=%d bytes, free heap=%u\r\n",
				  (int)sizeof(Record_t), (unsigned)xPortGetFreeHeapSize());
;
	
	/* 只创建"装配工"这一个任务，其余任务由它统一建（正点原子惯例，见 start_task） */
	xTaskCreate((TaskFunction_t 			   )   start_task,
				(char *          			   )   "start_task",
				(configSTACK_DEPTH_TYPE        )   START_TASK_STACK_SIZE,
				(void *                        )   NULL,
				(UBaseType_t                   )   START_TASK_PRIO,
				(TaskHandle_t *                )   &start_task_handler);

	/* ★ R1 集成（第 6 课）：SD 交接对象（两条队列 + 两块缓冲的初始所有权）
	 *   必须在 vTaskStartScheduler() 之前就位。为什么这么严：
	 *   task_SD 优先级 1 低于 PROC 的 2，"大概率"后跑——但"大概率"不是保证；
	 *   若 PROC 先跑、队列还是 NULL，第一次 SD_BufAcquire 就是空指针解引用。
	 *   （task_SD 本体另有兜底：队列没就绪它原地等，见 task_sd.c 任务开头） */
	task_sd_Init();

	vTaskStartScheduler();
	}

/* 启动任务：创建其它任务，然后删掉自己 */
/* 注意：下面的创建代码包在临界区里，失败时的打印【不能】放在临界区里
 * （临界区里放打印 = 屏蔽中断约 3.5ms，见 30 号文档 §四） */
void start_task(void *pvParameters)
{
	/* 创建失败用位图攒着，出了临界区再打印（临界区里不能放打印） */
	uint32_t fail_mask = 0;

	taskENTER_CRITICAL();

	/* 返回值必须判（评审确认的 bug：heap 不够时任务会静默消失，掩盖 RAM 不足症状） */
	if (xTaskCreate(  (TaskFunction_t )   task_ACQ,
                   (char *         )   "ACQ",
                   (uint32_t       )   TASK_ACQ_STACK_SIZE,
                   (void *         )   NULL,
                   (UBaseType_t    )   TASK_ACQ_PRIO,
                   (TaskHandle_t * )   &task_acq_handler) != pdPASS)
	{
		fail_mask |= (1u << 0);		/* bit0 = ACQ 没建起来 */
	}
	if (xTaskCreate(  (TaskFunction_t )   task_PROC,
                   (char *         )   "PROC",
                   (uint32_t       )   TASK_PROC_STACK_SIZE,
                   (void *         )   NULL,
                   (UBaseType_t    )   TASK_PROC_PRIO,
                   (TaskHandle_t * )   &task_proc_handler) != pdPASS)
	{
		fail_mask |= (1u << 1);		/* bit1 = PROC 没建起来 */
	}
	if (xTaskCreate(  (TaskFunction_t )   task_KEY,
                   (char *         )   "KEY",
                   (uint32_t       )   TASK_KEY_STACK_SIZE,
                   (void *         )   NULL,
                   (UBaseType_t    )   TASK_KEY_PRIO,
                   (TaskHandle_t * )   &task_key_handler) != pdPASS)
	{
		fail_mask |= (1u << 2);		/* bit2 = KEY 没建起来 */
	}
	/* ★ task_DISP 也在这里创建（OLED 唯一所有者；prio 1 最低，不影响采集） */
	if (xTaskCreate(  (TaskFunction_t )   task_DISP,
                   (char *         )   "DISP",
                   (uint32_t       )   TASK_DISP_STACK_SIZE,
                   (void *         )   NULL,
                   (UBaseType_t    )   TASK_DISP_PRIO,
                   (TaskHandle_t * )   &task_disp_handler) != pdPASS)
	{
		fail_mask |= (1u << 3);		/* bit3 = DISP 没建起来 */
	}

	if (xTaskCreate(  (TaskFunction_t )   task_sensor,
                   (char *         )   "sensor",
                   (uint32_t       )   TASK_SENSOR_STACK_SIZE,
                   (void *         )   NULL,
                   (UBaseType_t    )   TASK_SENSOR_PRIO,
                   (TaskHandle_t * )   &task_sensor_handler) != pdPASS)
	{
		fail_mask |= (1u << 4);		/* bit4 = sensor 没建起来 */
	}

	/* ★ R1 集成（第 6 课）：task_SD 落盘任务 —— prio/栈取 task_sd.h 定版值
	 *   （1/256字=1KB）。prio 1 与 sensor 同级：SD 的慢（busy 最坏 250ms）
	 *   被隔离在最低优先级，ACQ→PROC 采集链永远先跑（task_sd.h 顶注释）。
	 *   建任务前 task_sd_Init() 已在 freertos_demo() 里建好队列，收缓冲
	 *   不会撞空指针（task_SD 本体仍保留"对象没就绪就等"的双保险） */
	if (xTaskCreate(  (TaskFunction_t )   task_SD,
                   (char *         )   "SD",
                   (uint32_t       )   TASK_SD_STACK_SIZE,
                   (void *         )   NULL,
                   (UBaseType_t    )   TASK_SD_PRIO,
                   (TaskHandle_t * )   &task_sd_handler) != pdPASS)
	{
		fail_mask |= (1u << 5);		/* bit5 = SD 没建起来 */
	}
	taskEXIT_CRITICAL();

	/* ★ 顺序必须是：出临界区 -> 打印 -> 再自删。临界区里不能放耗时打印；
	 *   自删之后代码不保证还能执行（详解见 30 号文档 §四） */
	if (fail_mask != 0)
	{
		Serial_Printf("!!! TASK CREATE FAILED: mask=0x%X (out of heap) !!!\r\n", (unsigned)fail_mask);
	}

	vTaskDelete(start_task_handler);
}


/* task_ACQ 四个要点：
 * ① xLastWakeTime 在 while【外面】初始化  ② vTaskDelayUntil 放在循环【开头】
 * ③ if (g_pause) continue 必须在【延时之后】 ④ xQueueSend 超时给 0（绝不阻塞） */
void task_ACQ(void *pvParameters)//100ms定时器
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	static uint32_t seq = 0; 
	Record_t rec;
	int16_t  ax, ay, az, gx, gy, gz, tchip;
	uint32_t t0, dt;
	while(1)
	{
	 vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100)); //毫秒翻译成tick数，毕竟api用的单位是tick

	/* KEY1 暂停开关：暂停时本拍不采样、seq 冻结；延时已在开头执行，节拍照走 */
	 if (g_pause)
	 {
		 continue;
	 }
	 t0 = DWT->CYCCNT;
	  /* ★ 读 MPU6050 前拿 I2C 锁 ✓（拿不到就一直等 ✓ —— 等待期间内核会跑别的任务 ✓）
	  *   ★ give 必须在【所有路径】上：下面 take 与 give 之间没有 return/continue ✓
	  *     （★ 前面那个 `if (g_pause) continue;` 在 take 之前 ✓ 不会漏锁 ✓）*/
	 xSemaphoreTake(s_i2c_lock, portMAX_DELAY);
	 MPU6050_GetData(&ax, &ay, &az, &gx, &gy, &gz, &tchip);
	 xSemaphoreGive(s_i2c_lock);
	 dt = (DWT->CYCCNT - t0) / 72u;
	  g_i2c_us_last = (dt > 60000u) ? 60000u : (uint16_t)dt;
	  if (g_i2c_us_last > g_i2c_us_max)
		  g_i2c_us_max = g_i2c_us_last;
	 rec.seq=++seq;
	 rec.tick=xTaskGetTickCount();
	 rec.press = (uint32_t)g_press_x10pa;      /* ★ 真实气压 ×10 Pa ✓（读失败=0 ✓）*/
	 rec.rtc=MyRTC_GetUnix();
	 rec.ax=ax;
	 rec.ay=ay;
	 rec.az=az;
	 rec.gx=gx;
	 rec.gy=gy;
     rec.gz=gz;
	 rec.temp_ds=g_temp_x100;
	 rec.temp_chip =MPU6050_TempX100(tchip);
	 rec.humi      = g_humi_x10;               /* ★ 2026-10-11：DHT22 真值 ✓
	                                            *   （未接 / 读失败 = 0xFFFF ✓ 契约见 30 号 §15.2 ✓）*/
	 rec.light= AD_Value[0];
	 rec.vbat      = AD_Value[1];              /* ★★ 2026-10-11：分压已装 ✓（20k+10k → 比 3 ✓）
	                                            *
	                                            *  ★ 来源：PB0 = ADC1_IN8 ✓ 由 DMA 后台连续刷新 ✓
	                                            *  ★★ 存的是【原始值 0~4095】✗ 不是电压 ✓（契约见 30 号 §15.2 行 429 ✓）
	                                            *     理由：原始值不丢信息 ✓ 换供电/换分压后【旧数据仍可用】✓✓
	                                            *  ★★★ 换算（PC 侧做 ✓）：
	                                            *     V = 原始值 × VDDA / 4095 × 3
	                                            *     （★ VDDA 想更准可以用 AD_ReadVrefint() 反推 ✓）
	                                            *  ★ 当前接的是【板子 5V 排针】= USB 的 5V ✗
	                                            *     预期原始值 ≈ 2068 ✓（= 5.0V ✓）
	                                            *     ★ 注意：5V 和 VDDA(3.3V) 不同源 → 能测出变化 ✓✓ */
	 rec.crc       = 0;
		
	
	 if (xQueueSend(xQueueRec, &rec, 0) != pdPASS)
	 {
		taskENTER_CRITICAL();
        g_stat.queue_full++;
        taskEXIT_CRITICAL(); 
	 }
	 else
	 {
		 UBaseType_t depth; 
	     taskENTER_CRITICAL();
         g_stat.acq_count++;
         taskEXIT_CRITICAL();

		 depth = uxQueueMessagesWaiting(xQueueRec);
		 if (depth > g_stat.q_max_depth)
			 g_stat.q_max_depth = depth;
	 }		 
	 
 }
	 
}
	
/* 打一次【任务状态表 + 每个任务的栈水位】（阶段1 验收项 4/5）。
 * 只在开机后调一次：任务表 200+ 字节，周期性打会把串口带宽吃光 */
static void print_task_diagnostics(void)
{
	UBaseType_t n, i;
	uint32_t    total_run_time = 0;

	/* ① 任务状态表：★ 用 vTaskListTasks(buf, len) 显式传长度。
	 *    不要用 vTaskList(buf) 那个宏 —— 它传的是 configSTATS_BUFFER_MAX_LENGTH
	 *    (=0xFFFF)，函数根本不知道缓冲区只有 512 字节，输出超了就是踩内存 */
	vTaskListTasks(s_task_list_buf, sizeof(s_task_list_buf));
	Serial_SendString("---- Task list ----\r\n");
	/* ★ 用 SendString 直接发：Serial_Printf 的缓冲只有 200B，
	 *   任务表比它长，会被 vsnprintf 截断（安全但内容不全） */
	Serial_SendString(s_task_list_buf);

	/* ② 栈水位：usStackHighWaterMark = 剩余余量（单位【字】），越小越危险 */
	n = uxTaskGetSystemState(s_task_status, 8, &total_run_time);
	Serial_SendString("---- Stack free (words) ----\r\n");
	for (i = 0; i < n; i++)
	{
		Serial_Printf("%-10s %u\r\n",
		              s_task_status[i].pcTaskName,
		              (unsigned)s_task_status[i].usStackHighWaterMark);
	}
}


/* ---- PROC → task_SD 拼批状态（Y8+⑤，第 6 课改造）----
 * s_cur / s_pos：手里正在填的乒乓缓冲 + 已填字节数（s_cur==NULL = 手里没缓冲）
 *   ★ 必须 static 全局：批与批之间要记住"填到哪了"，放栈上活不过一次循环
 * s_drop：因拿不到缓冲/交不出去而丢弃的记录条数 —— STAT 行 drop= 的数据源 */
static uint8_t *s_cur   = NULL;
static uint16_t s_pos   = 0;
static uint32_t s_drop  = 0;

/* 串口 REC 回显开关：调试期 1（边跑边看数据长什么样），正式跑改 0 ——
 * 正式数据走卡不走串口，回显纯属人眼验收，还占带宽（最坏 220B×10Hz） */
#define PROC_SERIAL_ECHO        1

/* task_PROC 五个要点（Y8+⑤ 改造后）：
 * ① receive 超时 500ms（用 portMAX_DELAY 的话通知唤不醒它）
 * ② 收到记录 → 拼 CSV 行 → 填 512B 乒乓缓冲 → 放不下先交批换新缓冲再拷
 * ③ 边界铁律：s_pos+行长+2 > SD_BUF_SIZE 就交批，绝不假设"一行一定放得下"
 * ④ 500ms 无数据 → 尾批落盘（len 单独传，不满 512B 也能写对）
 * ⑤ xTaskNotifyWait 按位收；n 只在【收到数据】时 ++ */
void task_PROC(void *pvParameters)
{
	Record_t rec;
	static uint32_t n = 0;
	static uint8_t  diag_done = 0;	/* 诊断只打一次 */

	Stat_t snap;
	uint32_t notify_val = 0;
    while (1)
    {
/* 超时 500ms 而不是 portMAX_DELAY：否则暂停时 KEY2 的通知永远轮不到看 */
if (xQueueReceive(xQueueRec, &rec, pdMS_TO_TICKS(500)) == pdPASS)
{
	char row[160];      /* CSV 一行最坏 123 字节：4×10 位(uint32 最坏值) + 8×6
	                       ("−32768") + 4×5 ("65535") + 15 个逗号；160 留足余量 */
	int  rowlen;

		 taskENTER_CRITICAL();
         g_stat.proc_count++;
         taskEXIT_CRITICAL();

	/* ★ 只在真实收到记录时 ++：否则空闲空转也计数，STAT 会漂移 */
		 n++;

	/* ---- 第①步：拼 CSV 行。列序必须与 task_sd.c 的 csv_hdr【一字不差】----
	 * seq,tick,press,rtc,ax,ay,az,gx,gy,gz,temp_ds,temp_chip,humi,light,vbat,crc
	 * ★ 改列序/列名 = 必须同步改 task_sd.c 的 csv_hdr，两边漂移 → PC 端解析全错 */
	
	/* ---- 第①步：拼 CSV 行（★ 先只拼【前 15 列】✓）----
	 * seq,tick,press,rtc,ax,ay,az,gx,gy,gz,temp_ds,temp_chip,humi,light,vbat,crc
	 * ★ 改列序/列名 = 必须同步改 task_sd.c 的 csv_hdr，两边漂移 → PC 端解析全错
	 *
	 * ★★ 2026-10-10：crc 不再恒 0 ✗ —— 改成对【前 15 列的文本】算 CRC16 ✓
	 *   为什么对文本算、而不是对 40 字节结构体算 ✗：
	 *     卡上存的就是【文本】✗ —— 校验必须覆盖【实际存储的东西】✓
	 *     对结构体算的话，PC 侧无法从 CSV 复现 → 校验形同虚设 ✓
	 *   ★ 契约（PC 侧照此实现 ✓，见 check_csv_crc.py）：
	 *     · 取整行，去掉末尾 \r\n ✓
	 *     · 取【最后一个逗号之前】的全部字符（= 前 15 列 ✓）
	 *     · 算 CRC-16/KERMIT（poly 0x8408 / init 0 / 反射 / 无输出异或 ✓）
	 *     · 与最后一个逗号之后的十进制值比较 ✓
	 *   ★ 此处 row 里【还没有 \r\n】✓（\r\n 是后面拷进缓冲时才加的 ✓）
	 *     —— 正合契约 ✓ 不需要额外处理 ✓ */
	rowlen = sprintf(row, "%lu,%lu,%lu,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%u",
	                  (unsigned long)rec.seq,  (unsigned long)rec.tick,
	                  (unsigned long)rec.press,(unsigned long)rec.rtc,
	                  rec.ax, rec.ay, rec.az,
	                  rec.gx, rec.gy, rec.gz,
	                  rec.temp_ds, rec.temp_chip,
	                  (unsigned)rec.humi, (unsigned)rec.light,
	                  (unsigned)rec.vbat);

	/* ★ 对【上面这段文本】算 CRC ✓（覆盖的正是"卡上那一行的前 15 列"✓）*/
	rec.crc = Crc16_Kermit((const uint8_t *)row, (uint16_t)rowlen);

	/* ★ 再把 crc 作为第 16 列追加 ✓（串口回显也跟着用它 ✓）*/
	rowlen += sprintf(row + rowlen, ",%u", (unsigned)rec.crc);

	/* ---- 第②步：保证手里有缓冲（数据落卡 > 一切，串口回显排后面）----
	 * 拿不到 = 两块都在 task_SD 手里写卡：丢本条保节拍，绝不阻塞等待
	 * （PROC 一阻塞就顶住 ACQ 的采集队列，违反架构前提） */
	if (s_cur == NULL)
	{
		if (SD_BufAcquire(&s_cur) != pdPASS)
		{
			s_drop++;
		}
	}

	/* ---- 第③步（Y8 核心）：放不下 → 先交批 → 再换新缓冲 ----
	 * +2 是给 "\r\n" 的。s_pos>0 才进分支：手里没攒任何半行时无可交；
	 * "单行就超 512B"的理论情形交给第④步的兜底闸拦（宁丢不越界） */
	if ((s_cur != NULL) && (s_pos > 0) &&
	    ((uint32_t)s_pos + (uint32_t)rowlen + 2u > SD_BUF_SIZE))
	{
		/* 交失败也不用操心缓冲去哪了：SD_LogEnqueue 内部要么把它还进
		 * 空闲池、要么已置 g_sd_error 报"所有权被破坏"——两种结局它
		 * 都不再属于我们，s_cur 清空重拿即可（不泄漏、不重复持有） */
		if (SD_LogEnqueue(s_cur, s_pos) != pdPASS)
		{
			s_drop++;
		}
		s_cur = NULL;
		s_pos = 0;

		if (SD_BufAcquire(&s_cur) != pdPASS)    /* 新缓冲拿不到：丢本条 */
		{
			s_drop++;
		}
	}

	/* ---- 第④步：拷行 + "\r\n"，推进写指针 ----
	 * 兜底闸：s_pos==0 还放不下 = 单行超 512B（row[160] 下不可能，
	 * 但这闸保证将来有人把 row 改大时也绝不写越界——宁丢一条不溢出一字节） */
	if ((s_cur != NULL) &&
	    ((uint32_t)s_pos + (uint32_t)rowlen + 2u <= SD_BUF_SIZE))
	{
		memcpy(&s_cur[s_pos], row, (size_t)rowlen);
		s_pos += (uint16_t)rowlen;
		s_cur[s_pos++] = '\r';
		s_cur[s_pos++] = '\n';
	}
	else if (s_cur != NULL)
	{
		s_drop++;       /* 走到这 = 单行放不下（见上）：丢本条 */
	}

#if PROC_SERIAL_ECHO
	/* ---- 串口 REC 回显（调试用；正式跑把 PROC_SERIAL_ECHO 改 0）----
	 * ★ 放在开关里：它和数据落卡互不相干，正式期只占带宽不产生价值 */
	Serial_Printf("REC,seq<%lu>,tick<%lu>,press<%lu>,rtc<%lu>,"
	              "ax<%d>,ay<%d>,az<%d>,gx<%d>,gy<%d>,gz<%d>,"
	              "temp_ds<%d>,temp_chip<%d>,humi<%u>,light<%u>,vbat<%u>,crc<%u>\r\n",
	              (unsigned long)rec.seq,  (unsigned long)rec.tick,
	              (unsigned long)rec.press,(unsigned long)rec.rtc,
	              rec.ax, rec.ay, rec.az,
	              rec.gx, rec.gy, rec.gz,
	              rec.temp_ds, rec.temp_chip,
	              rec.humi, rec.light, rec.vbat, rec.crc);
#endif
}
else
{
	/* ---- 500ms 没数据：尾批落盘（要点④的另一半）----
	 * 不满 512B 也交（len 单独传，task_SD 按实际长度 f_write）——
	 * 否则暂停采集后最后几条会一直捂在手里等下一批，违反秒级落盘口径 */
	if ((s_cur != NULL) && (s_pos > 0))
	{
		if (SD_LogEnqueue(s_cur, s_pos) != pdPASS)
		{
			s_drop++;
		}
		/* 只在"确实交过手"之后才清空：交成功 = 所有权已转移给 task_SD；
		 * 交失败 = 缓冲已被接口回收进空闲池（或已置错）——两种结局都不
		 * 再属于我们。而 s_pos==0（申请了还没写）时绝不能清空：
		 * 拿了不还又不用 = 512B 缓冲永久泄漏，整条链慢慢停摆 */
		s_cur = NULL;
		s_pos = 0;
	}
}

	/* KEY2 事件：与发送端 eSetBits 配对，按位收、按位清（0 超时=不阻塞） */
	if (xTaskNotifyWait(0, EVENTBIT_KEY2, &notify_val, 0) == pdPASS)
	{
		if (notify_val & EVENTBIT_KEY2)
		{
			taskENTER_CRITICAL();
			snap = g_stat;
			taskEXIT_CRITICAL();

			/* ★ sd_err / sd_last：SD 故障与原因码（0 = 健康）。见下方 STAT 的说明
			 * ★★ 2026-10-09 新增 heap / heapmin（补 T2.6④ 的空白 ✗）：
			 *   原来 STAT 只有 SD 状态，【没有堆的信息】✗ ->
			 *   "heap 最低水位 > 2KB" 这条判据【根本没有采集点】✗✓
			 *   · heap    = 当前空闲堆（临时对象会让它抖动 ✓ 正常 ✓）
			 *   · heapmin = ★【历史最低水位】—— ★ 判据看【它】，不是看 heap ✗
			 *       heapmin 下降后【不再回升】= 内存泄漏 ✗✓（慢性故障唯一可靠的信号 ✓）
			 *   本工程 12KB heap：开机实测 heap=11856 ✓（已用 432B ✓）
			 *   长稳时 heapmin 保持稳定 = 无泄漏 ✓✓ */
			Serial_Printf("STAT(key) acq=%lu proc=%lu full=%lu qmax=%lu drop=%lu i2c=%u i2c_max=%u sd_err=%u sd_last=%u heap=%u heapmin=%u\r\n",
			          (unsigned long)snap.acq_count,
                      (unsigned long)snap.proc_count,
                      (unsigned long)snap.queue_full,
                      (unsigned long)snap.q_max_depth,
                      (unsigned long)s_drop,
                      g_i2c_us_last, g_i2c_us_max,
                      (unsigned)g_sd_error, (unsigned)g_sd_last_err,
                      (unsigned)xPortGetFreeHeapSize(),
                      (unsigned)xPortGetMinimumEverFreeHeapSize());
		}
	}

	/* 开机 3 秒后打一次诊断（只打一次）：任务状态表 + 栈水位 */
	if (!diag_done && xTaskGetTickCount() > pdMS_TO_TICKS(3000))
	{
		diag_done = 1;
		print_task_diagnostics();
	}
	if (n >= 100)
	{
		n=0;
		taskENTER_CRITICAL();
		snap = g_stat;
		taskEXIT_CRITICAL();
		
		/* ★ sd_err / sd_last：把 SD 侧状态搬上串口 —— 只为让阶段 2 验收【可自证】★
		 *   动机：task_sd 的"挂载成功"目前只有 OLED 能看见，串口看不见 ✗ ->
		 *         验收时无法判断"卡挂上了没有"、"是不是一直在重试"。
		 *   读法：
		 *     sd_err=0          -> 卡在挂载态（健康 ✓）；配合 drop=0 即"写卡不阻塞采集" ✓
		 *     sd_err=1          -> 故障（未挂载 / 连续 3 批写失败已卸载），原因看 sd_last
		 *     sd_last=0         -> 无错误史
		 *     sd_last=3(FR_NOT_READY) -> 没插卡 / 接触不良（最常见）
		 *   ⚠️ sd_err=0 只说明【任务侧认为卡在】，最终仍以"拔卡插 PC 能读到完整
		 *      YYMMDDNN.CSV"为准（T2.6①）—— 这个字段是给现场快速判读用的，不是替代物
		 *      （★ 2026-10-11 更新：文件名已从 LOGxxxx.CSV 改成 YYMMDDNN.CSV ✓）
		 * ★★ heap / heapmin（2026-10-09 加）：判据看【heapmin】不是 heap ✗ ——
		 *    heapmin 下降后不再回升 = 内存泄漏 ✗✓；完整说明见上面 STAT(key) 那段 ✓ */
		Serial_Printf("STAT acq=%lu proc=%lu full=%lu qmax=%lu drop=%lu i2c=%u i2c_max=%u sd_err=%u sd_last=%u heap=%u heapmin=%u\r\n",
		          (unsigned long)snap.acq_count,
                  (unsigned long)snap.proc_count,
                  (unsigned long)snap.queue_full,
                  (unsigned long)snap.q_max_depth,
                  (unsigned long)s_drop,
				  g_i2c_us_last, g_i2c_us_max,
				  (unsigned)g_sd_error, (unsigned)g_sd_last_err,
				  (unsigned)xPortGetFreeHeapSize(),
				  (unsigned)xPortGetMinimumEverFreeHeapSize());
	}
	
    }
}


/* task_KEY：KEY1=状态->标志位；KEY2=事件->任务通知。
 * 20ms 扫一次够用（按键抖动 <10ms），本任务不许打印 */
/* 按键任务：扫键 -> 状态改标志 / 事件发通知 -> vTaskDelay。★ 禁止打印 */
void task_KEY(void *pvParameters)
{
	for (;;)
	{
		uint8_t ev = Key_Scan();

		if (ev & KEY1_EVENT)                    /* 状态类：翻转暂停标志 */
		{
			g_pause = !g_pause;
			/* ★★★ 按键反馈（2026-10-13 加 ✓）
			 *   ★ 为什么可以在这里调 ✗：★ `Buzz_Request` 【只置一个标志位】✓
			 *     · ★★ 不打印 ✗ 不阻塞 ✗ 不碰 GPIO ✓
			 *     → ★★★ 【完全符合本任务"不许打印"的约束】✓✓
			 *   ★★★★ 实际发声由 task_DISP 的 Buzz_Poll() 负责 ✓ */
			Buzz_Request(BUZZ_EV_KEY);
		}

		if (ev & KEY11_EVENT)                   /* 事件类：通知 PROC 立刻打统计 */
		{
			xTaskNotify(task_proc_handler, EVENTBIT_KEY2, eSetBits);
			Buzz_Request(BUZZ_EV_KEY);          /* ★ 按键反馈 ✓（同上 ✓）*/
		}

		vTaskDelay(pdMS_TO_TICKS(20));
	}
}


/*==============================================================================
 *  ★★★★ 串口对时（2026-10-12 加 ✓）
 *        解决"没 VBAT 时，上电时间只能到【编译时刻】"✗
 *
 *  ★ 为什么需要它 ✗：
 *    · ★ 没装 VBAT ✗ → 【断电后 RTC 丢失】✓
 *    · ★★ 上电时用【编译时刻】重设 ✗ → 【不是"现在的真实时间"】✓
 *    · ★★★ 而且【备份域只要有电 ✗ 魔数就在 ✗
 *      连 Rebuild + 烧录都【不会】重设】**✗✓（用户实测确认 ✓）
 *    → ★★★★ 所以必须有【外部时间源】✗ —— ★【PC 就是现成的】✓✓
 *
 *  ★★ 协议（★ 越简单越好 ✗）：
 *    PC → MCU ：`T<10 位 Unix 秒>\r\n`     例：`T1791820005\r\n`
 *    MCU → PC ：`TIME OK -> 2026-10-12 23:46:45 (unix=...)`
 *               `TIME BAD (...)`      ← ★ 格式错 / 超范围 / RTC 没配 ✓
 *
 *  ★★★ 为什么【放进 task_DISP】✗：
 *    · ★ task_KEY 的注释明确写着【本任务不许打印】✓（★ 它优先级 2 ✗ 偏高 ✓）
 *    · ★★ 而 Serial_Printf 是【忙等发送】✗ 一行约 5ms ✓
 *      → 【在高优先级任务里打印会压住低优先级任务】✓
 *    · ★★★ task_DISP 优先级 1 ✗ 而且是【输出类任务】✓
 *      → 【回显确认信息是它的本职】✓✓
 *    · ★★★★ 代价：★ 延迟 ≤200ms ✗ —— 对"对时"【完全无所谓】✓✓
 *
 *  ★★★★ 线程安全 ✗：★ 环形缓冲是【单生产者（ISR）+ 单消费者（任务）】✓
 *    → 【天然无锁 ✗ 不需要临界区】✓✓（★ 详见 Serial.c 的说明 ✓）
 *============================================================================*/
#define TIMESYNC_LINE_MAX  24u

/*==============================================================================
 *  ★★★★ App 侧：请求升级（2026-10-13 加 ✓ 第二步）
 *
 *  ★ 做什么 ✗：
 *    · ★ ① 往 **`BKP_DR2`** 写【升级请求魔数】✓
 *    · ★★ ② **软复位** ✗ → 【CPU 从 `0x08000000` 重新起 ✗
 *      也就是【从 Bootloader 起】】✓✓
 *
 *  ★★ 为什么【标志能在复位后活下来】✗：
 *    · ★ `BKP` 【属于【备份域】✗ **软复位【不会】动它**】✗✓
 *    · ★★ 而【Bootloader 一上电就读它】✓✓
 *
 *  ★★★ 为什么【复位前【不用】关外设 / 停 SD / f_sync**✗：
 *    · ★ 软复位是【硬件级重启 ✗ 所有外设【自动回到初始】】✓
 *    · ★★ 数据每批已经 `f_sync` ✗ 最多丢【最后半秒】✓
 *    → ★★★ 【所以【直接复位就好 ✗ 不需要"优雅收尾"】】✓✓
 *
 *  ★★★★ 注意 ✗：★ 这里的寄存器和魔数 ✗
 *    必须与 `Boot\boot_flag.h` **完全一致** ✓✓
 *    （★ 我们用【两份【逐字节相同】的 `boot_flag.h`】✗
 *      ★★ 改的时候【两边一起改】✓）
 *============================================================================*/
static void App_RequestUpgrade(void)
{
	Serial_Printf("\r\nAPP: upgrade requested\r\n");

	/* ★ 等串口把话【发完】✗ 再往下走 ✓ */
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) { }

	/* ★★ 解锁备份域 ✗✓
	 *   ★ 为什么【这里还要自己做一遍**✗：
	 *     · ★ `MyRTC_Init()` 里【确实解锁过】✓
	 *     · ★★ 但【那是【启动时】的事 ✗
	 *       而且【备份域访问【可能被锁回去】】✓
	 *     → ★★★ 【这里重来一遍 ✗ 最稳妥 ✗ 而且【几乎不花时间】】✓✓
	 *   ★★★★ 做法【照抄 `MyRTC.c:28-29`】✓ */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	PWR_BackupAccessCmd(ENABLE);

	/* ★★★ 写标志 ✓ */
	BKP_WriteBackupRegister(BOOT_FLAG_DR, BOOT_FLAG_MAGIC);

	/* ★ 回读确认 ✗ —— ★【不信任"我写了"✗ 要看"它真的进去了"】✓✓
	 *   ★★ 这一步【很有用 ✗：★ 如果【备份域没解锁**✗
	 *     写会【静默失败】✗ 而【回读立刻暴露】✓✓ */
	Serial_Printf("APP: flag readback = 0x%04X (expect 0x%04X) -> resetting\r\n",
	              (unsigned)BKP_ReadBackupRegister(BOOT_FLAG_DR),
	              (unsigned)BOOT_FLAG_MAGIC);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) { }

	/* ★★★★ 软复位 ✗✓ */
	NVIC_SystemReset();

	/* ★ 正常情况下【不会到这里】✗ 写上更安全 ✓ */
	for (;;) { }
}


static void TimeSync_Handle(const char *s, uint8_t n)
{
	uint32_t ts = 0u;
	uint8_t  i;
	RTC_DateTime_t dt;

	/* ★★★★ 新增命令 ✗：**`U`** = 请求升级（2026-10-13 加 ✓）
	 *   ★ 为什么用【单个字母**✗：★ 好敲 ✗ 而且【不会和 `T<数字>` 混】✓✓ */
	if ((n == 1u) && (s[0] == 'U'))
	{
		App_RequestUpgrade();
		return;                     /* ★ 理论上不会返回 ✗ 但写上更安全 ✓ */
	}

	/* ★ 只认 `T` + 10 位数字 ✗（★ 其它一律【安静丢弃 ✗ 不刷屏】✓）*/
	if ((n != 11u) || (s[0] != 'T')) { return; }

	for (i = 1u; i < 11u; i++)
	{
		if ((s[i] < '0') || (s[i] > '9')) { return; }
		ts = ts * 10u + (uint32_t)(s[i] - '0');
	}

	/* ★★ 合理性护栏 ✗：★ 只接受 2020-01-01 ~ 2100-01-01 ✓
	 *   · ★ 防呆 ✓（★ 也防【串口噪声拼出天文数字】✓）
	 *   · ★★ 这是【最后一道闸 ✗ 不合理就【绝不写进 RTC】】✓✓ */
	if ((ts < 1577836800u) || (ts > 4102444800u))
	{
		Serial_Printf("TIME BAD (out of range: %lu)\r\n", (unsigned long)ts);
		return;
	}

	if (MyRTC_IsConfigured() == 0u)
	{
		Serial_Printf("TIME BAD (RTC not configured - check LSE)\r\n");
		return;
	}

	MyRTC_SetUnix(ts);

	/* ★ 回读确认 ✗ —— ★【不信任"我设了"✗ 要看"它变成什么了"】✓✓ */
	MyRTC_GetDateTime(&dt);
	Serial_Printf("TIME OK -> 20%02u-%02u-%02u %02u:%02u:%02u (unix=%lu)\r\n",
	              (unsigned)dt.year % 100u, (unsigned)dt.month, (unsigned)dt.day,
	              (unsigned)dt.hour, (unsigned)dt.minute, (unsigned)dt.second,
	              (unsigned long)MyRTC_GetUnix());
}

/* ★ 每帧排空串口缓冲 ✗ → 拆行 → 交给 Handle ✓
 *   ★ 用 static ✗：【跨帧保留半截命令】✓（★ 串口是按字节来的 ✗ 可能一帧只到一半 ✓）*/
static void TimeSync_Poll(void)
{
	static char    s_line[TIMESYNC_LINE_MAX];
	static uint8_t s_n = 0u;
	int16_t ch;

	while ((ch = Serial_RxPop()) >= 0)
	{
		char c = (char)ch;

		if ((c == '\n') || (c == '\r'))
		{
			if (s_n > 0u)
			{
				s_line[s_n] = '\0';
				TimeSync_Handle(s_line, s_n);
				s_n = 0u;
			}
		}
		else if (s_n < (uint8_t)(TIMESYNC_LINE_MAX - 1u))
		{
			s_line[s_n] = c;
			s_n++;
		}
		else
		{
			s_n = 0u;       /* ★ 太长 → 丢整行 ✓（★ 免得拼出半截命令 ✓）*/
		}
	}
}


/* task_DISP：① 先拷快照 ② 状态字定长防残影
 * ③ 第4行的心跳数字用来区分【暂停】和【卡死】——没有它这个验收项无法判定
 * ④ 第4行 13~16 列是 SD 错误码 E:xx（Y2）——OLED 从"坏了"升级到"为什么" */
void task_DISP(void *pvParameters)
{
	Stat_t snap;
	uint32_t frame = 0;		/* 心跳计数：每帧 +1（用来证明屏是活的） */

	       

	for (;;)
	{
		/* ① 快照 */
		taskENTER_CRITICAL();
		snap = g_stat;
		taskEXIT_CRITICAL();

		/* 刷新（OLED 16 列 x 4 行）：FUL=队列满次数  QMX=历史最大积压 */
		OLED_ShowString(1, 1,  "ACQ:");  OLED_ShowNum(1, 5,  snap.acq_count,    6);
		OLED_ShowString(2, 1,  "PRC:");  OLED_ShowNum(2, 5,  snap.proc_count,   6);
		OLED_ShowString(3, 1,  "FUL:");  OLED_ShowNum(3, 5,  snap.queue_full,   3);
		OLED_ShowString(3, 9,  "QMX:");  OLED_ShowNum(3, 13, snap.q_max_depth,  2);
		OLED_ShowString(4, 1,  g_pause ? "PAUSE" : "RUN  ");
		/* ★ 心跳：用来区分【暂停】和【卡死】 */
		OLED_ShowNum(4, 7, ++frame, 5);

		/* ★ Y2：SD 错误码上浮展示（第 4 行 13~16 列，"E:xx"格式）
		 *   码表（= FRESULT 值）：E:00 无错误史；E:01 FR_DISK_ERR（写中拔卡）；
		 *   E:03 FR_NOT_READY（没插卡/接触不良）；E:07 FR_DENIED（卷/根目录满）；
		 *   E:13 FR_NO_FILESYSTEM（卡没格式化）。
		 *   挂载成功时码随 g_sd_error 一起清 0 —— E:00 即"健康"。
		 *   心跳占 7~11 列，这里从 13 列起正好不重叠（16 列屏用满） */
		OLED_ShowString(4, 13, "E:");
		OLED_ShowNum(4, 15, g_sd_last_err, 2);

		/* ★★★★ 串口对时 ✗（2026-10-12 加 ✓）
		 *   ★ 放在这里而不是 task_KEY ✗ ——
		 *     因为本任务【优先级最低 ✗ 而且是输出类任务】✓
		 *     → 【打印确认信息不会压住别人】✓✓
		 *   ★★ 每帧都排空缓冲 ✗ → 【延迟 ≤200ms】✓ */
		TimeSync_Poll();

		/* ★★★★ 蜂鸣器服务（2026-10-13 加 ✓）
		 *   ★ 为什么放这里 ✗：★ 需要一个【周期性的"节奏发生器"】✓
		 *     · ★★ 本任务是【200ms 一帧 ✗ 正好够翻转出清晰的声音】✓
		 *     · ★★★ 而且它是【最低优先级 ✗ 输出类任务】✗
		 *       → 【不会干扰数据链路】✓✓
		 *
		 *   ★ ① SD 故障 → 持续间歇响 ✓
		 *     · ★ 判据：★ `g_sd_error != 0` ✗（★ 任何 SD 层错误 ✓）
		 *     · ★★ 为什么最重要 ✗：★ 记录仪【无人值守】✗
		 *       → ★★★ 【"数据存不进去"必须【有物理反馈】】✓✓
		 *     · ★★★★ SD 一恢复就【自动停】✗（★ Clear ✓）✓
		 *   ★ ② 按键反馈在 task_KEY 里请求 ✗ 这里只负责发声 ✓ */
		if (g_sd_error != 0u) { Buzz_Request(BUZZ_EV_SD_ERR); }
		else                  { Buzz_Clear(BUZZ_EV_SD_ERR);   }
		Buzz_Poll();

		vTaskDelay(pdMS_TO_TICKS(200));
	}
}

void task_sensor(void *pvParameters)
{
	int16_t raw;
	/* ★★ DHT22 分频计数器（2026-10-11 加 ✓）
	 *   · DHT22 要求采样间隔【≥2 秒】✗ 而本任务节奏是 1 秒一拍 ✓
	 *   · 所以每 2 拍读一次 ✓（★ 初值给 1 → 第一拍就变 2 → 首次立即读 ✓）
	 *   ★ 用 static：生命周期贯穿整个任务 ✓ */
	static uint32_t s_sensor_tick = 1u;

	(void)pvParameters;

	/* ★ 错开相位：ACQ 在 0/100/200…ms 上跑，这里先偏 500ms，
	 *   别让"单总线临界区"和"I2C 读(5.24ms)"挤在同一拍 */
	vTaskDelay(pdMS_TO_TICKS(500));

	for (;;)
	{
		DS18B20_Start();                        /* 启动转换，立即返回（不阻塞 ✓）*/
		vTaskDelay(pdMS_TO_TICKS(750));         /* ★ 这 750ms CPU 让出去 ✓ */

		if (DS18B20_Read(&raw))
		{
			g_temp_x100 = DS18B20_ToX100(raw);
		}
		else
		{
			g_temp_x100 = DS18B20_INVALID;      /* 明确给一个"无效"值，别留旧值 ✓ */
		}
		/* ★★ BMP280 气压 —— 1 秒读一次 ✓
		 *   ★ 与 ACQ 里的 MPU6050 共用同一条软件 I2C（PA2/PA3）✗
		 *   ★★ 2026-10-09 升级：用【I2C 互斥量】串行化 ✓（见 30 号 §二十 ✓）
		 *      —— 只挡另一个 I2C 用户（ACQ）✓ 其它任务不受影响 ✓✓
		 *      （原来的 vTaskSuspendAll 会冻全场 10ms ✗ 已弃用 ✓）*/
		{
			int32_t p_x10, t_c100;
			xSemaphoreTake(s_i2c_lock, portMAX_DELAY);
			if (BMP280_Read(&p_x10, &t_c100) == 0u)
			{
				g_press_x10pa   = p_x10;
				g_bmp_temp_x100 = (int16_t)t_c100;
			}
			else
			{
				g_press_x10pa   = 0;            /* ★ 无效 → 0（"未接"约定 ✓）*/
			}
			xSemaphoreGive(s_i2c_lock);
		}

		/*========================================================================
		 *  ★★ DHT22 温湿度（2026-10-11 加 ✓）—— 每 2 拍读一次（= 2 秒 ✓）
		 *
		 *  ★ 为什么放在这里（BMP280 之后 ✓）：
		 *    · DS18B20 的 750ms 等待已经过去 ✗ → 这一拍不急 ✓
		 *    · ★★ 和 DS18B20 在本任务里【顺序执行】✗
		 *      → 两个临界区【不会重叠】✓（最坏是连续约 12ms ✓）
		 *
		 *  ★★ 中断占用（改之前已核算 ✓ 见 30 号）：
		 *    · DHT22 整帧 4.8ms 全在临界区 ✗（位间隙没有容错空间 ✓ 必须整帧包 ✓）
		 *    · 每 2 秒一次 → 2.4ms/s ✗
		 *    · 加 DS18B20 的 7.2ms/s → 合计 ≈ 0.96% ✓ 可接受 ✓
		 *    ⚠️ 代价：最坏连续约 12ms 关中断 ✗ → 会丢几个 tick ✓
		 *       → vTaskDelay 漂移 <1.2% ✓（★ 长稳时要重新量 tick 步进分布 ✓）
		 *========================================================================*/
		s_sensor_tick++;
		if ((s_sensor_tick & 1u) == 0u)
		{
			uint16_t h_x10 = 0xFFFF;
			int16_t  t_x10 = 0;

			if (DHT22_Read(&h_x10, &t_x10))
			{
				g_humi_x10     = h_x10;
				g_dht_temp_x10 = t_x10;
			}
			else
			{
				g_humi_x10 = 0xFFFF;            /* ★ 无效 → 0xFFFF（"未接"约定 ✓）*/
			}
		}

		vTaskDelay(pdMS_TO_TICKS(250));         /* 750 + 250 = 1 秒一拍 */
	}
}
/* 应用钩子：★ 名字一字不差（写错不报错、只是永远不被调用）
 * 结尾死循环 + 关中断；打印是【尽力而为】。三个钩子都做过故意触发验证 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
	(void)xTask;
	Serial_Printf("\r\n!!! STACK OVERFLOW: task=%s !!!\r\n", pcTaskName);
	taskDISABLE_INTERRUPTS();
	while(1){}
}

void vApplicationMallocFailedHook(void)
{
	Serial_Printf("\r\n!!! MALLOC FAILED: free=%u, min_ever_free=%u !!!\r\n",
				  (unsigned)xPortGetFreeHeapSize(),
				  (unsigned)xPortGetMinimumEverFreeHeapSize());
	taskDISABLE_INTERRUPTS();
	while(1){}
}

void vAssertCalled(const char *pcFile, unsigned long ulLine)
{
	Serial_Printf("\r\n!!! ASSERT FAILED: %s:%lu !!!\r\n", pcFile, ulLine);
	
	taskDISABLE_INTERRUPTS();
	while(1){}
}

/*==============================================================================
 *  ★★★ IWDG 独立看门狗（2026-10-12 加 ✓ 阶段 3 最后一块）
 *
 *  ★ 为什么现在需要它 ✗：
 *    · 今天加了 DHT22 ✗ → 有 4.8ms 的【整帧临界区】✓
 *    · 最坏连续约 12ms 关中断 ✗（DHT22 和 DS18B20 在同一任务里连着跑 ✓）
 *    · ★★ 一旦哪里死循环 / 卡死 ✗ → 【目前没有任何兜底】✓✓
 *    · ★★★ 而长稳要跑一整天 ✗ → 半夜死机【第二天才发现，丢十几个小时】✓
 *
 *  ★★ 喂狗点 = vApplicationIdleHook() ✗（在下面 ✓）
 *    · 空闲任务【优先级 0 ✗ 最低】✓
 *    · ★ 正常时：所有任务都会 vTaskDelay ✗ → 每 100ms 内必然有空闲窗口 ✓
 *    · ★★ 任何任务【死循环】✗ → 空闲任务【拿不到 CPU】✗ → 不喂 → 复位 ✓✓
 *
 *  ★★ 超时 = 6.55 秒 ✗：
 *    · LSI 40kHz ÷ 预分频 64 = 625Hz ✓
 *    · 重装值 4095 → 4096 / 625 ≈ 【6.55 秒】✓
 *    · ★ LSI 有 ±50% 公差 ✗ → 实际 【4.4 ~ 8.7 秒】✓
 *    · ★★ 对 100ms 的任务周期【非常宽松】✓✓
 *
 *  ★★★ 启动时机【很关键 ✗】：
 *    · IWDG 一旦 Enable() 就【不能停】✗✓
 *    · 而启动阶段【SD 初始化可能耗好几秒】✗
 *      （★ 实测 ACMD41 重试过 142 次 ✓ 那是好几秒 ✓）
 *    · → ★★ 所以【必须在所有耗时初始化之后】才调 WD_Init() ✓✓
 *      （★ 在 main.c 里，紧挨着 freertos_demo() 之前 ✓）
 *============================================================================*/
void WD_Init(void)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);   /* 允许写 PR / RLR ✓ */
	IWDG_SetPrescaler(IWDG_Prescaler_64);           /* 40kHz / 64 = 625Hz ✓ */
	IWDG_SetReload(4095);                           /* 4096 / 625 ≈ 6.55 秒 ✓ */
	IWDG_ReloadCounter();                           /* ★ 先喂一次再启动 ✓ */
	IWDG_Enable();                                  /* ★★ 启动后【不可停】✗ */
}

/*==============================================================================
 *  ★★★ 空闲钩子：IWDG 的喂狗点（2026-10-12 加 ✓）
 *       配合 FreeRTOSConfig.h 里 configUSE_IDLE_HOOK = 1 ✓
 *
 *  ★ 为什么【放这里就能检测死机】✗：
 *    · 空闲任务【优先级 0 ✗ 最低】✓
 *    · ★★ 只要【有一个任务不让出 CPU】✗ → 空闲任务【永远轮不到】✓
 *      → ★★★ 钩子不跑 → 不喂 → 【6.55 秒后 IWDG 复位】✓✓
 *
 *  ★★ 为什么【不用担心漏喂】✗：
 *    · 空闲任务在【每轮空闲循环】都会调这个钩子 ✓
 *    · 系统正常时【空闲时间远多于 6.55 秒】✓ → 喂得非常勤 ✓✓
 *
 *  ★ 注意 ✗：
 *    · 本钩子在【任务上下文】里跑 ✓（不是中断 ✓）可以用普通 API ✓
 *    · ★★ 但它【霸占空闲任务】✗ → 必须极短 ✓（这里只有一条寄存器写 ✓）
 *    · ★★ 复位后【不打印任何东西】✗ ——
 *      想确认它触发过 ✗ → 看串口会【重新打一遍 "=== boot ==="】✗✓
 *============================================================================*/
void vApplicationIdleHook(void)
{
	IWDG_ReloadCounter();           /* ★ 空闲任务能跑到 = 系统还活着 ✓ */
}
