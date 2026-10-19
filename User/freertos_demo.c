/**
  ******************************************************************************
  * @file    freertos_demo.c
  * @brief   FreeRTOS **动态**方式创建任务（★ 全部 `xTaskCreate` ✗ 栈/TCB 走 heap_4 ✓）
  *         ★★ 原写"静态方式"✗ 与实际不符 ✓（★ 改于 2026-10-19 ✓）
  
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
#include "golden.h"                /* ★★★★ 金副本（版本备份/回退 ✓ 2026-10-14 加 ✓）*/
#include "MyRTC.h" 
#include "boot_flag.h"             /* ★ 升级请求标志（★ 和 Boot\boot_flag.h 逐字节一致 ✓）*/
#include "Crc16.h"
/*==============================================================================
 *  数据记录仪 —— 应用层（任务总装）
 *      数据流： task_ACQ --Record_t--> [xQueueRec] --> task_PROC
 *              --拼CSV行+乒乓缓冲(SD_BufAcquire/SD_LogEnqueue)--> task_SD --> FatFs --> SD 卡
 *      任务表（★ 2026-10-19 按代码【一个个核过】✓）：
 *              ACQ 6/256/100ms   PROC 2/256/队列   KEY 2/128/20ms
 *              DISP 1/256/200ms  SENSOR 1/128/队列 GOLDEN 1/256/后台
 *              SD 1/384/队列 （★ 384 字 = 2026-10-19 修正后的值 ✗ 见 `task_sd.h` ✓）
 *          ★ 共 **7 个**（+ START 建完自删 + IDLE 系统自带 ✓）
 *          ★★ 原表【漏了 SENSOR 与 GOLDEN】✗ 且 SD 的栈写成 256 ✓
 *  ★ 每个【为什么】（优先级/栈大小/快照为何要进临界区/踩过哪些坑）都在：
 *      stm32\学习笔记\30-项目设计说明-为什么这么写.md
 *============================================================================*/
/******************************************************************************
 *  任务配置（★ **动态**方式 ✗ 栈与 TCB 都由 `xTaskCreate` 从 heap_4 里分配 ✓
 *            ★★ 原写"静态方式：句柄 + 栈数组 + TCB 三样都要自己定义"✗ 已过期 ✓）
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
#define TASK_ACQ_STACK_SIZE        128  /* ★★ 2026-10-19 R4：256 → 128 ✗ 省 512 B ✓
                                         *  ★ 实测最深只用 **69 字**（水位 187/256）✓
                                         *  ★★ 68 × 1.3 ≈ 90 ⇒ 128 有 86% 余量 ✓✓ */
TaskHandle_t    task_acq_handler;
void task_ACQ(void *pvParameters);

/* task_PROC：出队 → 拼 CSV 行 → 填乒乓缓冲交 task_SD（Y8+⑤ 改造）
 * ★ 仍是【唯一】串口打印者；栈 **384 字**的依据是 Serial_Printf 调用链 352 字节 */
/* TASK_PROC 任务配置：出队+拼批落卡+串口回显*/
#define TASK_PROC_PRIO              2
#define TASK_PROC_STACK_SIZE        320  /* ★★★ 2026-10-19 R4：256 → **320**（★ 反而加 ✓）
                                          *  ★ 实测最深 **206 字**（水位只剩 50/256 = 19.5%）✗
                                          *    比 SD 还紧 ✗✓
                                          *  ★★ 206 × 1.3 ≈ 268 ⇒ 320 有 55% 余量 ✓✓
                                          *  （★ 老注释说的"256 够用"是【没跑过 S 路径】的数据 ✓）*/
TaskHandle_t    task_proc_handler;
void task_PROC(void *pvParameters);

/* task_KEY：20ms 扫键（扫描周期本身就是消抖基准）
 * ★ 本任务【禁止 Serial_Printf】：栈 128 字只剩约 96 字节余量，打印甩给 PROC */
#define TASK_KEY_PRIO               2
#define TASK_KEY_STACK_SIZE         64   /* ★★ 2026-10-19 R4：128 → 64 ✗ 省 256 B ✓
                                          *  ★ 实测最深 **23 字**（水位 105/128）✓
                                          *  ★★ 23 × 1.3 ≈ 30 ⇒ 64 有 178% 余量 ✓✓ */
TaskHandle_t    task_key_handler;
void task_KEY(void *pvParameters);

/* task_DISP：OLED 显示（200ms 刷一次）
 * ★ 本任务是 OLED 的【唯一所有者】，别的任务一律不许碰屏 */
/* ★ 2026-10-19 记录一个坑（★ 曾把优先级改成 3 ✗ 已改回 1 ✓）：
 *   当时以为"屏冻住"是【同优先级被时间片切碎 ⇒ 软件 I2C 时序被拉长】✗
 *   ★★ 实测证明【不是】✗ 真因是 `Clk_CalibOnce()` 阻塞本任务 11 秒 ✓
 *     （已搬到 task_GOLDEN ✓ 见本文件 `task_GOLDEN` ✓）
 *   ⇒ 优先级【恢复为 1】✗ 保持 20 小时长稳验证过的那套调度 ✓ */
#define TASK_DISP_PRIO              1
#define TASK_DISP_STACK_SIZE        224  /* ★★★ 2026-10-19 R4：256 → 224 ✗ 省 128 B ✓
                                          *  ★ 实测最深 **150 字** ✗ 而且【是 `S` 命令那趟】
                                          *    撑上去的 ✓✓
                                          *  ★★ 为什么 ✗ `App_RequestUpgrade()` /
                                          *     `S` 那条路会【在 DISP 栈上】调
                                          *     `Golden_SaveCurrent()`（★ 2~3 秒的整份备份 ✓）
                                          *     ⇒ 水位从 126 掉到 106（★ 多吃了 20 字 ✓）
                                          *  ★★★ 150 × 1.3 ≈ 195 ⇒ 224 有 49% 余量 ✓✓ */
TaskHandle_t    task_disp_handler;
void task_DISP(void *pvParameters);

/* ★★★★ 2026-10-18 加：OLED 超时黑屏（★ 低功耗改造 序 3 ✓）
 *
 *  ★ 为什么要有 ✗：本板实测【屏占约 6~7 mA】✗ 而 25 mA 里有 13 mA 是
 *    纯硬件底噪（LDO+LED ✗ 软件动不了）⇒ 屏是【唯一还能靠软件砍的一块】✓
 *
 *  ★★ 取值 ✗：30000 = 30 秒没人按键就黑屏 ✓
 *     · ★ 设 0 = 【永不黑屏】✗ 台面上调试时用这个 ✓
 *       （改了要重新编译烧录 ✓ 因为它是编译期常量 ✓）
 *
 *  ★ 谁算"有人在看" ✗：现在只有【按键】会刷新 `g_ui_act_tick` ✓
 *     · ★ 串口命令【不算】✗ —— 现场没有 PC ✗ 不必为它亮屏 ✓
 *
 *  ★★★ 关机态（SAFE TO OFF）【绝不黑屏】✗ 见 task_DISP 里的判断 ✓
 *     因为那正是"必须让人看见"的唯一状态 ✓ */
#define OLED_BLANK_MS               30000u

/* task_SD：SD 卡落盘任务（乒乓缓冲的接收方，task_sd.c 实现本体）
 * ★ 优先级/栈大小用 task_sd.h 的定版宏（1 / **384 字**），别在这里手抄数字 ——
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

/* ★★★★ task_GOLDEN：金副本后台备份（★ 2026-10-14 加 ✓ 设计见 `golden.h`）
 *   ★ 优先级 1 ✗ 最低业务级 ✓
 *     （★ 纯后台活 ✗ 而且软件 SPI 是【忙等】✗ 采集链必须能随时抢占它 ✓）
 *   ★★ 栈 **384 字 = 1536 B** ✗ 够用（★ 实测水位 `sdstk` 剩 76~84 字 ✓
 *     （★ 只见一个 64B 头结构 + `Serial_Printf` 调用链 ✓
 *       ★★ 页数据【直接从内部 Flash 指过去 ✗ 不要 RAM 缓冲】✓）✓ */
#define TASK_GOLDEN_PRIO            1
#define TASK_GOLDEN_STACK_SIZE      192  /* ★★★ 2026-10-19 R4：256 → 192 ✗ 省 256 B ✓
                                          *  ★ 实测最深 **129 字** ✗ 而且【是 RTC 校准那趟】
                                          *    撑上去的（★ 水位 235 → 127 ✓）
                                          *  ★★ `Clk_CalibOnce()` 在 task_GOLDEN 里跑
                                          *     （★ 10 秒 RTC 校准 + printf ✓）
                                          *  ★★★ 129 × 1.3 ≈ 168 ⇒ 192 有 49% 余量 ✓✓ */
TaskHandle_t    task_golden_handler;
void task_GOLDEN(void *pvParameters);


/* Record_t：一条记录，定长 40 字节（队列是值拷贝，定长最省事）
 * ★ 布局不要随便改；改字段要同步改 SD 卡格式 和 下面的编译期断言 */
typedef struct {
    uint32_t seq;          /* 序号：从 1 开始，只有 task_acq 会自增 */
    uint32_t tick;         /* 时间戳：产生这一条时的 tick（阶段3 换成 RTC 时间）*/
	uint32_t press;     /* ×10 Pa（BMP280；读失败/未接=0；★ 2026-10-18 起负值也钳 0 ✓）*/
	uint32_t rtc;       /* ★ 真 Unix 秒(UTC) ✓ —— 2026-10-18 修正：原"现在恒 0"【早已过期】✗
     *   ★★ 可信度【不在数据里】（16 列没有 trust 位）⇒ 只能靠屏上 `T?`（复盘 T2）✓ */
    int16_t  ax, ay, az;   /* 加速度原值（阶段2 换 MPU6050） */
    int16_t  gx, gy, gz;   /* 角速度原值 */
	int16_t  temp_ds;   /* ★ ×100 °C（DS18B20）*/
    int16_t  temp_chip;    /* 芯片内部温度 ×100 */
	uint16_t humi;      /* ×10 %RH（DHT22；未接时约定 0xFFFF）*/
    uint16_t light;        /* 光敏：PB13 是数字量，只能判亮/暗，不是 ADC */
	uint16_t vbat; 
	/* ★★★★★ 2026-10-19：**CRC-32 方案已废弃 ✗ 退回 CRC-16** ✓
	 *   ★ 为什么废弃 ✗（★ 用户拍板 ✓）：
	 *     · 硬件 CRC **不省 RAM**（一样 0 静态 ✓）✗ 却**增加难度** ✓
	 *       （★ 要按 32 位字喂变长数据 + 补齐规则 ✗ 算法非标准
	 *         ⇒ **PC 侧不能用 `zlib.crc32()`** ✗✓）
	 *     · ★★ 而收益只有"16 位漏检率 1/65536 ⇒ 38,440 行垃圾里
	 *       期望约 0.6 行误判通过"✗ 实际可忽略 ✓
	 *     · ★ 代价却是【每行 +5 字节】✗（20 万行 ⇒ +1 MB ✓）
	 *   ⇒ ★ 结论：**性价比不划算 ✗ 维持 CRC-16/KERMIT** ✓✓
	 *   ★★ 已回滚 ✗：`Crc32_Calc()` 也一并删除（★ 不留死代码 ✓）*/
	uint16_t crc;       /* ★ CRC-16/KERMIT，算的是【CSV 前 15 列文本】而不是本结构体 ✓ */
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

/* ★★★★ 2026-10-18 加：空闲钩子【进入次数】—— 低功耗改造 序 1 的观测点 ✓
 *
 *  ★ 它量的是什么 ✗：
 *    · `vApplicationIdleHook()` 里加了 `__WFI()` 之后 ✗
 *      钩子【每进一次 ≈ 睡到下一个 1ms SysTick】✓
 *    ⇒ 【每秒进入次数 ÷ 1000 ≈ 空闲占比】✓✓
 *      例：500 次/秒 ⇒ 约 50% 时间在睡 ✓
 *
 *  ★★ 为什么不能拿这个次数直接当"CPU 占用率"✗：
 *    · 没加 WFI 时钩子是【死循环空转】✗ ⇒ 每秒上万次 ✗ 没有物理意义 ✓
 *    · ⇒ ★ 只有【加了 WFI 之后】这个换算才成立 ✓
 *      所以它同时也是"WFI 到底生效没有"的判据 ✓（前后差一个数量级 ✓）
 *
 *  ★ 必须 volatile：空闲任务写、task_DISP(prio2) 读 ✓
 *    （不加会被编译器缓存进寄存器 ✗ 读到永远是旧值 ✓）*/
volatile uint32_t g_idle_cnt = 0u;

/* ★★ 2026-10-18 加：`低功耗改造方案.md` §7.2 的指标 —— "每秒有多少毫秒闲过"✓
 *   · `g_idle_ms`：计的是【某一个 tick 里至少进过一次空闲】⇒ +1 ✓
 *     同一 tick 里进很多次【只算一次】（靠下面的 `g_last_idle_tick` 去重 ✓）
 *   ★★ 它和 `g_idle_cnt` 的区别（★ 别混用 ✓）：
 *     · `g_idle_ms` 量的是【还有多少可睡的空间】✗ 不随 WFI 变化 ✓
 *       ⇒ 适合"改之前量一次、改之后量一次"对比 ✓
 *     · `g_idle_cnt` 量的是【钩子被调了多少次】✗ 一加 WFI 就掉一个数量级 ✓
 *       ⇒ 适合"证明 WFI 真的生效"✓
 *   ★ 已知局限（★ 别把它当精确占用率 ✓）：
 *     只要某一毫秒里【闲过一次】就算 1 ⇒ 被切得很碎的忙时会被"算成闲"✗
 *     ⇒ 它读出来的是【可睡时间的下界】✓ 不是"空闲百分比" ✓ */
volatile uint32_t g_idle_ms       = 0u;
volatile uint32_t g_last_idle_tick = 0u;

/* ★★★★ 2026-10-18 加：最近一次"有人在看屏"的 tick（★ 低功耗 序 3 ✓）
 *   · 现在只有【按键】会刷新它 ✓（`task_KEY` 里写 ✓）
 *   · ★ 初值 0 ⇒ 开机 30 秒没人按键就【自动黑屏】✓
 *     现场无人值守本来就该这样 ✓ 想看屏按一下键就行 ✓ */
volatile uint32_t g_ui_act_tick = 0u;

/* ★★★★ 2026-10-18 加：屏【现在】是不是黑着的（★ 给 `task_KEY` 判断用 ✓）
 *   ★ 为什么要跨任务告诉按键任务 ✗：
 *     · 黑屏时【第一下按键只该唤醒 ✗ 不该执行功能】✓
 *     · 而按键里有个【暂停采集】✗ 用户只想看一眼屏 ⇒ 结果把记录停了 ✓
 *       ★★ 现场无人值守 ⇒ 停了也没人知道 ✗ 这是真会丢数据的动作 ✓✓
 *   ★ 谁写谁读：`task_DISP` 写、`task_KEY` 读 ⇒ 必须 volatile ✓ */
volatile uint8_t  g_oled_blanked = 0u;

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
/* ★★★★ 2026-10-19 R3：这里原来有一块 `static char s_task_list_buf[512]` ✗ 【已删除】✓
 *   · 它只喂 `vTaskListTasks()` ✗ 而那个函数内部会 `pvPortMalloc`
 *     ⇒ 它是【全工程最后一个还会碰堆的人】✓✓
 *   · ★★ 换成 `uxTaskGetSystemState()`（数组由调用者给）
 *     ⇒ 这 512 B 缓冲和那个 1 KB 堆【一起不用了】✓ 并且 `heap_4.o` 会被链接器丢弃 ✓✓ */
/* ★★ 2026-10-18 改：8 → 12 ✗✓
 *   ★ 现在实际有：START(已删) / ACQ / PROC / KEY / DISP / sensor / SD / GOLD / IDLE
 *     = **9 个**（含 IDLE，不含已自删的 start_task）✓
 *   ★★ 而原来只给 8 个槽 ⇒ `uxTaskGetSystemState` 只会填前 8 个
 *     ⇒ 【栈水位表被截断】✗ 最后一个（通常是 IDLE 或 GOLD）看不到 ✓✓
 *   ★ 为什么这事要紧 ✗：上次 SD 栈只剩 16 字那次险情
 *     正是靠这张表才发现的 ✓✓ */
static TaskStatus_t s_task_status[12]; 
/******************************************************************************
 *  FreeRTOS 例程入口
 ******************************************************************************/
 
 
/*==============================================================================
 *  ★★★★★ 2026-10-19 加：**全静态分配的内存块**（★ RAM 优化 R2 ✓）
 *
 *  ★ 为什么放这个文件 ✗ 本工程铁律【不许新建 .c】✗ 所以就近放在"建任务的地方"✓
 *  ★★ 为什么值得改 ✗（★ 实测 ✓）：
 *    · 原来 12KB 堆 ✗ `STAT` 显示 `heapmin=2664` ⇒ **2,664 B 永远空转**✓✓
 *    · 改成静态 ✗ 这些数组就变成 **.bss 里有名字的对象**
 *      ⇒ `map` 里能逐项对账 ✗ 没有块头开销、没有碎片、没有"分配失败"路径 ✓✓
 *  ★★★ `MakeTask()` 这个小包装 ✗ 只是把
 *    `xTaskCreateStatic` 的【NULL = 失败】翻译回 `pdPASS/pdFAIL`
 *    ⇒ 上层那 8 个 `if (... != pdPASS)` **一个字都不用改** ✓✓
 *  ★★★★ 栈大小用的还是原来的宏 ✗ 所以**行为完全等价** ✗ 只是"内存从堆搬到了 .bss" ✓
 *============================================================================*/
static StackType_t  s_start_stack [START_TASK_STACK_SIZE];
static StaticTask_t s_start_tcb;
static StackType_t  s_acq_stack   [TASK_ACQ_STACK_SIZE];
static StaticTask_t s_acq_tcb;
static StackType_t  s_proc_stack  [TASK_PROC_STACK_SIZE];
static StaticTask_t s_proc_tcb;
static StackType_t  s_key_stack   [TASK_KEY_STACK_SIZE];
static StaticTask_t s_key_tcb;
static StackType_t  s_disp_stack  [TASK_DISP_STACK_SIZE];
static StaticTask_t s_disp_tcb;
static StackType_t  s_sensor_stack[TASK_SENSOR_STACK_SIZE];
static StaticTask_t s_sensor_tcb;
static StackType_t  s_sd_stack    [TASK_SD_STACK_SIZE];
static StaticTask_t s_sd_tcb;
static StackType_t  s_golden_stack[TASK_GOLDEN_STACK_SIZE];
static StaticTask_t s_golden_tcb;

/* ★★ 队列/互斥量的静态存储（★ 控制块 + 缓冲，都是 .bss ✓）
 *   ★ 缓冲用 `uint32_t` 数组 ⇒ 天然 4 字节对齐 ✓
 *     （★ 实测 `xQueueGenericCreateStatic` **没有** 8 字节对齐断言 ✓ 4 即可 ✓）*/
static StaticQueue_t s_qrec_cb;
static uint32_t      s_qrec_storage[(REC_QUEUE_LEN * sizeof(Record_t) + 3u) / 4u];
static StaticSemaphore_t s_i2c_mutex_cb;

/* ★★★★ 2026-10-19：与 `xTaskCreate` **同签名、同返回约定** ✗ 但内部走静态分配 ✓
 *
 *  ★ 为什么这么设计 ✗ 这样 8 个调用点【只把函数名换掉】**✗ 参数一个都不用动 ✓✓
 *    （★ 少改 40 行 = 少 40 个改错的机会 ✓）
 *  ★★ 静态内存按【任务函数指针】挑 ✗ 各任务在 55~131 行都有前置声明 ✓
 *    ⇒ 这里能直接比较函数地址 ✓ 表里没登记的直接 `pdFAIL`（★ 不会静默 ✓）*/
static BaseType_t MakeTask(TaskFunction_t fn, const char *name, uint32_t depth,
                           void *pv, UBaseType_t prio, TaskHandle_t *out)
{
	StackType_t  *stk = NULL;
	StaticTask_t *tcb = NULL;

	if      (fn == task_ACQ)    { stk = s_acq_stack;    tcb = &s_acq_tcb;    }
	else if (fn == task_PROC)   { stk = s_proc_stack;   tcb = &s_proc_tcb;   }
	else if (fn == task_KEY)    { stk = s_key_stack;    tcb = &s_key_tcb;    }
	else if (fn == task_DISP)   { stk = s_disp_stack;   tcb = &s_disp_tcb;   }
	else if (fn == task_sensor) { stk = s_sensor_stack; tcb = &s_sensor_tcb; }
	else if (fn == task_SD)     { stk = s_sd_stack;     tcb = &s_sd_tcb;     }
	else if (fn == task_GOLDEN) { stk = s_golden_stack; tcb = &s_golden_tcb; }
	else if (fn == start_task)  { stk = s_start_stack;  tcb = &s_start_tcb;  }

	if ((stk == NULL) || (tcb == NULL)) { return pdFAIL; }

	*out = xTaskCreateStatic(fn, name, depth, pv, prio, stk, tcb);
	return (*out != NULL) ? pdPASS : pdFAIL;
}

/* freertos_demo：内核对象必须在 vTaskStartScheduler() 之前创建，NULL 要查 */
void freertos_demo(void)
{
    xQueueRec = xQueueCreateStatic(REC_QUEUE_LEN, sizeof(Record_t),
                                   (uint8_t *)s_qrec_storage, &s_qrec_cb);  /* ★ 静态 ✓ */
	if (xQueueRec == NULL)
	{
		Serial_Printf("!!! xQueueCreateStatic FAILED (bad args) !!!\r\n");
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
	s_i2c_lock = xSemaphoreCreateMutexStatic(&s_i2c_mutex_cb);   /* ★ 静态 ✓ */
	
	if (s_i2c_lock == NULL)
	{
		Serial_Printf("!!! xSemaphoreCreateMutexStatic FAILED !!!\r\n");
		return;
	}

	Serial_Printf("queue ok: item=%d bytes\r\n", (int)sizeof(Record_t));
;
	
	/* 只创建"装配工"这一个任务，其余任务由它统一建（正点原子惯例，见 start_task） */
	(void)MakeTask((TaskFunction_t 			   )   start_task,
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

	/*==========================================================================
	 *  ★★★★ 金副本初始化（★ 2026-10-14 加 ✓ 完整设计见 `golden.h`）
	 *
	 *  ★★ 为什么【放在【调度器启动之前】**✗✓：
	 *    · ★ 它要做三件事 ✗【读元数据（黑名单）/ 算自己镜像的 CRC /
	 *      【决定"要不要后台填备份"】】✓
	 *    · ★★ 【在【任何任务跑起来之前【把状态定下来 ✗ 后面不再有分歧】】✓✓
	 *
	 *  ★★★ 它【不会卡**✗：
	 *    · ★ W25Q64 不在位 → 【直接关掉整个功能 ✗ 一切照旧】✓
	 *    · ★★ 算一次 CRC 约 40~60ms ✗ 【只此一次】✓✓
	 *========================================================================*/
	(void)Golden_Init();

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
	if (MakeTask(  (TaskFunction_t )   task_ACQ,
                   (char *         )   "ACQ",
                   (uint32_t       )   TASK_ACQ_STACK_SIZE,
                   (void *         )   NULL,
                   (UBaseType_t    )   TASK_ACQ_PRIO,
                   (TaskHandle_t * )   &task_acq_handler) != pdPASS)
	{
		fail_mask |= (1u << 0);		/* bit0 = ACQ 没建起来 */
	}
	if (MakeTask(  (TaskFunction_t )   task_PROC,
                   (char *         )   "PROC",
                   (uint32_t       )   TASK_PROC_STACK_SIZE,
                   (void *         )   NULL,
                   (UBaseType_t    )   TASK_PROC_PRIO,
                   (TaskHandle_t * )   &task_proc_handler) != pdPASS)
	{
		fail_mask |= (1u << 1);		/* bit1 = PROC 没建起来 */
	}
	if (MakeTask(  (TaskFunction_t )   task_KEY,
                   (char *         )   "KEY",
                   (uint32_t       )   TASK_KEY_STACK_SIZE,
                   (void *         )   NULL,
                   (UBaseType_t    )   TASK_KEY_PRIO,
                   (TaskHandle_t * )   &task_key_handler) != pdPASS)
	{
		fail_mask |= (1u << 2);		/* bit2 = KEY 没建起来 */
	}
	/* ★ task_DISP 也在这里创建（OLED 唯一所有者；prio 1 最低，不影响采集） */
	if (MakeTask(  (TaskFunction_t )   task_DISP,
                   (char *         )   "DISP",
                   (uint32_t       )   TASK_DISP_STACK_SIZE,
                   (void *         )   NULL,
                   (UBaseType_t    )   TASK_DISP_PRIO,
                   (TaskHandle_t * )   &task_disp_handler) != pdPASS)
	{
		fail_mask |= (1u << 3);		/* bit3 = DISP 没建起来 */
	}

	if (MakeTask(  (TaskFunction_t )   task_sensor,
                   (char *         )   "sensor",
                   (uint32_t       )   TASK_SENSOR_STACK_SIZE,
                   (void *         )   NULL,
                   (UBaseType_t    )   TASK_SENSOR_PRIO,
                   (TaskHandle_t * )   &task_sensor_handler) != pdPASS)
	{
		fail_mask |= (1u << 4);		/* bit4 = sensor 没建起来 */
	}

	/* ★ R1 集成（第 6 课）：task_SD 落盘任务 —— prio/栈取 task_sd.h 定版值
	 *   （1 / **384 字 = 1.5KB**）。prio 1 与 sensor 同级：SD 的慢（busy 最坏 250ms）
	 *   被隔离在最低优先级，ACQ→PROC 采集链永远先跑（task_sd.h 顶注释）。
	 *   建任务前 task_sd_Init() 已在 freertos_demo() 里建好队列，收缓冲
	 *   不会撞空指针（task_SD 本体仍保留"对象没就绪就等"的双保险） */
	if (MakeTask(  (TaskFunction_t )   task_SD,
                   (char *         )   "SD",
                   (uint32_t       )   TASK_SD_STACK_SIZE,
                   (void *         )   NULL,
                   (UBaseType_t    )   TASK_SD_PRIO,
                   (TaskHandle_t * )   &task_sd_handler) != pdPASS)
	{
		fail_mask |= (1u << 5);		/* bit5 = SD 没建起来 */
	}

	/* ★★★★ 金副本后台备份任务（★ 2026-10-14 加 ✓）
	 *   ★ 为什么【优先级 1**✗✓：
	 *     · ★ 它干的是【"把内部 Flash 慢慢拷到 W25Q64"】✗ 【纯后台活】✓
	 *     · ★★ 【软件 SPI 是【忙等】✗ 一页 256B ≈ 1.4ms 一直占着 CPU】✓
	 *     → ★★★ 【放最低业务优先级 ✗ 采集链【随时能抢占它】】✓✓
	 *   ★★ 【闲着的时候睡 5 秒**✗
	 *     【备份做完之后【就永久安静了】】✓✓ */
	if (MakeTask(  (TaskFunction_t )   task_GOLDEN,
                   (char *         )   "GOLD",
                   (uint32_t       )   TASK_GOLDEN_STACK_SIZE,
                   (void *         )   NULL,
                   (UBaseType_t    )   TASK_GOLDEN_PRIO,
                   (TaskHandle_t * )   &task_golden_handler) != pdPASS)
	{
		fail_mask |= (1u << 6);		/* bit6 = GOLD 没建起来 */
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
	 rec.press = (g_press_x10pa > 0) ? (uint32_t)g_press_x10pa : 0u;   /* ★ 真实气压 ×10 Pa ✓（读失败=0 ✓；★ 负值也钳 0 —— T6 防回绕成 42 亿 ✓）*/
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
        Loss_Add(LOSS_Q_FULL, 1u);   /* ★ 队列满 ⇒ 丢 1 条 ✓（★ 此刻在临界区里 ✗ 嵌套安全 ✓）*/
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

	/* ★★★★ 2026-10-19 改（★ RAM 优化 R3 ✓）：
	 *   ★ 原来调 `vTaskListTasks(s_task_list_buf, ...)` ✗
	 *     · ★★ 它内部会 `pvPortMalloc(uxCurrentNumberOfTasks × sizeof(TaskStatus_t))`
	 *       ⇒ **全工程最后一个还会碰堆的地方** ✓✓
	 *   ★★ 现在直接用 `uxTaskGetSystemState()` ✗ 数组由【调用者提供】
	 *     ⇒ 一次 malloc 都不需要 ✓
	 *   ★★★ 于是 `pvPortMalloc` 彻底没人引用 ⇒ 链接器会把 `heap_4.o`
	 *     （★ 含 12 KB 的 `ucHeap[]` + ~960 B 代码）**整体丢弃** ✓✓
	 *   ⚠️ ★★★★ 【别】把 `configSUPPORT_DYNAMIC_ALLOCATION` 置 0 ✗
	 *     因为 `heap_4.c` 开头有 `#error` 拦着 ✗ 而工程文件【不许改】✓
	 *     ⇒ 保持 1、靠"没引用"让它自然消失 ✓ */
	n = uxTaskGetSystemState(s_task_status, 12, &total_run_time);

	Serial_Printf("---- Task table (%u) ----\r\n", (unsigned)n);
	for (i = 0u; i < n; i++)
	{
		const char *st;

		switch (s_task_status[i].eCurrentState)
		{
		case eRunning:   st = "RUN"; break;
		case eReady:     st = "RDY"; break;
		case eBlocked:   st = "BLK"; break;
		case eSuspended: st = "SUS"; break;
		case eDeleted:   st = "DEL"; break;
		default:         st = "INV"; break;
		}

		/* ★ 一行一个任务 ✗ `Serial_Printf` 的 200 B 缓冲够用 ✓
		 *   （★ 老代码整表一次发才必须用 `Serial_SendString` ✓）*/
		Serial_Printf("%-8s %s prio=%u stkfree=%u\r\n",
		              s_task_status[i].pcTaskName, st,
		              (unsigned)s_task_status[i].uxCurrentPriority,
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

/* ★★★ 2026-10-18 加：当前缓冲里【已经攒了几条】✗✓
 *   ★ 为什么需要 ✗：交批失败时【整批都丢】✓
 *     → 要按【条】记账 ✗ 就得知道这一批是几条 ✓
 *   ★ 什么时候清 ✗：缓冲交出去 / 换新缓冲时归零 ✓ */
static uint16_t s_nrec  = 0;

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
	static uint8_t  diag_done = 0;	/* ★ 诊断只打一次（★ R4 定容完已改回 ✓）
	                                 *   ★★★ TEMP 期间改成过"每 20 秒"✗ 为了量出
	                                 *     `S`/掉卡重挂载/RTC 校准这些【重路径】的水位 ✓✓
	                                 *   ★★ 为什么不留着周期打印 ✗ 它跑在 `task_PROC`
	                                 *     （prio 2）上 ✗ 8 行 ≈ **160 ms 阻塞**
	                                 *     ⇒ 会拖慢采集链 ✓ 所以只留开机一次 ✓ */

	Stat_t snap;
	uint32_t notify_val = 0;
    while (1)
    {
/* 超时 500ms 而不是 portMAX_DELAY：否则暂停时 KEY2 的通知永远轮不到看 */
if (xQueueReceive(xQueueRec, &rec, pdMS_TO_TICKS(500)) == pdPASS)
{
	char row[160];      /* CSV 一行最坏 **124 字节**：4×10 位(uint32 最坏值) + 8×6
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

	/* ★ 对【上面这段文本】算 CRC ✓（覆盖的正是"卡上那一行的前 15 列"✓）
	 *   ★★ 2026-10-19 ✗：**CRC-32 方案已废弃 ⇒ 退回 CRC-16/KERMIT** ✓
	 *     · 理由见 `Record_t` 里那段注释 ✓（硬件不省 RAM ✗ 软件收益太小 ✓）*/
	rec.crc = Crc16_Kermit((const uint8_t *)row, (uint16_t)rowlen);

	/* ★ 再把 crc 作为第 16 列追加 ✓（串口回显也跟着用它 ✓）
	 *   ★ 5 位十进制 ✓（16 位最大值 65535 ✓）*/
	rowlen += sprintf(row + rowlen, ",%u", (unsigned)rec.crc);

	/* ---- 第②步：保证手里有缓冲（数据落卡 > 一切，串口回显排后面）----
	 * 拿不到 = 两块都在 task_SD 手里写卡：丢本条保节拍，绝不阻塞等待
	 * （PROC 一阻塞就顶住 ACQ 的采集队列，违反架构前提） */
	if (s_cur == NULL)
	{
		if (SD_BufAcquire(&s_cur) != pdPASS)
		{
			s_drop++;
			Loss_Add(LOSS_NO_BUF, 1u);      /* ★ 拿不到缓冲 ⇒ 丢 1 条 ✓ */
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
		if (SD_LogEnqueue(s_cur, s_pos, s_nrec) != pdPASS)
		{
			/* ★ 2026-10-18 改：按【条】记账 ✗ 不是"加 1 次事件" ✓
			 *   ★ 这一批（s_nrec 条）全都进不了卡 ⇒ 全丢 ✓ */
			s_drop += s_nrec;
			Loss_Add(LOSS_ENQ_FAIL, s_nrec);
		}
		s_cur = NULL;
		s_pos = 0;
		s_nrec = 0;

		if (SD_BufAcquire(&s_cur) != pdPASS)    /* 新缓冲拿不到：丢本条 */
		{
			s_drop++;
			Loss_Add(LOSS_NO_BUF, 1u);          /* ★ 丢 1 条 ✓ */
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
		s_nrec++;                           /* ★ 2026-10-18：这批又多 1 条 ✓ */
	}
	else if (s_cur != NULL)
	{
		s_drop++;       /* 走到这 = 单行放不下（见上）：丢本条 */
		Loss_Add(LOSS_ROW_BIG, 1u);         /* ★ 丢 1 条 ✓（★ 理论不发生 ✓）*/
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
		if (SD_LogEnqueue(s_cur, s_pos, s_nrec) != pdPASS)
		{
			s_drop += s_nrec;                   /* ★ 尾批整批丢 ⇒ 按条记 ✓ */
			Loss_Add(LOSS_ENQ_FAIL, s_nrec);
		}
		/* 只在"确实交过手"之后才清空：交成功 = 所有权已转移给 task_SD；
		 * 交失败 = 缓冲已被接口回收进空闲池（或已置错）——两种结局都不
		 * 再属于我们。而 s_pos==0（申请了还没写）时绝不能清空：
		 * 拿了不还又不用 = 512B 缓冲永久泄漏，整条链慢慢停摆 */
		s_cur = NULL;
		s_pos = 0;
		s_nrec = 0;                         /* ★ 尾批交完 ⇒ 计数归零 ✓ */
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
			 *   长稳时 heapmin 保持稳定 = 无泄漏 ✓✓
			 *
			 * ★★★★★ 2026-10-17 更新：判据从" > 2KB "改成【不降 + ≥ 1KB 】✗✓
			 *   ★ 改的原因（★ 实测 ✓）✗
			 *     · 20.37 小时长稳实测：heap=2080 ✗ heapmin=【1456】✓
			 *     · 而这里一直写着" > 2KB " ✗ ⇒ 按字面【这条判据不通过】✓
			 *     · ★★ 但 heapmin【20 小时恒为 1456 不降】⇒ 真正要看的
			 *       【"泄不泄漏"】是【通过】的 ✓✓
			 *   ★★ 为什么"不降"比"绝对余量"更重要 ✗
			 *     · 绝对余量【取决于当次功能的临时开销】✗ 会随版本起落 ✓
			 *     · 而"降下去不再回升"才是【泄漏】的签名 ✓✓
			 *   ★★★ 为什么是 1KB 下限 ✗
			 *     · 2026-10-17 给 `Serial_Printf` 加了互斥量（★ 约 80~88 B ✓）
			 *        ⇒ heapmin 预计还要降一点 ✓
			 *     · 1KB 是【还能再分配几个小对象】的底线 ✓
			 *   ★ 判据 ✗：heapmin ≥ 1024 且【整段长稳内不降】⇒ 通过 ✓✓ */
			/* ★★ 2026-10-19 R3：`heap=` / `heapmin=` 两列【已删除】✓
			 *   · 为什么 ✗ 它们调 `xPortGetFreeHeapSize()` ✗ 而那正是
			 *     【最后一个把 heap_4.o 拉进链接的原因】✓
			 *   · ★★★ 删掉它们 ⇒ `heap_4.o` 整体被丢弃 ⇒ **省 1 KB RAM + ~960 B 代码** ✓✓
			 *   · ★ 想看"有没有人偷偷分配" ✗ 现在【不需要】了：
			 *     全静态 ⇒ 分配不了（`pvPortMalloc` 根本不在镜像里）✓✓ */
			Serial_Printf("STAT(key) acq=%lu proc=%lu full=%lu qmax=%lu drop=%lu i2c=%u i2c_max=%u sd_err=%u sd_last=%u\r\n",
			          (unsigned long)snap.acq_count,
                      (unsigned long)snap.proc_count,
                      (unsigned long)snap.queue_full,
                      (unsigned long)snap.q_max_depth,
                      (unsigned long)s_drop,
                      g_i2c_us_last, g_i2c_us_max,
                      (unsigned)g_sd_error, (unsigned)g_sd_last_err);
		}
	}

	/* 开机 3 秒后打一次诊断（只打一次）：任务状态表 + 栈水位 */
	/* ★ 开机 3 秒后打一次诊断（任务状态表 + 每个任务的栈水位）✓
	 *   ★★★ R4 定容就是靠它量的 ✗ 见上面注释 ✓ */
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
		/* ★★ 2026-10-19 R3：`heap=` / `heapmin=` 两列【已删除】✗
		 *   理由同上（★ 它们是 `heap_4.o` 最后的两根引用绳 ✓）
		 *   ★★ 现在 `STAT` 的 RAM 判据换成 `sdstk=`（SD 任务栈余量）
		 *      ✗ 而且全静态之后【不可能有"运行期分配"这件事】✓ */
		Serial_Printf("STAT acq=%lu proc=%lu full=%lu qmax=%lu drop=%lu i2c=%u i2c_max=%u sd_err=%u sd_last=%u sdstk=%u\r\n",
		          (unsigned long)snap.acq_count,
                  (unsigned long)snap.proc_count,
                  (unsigned long)snap.queue_full,
                  (unsigned long)snap.q_max_depth,
                  (unsigned long)s_drop,
				  g_i2c_us_last, g_i2c_us_max,
				  (unsigned)g_sd_error, (unsigned)g_sd_last_err,
				  (unsigned)uxTaskGetStackHighWaterMark(task_sd_handler));
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

		/* ★★★ 2026-10-18 加：有按键就记一笔"有人在看"✗ 供 task_DISP 超时黑屏用 ✓
		 *   ★ 本任务【禁止打印】✗ 这里只写一个 uint32 ✓ 不违反约定 ✓ */
		if (ev != 0u)
		{
			g_ui_act_tick = (uint32_t)xTaskGetTickCount();

			/* ★★★★ 黑屏时的【第一下按键只唤醒 ✗ 不执行功能】✓
			 *   · 判据 `g_oled_blanked` 由 task_DISP 维护 ✓
			 *   · ★★ 为什么要"吃掉"这一下 ✗：按键里有【暂停采集/安全关机】✗
			 *     用户只想看一眼屏 ⇒ 结果把记录停了 ✓ 而现场没人知道 ✓✓
			 *   · ★ 屏会在 ≤200ms 内自己亮起（task_DISP 下一帧 ✓）不用管 ✓ */
			if (g_oled_blanked != 0u)
			{
				ev = 0u;                /* ★ 这一下只唤醒，不往下面走 ✓ */
			}
		}

		/* ★★★★★ 2026-10-18 加：关机态【屏蔽 KEY1】✗✓（★ 复盘 §7 ★★ 第 5 条 ✓）
		 *
		 *  ★ 原来会怎样 ✗：
		 *    · 关机态下按一下 KEY1 ⇒ `g_pause` 翻回 0 ⇒ 【采集又起来了】✓
		 *    · ★★ 而 task_SD 正处在"已卸卷、不再排空队列"的状态 ✓
		 *    · ★★★ 于是数据【以 10 条/秒静默丢弃】✗
		 *      而屏上还写着 `SAFE TO OFF` ⇒ 【人以为在记，其实在丢】✓✓
		 *
		 *  ★ 为什么选择"直接不理"，而不是"顺手恢复记录" ✗：
		 *    · ★ 恢复记录是【电源键长按】的语义 ✗ 两个键做同一件事只会乱 ✓
		 *    · ★★ 而且此时 SD 未必挂得回来 ⇒ 又是静默丢数据 ✓
		 *    ⇒ ★ 关机态只认电源键 ✓✓ */
		if ((ev & KEY1_EVENT) && (g_shutdown_req == 0u) && (g_shutdown_done == 0u))
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

		/* ★★★★ 电源键（★ 长按 1 秒 ✗ 2026-10-13 深夜加 ✓）
		 *   ★ 它是一个【软开关】**✗✓：
		 *     · ★ 第一次长按 → 【安全关机】（★ 落盘 + 关文件 + 屏显 ✓）
		 *     · ★★ 再长按一次 → 【恢复记录】（★ 【不用断电】✓）✓✓
		 *
		 *   ★★ 为什么【本任务里只置标志 ✗ 什么都不干**✗✓：
		 *     · ★ 本任务【栈只有 128 字 ✗ 而且约定【禁止打印】】✓
		 *     · ★★ 【真正的落盘在 task_SD ✗ 真正的显示在 task_DISP】✓✓
		 *
		 *   ★★★ 为什么【req 和 done 两个都要判**✗✓：
		 *     · ★ `done` 是【最终状态】✗ 但它【要等 SD 落完盘才置位】（★ 最多 1 秒 ✓）
		 *     · ★★ 只看 done ✗：【关机过程中的那 1 秒里再长按 ✗
		 *       会被当成"再关一次"✗ 【什么都不会发生 ✗ 用户以为按键坏了】】✓✓
		 *     → ★★★ 【req 或 done 只要有一个在 ✗ 就算"已关机"✗ 再按就是恢复】✓✓ */
		if (ev & KEY_PWR_EVENT)
		{
			if ((g_shutdown_req != 0u) || (g_shutdown_done != 0u))
			{
				/* ★ 恢复 ✗：清请求 ✗ 【SD 任务会自己重挂 ✗ 不用写新代码】✓ */
				g_shutdown_req  = 0u;
				g_shutdown_done = 0u;
				g_pause         = 0u;           /* ★ 回到采集运行 ✓ */
				Buzz_Request(BUZZ_EV_KEY);
			}
			else
			{
				/* ★ 关机 ✗：★【先停采集 ✗ 再请求落盘】✗ 顺序不能反 ✓
				 *   （★ 先 `g_pause` 才能【保证不再产生新数据】✓）*/
				g_pause        = 1u;
				g_shutdown_req = 1u;
				Buzz_Request(BUZZ_EV_KEY);
			}
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
/* ★★★★ 请求升级（★ 2026-10-14 加 `force` 参数 ✗ 审计 G6 ✓）
 *   ★ `force = 0` ✗【备份失败就【中止升级】】✓（★ 默认 ✗ 安全 ✓）
 *   ★★ `force = 1` ✗【明知没有备份也继续】（★ 只有 PC 发 `U!` 才会走到这儿 ✓）*/
/* ★★★★★ 2026-10-18 加：升级 / 回退 / 备份期间的【专用界面】✗✓
 *
 *  ★ 为什么可以直接在这里画屏 ✗：
 *    · `task_DISP` 是 OLED 的【唯一所有者】✓
 *    · ★★ 而这几条命令的解析（`TimeSync_Handle`）【就跑在 task_DISP 里】✓✓
 *      （由帧尾的 `TimeSync_Poll()` 调进来 ✓）
 *    ⇒ ★ 所以这里画屏【不违反"唯一所有者"规矩】✓
 *
 *  ★★ 为什么每行【必须补齐 16 个字符】✗：
 *    · 本工程老规矩：写不满 16 列 ⇒ 【右边残留上一帧的字】✓
 *    · ★ 而升级界面【正好压在正常画面上】⇒ 不补满就露馅 ✓✓
 *
 *  ★★★ 为什么这个界面【特别重要】✗：
 *    · 存金副本要【2~3 秒】✗ 这期间整个 DISP 任务【卡住不动】✓
 *      → 屏上还停着"正常采集画面" ⇒ 【看起来像没反应 / 像死机】✓✓
 *    · ★★ 而这 2~3 秒【恰恰是最不能断电的时候】✓
 *
 *  ★★★★ 为什么【不做进度百分比】✗✓（★ 这是零成本的关键 ✓）：
 *    · 真正把固件搬进 Flash 的是【Bootloader】✗ 它【没有 OLED 驱动】✓
 *    · ★★ 但 SSD1306 的面板在【复位后仍显示最后一帧】✓
 *      ⇒ 所以这里画好的"升级中"，在 Boot 搬 112KB 的整个过程中
 *        【一直显示着】✓✓
 *    ★ 想要真进度条 ⇒ 要给 Boot 加 3KB 显示代码 ✗ 先不做 ✓
 *      （★ 而且会成为"第二份 OLED 实现"✗ 两份迟早不一致 ✓）*/
static void Ui_Msg(const char *l1, const char *l2, const char *l3, const char *l4)
{
	/* ★★★★★ 2026-10-19 加（★ 审计 A2 ✗ 黑屏下 `U`/`R`/`S` 零反馈 ✓）：
	 *   ★ 现象 ✗：屏在 30 秒后已经熄灭（★ `0xAE` + 电荷泵关 ✓）
	 *     · 这时从串口发 `U` / `R` / `S` ✗ 解析【照旧在跑】✓
	 *     · ★★ 但本函数【只是往 GDDRAM 写字】✗
	 *       ⇒ 【面板熄着 ✗ 写进去也看不见】✓✓
	 *     · ★★★ 而 `U` 的时序【等不起】✗：
	 *         `Ui_Msg()` → `printf` → `Golden_SaveCurrent()`（2~3 秒）
	 *         → `NVIC_SystemReset()` ✓
	 *       ⇒ 【根本轮不到下一帧去点亮它】✓✓
	 *     · ★★★★ 更要命 ✗：Bootloader【没有 OLED 驱动】✗
	 *       它只能靠"面板上【留着的那一帧】"✓
	 *       而那一帧【是熄灭的】⇒ 【升级全程黑屏 ✗ 用户零反馈】✓✓
	 *       （★ 而这恰恰是【最不能断电】的窗口 ✓）
	 *   ★ 修法 ✗：画之前【自己先把面板点亮】✓
	 *     · 不能靠黑屏状态机的"下一帧边沿"✗ 上面说了轮不到 ✓
	 *     · ★★ `task_DISP` 是 OLED 的【唯一所有者】✗
	 *       而本函数【只被 task_DISP 里的命令处理调用】✓ 不越权 ✓
	 *     · ★★★ 这里【不碰】那两个 `static` 状态变量 ✗
	 *       （★ 它们是 `task_DISP` 的私有 static ✗ 这里够不着 ✓）
	 *       而是【顺手续期 idle 计时】✗
	 *       ⇒ 下一帧状态机看到 `idle_ms ≈ 0` ✗ 自己就会走"唤醒边沿"✓
	 *         把 `s_blanked` / `g_oled_blanked` 一并归位 ✓✓
	 *       （★ `U` 没有下一帧 ✗ 但那时已经复位 ✗ 无所谓 ✓）
	 *   ★ 幂等性 ✗：已经亮着时再发一遍 `OLED_DisplayOn()` ✓
	 *     = 多两条 I2C 写 ✗ 无害 ✓（★ 只在发命令这一刻 ✓）*/
	g_ui_act_tick = (uint32_t)xTaskGetTickCount();
	OLED_DisplayOn();

	OLED_Clear();                               /* ★ 先清 ✗ 免残影 ✓ */
	OLED_ShowString(1, 1, (char *)l1);          /* ★ 每行都是 16 字符 ✓ */
	OLED_ShowString(2, 1, (char *)l2);
	OLED_ShowString(3, 1, (char *)l3);
	OLED_ShowString(4, 1, (char *)l4);
}

static void App_RequestUpgrade(uint8_t force)
{
	Serial_Printf("\r\nAPP: upgrade requested\r\n");

	/* ★ 等串口把话【发完】✗ 再往下走 ✓ */
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) { }

	/* ★★★★ 2026-10-19 加（★ 审计 A2 的另一半 ✓）：
	 *   ★ 为什么【必须在存备份之前】先画一屏 ✗：
	 *     · 下面 `Golden_SaveCurrent()` 要【2~3 秒】✗
	 *       而那条"STEP 2/2"的 `Ui_Msg()` 【在它后面】才执行 ✓
	 *     · ★★ 屏【早就黑透了】✗
	 *       ⇒ 这 2~3 秒【仍然是全黑 ✗ 用户看到"发了 U 一点反应都没有"】✓✓
	 *     · ★★★ 所以这里补一屏"第 1 步：正在存备份"✓
	 *       （★ 命名与下面的 `STEP 2/2` 对齐 ✗ 不另起一套 ✓）
	 *   ★ 代价 ✗：`U` 会连着看到两屏（各 2~3 秒）✗ 不闪 ✓ 正是想要的 ✓ */
	Ui_Msg("UPGRADING       ",
	       "STEP 1/2 BACKUP ",
	       "DO NOT POWER OFF",
	       "PLEASE WAIT ... ");

	/*==========================================================================
	 *  ★★★★ 【升级之前 ✗ 先把【当前这一版】存成备份】✗✓（2026-10-14 加 ✓）
	 *
	 *  ★★★ 为什么【必须在这里做**✗✓：
	 *    · ★ 【此刻正在跑的版本 ✗ 才是我们【确定它好】的版本】✓
	 *    · ★★ 一旦【Boot 开始收新固件 ✗ 【旧版本就被擦了 ✗ 来不及了】】✓✓
	 *
	 *  ★★ 为什么【同步做 ✗ 不丢给后台任务**✗：
	 *    · ★ 后台是【500ms 一页 ✗ 传完要 110 秒】✓
	 *    · ★★ 而【升级请求【马上】就要软复位 ✗ 【等不起】】✓✓
	 *
	 *  ★★★ 代价 ✗：★ 【本任务（DISP）会卡约 2~3 秒】**✓
	 *    · ★ 但【这是【用户主动要求升级】✗ 慢一点【完全可以接受】】✓
	 *    · ★★ 【而且【采集链优先级更高 ✗ 它抢占得到 CPU ✗ 不受影响】】✓✓
	 *
	 *  ⚠️ ★★★★ 【备份失败【也必须如实说出来】✗✓
	 *    【绝不能"静默地不带备份就升级"✗ 那是拿唯一的命去赌】✓✓
	 *========================================================================*/
	if (Golden_SaveCurrent() == 0u)
	{
		/* ★★★★ 审计 G6 ✗ 【默认中止】✗✓ */
		if (force == 0u)
		{
			Serial_Printf("APP: UPGRADE ABORTED (no backup).\r\n");
			Serial_Printf("APP: if you accept the risk, send `U!` to force.\r\n");
			return;                 /* ★★ 不复位 ✗ 【原地停在 App】✓ */
		}
		Serial_Printf("APP: forced upgrade WITHOUT backup (you asked for it)\r\n");
	}

	/* ★★ 解锁备份域 ✗✓
	 *   ★ 为什么【这里还要自己做一遍**✗：
	 *     · ★ `MyRTC_Init()` 里【确实解锁过】✓
	 *     · ★★ 但【那是【启动时】的事 ✗
	 *       而且【备份域访问【可能被锁回去】】✓
	 *     → ★★★ 【这里重来一遍 ✗ 最稳妥 ✗ 而且【几乎不花时间】】✓✓
	 *   ★★★★ 做法【照抄 `MyRTC.c:28-29`】✓ */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	PWR_BackupAccessCmd(ENABLE);

	/* ★ 2026-10-18 加：备份完了 ⇒ 屏上进第 2 步 ✓
	 *   ★★ 这一帧会【一直留到 Boot 把固件搬完】✓（面板复位不掉内容）
	 *     ⇒ 用户看到的是"等 PC 发固件"，而不是一片死寂 ✓ */
	Ui_Msg("UPGRADING       ",
	       "STEP 2/2 RESET  ",
	       "DO NOT POWER OFF",
	       "WAIT FOR PC ... ");

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
		/* ★ 2026-10-18 加：先画"升级中"✗ 再去做那 2~3 秒的备份 ✓
		 *   ★ 不画的话：屏上停着正常采集画面 ⇒ 【看起来像没反应】✓ */
		Ui_Msg("UPGRADING       ",
		       "STEP 1/2 BACKUP ",
		       "DO NOT POWER OFF",
		       "WAIT ...        ");

		App_RequestUpgrade(0u);     /* ★ 默认：备份失败就中止 ✓ */

		/* ★ 只有【中止】才会走到这里 ✗ 把屏恢复成正常画面 ✓
		 *   ★★ 必须 Clear ✗ 否则第 1/2 行右边会【残留 "GRADING" 之类】✓ */
		OLED_Clear();
		return;                     /* ★ 理论上不会返回 ✗ 但写上更安全 ✓ */
	}

	/* ★★★★ 强制升级（★ 审计 G6 的"逃生通道"✓ 2026-10-14 加 ✓）
	 *   ★ 为什么【必须留这个口子**✗✓：
	 *     · ★ 万一【W25Q64 坏了 / 还没焊**✗
	 *       【"备份失败就中止"会【把升级路【彻底堵死】】】✓✓
	 *     · ★★ 所以【要能绕过 ✗ 但【必须是用户【显式、知情】地绕过】】✓✓
	 *   ★★★ 为什么【用 `U!`**✗：★ 多打一个感叹号 ✗
	 *     【手不会"顺手"打出来 ✗ 心里会过一下】✓✓ */
	if ((n == 2u) && (s[0] == 'U') && (s[1] == '!'))
	{
		/* ★ 2026-10-18 加：强制升级也画屏 ✗ 而且【明说没有备份】✓
		 *   ★ 这句 `NO BACKUP!` 是【故意吓人的】**✗
		 *     用户选了 `U!` 就该在屏上也看到代价 ✓ */
		Ui_Msg("FORCED UPGRADE  ",
		       "NO BACKUP!      ",
		       "DO NOT POWER OFF",
		       "WAIT ...        ");

		App_RequestUpgrade(1u);     /* ★ 我知情 ✗ 接受"没有备份"的风险 ✓ */
		OLED_Clear();
		return;
	}

	/* ★★★★ 金副本的两条新命令（★ 2026-10-14 加 ✓ 完整设计见 `golden.h`）
	 *   · ★ **`R`** = 【回退到备份】（★ 置标志 + 软复位 ✗ 真正的恢复由 Boot 做 ✓）
	 *   · ★★ **`B`** = 【清掉备份 + 把当前版本记成黑名单】✓
	 *     （★ 用来【防止一个"有 bug 但能跑"的版本被当成备份存下去】✓）
	 *   ★★★ 为什么【也用单字母**✗：★ 和 `U` 同一个理由 ✗
	 *     【好敲 ✗ 而且【不会和 `T<10位数字>` 混】】✓✓
	 *   ⚠️ ★★ 【别担心和 Boot 的协议字母撞**✗：
	 *     · Boot 用 `C`/`D`/`X` ✓
	 *     · ★★ 但【那是【另一个阶段】✗ 【App 在跑的时候 Boot 根本不在】】✓✓ */
	if ((n == 1u) && (s[0] == 'R'))
	{
		/* ★ 2026-10-18 加：回退也画屏 ✓
		 *   ★ 回退同样是"复位 + Boot 搬 112KB"✗ 同样不能断电 ✓ */
		Ui_Msg("ROLLING BACK    ",
		       "RESTORE BACKUP  ",
		       "DO NOT POWER OFF",
		       "WAIT ...        ");

		Golden_RequestRollback();

		/* ★ 走到这里 = 【就地拒绝】（没有有效备份 ✗ 不复位）✓
		 *   ⇒ 把屏恢复成正常画面 ✓ */
		OLED_Clear();
		return;                     /* ★ 不会返回 ✗ 但写上更安全 ✓ */
	}
	if ((n == 1u) && (s[0] == 'B'))
	{
		Golden_ClearBackup();
		return;
	}

	/*==========================================================================
	 *  ★★★★★ `S` —— 手动宣告"这一版是好的 ✗ 存成备份"（★ 2026-10-14 加 ✓）
	 *
	 *  ★★★ 为什么需要它 ✗✓（★ 用户当场提出的需求 ✓）✗：
	 *    · ★ 原来【备份只会被【升级流程（`U`）】更新】✓
	 *      → ★ 语义 ✗【"升级前那一版 = 我们确定能跑的"】✓✓
	 *    · ★★ 但【用 Keil / ST-Link 烧进去的版本 ✗ 走不到那条路】✗
	 *      → ★★★ 【于是【备份【永远停在最早那一版】】】
	 *        → 【用户："我想让现在这版当金副本 ✗ 怎么办？"】✓
	 *
	 *  ★★ 语义 ✗：★ 这一版是【人【亲自确认过】的】✓
	 *    —— 【不是"跑了几分钟没死"✗ 而是【判断权在人手里】】✓✓
	 *
	 *  ★★★ 它做的事【和升级前那一遍【完全相同】】✗✓：
	 *    · 直接调 `Golden_SaveCurrent()` ✓
	 *    · 擦 64KB 块 → 逐页写 → 【最后写头】✓
	 *    → ★★ 【所以【中途掉电 ✗ 头没写 ✗ 旧备份【自动保留】】✓✓
	 *      （★ 绝不会出现"存到一半把好备份毁了" ✓）
	 *
	 *  ★★★★ 副作用 ✗：★【DISP 任务会卡约 2~3 秒】✓
	 *    · ★ 和 `U` 那条路一样 ✓【采集链优先级更高 ✗ 不受影响】✓✓
	 *    · ★★ 【期间 OLED 不刷新 ✗ 正常】✓
	 *
	 *  ⚠️ ★★ 注意 ✗：★ 它【不会】清黑名单 ✓
	 *    · ★ 刚发过 `B` 之后再发 `S` ✗ = 【把刚作废的那版又存回去】✓
	 *    · ★★ 【正确顺序 ✗ `B` → 【烧好版本】→ 它会【自己填进去】】✓✓
	 *========================================================================*/
	if ((n == 1u) && (s[0] == 'S'))
	{
		/* ★ 2026-10-18 加：存备份也要画屏 ✗
		 *   ★ 这一步同样要【2~3 秒】✗ 而且【正在擦 W25 的块】✓
		 *   ★★ 用户最容易在这时候以为"卡死了"然后去断电 ✓✓ */
		Ui_Msg("SAVING BACKUP   ",
		       "TO SPI FLASH    ",
		       "DO NOT POWER OFF",
		       "PLEASE WAIT ... ");

		(void)Golden_SaveCurrent();

		/* ★ 存完/失败都恢复画面 ✓（正常刷新会在 200ms 内重画满 4 行 ✓）*/
		OLED_Clear();
		return;
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


/* ★★★★★ 2026-10-17 加：用 RTC（LSE）校准 SysTick（HSE）✗ 不需要任何仪器 ✓
 *
 *  ★★★★ 为什么要它 ✗✓：
 *    2026-10-17 用【PC 时钟当裁判】量出来【系统时钟偏慢 ≈1900 ppm】✗
 *    （★ 三方比速率 ✗ PC 9.9811 / RTC 9.9822 / SysTick 10.0000 条每秒
 *      ⇒ 只有 SysTick 是离群的 ⇒ 偏的是 HSE 那一路 ✓✓）
 *    而 1900 ppm 远超石英晶振规格（±20 ppm）
 *    ⇒ ★★ 怀疑那颗 8MHz 是【陶瓷谐振器】✗ 不是石英晶振 ✓
 *
 *  ★★ 为什么要测【10 秒】✗：tick 的分辨率是 1 ms ✓
 *    · 测 1 秒 ⇒ 量化误差 ±1000 ppm ⇒ 【和要测的量同级 ⇒ 没用】✓
 *    · 测 10 秒 ⇒ 误差 ±100 ppm ⇒ 够用 ✓✓
 *
 *  ★★★ 为什么放在【本任务（task_DISP）】✗✓：
 *    · 它是【最低优先级 + 输出类任务】✗ 卡它 10 秒不伤数据链路 ✓
 *    · ★★ 绝对不能放 task_PROC ✗ 那里停 10 秒 = 【数据断流 10 秒】✓✓
 *    · ★ 等的时候用 `vTaskDelay` ✗ 不空转 ⇒ 不烧 CPU ✓
 *
 *  ★ 判据 ✗：|ppm| ≤ 100 ⇒ 合格 ✓ ／ > 500 ⇒ 去查晶振型号与负载电容 ✓ */
static void Clk_CalibOnce(void)
{
	static uint8_t s_done = 0u;
	uint32_t r0, k0, k1;
	int32_t  dms, ppm;

	if (s_done != 0u) { return; }
	s_done = 1u;

	/* ★ 先对齐到 RTC 的【秒沿】✗ 否则起点误差最大 1 秒 ✓ */
	r0 = MyRTC_GetUnix();
	while ((uint32_t)(MyRTC_GetUnix() - r0) < 1u) { vTaskDelay(pdMS_TO_TICKS(1)); }

	k0 = xTaskGetTickCount();
	r0 = MyRTC_GetUnix();
	/* ★ 再等 RTC 走满 10 秒 ✓ */
	while ((uint32_t)(MyRTC_GetUnix() - r0) < 10u) { vTaskDelay(pdMS_TO_TICKS(1)); }
	k1 = xTaskGetTickCount();

	/* ★ tick 配成 1ms/tick ⇒ 差值就是毫秒数 ✓ */
	dms = (int32_t)(k1 - k0);
	if (dms <= 0) { return; }

	/* ppm = (dms-10000)/10000 × 1e6 = (dms-10000) × 100 ✓ */
	ppm = (dms - 10000) * 100;

	Serial_Printf("CLK: over RTC 10s, tick advanced %ld ms (expect 10000) => %+ld ppm %s\r\n",
	              (long)dms, (long)ppm,
	              ((ppm > 100) || (ppm < -100)) ?
	                  "*** SYSTEM CLOCK OFF (check the 8MHz XTAL: crystal or ceramic resonator?)" : "OK");
}


/* task_DISP：① 先拷快照 ② 状态字定长防残影
 * ③ 第4行的心跳数字用来区分【暂停】和【卡死】——没有它这个验收项无法判定
 * ④ 第4行 13~16 列是 SD 错误码 E:xx（Y2）——OLED 从"坏了"升级到"为什么"
 * ★★ 2026-10-17 加 ⑤：时间不可信时第 4 行【常驻】显示 `T?` ✓ */
void task_DISP(void *pvParameters)
{
	Stat_t snap;
	uint32_t frame = 0;		/* 心跳计数：每帧 +1（用来证明屏是活的） */
	static uint8_t s_shutdown_seen = 0u;	/* ★ 关机长响【只响一次】的边沿标志 ✓ */

	       
	/* ★★★ 2026-10-14 加：刷屏耗时统计 ✓
	 *   ★ t0 用 DWT 周期数（★ 和你 task_ACQ 里那个写法一样 ✓）
	 *   ★★ s_us_max 只看【历史最大】✗ 平均会掩盖偶发卡顿 ✓✓ */
	uint32_t t0;
	uint32_t dt_us;
	uint8_t  did_refresh;  
	static uint32_t s_us_max = 0u;
	static uint32_t s_us_sum = 0u;
	static uint16_t s_cnt    = 0u;

	/* ★★ 空闲占比采样的"上次值"（★ 2026-10-18 加，低功耗改造 序 1 ✓）
	 *   · s_idle_last：上次快照时 `g_idle_cnt` 的值 ✗ 做差得"这段时间进了几次" ✓
	 *   · s_idle_tick：上次快照时的 tick ✗ 做差得"这段时间多长" ✓
	 *   ★ 两个必须【同时更新】✗ 否则算出来的速率是错的 ✓ */
	static uint32_t s_idle_last = 0u;
	static uint32_t s_idle_tick = 0u;
	static uint32_t s_idle_ms_last = 0u;   /* ★ §7.2 指标的上次快照 ✓ */

	/* ★★ 2026-10-18 加：超时黑屏的三个状态（★ 低功耗 序 3 ✓）
	 *   · s_blank     ：本帧【逻辑上】该不该黑 ✓
	 *   · s_blanked   ：屏【物理上】已经关掉了吗 ✓（用来做边沿，别每次都发命令 ✓）
	 *   · s_need_full ：下一帧要不要【整屏重画】✓
	 *     ★ 从黑屏恢复后【必须】重画 ✗ 因为 GDDRAM 里还是旧内容 ✓ */
	static uint8_t  s_blank     = 0u;
	static uint8_t  s_blanked   = 0u;
	static uint8_t  s_need_full = 0u;

	DWT_Delay_Init();       /* ★★ 保险：确保周期计数器在跑（重复调用无害 ✓）*/

	for (;;)
	{
		/* ★★★★★ 2026-10-18 校准挪到这里（★ 计时起点【之前】✓）
		 *
		 *  ★★★ 为什么必须在这 ✗✓：
		 *    · `Clk_CalibOnce()` 故意卡本任务约 10 秒（★ 等 RTC 走满 10 秒 ✓）
		 *    · 它若在 `t0 = DWT->CYCCNT` 之后 ⇒ 那 10 秒
		 *      【被算成"一帧的耗时"】✗ ⇒ `max` 被顶到 10.58 秒 ✓✓
		 *      （★ 2026-10-18 第一次烧上去就实测到了 ✓）
		 *    · 而 `s_us_max` 【不清零】⇒ 每次开机污染一次
		 *      ⇒ ★★ 判据「max 比 avg 重要」彻底作废 ✓✓
		 *  ★ 为什么阈值是 `frame == 200` ✗
		 *    · frame 每 200ms +1 ⇒ 约 40 秒 ✓
		 *    · 不在刚开机就量 ✗ 那时还在初始化 / 挂卡 ✗ 时机不干净 ✓
		 *  ★★ 位置在 `frame++` 之前 ⇒ 此处 `frame` 还是上一轮的值
		 *    ⇒ `== 200` 对应"第 201 轮开头" ✗ 时间上无差别 ✓ */
		/* ★★★ 2026-10-19 教训（★ 保留成注释 ✗ 不再留死代码 ✓）：
		 *   校准【绝不能放在 task_DISP 里】✗
		 *   · `Clk_CalibOnce()` 故意阻塞约 11 秒 ✓
		 *   · 放这里 ⇒ 【屏 11 秒不更新 + 蜂鸣器卡在最后状态】✗✓
		 *     （`Buzz_Poll()` 也在本任务里 ✓）
		 *   · ★★ 现象"看起来像拔卡导致的"✗ 其实是
		 *     `frame == 200` ⇒ 开机后 40~55 秒的固定窗口 ✓✓
		 *   ⇒ ★ 已搬到 `task_GOLDEN`（没人等它 ✓）*/

		/* ★★★ 帧计时【从这里开始】✓ */
		t0 = DWT->CYCCNT;
		did_refresh = 0u;   
		/* ① 快照 */
		taskENTER_CRITICAL();
		snap = g_stat;
		taskEXIT_CRITICAL();

		/* ★★ 心跳计数【两种状态都要走】**✗ 所以先加 ✓ */
		frame++;

		/* ★★★★★ 2026-10-18 加：OLED 超时黑屏判定（★ 低功耗改造 序 3 ✓）
		 *
		 *  ★ 为什么放在【屏幕的唯一所有者】里 ✗：
		 *    · `task_DISP` 是 OLED 的【唯一所有者】✗ 别人一律不许碰屏 ✓
		 *    · ⇒ 开关屏也只能在这里判 ✓✓
		 *
		 *  ★★★ 关机态【绝不黑屏】✗：
		 *    · `SAFE TO OFF` 是【唯一必须让人看见】的状态 ✓
		 *    · 屏一黑 ✗ 用户就不知道能不能断电了 ✓✓
		 *
		 *  ★★ 边沿判断（s_blanked）✗：只在【状态变化】的那一帧发命令 ✓
		 *     否则每 200ms 都发一遍 `0xAE` ✗ 白跑 I2C 时序 ✓
		 *
		 *  ★ 判据（烧进去看）✗：
		 *    · 静置 30 秒 ⇒ 屏灭，串口打 `OLED: blank`
		 *    · 按任意键 ⇒ 屏亮，串口打 `OLED: wake`，且【整屏立刻重画】✓ */
		{
			uint32_t now_tick;
			uint32_t idle_ms;

			now_tick = (uint32_t)xTaskGetTickCount();
			idle_ms  = now_tick - g_ui_act_tick;

			/* ★★★ 2026-10-18 加：这三种情况【强制亮屏】✗ 一个键都不占 ✓
			 *   ① `g_shutdown_done` ── SAFE TO OFF 必须看得见 ✓
			 *   ② `OLED_BLANK_MS == 0` ── 有人把黑屏关了（台面调试 ✓）
			 *   ③ `g_sd_error != 0` ── ★ SD 故障：无人值守时"数据存不进去"
			 *      必须【看得见】✓ 光靠蜂鸣器不够 ✓（★ 屏再黑着 = 彻底没提示 ✓）
			 *   ★★ 特别注意【不要】把"时间不可信 `T?`"加进来 ✗
			 *     本板没 VBAT ⇒ 每次上电都是 `T?` ⇒ 加进去屏就【永不黑】✓ */
			if ((g_shutdown_done != 0u) || (OLED_BLANK_MS == 0u) || (g_sd_error != 0u))
			{
				s_blank = 0u;                      /* ★ 关机 / 禁用 / SD 故障 ⇒ 常亮 ✓ */

				/* ★★★★ 2026-10-18 修：强制亮屏期间【持续续期】✗✓
				 *   （★ 用户实测抓到的 bug："恢复正常之后直接一下子黑屏"✓）
				 *
				 *  ★ 原来的错 ✗：`g_ui_act_tick` 【只有按键会刷新】✗
				 *     · SD 故障期间，屏靠上面这个条件【强制】亮着 ✓
				 *     · ★★ 但 30 秒计时【在后台一直往下跑】✗ 没人管它 ✓
				 *     · ⇒ 故障一解除 ⇒ `idle_ms` 早就 > 30000 ⇒ 【屏立刻又黑】✓✓
				 *       （★ 插回卡、关机态恢复 都会撞上 ✓）
				 *
				 *  ★ 改法 ✗：只要还在"强制亮屏"，就【当成刚刚有人看过】✓
				 *     ⇒ 强制条件解除后，仍然有【完整 30 秒】✓✓
				 *
				 *  ★ 为什么由 task_DISP 写这个全局也安全 ✗：
				 *     · 写的是 uint32 ⇒ CM3 上单条 STR ✗ 天然原子 ✓
				 *     · 另一个写者是 task_KEY ✗ 两边都只是"更新成当前 tick"✓
				 *       ⇒ 谁后写都无害 ✓ */
				g_ui_act_tick = now_tick;
			}
			else
			{
				s_blank = (idle_ms >= OLED_BLANK_MS) ? 1u : 0u;
			}

			if ((s_blank != 0u) && (s_blanked == 0u))
			{
				OLED_DisplayOff();                 /* ★ 连电荷泵一起关 ✓ */
				s_blanked   = 1u;
				g_oled_blanked = 1u;               /* ★ 告诉 task_KEY：第一下只唤醒 ✓ */
				s_need_full = 1u;                  /* ★ 恢复时要重画 ✓ */
				/* ★★★ 2026-10-19 修（★ 审计 A3 ✓）：
				 *   · 原来这里打的是【阈值常量 `OLED_BLANK_MS`】✗
				 *     ⇒ 每次都打 `30000` ✗【量不到"实际闲了多久"】✓✓
				 *   · ★★ 而低功耗实验【恰恰要看这个数】✗
				 *     打常量 = 【把判据自己污染掉】✓
				 *   · ★ 现在打【真实的 `idle_ms`】✓ */
				Serial_Printf("OLED: blank (%lu ms idle)\r\n",
				              (unsigned long)idle_ms);
			}
			else if ((s_blank == 0u) && (s_blanked != 0u))
			{
				OLED_DisplayOn();
				s_blanked   = 0u;
				g_oled_blanked = 0u;
				s_need_full = 1u;
				Serial_Printf("OLED: wake\r\n");
			}
		}

		/* ★★★★ 2026-10-19 加：SD 故障期【每秒做一次 OLED 总线恢复】✗✓
		 *
		 *  ★ 为什么 ✗（★ 实测驱动 ✓）：
		 *    · 拔卡期间 `oled` 计数【稳定上涨】⇒ 每帧都画了 ✓
		 *    · ★★ 但屏上数字【冻住 11 秒】⇒ 【面板没接受写入】✓✓
		 *    · ★★★ 怀疑：I2C 从机被留在"事务中途"（SDA 被拉住）
		 *      ⇒ 之后【每一次写入都失败】✓
		 *    · 驱动里【本来就有】`OLED_BusRecover()`（9 时钟 + Stop）
		 *      但【只在初始化时调过一次】✗✓
		 *  ★ 做法 ✗：故障期每秒恢复一次 + 强制整屏重画 ✓
		 *    · 一次最多 1 秒的屏滞后 ✗ 换来"面板一定回得来"✓✓
		 *  ★ 为什么【只在故障期】做 ✗：
		 *    · 正常时 I2C 没有任何被打断的压力 ✗ 不做也无害 ✓
		 *    · ★★ 而且恢复动作会重发面板命令 ✗ 正常期没必要增加总线负载 ✓ */
		/* ★★★ 2026-10-19 教训（★ 注释保留 ✗ 死代码已删 ✓）：
		 *   曾加过"SD 故障期每秒做一次 OLED 总线恢复"✗
		 *   · 依据是"屏冻住 = I2C 从机被卡在事务中途"✗
		 *   · ★★ 实测推翻：`sda/scl` 全程高电平（总线从未被拉低）✓
		 *     真因 = `Clk_CalibOnce()` 阻塞 DISP 11 秒 ✓
		 *   · ★★★ 而且它有副作用：每秒重发 `0x8D 0x14`
		 *     （电荷泵 关→开）⇒ 面板【瞬间灭再亮】
		 *     ⇒ 用户实测"屏幕切换时闪得挺明显"✓✓
		 *   ⇒ 调用点和这个函数【都已删除】✓
		 *     （★ 驱动里原有的 `OLED_BusRecover()` 仍在 ✗
		 *       只在 `OLED_Init` 里用一次 ✓ 那是它本来该在的位置 ✓）*/

		/* ★★★★★ 2026-10-17 加：第一帧先【整屏清一次】✗✓
		 *   ★ 为什么需要 ✗：`main.c` 启动时会在屏上打 4 行警告
		 *     （★ "RTC NOT SET / RUN time_sync / …" ✓）
		 *   ★★ 而本任务的正常画面【只写固定那几列】✗
		 *     例：第 1 行只写 1~10 列（"ACQ:" + 6 位数字 ✓）
		 *     ⇒ 警告残留在 11~16 列 ⇒ 【像多出几个怪字符】✓✓
		 *     （★ 实测：第 1 行末尾冒出 `ET!` ✗ 第 2 行末尾冒出 `ync`
		 *        —— 正是 ` RTC NOT SET !` 与 `RUN time_sync` 的尾巴 ✓）
		 *   ★★★ 这就是本项目"状态字定长防残影"规矩的【另一面】✗
		 *     · 老代码守的是"自己写的东西【自己写满】"✓
		 *     · ★★ 而【别人写过的内容】必须【有人负责清】✓✓
		 *     ⇒ 在【接管屏幕的那一帧】清一次 ✗ 最省事也最保险 ✓ */
		if ((s_blank == 0u) && ((frame == 1u) || (s_need_full != 0u))) { OLED_Clear(); }

		/* ★★★★ 安全关机态 → 【整屏重画】**✗✓（★ 2026-10-13 深夜加 ✓）
		 *
		 *  ★ 为什么【必须整屏重画**✗✓：
		 *    · ★ 第 4 行【16 列已经占满】（★ `PAUSE` + 心跳 + `E:xx` ✓）
		 *    · ★★ 【"SAFE TO OFF" 【硬塞塞不进去】】✓✓
		 *  ★★ 所以【每一行都写【正好 16 个字符】】**✗✓
		 *    · ★ 写不足 16 ✗ → 【右边【残留上一屏的数字】✗ 看起来像乱码】✓✓
		 *
		 *  ★★★ 心跳【保留在**✗（★ 第 4 行 ✓）✗：
		 *    · ★ 区分【"安全停机"和"死机"】✓
		 *    · ★★ 屏上数字还在跳 ✗ 【说明板子活着 ✗ 只是故意不记了】✓✓
		 *
		 *  ★★★★ 长响【只在"刚进入这一态"的那一帧响一次】✗✓
		 *    · ★ 不加边沿判断 ✗ → 【每 200ms 请求一次 ✗ 一直响】✓✓ */
		if (g_shutdown_done != 0u)
		{
			/* ★★★★★ 2026-10-18 改：屏必须【说实话】+ 关机态【别每帧刷】✗✓
			 *
			 *  ★ 改动一：`DATA SAVED` 改成【有条件的】✗
			 *    · 原来【无条件】写 `DATA SAVED` ✗
			 *    · ★★ 而 `g_shutdown_saved` 才代表"文件真的关成功了"
			 *      → 写失败、没插卡、从没挂载过 ⇒ 【一个字节都没进卡】✓✓
			 *    ★★★ 这是复盘里最让人难受的一条 ✗：
			 *      没卡的时候按电源键，屏上写着"数据已保存，可以断电"✓
			 *
			 *  ★ 改动二：静态那 3 行【只画一次】✗ 心跳【1 秒刷一次】✗
			 *    · 原来【每一帧都全屏重画】✗
			 *      → 帧周期 = 200 + 85 = 285ms ⇒ 3.5 次/秒 × 85ms
			 *      → 【30% CPU】✗ 而正常态才 17.5% ✓✓
			 *    · ★★ 于是"关机态最省电"这句话【实际是反的】✓
			 *    ⇒ 现在：进态清一次屏 + 只重画心跳那 5+10 个字符
			 *      ⇒ 30% → 【约 2%】✓✓ */
			if (s_shutdown_seen == 0u)
			{
				s_shutdown_seen = 1u;

				OLED_Clear();                            /* ★ 清掉上一屏残留 ✓ */
				OLED_ShowString(1, 1, "SHUTDOWN        ");   /* ★ 8 + 8 空格 = 16 ✓ */

				if (g_shutdown_saved != 0u)
				{
					OLED_ShowString(2, 1, "DATA SAVED      ");   /* ★ 10 + 6 = 16 ✓ */
					OLED_ShowString(3, 1, "SAFE TO OFF     ");   /* ★ 11 + 5 = 16 ✓ */
				}
				else
				{
					OLED_ShowString(2, 1, "NOT SAVED !!    ");   /* ★ 12 + 4 = 16 ✓ */
					OLED_ShowString(3, 1, "CHECK SD CARD   ");   /* ★ 13 + 3 = 16 ✓ */
				}

				Buzz_Request(BUZZ_EV_LONG);              /* ★ 1.6 秒长响 ✓ */

				if (g_shutdown_saved != 0u)
				{
					Serial_Printf("SYS: shutdown done -> SAFE TO POWER OFF\r\n");
				}
				else
				{
					Serial_Printf("SYS: shutdown done but ** DATA NOT SAVED ** -> check SD card\r\n");
				}
			}

			/* ★ 心跳：1 秒刷一次（★ 只重画第 4 行，约 20ms ✓）
			 *   ★★ 为什么还要留心跳 ✗：区分"安全停机"和"死机"✓ */
			if ((frame % 5u) == 0u)
			{
				OLED_ShowNum   (4, 1, frame, 5);
				OLED_ShowString(4, 7, " OFF      ");         /* ★ 10 字符 = 第 7~16 列 ✓ */
				did_refresh = 1u;
			}
		}
		else
		{
			s_shutdown_seen = 0u;                        /* ★ 恢复时重新武装 ✓ */
			if ((((frame % 2u) == 0u) || (s_need_full != 0u)) && (s_blank == 0u))
			{
			/* 刷新（OLED 16 列 x 4 行）：FUL=队列满次数  QMX=历史最大积压 */
			OLED_ShowString(1, 1,  "ACQ:");  OLED_ShowNum(1, 5,  snap.acq_count,    6);
			OLED_ShowString(2, 1,  "PRC:");  OLED_ShowNum(2, 5,  snap.proc_count,   6);
			OLED_ShowString(3, 1,  "FUL:");  OLED_ShowNum(3, 5,  snap.queue_full,   3);
			OLED_ShowString(3, 9,  "QMX:");  OLED_ShowNum(3, 13, snap.q_max_depth,  2);
			/* ★★★★ 2026-10-17 改：时间不可信时【常驻】显示 `T?` ✗✓
			 *   · 原来只有 RUN / PAUSE 两个状态 ✓
			 *   · ★★ 为什么要常驻 ✗：开机那 4 行警告【0.4 秒就被本任务刷掉了】✗
			 *     而记录仪是【无人值守】的 ⇒ 必须有个【一直看得见】的记号 ✓✓
			 *   · ★ 判据 ✗：`T?` = 本次上电后【没对过时】✗ 数据时间戳不可信 ✓ */
			OLED_ShowString(4, 1, g_pause ? "PAUSE" :
			                        ((MyRTC_IsTrusted() != 0u) ? "RUN  " : "T?   "));
			/* ★ 心跳：用来区分【暂停】和【卡死】 */
			OLED_ShowNum(4, 7, frame, 5);

			/* ★ Y2：SD 错误码上浮展示（第 4 行 13~16 列，"E:xx"格式）
			 *   码表（= FRESULT 值）：E:00 无错误史；E:01 FR_DISK_ERR（写中拔卡）；
			 *   E:03 FR_NOT_READY（没插卡/接触不良）；E:07 FR_DENIED（卷/根目录满）；
			 *   E:13 FR_NO_FILESYSTEM（卡没格式化）。
			 *   挂载成功时码随 g_sd_error 一起清 0 —— E:00 即"健康"。
			 *   心跳占 7~11 列，这里从 13 列起正好不重叠（16 列屏用满） */
			OLED_ShowString(4, 13, "E:");
			OLED_ShowNum(4, 15, g_sd_last_err, 2);
			did_refresh = 1u;                            /* ★★★ 正常画面【每 3 帧】刷一次 ✓ */
			s_need_full = 0u;                            /* ★ 重画完了 ⇒ 清标记 ✓ */
			}
		}

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

		/* ★★★★ 2026-10-18 从这里【移走】：校准不能放在 t0 之后 ✗✓
		 *   ★ 原因 ✗ 它故意卡本任务约 10 秒（★ 等 RTC 走满 ✓）
		 *     而下面那行的位置在 `t0 = DWT->CYCCNT` 之后 ⇒
		 *     【这 10 秒被当成"一帧的耗时"】✓
		 *   ★★ 后果（★ 2026-10-18 烧完第一次实测就撞上了 ✓）✗
		 *     `DISP: avg=82044 us  max=10581722 us` ⇒ 【max = 10.58 秒】✓
		 *     而 `s_us_max` 【不清零】⇒ 每次开机 max 都被顶到 ~10 秒
		 *     ⇒ ★★ 判据「max 比 avg 重要」【直接作废】✓✓
		 *   ★★★ 已挪到【循环开头 ✗ `t0 = DWT->CYCCNT` 之前】
		 *     （★ 见本函数 `for (;;)` 的第一句 ✓✓）*/

			/* ★★★★ 2026-10-14 改：只统计【真的刷了屏】的那些帧 ✓
		 *   ★ 为什么 ✗：现在每 3 帧才刷一次 ✗
		 *     另外两帧只花几十µs ✓ 全算进去会把 avg 拉得很低 ✗ 看不出真实开销 ✓
		 *   ★★ 用 did_refresh 标记 ✗【不用 frame % 3 去猜】✓
		 *       因为【关机画面那一支是每帧都刷的】✗ 猜不出来 ✓✓
		 *   ★★★ 想让汇总还是"每 5 秒一次"✗ 25 改成 8 ✓
		 *        （★ 8 次刷新 × 3 帧 × 200ms ≈ 4.8 秒 ✓）*/
		if (did_refresh != 0u)
		{
		/* ★★★ 2026-10-14 加：帧结束计时 + 统计 ✓
		 *   ★ 注意：下面的打印【不算进帧时间】✗ 所以要放在 dt_us 之后 ✓ */
		dt_us = (DWT->CYCCNT - t0) / 72u;      /* 72MHz：1µs = 72 周期 ✓ */
		if (dt_us > s_us_max) { s_us_max = dt_us; }
		s_us_sum += dt_us;
		s_cnt++;

		/* ★★★ 平时【安静】✗ 只在"这一帧明显偏慢"时才报 ✓
		 *   （★ 115200 下发一行约 30 字符 ≈ 2.6ms ✗ 每帧都报会污染测量 ✓）
		 *   ★★★★ 2026-10-17 改：阈值 60ms → 120ms ✗✓
		 *     · ★ 60ms 那个阈值【早就失效了】✗ 因为 OLED 优化（148→84ms）之后
		 *       【常态帧耗时就是 64~84 ms】✓
		 *     · ⇒ 【每一帧都触发】✗ 20.37 小时长稳刷了 147,238 条 ✓✓
		 *     · ★★ 而它原本的设计意图是"平时安静 ✗ 只在明显偏慢时报" ✓
		 *     · ★★★ 新阈值 120ms 的依据 ✗
		 *        常态 max ≈ 84~98 ms ✗ 留约 25% 余量 ⇒ 120 ms ✓
		 *        （★ 超过它 = 真的被谁拖了 ✗ 值得看一眼 ✓✓）*/
		if (dt_us > 120000u)
		{
			Serial_Printf("DISP: SLOW frame %u us !\r\n", (unsigned)dt_us);
		}

		/* ★★ 每 8 次刷新（= 5 秒）报一次汇总 ✗ 看趋势用 ✓ */
		if (s_cnt >= 8u)
		{
			Serial_Printf("DISP: avg=%u us  max=%u us  (per refresh)\r\n",
			              (unsigned)(s_us_sum / s_cnt), (unsigned)s_us_max);
			s_cnt    = 0u;
			s_us_sum = 0u;
			/* ★ s_us_max【不清】✗ 保留历史最大 ✓（看最坏情况 ✓）*/
		}
		}

		/* ★★★★ 2026-10-18 加：空闲占比（★ 低功耗改造 序 1 的判据 ✓）
		 *
		 *   ★★★ 为什么【从上面那个 `did_refresh` 块里挪出来】✗✓
		 *     （★ 2026-10-18 加黑屏后【自己踩到的坑】✓）
		 *     · 它原来在 `if (did_refresh != 0u)` 里面 ✗
		 *     · ★★ 而屏一黑就【不再刷屏】⇒ `did_refresh` 恒 0
		 *       ⇒ 【这一整块统计全停】✗ 恰恰是做省电实验时最需要看它的时候 ✓✓
		 *     ⇒ ★ 挪到【循环末尾 ✗ 刷不刷屏都跑】✓
		 *
		 *   ★ 换算依据 ✗：`__WFI()` 生效后，钩子每进一次 ≈ 睡到下一个
		 *     1ms SysTick ⇒ 【次数/秒 ÷ 1000 ≈ 空闲占比】✓
		 *   ★★ 为什么【先除后除】而不是 `dn * 1000 / dms` ✗：
		 *     · 没加 WFI 时 dn 可能是【每秒几十万】(空转 ✗)
		 *     · `dn * 1000` 会【溢出 uint32】✗ ⇒ 算出个假数 ✓✓
		 *   ★ 判据 ✗：
		 *     · WFI 生效 ⇒ 从"上万"掉到【几百】✓
		 *     · 500/s ⇒ 约 50% 空闲 ✓（和 CPU 预算吻合 ✓）
		 *     · 若还是上万 ⇒ ★ WFI 没起作用 ✗ 回头查钩子有没有真被调用 ✓
		 *
		 *   ★★ 代价（说清楚）✗：现在【每秒一行】✗ 约 30 字节 ≈ 2.6ms/s ✓
		 *      占 CPU 0.26% ✗ 可忽略 ✓ 换来的是"黑屏时也看得见"✓ */
		{
			TickType_t now = xTaskGetTickCount();
			uint32_t   dms = (uint32_t)(now - s_idle_tick);
			if (dms >= 1000u)
			{
				uint32_t dn    = (uint32_t)(g_idle_cnt - s_idle_last);
				uint32_t per_s = (uint32_t)(dn / (dms / 1000u));
				/* ★ 归一化成【每 1000ms 有多少 ms 闲过】✓
				 *   ★ 先乘后除可能溢出 ⇒ 先取商再补 ✗ 这里 ms ≤ 4000、×1000 安全 ✓ */
				uint32_t ms_raw = (uint32_t)(g_idle_ms - s_idle_ms_last);
				uint32_t ms_per = (uint32_t)((unsigned long)ms_raw * 1000u / (unsigned long)dms);
				Serial_Printf("IDLE: %lu/1000 ms  hook %lu/s  blank %u\r\n",
				              (unsigned long)ms_per, (unsigned long)per_s,
				              (unsigned)s_blanked);
				s_idle_last    = g_idle_cnt;
				s_idle_ms_last = g_idle_ms;
				s_idle_tick    = now;
			}
		}

		vTaskDelay(pdMS_TO_TICKS(200));
	}
}
	


/*==============================================================================
 *  ★★★★ task_GOLDEN：金副本后台备份（★ 2026-10-14 加 ✓）
 *
 *  ★ 为什么【任务体这么薄**✗✓：
 *    · ★ 真正的逻辑（★ 判断要不要填 / 擦块 / 逐页写 / 最后写头 ✓）
 *      【全在 `golden.c` 里 ✗ 【这个任务体【只是个壳】】】✓✓
 *    ★★ 为什么【要【单独开一个任务】**✗ 不塞进现有的 ✗：
 *      · ★ 它要【主动让出 CPU ✗ 每页停 500ms】✓
 *      · ★★ 【塞进 DISP → OLED 刷新会被拖慢 ✗
 *        塞进 SD → 落盘会被拖慢】✓✓
 *      → ★★★ 【独立一个最低优先级任务 ✗ 【谁都不影响】】✓✓
 *============================================================================*/
void task_GOLDEN(void *pvParameters)
{
	(void)pvParameters;

	/* ★★★★★ 2026-10-19 加：**时钟校准搬到这里做** ✗✓
	 *   ★ 为什么搬 ✗：它故意阻塞约 11 秒（等 RTC 走满 10 秒 ✓）
	 *     · 原来在 `task_DISP` 里 ✗ ⇒ 【屏冻 11 秒 + 蜂鸣器卡住】✓✓
	 *       （★ 实测：IDLE 空档 11.36 秒，用户反复报告 ✓）
	 *   ★★ 为什么放这里 ✗：
	 *     · 本任务优先级 1 ✗ 本来就"闲睡 5 秒"✓ 无时限要求 ✓
	 *     · ★ 它唯一的活是金副本后台填 ✗ 晚 11 秒毫无影响 ✓✓
	 *   ★★★ 为什么先等 30 秒 ✗：
	 *     · 开机初期 SD 挂载 / 金副本判断都在跑 ✗ 别插队 ✓
	 *     · ★★ 而且此时 RTC 早已对过时 / 走稳 ✓ 测出来更准 ✓
	 *   ★ 注意 ✗：`Clk_CalibOnce()` 内部有 `s_done` 保护
	 *     ⇒ 只跑一次 ✓（这里也只调一次 ✓ 双保险 ✓）*/
	vTaskDelay(pdMS_TO_TICKS(30000));
	Clk_CalibOnce();

	for (;;)
	{
		Golden_Task();          /* ★ 内部自己 `vTaskDelay` ✗ 闲着睡 5 秒 ✓ */
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

#if (configUSE_MALLOC_FAILED_HOOK == 1)
/* ★★ 2026-10-19 R3：整个钩子【包在宏里】✗ `configUSE_MALLOC_FAILED_HOOK=0` 时它消失 ✓
 *   ★ 为什么必须包 ✗ 不包的话它仍会引用 `xPortGetFreeHeapSize()` ✗
 *     而 ARM 链接器【按 .o 段】丢弃（不是按函数）✗
 *     ⇒ `freertos_demo.o` 被别处用到 ⇒ 这个函数【也会被保留】⇒ 堆仍然活着 ✗✗
 *   ★★ 全静态之后【没有 malloc 可失败】⇒ 这个钩子本来就没意义 ✓ */
void vApplicationMallocFailedHook(void)
{
	Serial_Printf("\r\n!!! MALLOC FAILED: free=%u, min_ever_free=%u !!!\r\n",
				  (unsigned)xPortGetFreeHeapSize(),
				  (unsigned)xPortGetMinimumEverFreeHeapSize());
	taskDISABLE_INTERRUPTS();
	while(1){}
}
#endif /* configUSE_MALLOC_FAILED_HOOK */

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
	/* ★★★★ 2026-10-18 加（低功耗改造 序 1 ✓）：空闲就【睡】，别空转 ✓
	 *
	 *  ★ 为什么是这里 ✗：空闲任务是"系统确实没事干"的【唯一可靠信号】✓
	 *    别的地方睡都会撞上"其实还有活没干"✗
	 *
	 *  ★★ 为什么只敢 `__WFI()` ✗ 不敢上停止模式 ✓：
	 *    · `__WFI()` = 停【内核时钟】，等任意中断唤醒 ✓
	 *      最近的唤醒源就是【1ms 的 SysTick】⇒ 最多睡 1ms ✓
	 *    · ★★★ Sleep 下 SysTick【照跑】✓ ⇒ tick 不丢 ✓
	 *      ⇒ 【所有任务的周期一个 tick 都不变】✓✓（含 100ms 的 ACQ ✓）
	 *    · ✗✗ 换成 `PWR_EnterSTOPMode` 就死锁：
	 *      Stop 停掉内核时钟 ⇒ SysTick 不再产生 ⇒ FreeRTOS 心跳没了
	 *      ⇒ 所有 `vTaskDelay` 永远等下去 ✓（`低功耗改造方案.md` §九 已列为禁止 ✓）
	 *
	 *  ★★ 喂狗和睡眠【不冲突】✓：
	 *    睡 ≤1ms 就醒 ⇒ 醒来又进这个钩子 ⇒ 喂狗照旧 ✓
	 *    IWDG 超时 6.55 秒 vs 这里 1ms ⇒ 差【三个数量级】✓✓
	 *    （★ 所以 IWDG × 回退炸弹【只在停止模式成立】✗ 与 WFI 无关 ✓）*/
	g_idle_cnt++;                   /* ★ 先记一笔：这一步证明钩子真被调到 ✓ */

	IWDG_ReloadCounter();           /* ★ 空闲任务能跑到 = 系统还活着 ✓ */

	/* ★★ §7.2 的指标：同一个 tick 里进多次【只记一次】✓
	 *   ★ 用 `!=` 而不是 `>`：tick 是 uint32 会回绕 ✗ `!=` 天然安全 ✓ */
	{
		uint32_t now = (uint32_t)xTaskGetTickCount();
		if (now != g_last_idle_tick)
		{
			g_last_idle_tick = now;
			g_idle_ms++;
		}
	}

	__WFI();                        /* ★★ 睡到下一个中断（≤1ms）✓ */
}
