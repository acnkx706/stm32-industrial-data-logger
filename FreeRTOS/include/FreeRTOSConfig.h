#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* ============================================================================
 * FreeRTOSConfig.h —— FreeRTOS 的用户配置文件（"用户覆盖层"）
 *   适配：STM32F103C8T6（20KB RAM / 64KB Flash）+ 江协标准库 + Keil
 *   规则：这里只写"你要改的"配置项；没写的项由内核 FreeRTOS.h 里的默认值兜底
 *   （所以找不到某个宏时，去 FreeRTOS.h 里搜，那里有 #ifndef 默认值）
 * ============================================================================ */

#include <stdint.h>
extern uint32_t SystemCoreClock;   /* 在 system_stm32f10x.c 中定义，默认 72MHz */

/* ======================== 一、调度器基础配置 ======================== */

/* 是否使用抢占式调度
 *   1 = 抢占式：高优先级任务一就绪，立刻抢走 CPU（RTOS 的常态，本工程用这个）
 *   0 = 协作式：任务必须主动让出（调用 vTaskDelay / 等事件）才会切换 */
#define configUSE_PREEMPTION                     1

/* 是否使用时间片
 *   1 = 同一优先级的多个就绪任务，每个 tick 轮流执行一次
 *   0 = 同优先级任务不自动轮换（先跑的会一直跑下去） */
#define configUSE_TIME_SLICING                   1

/* 是否用"硬件指令"挑选下一个要运行的任务（Cortex-M3 有前导零指令 CLZ）
 *   1 = 用 __clz 一步算出最高优先级（更快，CM3 推荐）
 *   0 = 用软件循环查找（更慢，但内核代码稍小）
 *   注意：置 1 时 configMAX_PRIORITIES 必须 ≤ 32 */
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  1

/* 是否启用"无节拍空闲"低功耗模式（Tickless Idle）
 *   0 = 不用（空闲任务照常每个 tick 醒来一次）
 *   1 = 空闲时关掉 SysTick 进入低功耗，实现复杂，需要额外回调 */
#define configUSE_TICKLESS_IDLE                  0

/* CPU 主频（Hz）—— 内核用它计算"1 个 tick 需要多少个计数"
 *   这里用 SystemCoreClock 变量（system_stm32f10x.c 里定义），本工程为 72MHz */
#define configCPU_CLOCK_HZ                       (SystemCoreClock)

/* 系统节拍频率（Hz）= 每秒多少个 tick
 *   1000 表示 1 tick = 1ms（所以 vTaskDelay(500) 就是延时 500ms）
 *   ?? 记住：vTaskDelay 的单位永远是 tick，不是毫秒！ */
#define configTICK_RATE_HZ                       (1000)

/* 优先级数量（可用优先级 0 ~ configMAX_PRIORITIES-1）
 *   数值越大优先级越高；0 号优先级保留给空闲任务
 *   这个值越大，内核占用的 RAM 越多（每个优先级对应一条就绪链表） */
#define configMAX_PRIORITIES                     (32)

/* 最小栈大小（单位：**字**，1 字 = 4 字节）
 *   128 字 = 512 字节；空闲任务就用这个大小
 *   ?? 单位是"字"不是"字节"，写小了会栈溢出 */
#define configMINIMAL_STACK_SIZE                 ((unsigned short)128)

/* 任务名的最大长度（含结尾的 '\0'）
 *   16 表示任务名最多 15 个字符，超长会被静默截断（不报错） */
#define configMAX_TASK_NAME_LEN                  (16)

/* tick 计数变量用多少位
 *   0 = 32 位（推荐，溢出周期约 49.7 天）
 *   1 = 16 位（约 65 秒就溢出，极少使用） */
#define configUSE_16_BIT_TICKS                   0

/* 空闲任务是否主动让出 CPU 给同优先级的任务
 *   1 = 让出（避免空闲任务和用户任务抢 CPU，推荐保持 1） */
#define configIDLE_SHOULD_YIELD                  1

/* ======================== 二、同步原语（信号量/互斥量） ======================== */

/* 是否支持互斥量 Mutex（带优先级继承，用来解决"优先级翻转"问题） */
#define configUSE_MUTEXES                        1

/* 是否支持递归互斥量（同一个任务可以重复获取同一把锁而不死锁） */
#define configUSE_RECURSIVE_MUTEXES              1

/* 是否支持计数型信号量（既能当"事件计数"，也能当"资源计数"） */
#define configUSE_COUNTING_SEMAPHORES            1

/* 是否支持队列集（可以一次阻塞等待多个队列/信号量，实际项目用得少） */
#define configUSE_QUEUE_SETS                     1

/* 队列注册表大小：可以给队列起名字登记，调试时能看到队列名
 *   0 = 关闭登记（省 RAM）；8 = 最多登记 8 个队列 */
#define configQUEUE_REGISTRY_SIZE                8

/* 是否支持任务通知（Task Notification）
 *   比信号量更轻量、更快的任务间通信方式，每个任务自带一个 32 位通知值 */
#define configUSE_TASK_NOTIFICATIONS             1

/* ======================== 三、内存管理 ======================== */

/* 是否允许"动态创建"任务/队列（TCB 和栈从 heap_4 的堆里分配） */
#define configSUPPORT_DYNAMIC_ALLOCATION         1

/* 是否允许"静态创建"（TCB 和栈由用户自己定义数组提供，不经过堆）
 *   0 = 关闭
 *   1 = 打开后【必须】实现两个回调，否则链接报 undefined symbol：
 *         vApplicationGetIdleTaskMemory()   给空闲任务提供内存
 *         vApplicationGetTimerTaskMemory()  给定时器任务提供内存（configUSE_TIMERS=1 时） */
#define configSUPPORT_STATIC_ALLOCATION          0

/* FreeRTOS 自己管理的堆大小（字节）—— 即 heap_4.c 里那个静态大数组
 *   动态创建的 TCB / 栈 / 队列内存都从这里分配
 *   C8T6 只有 20KB RAM，这里的值经 RAM 总账核算（执行表第八节）：
 *   7 个任务栈(约 6.7KB) + TCB(约 0.7KB) + 队列(约 0.5KB) —— 10KB 时开机只剩约 2.3KB，
 *   会让"运行期最低水位 >2KB"这条验收必挂，故 2026-10-05 按计划提到 12KB。 */
#define configTOTAL_HEAP_SIZE                    ((size_t)(12*1024))

/* ======================== 四、钩子函数（内核回调） ======================== */

/* 空闲任务钩子：1 = 空闲时每个 tick 调用一次 vApplicationIdleHook()（适合做低功耗） */
#define configUSE_IDLE_HOOK                      0

/* 节拍钩子：1 = 每个 tick 调用 vApplicationTickHook()
 *   ?? 它在【中断上下文】里执行，里面只能用 FromISR 系列 API，且必须极短 */
#define configUSE_TICK_HOOK                      0

/* 栈溢出检测：0 = 关闭；1 = 只检查栈指针越界；2 = 检查栈尾标记（更可靠）
 *   非 0 时【必须】实现 vApplicationStackOverflowHook()，否则出问题时无法定位 */
#define configCHECK_FOR_STACK_OVERFLOW           2

/* 堆分配失败钩子：1 = 分配失败时调用 vApplicationMallocFailedHook() 便于定位 */
#define configUSE_MALLOC_FAILED_HOOK             1

/* ======================== 五、统计与调试（学习阶段全关，省 RAM） ======================== */

/* 是否统计各任务的 CPU 占用率（需要一个额外的高频定时器作基准，费 RAM） */
#define configGENERATE_RUN_TIME_STATS            0

/* 是否开启任务状态查询功能（vTaskList / vTaskGetRunTimeStats 依赖它） */
#define configUSE_TRACE_FACILITY                 1

/* 是否提供把状态数据格式化成字符串的函数（配合上面的调试功能使用） */
#define configUSE_STATS_FORMATTING_FUNCTIONS     1

/* ★ 栈高水位查询（2026-10-05 审查发现缺失）：
 *   task_sd.h 里"用 uxTaskGetStackHighWaterMark() 实测后 ×1.5 定容"这条计划
 *   正是靠这个开关 —— FreeRTOS 默认不定义（=0），该函数不参与编译，写了会链接失败。
 *   打开只多几十字节 Flash，不占额外 RAM。 */
#define INCLUDE_uxTaskGetStackHighWaterMark      1

/* ======================== 六、协程（已被任务取代，不用） ======================== */

/* 是否使用协程 CoRoutine（比任务轻但功能受限，现代项目基本不用） */
#define configUSE_CO_ROUTINES                    0

/* 协程优先级数量（仅当上面为 1 时有效） */
#define configMAX_CO_ROUTINE_PRIORITIES          (2)

/* ======================== 七、软件定时器 ======================== */

/* 是否启用软件定时器
 *   1 = 内核会自动创建一个"定时器服务任务"和一个定时器命令队列 */
#define configUSE_TIMERS                         1

/* 定时器服务任务的优先级（这里 = configMAX_PRIORITIES-1 = 31，最高）
 *   ?? 因为它优先级最高，所以定时器回调函数里【绝不能】写耗时或阻塞的代码 */
#define configTIMER_TASK_PRIORITY                (configMAX_PRIORITIES-1)

/* 定时器命令队列长度（同时能排多少条"启动/停止/复位定时器"的命令） */
#define configTIMER_QUEUE_LENGTH                 5

/* 定时器服务任务的栈大小（单位：字）= configMINIMAL_STACK_SIZE*2 = 256 字 = 1KB */
#define configTIMER_TASK_STACK_DEPTH             (configMINIMAL_STACK_SIZE*2)

/* ======================== 八、可选 API（不用就置 0，能省 Flash 和 RAM） ======================== */

#define INCLUDE_vTaskPrioritySet                 1   /* 允许运行时修改任务优先级 */
#define INCLUDE_uxTaskPriorityGet                1   /* 允许查询任务优先级 */
#define INCLUDE_vTaskDelete                      1   /* 允许删除任务 */
#define INCLUDE_vTaskSuspend                     1   /* 允许挂起 / 恢复任务 */
#define INCLUDE_vTaskDelayUntil                  1   /* 允许绝对延时（精确定周期，采样必备） */
#define INCLUDE_vTaskDelay                       1   /* 允许相对延时 */
#define INCLUDE_xTaskGetSchedulerState           1   /* 允许查询调度器是否已启动 */
#define INCLUDE_xTimerPendFunctionCall           1   /* 允许把函数"委托"给定时器任务去执行 */

/* ======================== 九、中断优先级配置（最容易出错的一段） ======================== */

/* 本芯片 NVIC 优先级用几位：STM32F103 是 4 位（优先级取 0~15，共 16 级）
 * 该宏由 stm32f10x.h 中的 __NVIC_PRIO_BITS 提供（值为 4） */
#ifdef __NVIC_PRIO_BITS
#define configPRIO_BITS        __NVIC_PRIO_BITS
#else
#define configPRIO_BITS        4
#endif

/* ???? 中断优先级的方向和任务优先级【相反】：
 *      中断优先级数值越小 → 越紧急 → 级别越高
 *      任务优先级数值越大 → 越紧急 → 级别越高 */

/* 最低的中断优先级（15）：内核的 PendSV 和 SysTick 就设成这个
 *   让它们"排在最后"，不打断用户更紧急的中断 */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY      15

/* 可以调用 FreeRTOS API 的最高中断优先级（5）
 *   含义：中断优先级数值 ≥ 5 的中断（即不如它紧急的中断）才可以调用 FromISR 系列 API
 *       中断优先级数值 < 5 的中断（更紧急）【绝对不能】调用任何 FreeRTOS API → 否则随机死机
 *   按键 EXTI 中断如果要用 vTaskNotifyGiveFromISR，优先级就填 5、6、7... 都可以 */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

/* 把上面两个值"左移到优先级寄存器的高 4 位"（F103 只用高 4 位）
 *   configKERNEL_INTERRUPT_PRIORITY      = 15 << 4 = 0xF0 = 240
 *   configMAX_SYSCALL_INTERRUPT_PRIORITY = 5  << 4 = 0x50 = 80
 * 这两个宏会被内核写进 BASEPRI 寄存器，用来"选择性屏蔽中断"（临界区就用它） */
#define configKERNEL_INTERRUPT_PRIORITY \
    (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/* ======================== 十、中断函数名映射 ======================== */

/* port.c 里的函数名叫 vPortSVCHandler 等，这里把它们"改名"成启动文件
 * 向量表里的标准名字，两者才能对应上（否则这三个中断进不来） */
#define xPortPendSVHandler    PendSV_Handler    /* 任务切换（汇编实现） */
#define vPortSVCHandler       SVC_Handler       /* 启动第一个任务（汇编实现） */
#define xPortSysTickHandler   SysTick_Handler   /* 系统节拍：每个 tick 进来一次 */

extern void vAssertCalled( const char *pcFile, unsigned long ulLine );
#define configASSERT( x )					\
	if(( x )==0)							\
	{										\
		vAssertCalled(__FILE__,__LINE__);	\
	}										


#endif /* FREERTOS_CONFIG_H */
