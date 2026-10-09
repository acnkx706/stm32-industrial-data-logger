/*
 * FreeRTOS Kernel V11.1.0
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * SPDX-License-Identifier: MIT
 * （以下为 MIT 许可证原文，保留不动：允许免费使用/修改/分发，需保留版权声明，软件按"原样"提供不担保）
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */

/*
 * portmacro.h —— 移植层的"宏与类型定义"（Cortex-M3 / ARM Compiler，RVDS 移植层）
 *
 * 这个文件是 port.c 的配套头文件，定义了：
 *   1) 内核用的数据类型（StackType_t / TickType_t 等）
 *   2) 任务切换的触发方式（portYIELD → 挂起 PendSV）
 *   3) 临界区（关/开中断）的实现方式（BASEPRI 寄存器）
 *   4) 优先级位图（用一条指令找出最高优先级）
 */

#ifndef PORTMACRO_H
#define PORTMACRO_H

/* *INDENT-OFF* */
#ifdef __cplusplus
    extern "C" {
#endif
/* *INDENT-ON* */

/*-----------------------------------------------------------
 * 移植层专属定义
 *
 * 本文件里的设置是针对"指定硬件 + 指定编译器"配置好的，
 * 正常情况下【不要修改】。
 *-----------------------------------------------------------
 */

/* 数据类型定义（把标准 C 类型重新取个内核用的名字） */
#define portCHAR          char
#define portFLOAT         float
#define portDOUBLE        double
#define portLONG          long
#define portSHORT         short
#define portSTACK_TYPE    uint32_t   /* 栈里存放的元素类型：32 位无符号 */
#define portBASE_TYPE     long

typedef portSTACK_TYPE   StackType_t;
typedef long             BaseType_t;
typedef unsigned long    UBaseType_t;

/* tick 计数用什么位宽（由 FreeRTOSConfig.h 里的配置决定） */
#if ( configTICK_TYPE_WIDTH_IN_BITS == TICK_TYPE_WIDTH_16_BITS )
    typedef uint16_t     TickType_t;
    #define portMAX_DELAY              ( TickType_t ) 0xffff     /* 16 位时的"最大延时" */
#elif ( configTICK_TYPE_WIDTH_IN_BITS == TICK_TYPE_WIDTH_32_BITS )
    typedef uint32_t     TickType_t;
    #define portMAX_DELAY              ( TickType_t ) 0xffffffffUL   /* 32 位时的"最大延时"（近乎无限等待） */

/* 32 位架构上 tick 计数是 32 位的，所以读取 tick 计数值不需要用临界区保护
 * （单次读写在 32 位总线上是原子的）。 */
    #define portTICK_TYPE_IS_ATOMIC    1
#else
    #error configTICK_TYPE_WIDTH_IN_BITS set to unsupported tick type width.
#endif
/*-----------------------------------------------------------*/

/* 处理器架构相关特性 */
#define portSTACK_GROWTH          ( -1 )        /* 栈向下增长（Cortex-M 特性） */
#define portTICK_PERIOD_MS        ( ( TickType_t ) 1000 / configTICK_RATE_HZ )   /* 1 个 tick = 几毫秒 */
#define portBYTE_ALIGNMENT        8             /* 栈按 8 字节对齐（Cortex-M 的要求，影响 FPU/双字访问） */

/* 内存屏障指令要用的常量（全读写屏障） */
#define portSY_FULL_READ_WRITE    ( 15 )

/*-----------------------------------------------------------*/

/* 调度器相关工具宏 */
#define portYIELD()                                     \
    {                                                   \
        /* 置位 PendSV，请求一次任务切换（真正的切换在 PendSV 中断里做） */ \
        portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT; \
                                                        \
        /* 通常不需要屏障指令，但加上可以确保代码行为完全符合架构规范 */ \
        __dsb( portSY_FULL_READ_WRITE );                           \
        __isb( portSY_FULL_READ_WRITE );                           \
    }
/*-----------------------------------------------------------*/

/* 0xE000ED04 = 内核外设 SCB 的中断控制与状态寄存器（ICSR）
 * bit28 = PENDSVSET：写 1 表示"挂起 PendSV 异常" → 触发任务切换 */
#define portNVIC_INT_CTRL_REG     ( *( ( volatile uint32_t * ) 0xe000ed04 ) )
#define portNVIC_PENDSVSET_BIT    ( 1UL << 28UL )

/* 中断服务函数在返回前，根据"是否需要切换任务"决定要不要触发 PendSV
 * （中断里恢复/唤醒了更高优先级任务时就该切） */
#define portEND_SWITCHING_ISR( xSwitchRequired ) \
    do                                           \
    {                                            \
        if( xSwitchRequired != pdFALSE )         \
        {                                        \
            traceISR_EXIT_TO_SCHEDULER();        \
            portYIELD();                         \
        }                                        \
        else                                     \
        {                                        \
            traceISR_EXIT();                     \
        }                                        \
    } while( 0 )
/* 中断里调用 FreeRTOS 的 xxxFromISR API 后，用这个宏决定是否立即切换任务 */
#define portYIELD_FROM_ISR( x )    portEND_SWITCHING_ISR( x )
/*-----------------------------------------------------------*/

/* 临界区管理（进入临界区 = 关中断；退出 = 开中断） */
extern void vPortEnterCritical( void );
extern void vPortExitCritical( void );

#define portDISABLE_INTERRUPTS()                  vPortRaiseBASEPRI()        /* 关中断（抬高 BASEPRI） */
#define portENABLE_INTERRUPTS()                   vPortSetBASEPRI( 0 )       /* 开中断（BASEPRI 清 0） */
#define portENTER_CRITICAL()                      vPortEnterCritical()       /* 进入临界区（可嵌套，带计数） */
#define portEXIT_CRITICAL()                       vPortExitCritical()        /* 退出临界区（计数归零才真开中断） */
#define portSET_INTERRUPT_MASK_FROM_ISR()         ulPortRaiseBASEPRI()       /* 中断里用的"关中断"，返回旧值 */
#define portCLEAR_INTERRUPT_MASK_FROM_ISR( x )    vPortSetBASEPRI( x )       /* 中断里恢复之前的屏蔽状态 */

/*-----------------------------------------------------------*/

/* 无节拍空闲（低功耗）功能：当所有任务都在等待时，可以关掉 SysTick 进入睡眠 */
#ifndef portSUPPRESS_TICKS_AND_SLEEP
    extern void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime );
    #define portSUPPRESS_TICKS_AND_SLEEP( xExpectedIdleTime )    vPortSuppressTicksAndSleep( xExpectedIdleTime )
#endif
/*-----------------------------------------------------------*/

/* 移植层专属优化：用硬件指令挑选最高优先级任务 */
#ifndef configUSE_PORT_OPTIMISED_TASK_SELECTION
    #define configUSE_PORT_OPTIMISED_TASK_SELECTION    1
#endif

#if configUSE_PORT_OPTIMISED_TASK_SELECTION == 1

/* 配置检查 */
    #if ( configMAX_PRIORITIES > 32 )
        #error configUSE_PORT_OPTIMISED_TASK_SELECTION can only be set to 1 when configMAX_PRIORITIES is less than or equal to 32.  It is very rare that a system requires more than 10 to 15 difference priorities as tasks that share a priority will time slice.
    #endif

/* 用"位图"记录哪些优先级上有就绪任务：
 *   置位 = 该优先级有就绪任务；清位 = 该优先级没有就绪任务 */
    #define portRECORD_READY_PRIORITY( uxPriority, uxReadyPriorities )    ( uxReadyPriorities ) |= ( 1UL << ( uxPriority ) )
    #define portRESET_READY_PRIORITY( uxPriority, uxReadyPriorities )     ( uxReadyPriorities ) &= ~( 1UL << ( uxPriority ) )

/*-----------------------------------------------------------*/

/* 用 CLZ（前导零计数）指令一步算出位图中最高的那一位 = 最高就绪优先级。
 *   例如位图 = 0b1000（只有第 3 位是 1）→ 31 - clz(0b1000) = 31 - 28 = 3
 * 这就是"多优先级"能 O(1) 找到最高优先级任务的原因。 */
    #define portGET_HIGHEST_PRIORITY( uxTopPriority, uxReadyPriorities )    uxTopPriority = ( 31UL - ( uint32_t ) __clz( ( uxReadyPriorities ) ) )

#endif /* taskRECORD_READY_PRIORITY */
/*-----------------------------------------------------------*/

/* 任务函数宏（FreeRTOS 官网的写法）。本移植层其实用不到，
 * 定义它们只是为了让"官方通用例程"能编译通过。 */
#define portTASK_FUNCTION_PROTO( vFunction, pvParameters )    void vFunction( void * pvParameters )
#define portTASK_FUNCTION( vFunction, pvParameters )          void vFunction( void * pvParameters )
/*-----------------------------------------------------------*/

/* 如果定义了 configASSERT，内核会在中断里检查"你的中断优先级是否合法"
 * （优先级高于 configMAX_SYSCALL_INTERRUPT_PRIORITY 的中断里不能调用 FreeRTOS API） */
#ifdef configASSERT
    void vPortValidateInterruptPriority( void );
    #define portASSERT_IF_INTERRUPT_PRIORITY_INVALID()    vPortValidateInterruptPriority()
#endif

/* 本移植层不需要 portNOP()（空操作），定义为空 */
#define portNOP()

#define portINLINE              __inline
#ifndef portFORCE_INLINE
    #define portFORCE_INLINE    __forceinline
#endif

/*-----------------------------------------------------------*/

/**
 * @brief  把 BASEPRI 设置为指定值（用来"开中断"或恢复之前的屏蔽状态）
 * @param  ulBASEPRI 要写入 BASEPRI 的值（0 = 不屏蔽任何中断）
 *
 * BASEPRI 的作用：屏蔽"优先级数值 ≥ BASEPRI"的中断，
 * 优先级更高（数值更小）的中断照常响应 —— 这正是 FreeRTOS
 * "选择性关中断"比"全部关中断"实时性好的原因。
 */
static portFORCE_INLINE void vPortSetBASEPRI( uint32_t ulBASEPRI )
{
    __asm
    {
        /* 这里不用屏障指令：本函数只用于"降低"BASEPRI 的值（放开屏蔽）。 */
/* *INDENT-OFF* */
            msr basepri, ulBASEPRI
/* *INDENT-ON* */
    }
}
/*-----------------------------------------------------------*/

/**
 * @brief  抬高 BASEPRI 到"最高系统调用优先级"，从而进入临界区（关中断）
 *
 * 效果：屏蔽所有优先级数值 ≥ configMAX_SYSCALL_INTERRUPT_PRIORITY 的中断
 * （即把不重要/普通的中断都挡住），保证临界区代码不被打断。
 */
static portFORCE_INLINE void vPortRaiseBASEPRI( void )
{
    uint32_t ulNewBASEPRI = configMAX_SYSCALL_INTERRUPT_PRIORITY;

    __asm
    {
        /* 把 BASEPRI 设为 configMAX_SYSCALL_INTERRUPT_PRIORITY 即形成临界区。 */
/* *INDENT-OFF* */
            msr basepri, ulNewBASEPRI
            dsb
            isb
/* *INDENT-ON* */
    }
}
/*-----------------------------------------------------------*/

/**
 * @brief  在中断里把 BASEPRI 清 0（解除屏蔽）
 *
 * 用于中断上下文：只用于"放开"屏蔽，所以不需要内存屏障指令。
 */
static portFORCE_INLINE void vPortClearBASEPRIFromISR( void )
{
    __asm
    {
        /* 把 BASEPRI 设为 0，就屏蔽不了任何中断了。
         * 本函数只用于在中断里"放开"屏蔽，所以不用内存屏障。 */
/* *INDENT-OFF* */
            msr basepri, # 0
/* *INDENT-ON* */
    }
}
/*-----------------------------------------------------------*/

/**
 * @brief  中断里用的"关中断"：抬高 BASEPRI，并返回原来的值
 * @return 进入之前 BASEPRI 的旧值（退出时用它恢复）
 *
 * 为什么中断里要"返回值"：中断可能与任务里正在进行的临界区嵌套，
 * 所以不能简单地"清 0 开中断"，必须把原来的屏蔽状态存下来、用完再恢复。
 * 这就是中断版（FromISR）和任务版（vPortEnterCritical 计数式）的区别。
 */
static portFORCE_INLINE uint32_t ulPortRaiseBASEPRI( void )
{
    uint32_t ulReturn, ulNewBASEPRI = configMAX_SYSCALL_INTERRUPT_PRIORITY;

    __asm
    {
        /* 先读旧值保存，再写入新的 BASEPRI 值形成临界区。 */
/* *INDENT-OFF* */
            mrs ulReturn, basepri
            msr basepri, ulNewBASEPRI
            dsb
            isb
/* *INDENT-ON* */
    }

    return ulReturn;
}
/*-----------------------------------------------------------*/

/**
 * @brief  判断"当前是否在中断上下文里"
 * @return pdTRUE = 在中断里；pdFALSE = 在任务里
 *
 * 原理：读取 IPSR 寄存器（当前异常编号），为 0 表示"不在任何异常/中断中"。
 */
static portFORCE_INLINE BaseType_t xPortIsInsideInterrupt( void )
{
    uint32_t ulCurrentInterrupt;
    BaseType_t xReturn;

    /* 读取当前正在执行的异常（中断）编号。 */
    __asm
    {
/* *INDENT-OFF* */
            mrs ulCurrentInterrupt, ipsr
/* *INDENT-ON* */
    }

    if( ulCurrentInterrupt == 0 )
    {
        xReturn = pdFALSE;      /* 编号 0 = 不在中断里（正在跑任务） */
    }
    else
    {
        xReturn = pdTRUE;       /* 其它编号 = 正在中断里 */
    }

    return xReturn;
}


/* *INDENT-OFF* */
#ifdef __cplusplus
    }
#endif
/* *INDENT-ON* */

#endif /* PORTMACRO_H */
