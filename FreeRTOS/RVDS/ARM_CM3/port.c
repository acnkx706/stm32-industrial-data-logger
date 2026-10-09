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

/*-----------------------------------------------------------
* port.c —— Cortex-M3 移植层实现（RVDS/ARM 编译器版本）
*
* 这个文件是"FreeRTOS 内核"和"STM32 硬件"之间的桥梁，它实现四件大事：
*   1) pxPortInitialiseStack()   造任务时，往任务栈里伪造一个"中断现场"
*   2) prvStartFirstTask()/vPortSVCHandler()  启动第一个任务（汇编）
*   3) xPortPendSVHandler()      任务切换（汇编，保存旧任务现场、恢复新任务现场）
*   4) xPortSysTickHandler()     系统节拍（每 1ms 进来一次，走时基、必要时切任务）
*   另外还实现了临界区（BASEPRI 关中断）和中断优先级合法性检查。
*----------------------------------------------------------*/

/* 调度器相关头文件 */
#include "FreeRTOS.h"
#include "task.h"

/* configMAX_SYSCALL_INTERRUPT_PRIORITY 绝对不能是 0：
 * 值为 0 时 BASEPRI 写 0 = 不屏蔽任何中断，临界区会失效；
 * 而且优先级为 0 的中断无法被 BASEPRI 屏蔽。 */
#if configMAX_SYSCALL_INTERRUPT_PRIORITY == 0
    #error configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to 0.  See http: /*www.FreeRTOS.org/RTOS-Cortex-M3-M4.html */
#endif

/* 仅用于向后兼容的老宏。它过去用来"替换配置节拍时钟的函数"(prvSetupTimerInterrupt)，
 * 现在那个函数被声明为 weak（弱符号），应用层只要定义同名函数
 * vApplicationSetupTickInterrupt() 就能覆盖它。 */
#ifndef configOVERRIDE_DEFAULT_TICK_CONFIGURATION
    #define configOVERRIDE_DEFAULT_TICK_CONFIGURATION    0
#endif

/* 操作内核外设需要的常量。先是寄存器地址…… */
#define portNVIC_SYSTICK_CTRL_REG             ( *( ( volatile uint32_t * ) 0xe000e010 ) )   /* SysTick 控制与状态 */
#define portNVIC_SYSTICK_LOAD_REG             ( *( ( volatile uint32_t * ) 0xe000e014 ) )   /* SysTick 重装载值 */
#define portNVIC_SYSTICK_CURRENT_VALUE_REG    ( *( ( volatile uint32_t * ) 0xe000e018 ) )   /* SysTick 当前计数值 */
#define portNVIC_SHPR3_REG                    ( *( ( volatile uint32_t * ) 0xe000ed20 ) )   /* 系统异常优先级（含 PendSV/SysTick） */
/* ……然后是寄存器里的位 */
#define portNVIC_SYSTICK_CLK_BIT              ( 1UL << 2UL )    /* 时钟源选择（1=内核时钟） */
#define portNVIC_SYSTICK_INT_BIT              ( 1UL << 1UL )    /* 计数到 0 时产生中断 */
#define portNVIC_SYSTICK_ENABLE_BIT           ( 1UL << 0UL )    /* 使能 SysTick */
#define portNVIC_SYSTICK_COUNT_FLAG_BIT       ( 1UL << 16UL )   /* 计数到 0 的标志 */
#define portNVIC_PENDSVCLEAR_BIT              ( 1UL << 27UL )   /* 清除 PendSV 挂起 */
#define portNVIC_PEND_SYSTICK_SET_BIT         ( 1UL << 26UL )   /* 挂起 SysTick */
#define portNVIC_PEND_SYSTICK_CLEAR_BIT       ( 1UL << 25UL )   /* 清除 SysTick 挂起 */

#define portMIN_INTERRUPT_PRIORITY            ( 255UL )         /* 最低的中断优先级（数值最大） */
#define portNVIC_PENDSV_PRI                   ( ( ( uint32_t ) portMIN_INTERRUPT_PRIORITY ) << 16UL )   /* PendSV 优先级（低 8 位之外的位置） */
#define portNVIC_SYSTICK_PRI                  ( ( ( uint32_t ) portMIN_INTERRUPT_PRIORITY ) << 24UL )   /* SysTick 优先级 */

/* 检查中断优先级合法性时用到的常量 */
#define portFIRST_USER_INTERRUPT_NUMBER       ( 16 )            /* 第一个用户中断的异常编号（0~15 是内核异常） */
#define portNVIC_IP_REGISTERS_OFFSET_16       ( 0xE000E3F0 )    /* 中断优先级寄存器组的基地址 */
#define portAIRCR_REG                         ( *( ( volatile uint32_t * ) 0xE000ED0C ) )   /* 应用中断与复位控制寄存器（优先级分组） */
#define portMAX_8_BIT_VALUE                   ( ( uint8_t ) 0xff )
#define portTOP_BIT_OF_BYTE                   ( ( uint8_t ) 0x80 )
#define portMAX_PRIGROUP_BITS                 ( ( uint8_t ) 7 )
#define portPRIORITY_GROUP_MASK               ( 0x07UL << 8UL )
#define portPRIGROUP_SHIFT                    ( 8UL )

/* 只保留 ICSR 寄存器里的 VECTACTIVE 位（当前正在执行的异常编号） */
#define portVECTACTIVE_MASK                   ( 0xFFUL )

/* 初始化任务栈时需要的常量 */
#define portINITIAL_XPSR                      ( 0x01000000 )     /* xPSR 的 bit24（T 位）必须为 1，表示 Thumb 状态 */

/* SysTick 是 24 位计数器 */
#define portMAX_24_BIT_NUMBER                 ( 0xffffffUL )

/* 一个估算系数：在 tickless idle 计算里，用来估计 SysTick 停止期间"漏掉"的计数 */
#define portMISSED_COUNTS_FACTOR              ( 94UL )

/* 严格符合 Cortex-M 规范：任务入口地址的 bit0 必须为 0
 * （因为它是从异常返回时被装进 PC 的，bit0 必须表示 Thumb 状态） */
#define portSTART_ADDRESS_MASK                ( ( StackType_t ) 0xfffffffeUL )

/* 允许用户自定义 SysTick 的时钟频率。若用户定义了 configSYSTICK_CLOCK_HZ，
 * 那么当控制寄存器里 CLK 位为 0 时，该值必须等于 SysTick 的时钟频率。 */
#ifndef configSYSTICK_CLOCK_HZ
    #define configSYSTICK_CLOCK_HZ             ( configCPU_CLOCK_HZ )
    /* 让 SysTick 使用与内核相同的时钟频率 */
    #define portNVIC_SYSTICK_CLK_BIT_CONFIG    ( portNVIC_SYSTICK_CLK_BIT )
#else
    /* 选择"SysTick 时钟与内核不同频"的配置 */
    #define portNVIC_SYSTICK_CLK_BIT_CONFIG    ( 0 )
#endif

/*
 * 配置产生节拍中断的定时器。本文件里的实现是 weak（弱符号），
 * 方便应用层替换成别的定时器来产生节拍中断。
 */
void vPortSetupTimerInterrupt( void );

/*
 * 异常（中断）服务函数
 */
void xPortPendSVHandler( void );    /* 任务切换（最低优先级异常） */
void xPortSysTickHandler( void );   /* 系统节拍 */
void vPortSVCHandler( void );       /* 启动第一个任务 */

/*
 * "启动第一个任务"单独做成一个函数，便于单独测试。
 */
static void prvStartFirstTask( void );

/*
 * 用来抓住"任务函数企图返回"的错误（任务函数不许返回）。
 */
static void prvTaskExitError( void );

/*-----------------------------------------------------------*/

/* 每个任务在"临界区嵌套计数"变量里维护自己的中断状态。
 * 初值 0xaaaaaaaa 是故意的"魔法数"：用来发现"还没初始化就被使用"的情况。 */
static UBaseType_t uxCriticalNesting = 0xaaaaaaaa;

/*
 * 一个 tick 周期对应多少个 SysTick 计数。
 */
#if ( configUSE_TICKLESS_IDLE == 1 )
    static uint32_t ulTimerCountsForOneTick = 0;
#endif /* configUSE_TICKLESS_IDLE */

/*
 * 最多能"抑制"多少个 tick 周期 —— 受 SysTick 24 位分辨率限制。
 */
#if ( configUSE_TICKLESS_IDLE == 1 )
    static uint32_t xMaximumPossibleSuppressedTicks = 0;
#endif /* configUSE_TICKLESS_IDLE */

/*
 * 补偿 SysTick 停止期间流逝的 CPU 周期（仅低功耗功能使用）。
 */
#if ( configUSE_TICKLESS_IDLE == 1 )
    static uint32_t ulStoppedTimerCompensation = 0;
#endif /* configUSE_TICKLESS_IDLE */

/*
 * portASSERT_IF_INTERRUPT_PRIORITY_INVALID() 宏会用到这些变量，
 * 用来确保"不会从优先级高于 configMAX_SYSCALL_INTERRUPT_PRIORITY 的中断里
 * 调用 FreeRTOS 的 API"。
 */
#if ( configASSERT_DEFINED == 1 )
    static uint8_t ucMaxSysCallPriority = 0;
    static uint32_t ulMaxPRIGROUPValue = 0;
    static const volatile uint8_t * const pcInterruptPriorityRegisters = ( uint8_t * ) portNVIC_IP_REGISTERS_OFFSET_16;
#endif /* configASSERT_DEFINED */

/*-----------------------------------------------------------*/

/**
 * @brief  初始化一个新任务的栈（关键：在栈里"伪造"一个中断现场）
 * @param  pxTopOfStack 栈顶（栈是向下增长的，这里传入栈的高地址端）
 * @param  pxCode       任务函数入口
 * @param  pvParameters 传给任务的形参
 * @return 伪造完成后的"栈顶指针"（内核把它保存进 TCB 的 pxTopOfStack）
 *
 * 原理：任务第一次被调度时，内核会像"从异常返回"那样恢复现场 —— 把栈里的值
 *       弹进 CPU 寄存器。所以我们提前把栈摆成"一个刚被中断打断的任务"的样子：
 *       只要执行"恢复现场"，PC 就被弹成任务函数入口，任务就"开始运行"了。
 *
 * 栈内容（从高地址到低地址）：
 *   xPSR（bit24=1）→ PC（任务入口）→ LR（退出处理函数）
 *   → R12, R3, R2, R1 → R0（形参）→ R11…R4
 * 注意：R4~R11 是软件手动保存/恢复的，其它是硬件自动保存/恢复的。
 */
StackType_t * pxPortInitialiseStack( StackType_t * pxTopOfStack,
                                     TaskFunction_t pxCode,
                                     void * pvParameters )
{
    /* 按"上下文切换中断创建栈帧"的方式伪造现场。 */
    pxTopOfStack--;                                                      /* 预留一格，对应 MCU 中断进出时的栈使用方式 */
    *pxTopOfStack = portINITIAL_XPSR;                                    /* xPSR */
    pxTopOfStack--;
    *pxTopOfStack = ( ( StackType_t ) pxCode ) & portSTART_ADDRESS_MASK; /* PC：任务函数入口（bit0 清零） */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) prvTaskExitError;                    /* LR：任务若返回会走到这里 */

    pxTopOfStack -= 5;                                                   /* R12, R3, R2, R1（默认 0） */
    *pxTopOfStack = ( StackType_t ) pvParameters;                        /* R0：任务形参 */
    pxTopOfStack -= 8;                                                   /* R11, R10, R9, R8, R7, R6, R5, R4（默认 0） */

    return pxTopOfStack;
}
/*-----------------------------------------------------------*/

/**
 * @brief  任务函数"返回"时的兜底处理（任务函数不允许返回）
 *
 * 任务函数如果返回，就没有任何地方可回 —— 想结束任务应该调用 vTaskDelete(NULL)。
 * 如果定义了 configASSERT()，这里会故意触发断言，方便定位错误；然后死循环停住。
 */
static void prvTaskExitError( void )
{
    /* 实现任务的函数不能退出、也不能返回到调用者，因为没有"调用者"。
     * 任务想结束自己，应该调用 vTaskDelete( NULL )。
     *
     * 如果定义了 configASSERT()，这里故意触发断言，然后停在这里，
     * 方便应用开发者发现并定位这个错误。 */
    configASSERT( uxCriticalNesting == ~0UL );
    portDISABLE_INTERRUPTS();

    for( ; ; )
    {
    }
}
/*-----------------------------------------------------------*/

/**
 * @brief  SVC 异常服务函数：启动"第一个任务"
 *
 * 由 prvStartFirstTask() 里的 `svc 0` 触发。它做的事：
 *   1) 从 pxCurrentTCB 取出任务栈顶（TCB 的第一个成员就是栈顶）
 *   2) 把 R4~R11 从栈里弹出来（这几个是软件保存的寄存器）
 *   3) 把栈指针写进 PSP（任务用 PSP，内核/中断用 MSP）
 *   4) 把 BASEPRI 清 0（开中断），设置 EXC_RETURN = 0xD
 *   5) bx r14 触发"异常返回"：硬件自动把剩下 8 个寄存器
 *      （xPSR, PC, LR, R12, R3, R2, R1, R0）从栈里弹出
 *      → PC 被弹成任务函数入口，任务正式开始运行
 */
__asm void vPortSVCHandler( void )
{
/* *INDENT-OFF* */
    PRESERVE8

    ldr r3, = pxCurrentTCB   /* 取 pxCurrentTCB 变量的地址 */
    ldr r1, [ r3 ] /* 读出 pxCurrentTCB（即当前任务的 TCB 地址） */
    ldr r0, [ r1 ]           /* TCB 的第一个成员就是"任务栈顶" */
    ldmia r0 !, { r4 - r11 } /* 从栈里恢复 R4~R11（异常进入时硬件不会自动保存这些） */
    msr psp, r0 /* 把栈指针写进 PSP（任务使用 PSP） */
    isb
    mov r0, # 0
    msr basepri, r0          /* BASEPRI=0：不屏蔽任何中断 */
    orr r14, # 0xd           /* EXC_RETURN=0xD：异常返回后使用 PSP、返回线程模式、Thumb 状态 */
    bx r14                   /* 异常返回：硬件自动弹出剩余 8 个寄存器并开始执行任务 */
/* *INDENT-ON* */
}
/*-----------------------------------------------------------*/

/**
 * @brief  启动第一个任务前的准备工作
 *
 * 步骤：
 *   1) 从向量表里取出初始 MSP 的值（向量表第 0 项就是上电时的栈顶）
 *   2) 把 MSP 设回去（清掉启动代码用过的栈内容）
 *   3) 开全局中断
 *   4) `svc 0` 进入 SVC 异常 → 由 vPortSVCHandler 真正启动第一个任务
 */
__asm void prvStartFirstTask( void )
{
/* *INDENT-OFF* */
    PRESERVE8

    /* 借助 NVIC 的偏移寄存器(0xE000ED08 = VTOR)找到向量表的地址。 */
    ldr r0, =0xE000ED08
    ldr r0, [ r0 ]      /* 取 VTOR 里的向量表地址 */
    ldr r0, [ r0 ]      /* 取向量表第 0 项 = 初始 MSP 的值 */

    /* 把 MSP 设回栈的起始位置。 */
    msr msp, r0
    /* 全局开中断。 */
    cpsie i
    cpsie f
    dsb
    isb
    /* 触发 SVC 来启动第一个任务。 */
    svc 0
    nop
    nop
/* *INDENT-ON* */
}
/*-----------------------------------------------------------*/

/*
 * 说明见头文件。
 */
BaseType_t xPortStartScheduler( void )
{
    #if ( configASSERT_DEFINED == 1 )
    {
        volatile uint8_t ucOriginalPriority;
        volatile uint32_t ulImplementedPrioBits = 0;
        volatile uint8_t * const pucFirstUserPriorityRegister = ( uint8_t * ) ( portNVIC_IP_REGISTERS_OFFSET_16 + portFIRST_USER_INTERRUPT_NUMBER );
        volatile uint8_t ucMaxPriorityValue;

        /* 判断"可以从多高的中断优先级里调用 ISR 安全的 FreeRTOS API"。
         * ISR 安全的函数就是名字以 "FromISR" 结尾的那些。
         * FreeRTOS 把"任务 API"和"中断 API"分开维护，是为了让中断进入尽可能快而简单。
         *
         * 先保存即将被我们覆盖的中断优先级寄存器的值。 */
        ucOriginalPriority = *pucFirstUserPriorityRegister;

        /* 判断硬件实现了几个优先级位：先往所有位写 1。 */
        *pucFirstUserPriorityRegister = portMAX_8_BIT_VALUE;

        /* 再读回来，看哪些位"留住了"（硬件没实现的位读回是 0）。 */
        ucMaxPriorityValue = *pucFirstUserPriorityRegister;

        /* 用同样的掩码处理"最高系统调用优先级"。 */
        ucMaxSysCallPriority = configMAX_SYSCALL_INTERRUPT_PRIORITY & ucMaxPriorityValue;

        /* 检查"扣除硬件未实现的位之后"，系统调用优先级不为 0。
         * 优先级 0 是非法的：BASEPRI 写 0 等于不屏蔽任何中断，
         * 而优先级为 0 的中断本来也无法被 BASEPRI 屏蔽。 */
        configASSERT( ucMaxSysCallPriority );

        /* 检查 configMAX_SYSCALL_INTERRUPT_PRIORITY 里"硬件未实现的位"必须为 0。 */
        configASSERT( ( configMAX_SYSCALL_INTERRUPT_PRIORITY & ( ~ucMaxPriorityValue ) ) == 0U );

        /* 根据读回的位数，计算可接受的最大"优先级分组"值。 */

        while( ( ucMaxPriorityValue & portTOP_BIT_OF_BYTE ) == portTOP_BIT_OF_BYTE )
        {
            ulImplementedPrioBits++;
            ucMaxPriorityValue <<= ( uint8_t ) 0x01;
        }

        if( ulImplementedPrioBits == 8 )
        {
            /* 当硬件实现 8 个优先级位时，软件无法配置 PRIGROUP 使"没有子优先级"。
             * 结果最低位总是用作子优先级：128 个抢占优先级 + 2 个子优先级。
             *
             * 这可能引起困惑：例如把 configMAX_SYSCALL_INTERRUPT_PRIORITY 设为 5 时，
             * 优先级 5 和 4 的中断会在临界区里同时被屏蔽（因为它们抢占优先级相同），
             * 看起来就像"更高的 4 也被屏蔽了"。
             * 若把它设为 4，这种困惑就不会出现，行为也不变。
             *
             * 下面这个断言确保 configMAX_SYSCALL_INTERRUPT_PRIORITY 的子优先位为 0。 */
            configASSERT( ( configMAX_SYSCALL_INTERRUPT_PRIORITY & 0x1U ) == 0U );
            ulMaxPRIGROUPValue = 0;
        }
        else
        {
            ulMaxPRIGROUPValue = portMAX_PRIGROUP_BITS - ulImplementedPrioBits;
        }

        /* 把优先级分组值移回它在 AIRCR 寄存器里的位置。 */
        ulMaxPRIGROUPValue <<= portPRIGROUP_SHIFT;
        ulMaxPRIGROUPValue &= portPRIORITY_GROUP_MASK;

        /* 恢复刚才被我们改动的那个中断优先级寄存器。 */
        *pucFirstUserPriorityRegister = ucOriginalPriority;
    }
    #endif /* configASSERT_DEFINED */

    /* 让 PendSV 和 SysTick 使用最低的中断优先级（它们必须"让着"别的中断）。 */
    portNVIC_SHPR3_REG |= portNVIC_PENDSV_PRI;

    portNVIC_SHPR3_REG |= portNVIC_SYSTICK_PRI;

    /* 启动产生节拍中断的定时器（此处中断仍是关闭状态）。 */
    vPortSetupTimerInterrupt();

    /* 初始化临界区嵌套计数，为第一个任务做好准备。 */
    uxCriticalNesting = 0;

    /* 启动第一个任务（不会返回）。 */
    prvStartFirstTask();

    /* 不应该走到这里！ */
    return 0;
}
/*-----------------------------------------------------------*/

/**
 * @brief  结束调度器（本移植层不需要实现：任务无处可返回）
 * 故意触发断言，提醒调用者这个函数不该被使用。
 */
void vPortEndScheduler( void )
{
    /* 在没有"可返回之处"的移植层里不实现本函数，故意触发断言。 */
    configASSERT( uxCriticalNesting == 1000UL );
}
/*-----------------------------------------------------------*/

/**
 * @brief  进入临界区（任务上下文用）：关中断 + 嵌套计数 +1
 *
 * 注意：这是"任务版"的临界区，不是中断安全版本。
 * 它用"计数"支持嵌套：只有计数减到 0 时才真正开中断。
 */
void vPortEnterCritical( void )
{
    portDISABLE_INTERRUPTS();
    uxCriticalNesting++;

    /* 这是"非中断安全"版本的进入临界区函数，所以如果在中断上下文里调用它，
     * 就用断言报错。中断里只能用名字以 "FromISR" 结尾的 API。
     * 只在嵌套计数为 1 时才断言，以防"断言函数自己也用临界区"造成递归。 */
    if( uxCriticalNesting == 1 )
    {
        configASSERT( ( portNVIC_INT_CTRL_REG & portVECTACTIVE_MASK ) == 0 );
    }
}
/*-----------------------------------------------------------*/

/**
 * @brief  退出临界区：嵌套计数 -1，减到 0 时才真正开中断
 */
void vPortExitCritical( void )
{
    configASSERT( uxCriticalNesting );
    uxCriticalNesting--;

    if( uxCriticalNesting == 0 )
    {
        portENABLE_INTERRUPTS();
    }
}
/*-----------------------------------------------------------*/

/**
 * @brief  PendSV 异常服务函数：任务切换的真正执行者（整个 RTOS 的心脏）
 *
 * 流程：
 *   1) 读 PSP：拿到"被切走的任务"的栈指针
 *   2) 把 R4~R11 压进它的栈（这几个寄存器硬件不自动保存）
 *   3) 把新的栈顶存回 TCB 的第一个成员（pxTopOfStack）
 *   4) 临时把 R3、R14 压栈保护（因为马上要调用 C 函数）
 *   5) 关中断（BASEPRI）→ 调用 vTaskSwitchContext() 选出新任务
 *      （这个函数会更新 pxCurrentTCB 指向新任务）
 *   6) 从新 TCB 取出栈顶，恢复 R4~R11，把新栈顶写进 PSP
 *   7) bx r14 异常返回 → 硬件恢复剩下 8 个寄存器 → 新任务继续运行
 *
 * 一句话：任务切换 = 把当前任务现场存进它的栈 + 把新任务的现场从它的栈里恢复。
 */
__asm void xPortPendSVHandler( void )
{
    extern uxCriticalNesting;
    extern pxCurrentTCB;
    extern vTaskSwitchContext;

/* *INDENT-OFF* */
    PRESERVE8

    mrs r0, psp         /* r0 = 当前任务的栈指针(PSP) */
    isb

    ldr r3, =pxCurrentTCB /* r3 = pxCurrentTCB 变量的地址 */
    ldr r2, [ r3 ]        /* r2 = 当前任务的 TCB 地址 */

    stmdb r0 !, { r4 - r11 } /* 把 R4~R11 压进当前任务的栈（软件负责保存的部分） */
    str r0, [ r2 ]           /* 把新的栈顶存回 TCB 的第一个成员（pxTopOfStack） */

    stmdb sp !, { r3, r14 }  /* 临时压栈保护 r3 和 r14：
                              * 接下来要调用 C 函数，返回地址会覆盖 r14；r3 之后还要用 */
    mov r0, #configMAX_SYSCALL_INTERRUPT_PRIORITY
    msr basepri, r0          /* 关中断：进入临界区，保护内核数据结构 */
    dsb
    isb
    bl vTaskSwitchContext    /* 调用 C 函数选出新任务（更新 pxCurrentTCB） */
    mov r0, #0
    msr basepri, r0          /* 开中断：退出临界区 */
    ldmia sp !, { r3, r14 }  /* 恢复刚才保护的 r3、r14 */

    ldr r1, [ r3 ]           /* r1 = 新任务的 TCB 地址 */
    ldr r0, [ r1 ]           /* 取 TCB 第一个成员 = 新任务的栈顶 */
    ldmia r0 !, { r4 - r11 } /* 从新任务栈里恢复 R4~R11 */
    msr psp, r0              /* 把新任务的栈指针写进 PSP */
    isb
    bx r14                   /* 异常返回：硬件自动弹出 xPSR/PC/LR/R12/R3/R2/R1/R0
                              * → PC 变成新任务的"上次执行点"，新任务接着跑 */
    nop
/* *INDENT-ON* */
}
/*-----------------------------------------------------------*/

/**
 * @brief  SysTick 异常服务函数：系统节拍（每 1ms 进来一次）
 *
 * 流程：关中断 → xTaskIncrementTick()（时基+1、把到期的任务从延时链挪回就绪链，
 *       并返回"是否需要切换任务"）→ 需要切换就置位 PendSV（真正的切换在 PendSV 里做）
 *       → 开中断。
 */
void xPortSysTickHandler( void )
{
    /* SysTick 运行在最低中断优先级，所以它执行时所有中断一定是未被屏蔽的。
     * 因此不需要"保存再恢复中断屏蔽状态"，直接用更快的 vPortRaiseBASEPRI()，
     * 而不用 portSET_INTERRUPT_MASK_FROM_ISR()。 */
    vPortRaiseBASEPRI();
    traceISR_ENTER();
    {
        /* 递增 RTOS 节拍计数。 */
        if( xTaskIncrementTick() != pdFALSE )
        {
            traceISR_EXIT_TO_SCHEDULER();

            /* 需要切换任务。切换工作由 PendSV 异常完成，这里只"挂起 PendSV"。 */
            portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
        }
        else
        {
            traceISR_EXIT();
        }
    }

    vPortClearBASEPRIFromISR();
}
/*-----------------------------------------------------------*/

#if ( configUSE_TICKLESS_IDLE == 1 )

    /**
     * @brief  tickless idle（低功耗）实现：空闲时停掉 SysTick 让 CPU 睡一会儿
     *
     * 本工程 configUSE_TICKLESS_IDLE = 0，所以下面这段不会参与编译。
     * 保留中文说明以便需要时查阅：核心思想是"算出要睡多久 → 重设 SysTick 计数
     * → 执行 WFI 睡眠 → 醒来后把漏掉的 tick 补上（vTaskStepTick）"。
     */
    __weak void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime )
    {
        uint32_t ulReloadValue, ulCompleteTickPeriods, ulCompletedSysTickDecrements, ulSysTickDecrementsLeft;
        TickType_t xModifiableIdleTime;

        /* 确保 SysTick 重装载值不会把计数器撑溢出。 */
        if( xExpectedIdleTime > xMaximumPossibleSuppressedTicks )
        {
            xExpectedIdleTime = xMaximumPossibleSuppressedTicks;
        }

        /* 进入临界状态，但不能用 taskENTER_CRITICAL()：
         * 那会把"本该唤醒睡眠的中断"也屏蔽掉。 */
        __disable_irq();
        __dsb( portSY_FULL_READ_WRITE );
        __isb( portSY_FULL_READ_WRITE );

        /* 如果已有待切换的任务，或正有任务等待调度器解除挂起，就放弃本次低功耗。 */
        if( eTaskConfirmSleepModeStatus() == eAbortSleep )
        {
            /* 重新开中断（原因见上面 __disable_irq() 处的注释）。 */
            __enable_irq();
        }
        else
        {
            /* 先临时停掉 SysTick。停掉期间的时间会尽量补偿，
             * 但使用 tickless 模式不可避免地会让内核时间相对日历时间产生微小漂移。 */
            portNVIC_SYSTICK_CTRL_REG = ( portNVIC_SYSTICK_CLK_BIT_CONFIG | portNVIC_SYSTICK_INT_BIT );

            /* 用 SysTick 当前值寄存器判断"距离下一个 tick 中断还剩多少次递减"。
             * 如果当前值为 0，实际剩余的是 ulTimerCountsForOneTick 次
             * （因为从 1 递减到 0 时才请求中断）。 */
            ulSysTickDecrementsLeft = portNVIC_SYSTICK_CURRENT_VALUE_REG;

            if( ulSysTickDecrementsLeft == 0 )
            {
                ulSysTickDecrementsLeft = ulTimerCountsForOneTick;
            }

            /* 计算"等待 xExpectedIdleTime 个 tick 周期"所需的重装载值。
             * 减 1 是因为这段代码通常是在第一个 tick 周期进行到一半时执行的。
             * 但如果 SysTick 中断已经挂起，就清掉它（抑制掉第一个 tick），
             * 并修正重装载值（此时第二个 tick 周期已经开始了）。
             * 期望空闲时间至少是 2 个 tick。 */
            ulReloadValue = ulSysTickDecrementsLeft + ( ulTimerCountsForOneTick * ( xExpectedIdleTime - 1UL ) );

            if( ( portNVIC_INT_CTRL_REG & portNVIC_PEND_SYSTICK_SET_BIT ) != 0 )
            {
                portNVIC_INT_CTRL_REG = portNVIC_PEND_SYSTICK_CLEAR_BIT;
                ulReloadValue -= ulTimerCountsForOneTick;
            }

            if( ulReloadValue > ulStoppedTimerCompensation )
            {
                ulReloadValue -= ulStoppedTimerCompensation;
            }

            /* 写入新的重装载值。 */
            portNVIC_SYSTICK_LOAD_REG = ulReloadValue;

            /* 清 SysTick 计数标志，并把当前计数值清零。 */
            portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;

            /* 重新启动 SysTick。 */
            portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;

            /* 睡到"有事情发生"为止。configPRE_SLEEP_PROCESSING() 可以把它的
             * 参数置 0，表示"它自己内部已经包含了等待中断/事件的指令"，
             * 此时不该再执行 wfi。但原始的空闲时间变量必须保持不变，所以先拷一份。 */
            xModifiableIdleTime = xExpectedIdleTime;
            configPRE_SLEEP_PROCESSING( xModifiableIdleTime );

            if( xModifiableIdleTime > 0 )
            {
                __dsb( portSY_FULL_READ_WRITE );
                __wfi();        /* 等待中断：CPU 进入睡眠 */
                __isb( portSY_FULL_READ_WRITE );
            }

            configPOST_SLEEP_PROCESSING( xExpectedIdleTime );

            /* 重新开中断，让"把 MCU 唤醒的那个中断"能立刻执行
             * （原因见上面 __disable_irq() 处的注释）。 */
            __enable_irq();
            __dsb( portSY_FULL_READ_WRITE );
            __isb( portSY_FULL_READ_WRITE );

            /* 再次关中断：因为时钟马上要停，而"时钟停止期间执行的中断"会加大
             * RTOS 时间与日历时间之间的偏差。 */
            __disable_irq();
            __dsb( portSY_FULL_READ_WRITE );
            __isb( portSY_FULL_READ_WRITE );

            /* 关闭 SysTick 时钟。这里不读 portNVIC_SYSTICK_CTRL_REG，
             * 以免把已经置位的 COUNT_FLAG 位清掉。同样地，停表期间的时间
             * 会尽量补偿，但 tickless 模式难免带来微小漂移。 */
            portNVIC_SYSTICK_CTRL_REG = ( portNVIC_SYSTICK_CLK_BIT_CONFIG | portNVIC_SYSTICK_INT_BIT );

            /* 判断 SysTick 是否已经数到 0。 */
            if( ( portNVIC_SYSTICK_CTRL_REG & portNVIC_SYSTICK_COUNT_FLAG_BIT ) != 0 )
            {
                uint32_t ulCalculatedLoadValue;

                /* 是节拍中断结束了睡眠（或它现在挂起），新的 tick 周期已经开始。
                 * 用"新 tick 周期剩余的部分"重设重装载寄存器。 */
                ulCalculatedLoadValue = ( ulTimerCountsForOneTick - 1UL ) - ( ulReloadValue - portNVIC_SYSTICK_CURRENT_VALUE_REG );

                /* 不允许太小、也不允许因"睡眠后钩子耗时过长"或"当前值寄存器为 0"
                 * 而出现下溢的值。 */
                if( ( ulCalculatedLoadValue <= ulStoppedTimerCompensation ) || ( ulCalculatedLoadValue > ulTimerCountsForOneTick ) )
                {
                    ulCalculatedLoadValue = ( ulTimerCountsForOneTick - 1UL );
                }

                portNVIC_SYSTICK_LOAD_REG = ulCalculatedLoadValue;

                /* 由于挂起的那个 tick 会在本函数退出时立刻被处理，
                 * 所以 tick 计数只需前进"等待时间 - 1"。 */
                ulCompleteTickPeriods = xExpectedIdleTime - 1UL;
            }
            else
            {
                /* 是别的中断（不是节拍中断）结束了睡眠。 */

                /* 用 SysTick 当前值判断"距离预期空闲结束还剩多少次递减"。 */
                ulSysTickDecrementsLeft = portNVIC_SYSTICK_CURRENT_VALUE_REG;
                #if ( portNVIC_SYSTICK_CLK_BIT_CONFIG != portNVIC_SYSTICK_CLK_BIT )
                {
                    /* 如果 SysTick 不用内核时钟，当前值寄存器可能仍为 0。
                     * 这种情况下 SysTick 还没从重装载寄存器加载，
                     * 预期空闲时间里剩余的是 ulReloadValue 次递减，而不是 0。 */
                    if( ulSysTickDecrementsLeft == 0 )
                    {
                        ulSysTickDecrementsLeft = ulReloadValue;
                    }
                }
                #endif /* portNVIC_SYSTICK_CLK_BIT_CONFIG */

                /* 计算睡眠时长，折算成"完整 tick 周期数"
                 * （不是 ulReload 那个含半个周期的值）。 */
                ulCompletedSysTickDecrements = ( xExpectedIdleTime * ulTimerCountsForOneTick ) - ulSysTickDecrementsLeft;

                /* 处理器等待期间过去了多少个完整 tick 周期？ */
                ulCompleteTickPeriods = ulCompletedSysTickDecrements / ulTimerCountsForOneTick;

                /* 重装载值设为"一个 tick 周期剩下那部分"对应的计数。 */
                portNVIC_SYSTICK_LOAD_REG = ( ( ulCompleteTickPeriods + 1UL ) * ulTimerCountsForOneTick ) - ulCompletedSysTickDecrements;
            }

            /* 重启 SysTick，让它从 portNVIC_SYSTICK_LOAD_REG 重新开始计数，
             * 然后把该寄存器恢复成标准值。
             * 如果 SysTick 不用内核时钟，就临时切到内核时钟，强制它立刻从
             * portNVIC_SYSTICK_LOAD_REG 加载（而不是等另一个时钟的下个周期），
             * 这样就能马上写入标准值。 */
            portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;
            portNVIC_SYSTICK_CTRL_REG = portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT | portNVIC_SYSTICK_ENABLE_BIT;
            #if ( portNVIC_SYSTICK_CLK_BIT_CONFIG == portNVIC_SYSTICK_CLK_BIT )
            {
                portNVIC_SYSTICK_LOAD_REG = ulTimerCountsForOneTick - 1UL;
            }
            #else
            {
                /* 临时使用内核时钟的目的已达到（见上面的说明），恢复使用另一个时钟。 */
                portNVIC_SYSTICK_CTRL_REG = portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT;

                if( ( portNVIC_SYSTICK_CTRL_REG & portNVIC_SYSTICK_COUNT_FLAG_BIT ) != 0 )
                {
                    /* 那部分 tick 周期已经结束，确保 SysTick 只把它计一次。 */
                    portNVIC_SYSTICK_CURRENT_VALUE_REG = 0;
                }

                portNVIC_SYSTICK_LOAD_REG = ulTimerCountsForOneTick - 1UL;
                portNVIC_SYSTICK_CTRL_REG = portNVIC_SYSTICK_CLK_BIT_CONFIG | portNVIC_SYSTICK_INT_BIT | portNVIC_SYSTICK_ENABLE_BIT;
            }
            #endif /* portNVIC_SYSTICK_CLK_BIT_CONFIG */

            /* 把已经过去的那几个 tick 补上（推进内核节拍计数）。 */
            vTaskStepTick( ulCompleteTickPeriods );

            /* 退出时保持中断开启。 */
            __enable_irq();
        }
    }

#endif /* #if configUSE_TICKLESS_IDLE */

/*-----------------------------------------------------------*/

/*
 * 配置 SysTick 定时器，按要求的频率产生节拍中断。
 *
 * 关键算式：
 *   重装载值 = (SysTick 时钟频率 / 节拍频率) - 1
 *   本工程：72MHz / 1000Hz - 1 = 71999 → 每 72000 个计数产生一次中断 = 1ms
 */
#if ( configOVERRIDE_DEFAULT_TICK_CONFIGURATION == 0 )

    __weak void vPortSetupTimerInterrupt( void )
    {
        /* 计算配置节拍中断所需的常量。 */
        #if ( configUSE_TICKLESS_IDLE == 1 )
        {
            ulTimerCountsForOneTick = ( configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ );
            xMaximumPossibleSuppressedTicks = portMAX_24_BIT_NUMBER / ulTimerCountsForOneTick;
            ulStoppedTimerCompensation = portMISSED_COUNTS_FACTOR / ( configCPU_CLOCK_HZ / configSYSTICK_CLOCK_HZ );
        }
        #endif /* configUSE_TICKLESS_IDLE */

        /* 先停止并清零 SysTick。 */
        portNVIC_SYSTICK_CTRL_REG = 0UL;
        portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;

        /* 配置 SysTick 按要求的频率产生中断。 */
        portNVIC_SYSTICK_LOAD_REG = ( configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ ) - 1UL;
        portNVIC_SYSTICK_CTRL_REG = ( portNVIC_SYSTICK_CLK_BIT_CONFIG | portNVIC_SYSTICK_INT_BIT | portNVIC_SYSTICK_ENABLE_BIT );
    }

#endif /* configOVERRIDE_DEFAULT_TICK_CONFIGURATION */
/*-----------------------------------------------------------*/

/**
 * @brief  读取 IPSR 寄存器（当前异常编号），返回 0 表示"不在中断里"
 */
__asm uint32_t vPortGetIPSR( void )
{
/* *INDENT-OFF* */
    PRESERVE8

    mrs r0, ipsr
    bx r14
/* *INDENT-ON* */
}
/*-----------------------------------------------------------*/

#if ( configASSERT_DEFINED == 1 )

    /**
     * @brief  校验"当前中断的优先级"是否允许调用 FreeRTOS API
     *
     * 规则：会用 FreeRTOS API 的中断，优先级必须【数值 ≥】
     *       configMAX_SYSCALL_INTERRUPT_PRIORITY（本工程为 5，即填 5、6、7…都行）。
     *       如果填了更紧急的优先级（数值更小，例如 0/1/2…），一旦在中断里调用
     *       FromISR 系列 API，内核数据结构就会被破坏 → 随机死机。
     *
     * 另外还检查"优先级分组"必须是全抢占位（NVIC_PriorityGroup_4 的效果）。
     */
    void vPortValidateInterruptPriority( void )
    {
        uint32_t ulCurrentInterrupt;
        uint8_t ucCurrentPriority;

        /* 取出当前正在执行的异常编号。 */
        ulCurrentInterrupt = vPortGetIPSR();

        /* 是不是用户定义的中断（编号 >= 16）？ */
        if( ulCurrentInterrupt >= portFIRST_USER_INTERRUPT_NUMBER )
        {
            /* 查出该中断的优先级。 */
            ucCurrentPriority = pcInterruptPriorityRegisters[ ulCurrentInterrupt ];

            /* 如果某个中断的优先级高于（数值小于）configMAX_SYSCALL_INTERRUPT_PRIORITY，
             * 而它的服务函数却调用了 ISR 安全的 FreeRTOS API，下面的断言就会失败。
             * ISR 安全的 API 只允许从优先级"不高于"（数值不小于）
             * configMAX_SYSCALL_INTERRUPT_PRIORITY 的中断里调用。
             *
             * 中断优先级"数值越小，逻辑优先级越高"，所以要用 FreeRTOS API 的中断，
             * 其优先级数值必须【大于等于】configMAX_SYSCALL_INTERRUPT_PRIORITY。
             *
             * 要用 FreeRTOS API 的中断不能保持默认优先级 0 —— 那是最高优先级，
             * 必然高于 configMAX_SYSCALL_INTERRUPT_PRIORITY，属于非法配置。
             *
             * FreeRTOS 把"任务 API"和"中断 API"分开维护，
             * 是为了让中断进入尽可能快而简单。 */
            configASSERT( ucCurrentPriority >= ucMaxSysCallPriority );
        }

        /* 优先级分组：NVIC 允许把"定义中断优先级的位"拆成抢占优先级位和子优先级位。
         * 为了简单，FreeRTOS 要求所有位都是抢占优先级位（对应
         * NVIC_PriorityGroup_4 的设置）。如果存在子优先级位，下面的断言就会失败。
         *
         * 如果应用只用 CMSIS 库配置中断，那么在启动调度器前调用
         * NVIC_SetPriorityGrouping( 0 ) 即可；但某些厂商库会假定"非 0 的分组设置"，
         * 那种情况下用 0 会导致不可预期的行为。 */
        configASSERT( ( portAIRCR_REG & portPRIORITY_GROUP_MASK ) <= ulMaxPRIGROUPValue );
    }

#endif /* configASSERT_DEFINED */
