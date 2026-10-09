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
 * list.h —— 链表的定义与访问宏（内核与应用程序都能用）
 *
 * 调度器用的就是这套链表实现。它虽然是为调度器量身定做的，但应用代码也能直接用。
 *
 * 要点：
 *   - 链表里只能存"指向链表节点的指针"（ListItem_t*）。
 *     每个 ListItem_t 里带一个数值 xItemValue，绝大多数情况下链表按它升序排列。
 *   - 链表创建时就已经包含一个节点（哨兵 xListEnd），它的值是"能取到的最大值"，
 *     所以永远排在链表最后，起到"结尾标记"的作用。
 *   - 每个节点除了值以外，还有：指向下一个节点的 pxNext、指向上一个节点的 pxPrevious、
 *     指向"自己所属链表"的 pxContainer、以及指向"包含自己的对象"（通常是 TCB）的 pvOwner。
 *     后两个指针是为了让链表操作更高效 —— 节点与它的宿主对象之间形成"双向链接"。
 */

#ifndef LIST_H
#define LIST_H

#ifndef INC_FREERTOS_H
    #error "FreeRTOS.h must be included before list.h"
#endif

/*
 * 链表结构的成员会被中断修改，按理说应该声明成 volatile。
 * 但实际上它们只在"功能上原子"的场景里被修改（在临界区内、或调度器挂起时），
 * 而且要么以引用方式传给函数，要么通过 volatile 变量索引访问。
 * 所以在目前测试过的所有场景里，都可以省略 volatile 以获得一点性能提升，
 * 而不影响功能。
 *
 * 已检查过 IAR / ARM / GCC 在最高优化选项下生成的汇编，结果符合预期。
 * 不过随着编译器技术进步（尤其启用激进的跨模块优化时），将来可能需要 volatile 才能正确优化。
 * 如果真的遇到"编译器把必要代码优化掉导致调度器彻底失效"的情况，
 * 可以在 FreeRTOSConfig.h 里加上下面这行（不含引号）来给链表结构加上 volatile：
 *     #define configLIST_VOLATILE volatile
 * 如果没定义 configLIST_VOLATILE，下面的预处理指令就把它定义成空。
 */
#ifndef configLIST_VOLATILE
    #define configLIST_VOLATILE
#endif /* configSUPPORT_CROSS_MODULE_OPTIMISATION */

/* *INDENT-OFF* */
#ifdef __cplusplus
    extern "C" {
#endif
/* *INDENT-ON* */

/* 这些宏用来往链表结构里写入"已知值"，运行时再检查这些已知值有没有被破坏。
 * 它们能发现"链表数据结构被内存写坏"的情况，
 * 但查不出"配置错误"或"API 用法错误"导致的逻辑问题。 */
#if ( configUSE_LIST_DATA_INTEGRITY_CHECK_BYTES == 0 )
    /* 关闭检查：把所有相关宏定义成"什么都不做"。 */
    #define listFIRST_LIST_ITEM_INTEGRITY_CHECK_VALUE
    #define listSECOND_LIST_ITEM_INTEGRITY_CHECK_VALUE
    #define listFIRST_LIST_INTEGRITY_CHECK_VALUE
    #define listSECOND_LIST_INTEGRITY_CHECK_VALUE
    #define listSET_FIRST_LIST_ITEM_INTEGRITY_CHECK_VALUE( pxItem )
    #define listSET_SECOND_LIST_ITEM_INTEGRITY_CHECK_VALUE( pxItem )
    #define listSET_LIST_INTEGRITY_CHECK_1_VALUE( pxList )
    #define listSET_LIST_INTEGRITY_CHECK_2_VALUE( pxList )
    #define listTEST_LIST_ITEM_INTEGRITY( pxItem )
    #define listTEST_LIST_INTEGRITY( pxList )
#else /* if ( configUSE_LIST_DATA_INTEGRITY_CHECK_BYTES == 0 ) */
    /* 开启检查：这些宏会往结构体里"插入新的成员"。 */
    #define listFIRST_LIST_ITEM_INTEGRITY_CHECK_VALUE     TickType_t xListItemIntegrityValue1;
    #define listSECOND_LIST_ITEM_INTEGRITY_CHECK_VALUE    TickType_t xListItemIntegrityValue2;
    #define listFIRST_LIST_INTEGRITY_CHECK_VALUE          TickType_t xListIntegrityValue1;
    #define listSECOND_LIST_INTEGRITY_CHECK_VALUE         TickType_t xListIntegrityValue2;

/* 这些宏把新成员设成"已知值"。 */
    #define listSET_FIRST_LIST_ITEM_INTEGRITY_CHECK_VALUE( pxItem )     ( pxItem )->xListItemIntegrityValue1 = pdINTEGRITY_CHECK_VALUE
    #define listSET_SECOND_LIST_ITEM_INTEGRITY_CHECK_VALUE( pxItem )    ( pxItem )->xListItemIntegrityValue2 = pdINTEGRITY_CHECK_VALUE
    #define listSET_LIST_INTEGRITY_CHECK_1_VALUE( pxList )              ( pxList )->xListIntegrityValue1 = pdINTEGRITY_CHECK_VALUE
    #define listSET_LIST_INTEGRITY_CHECK_2_VALUE( pxList )              ( pxList )->xListIntegrityValue2 = pdINTEGRITY_CHECK_VALUE

/* 这些宏检查成员是否仍等于预期值，不等就触发断言。 */
    #define listTEST_LIST_ITEM_INTEGRITY( pxItem )                      configASSERT( ( ( pxItem )->xListItemIntegrityValue1 == pdINTEGRITY_CHECK_VALUE ) && ( ( pxItem )->xListItemIntegrityValue2 == pdINTEGRITY_CHECK_VALUE ) )
    #define listTEST_LIST_INTEGRITY( pxList )                           configASSERT( ( ( pxList )->xListIntegrityValue1 == pdINTEGRITY_CHECK_VALUE ) && ( ( pxList )->xListIntegrityValue2 == pdINTEGRITY_CHECK_VALUE ) )
#endif /* configUSE_LIST_DATA_INTEGRITY_CHECK_BYTES */


/*
 * 链表能容纳的唯一一种对象：链表节点（ListItem_t）的定义。
 */
struct xLIST;
struct xLIST_ITEM
{
    listFIRST_LIST_ITEM_INTEGRITY_CHECK_VALUE           /**< 若 configUSE_LIST_DATA_INTEGRITY_CHECK_BYTES 为 1，会设成一个已知值（调试用） */
    configLIST_VOLATILE TickType_t xItemValue;          /**< 节点的排序值：多数情况下链表按它升序排列 */
    struct xLIST_ITEM * configLIST_VOLATILE pxNext;     /**< 指向链表中下一个节点 */
    struct xLIST_ITEM * configLIST_VOLATILE pxPrevious; /**< 指向链表中上一个节点 */
    void * pvOwner;                                     /**< 指向"包含这个节点的对象"（通常是 TCB）：节点与宿主之间是双向链接 */
    struct xLIST * configLIST_VOLATILE pxContainer;     /**< 指向这个节点所在的链表（不在任何链表时为 NULL） */
    listSECOND_LIST_ITEM_INTEGRITY_CHECK_VALUE          /**< 若 configUSE_LIST_DATA_INTEGRITY_CHECK_BYTES 为 1，会设成一个已知值（调试用） */
};
typedef struct xLIST_ITEM ListItem_t;

/* "迷你节点"：只有值 + 前后指针（哨兵 xListEnd 用它，省内存） */
#if ( configUSE_MINI_LIST_ITEM == 1 )
    struct xMINI_LIST_ITEM
    {
        listFIRST_LIST_ITEM_INTEGRITY_CHECK_VALUE /**< 若 configUSE_LIST_DATA_INTEGRITY_CHECK_BYTES 为 1，会设成一个已知值（调试用） */
        configLIST_VOLATILE TickType_t xItemValue;
        struct xLIST_ITEM * configLIST_VOLATILE pxNext;
        struct xLIST_ITEM * configLIST_VOLATILE pxPrevious;
    };
    typedef struct xMINI_LIST_ITEM MiniListItem_t;
#else
    typedef struct xLIST_ITEM      MiniListItem_t;
#endif

/*
 * 调度器使用的"链表"类型定义。
 */
typedef struct xLIST
{
    listFIRST_LIST_INTEGRITY_CHECK_VALUE      /**< 若 configUSE_LIST_DATA_INTEGRITY_CHECK_BYTES 为 1，会设成一个已知值（调试用） */
    configLIST_VOLATILE UBaseType_t uxNumberOfItems;    /**< 当前链表里的节点个数 */
    ListItem_t * configLIST_VOLATILE pxIndex; /**< 遍历用的游标：指向"上一次 listGET_OWNER_OF_NEXT_ENTRY() 返回的那个节点" */
    MiniListItem_t xListEnd;                  /**< 哨兵节点：值取最大，永远在链表末尾，充当"结尾标记" */
    listSECOND_LIST_INTEGRITY_CHECK_VALUE     /**< 若 configUSE_LIST_DATA_INTEGRITY_CHECK_BYTES 为 1，会设成一个已知值（调试用） */
} List_t;

/*
 * 设置节点的"宿主"（owner）—— 宿主就是"包含这个节点的对象"（通常是 TCB）。
 */
#define listSET_LIST_ITEM_OWNER( pxListItem, pxOwner )    ( ( pxListItem )->pvOwner = ( void * ) ( pxOwner ) )

/*
 * 读取节点的"宿主"（owner）。
 */
#define listGET_LIST_ITEM_OWNER( pxListItem )             ( ( pxListItem )->pvOwner )

/*
 * 设置节点的排序值 xItemValue。多数情况下这个值决定它在链表中的升序位置。
 */
#define listSET_LIST_ITEM_VALUE( pxListItem, xValue )     ( ( pxListItem )->xItemValue = ( xValue ) )

/*
 * 读取节点的排序值。它可以是任何含义 ——
 * 比如任务的优先级，或者"任务该被唤醒的时刻"。
 */
#define listGET_LIST_ITEM_VALUE( pxListItem )             ( ( pxListItem )->xItemValue )

/*
 * 读取"链表头节点"的排序值（即当前排在最前面的那个节点的值）。
 */
#define listGET_ITEM_VALUE_OF_HEAD_ENTRY( pxList )        ( ( ( pxList )->xListEnd ).pxNext->xItemValue )

/*
 * 返回链表头节点（排在最前面的那个节点）。
 */
#define listGET_HEAD_ENTRY( pxList )                      ( ( ( pxList )->xListEnd ).pxNext )

/*
 * 返回下一个节点。
 */
#define listGET_NEXT( pxListItem )                        ( ( pxListItem )->pxNext )

/*
 * 返回链表的"结尾标记"节点（哨兵 xListEnd）。
 */
#define listGET_END_MARKER( pxList )                      ( ( ListItem_t const * ) ( &( ( pxList )->xListEnd ) ) )

/*
 * 判断链表是否为空（为空时宏值为真）。
 */
#define listLIST_IS_EMPTY( pxList )                       ( ( ( pxList )->uxNumberOfItems == ( UBaseType_t ) 0 ) ? pdTRUE : pdFALSE )

/*
 * 返回链表里的节点个数。
 */
#define listCURRENT_LIST_LENGTH( pxList )                 ( ( pxList )->uxNumberOfItems )

/*
 * 取"下一个节点的宿主"（内核靠它实现同优先级任务轮转）。
 *
 * 原理：链表成员 pxIndex 用来遍历链表。每次调用本宏，
 *       它把 pxIndex 前移到下一个节点，并返回该节点的 pvOwner。
 *       于是反复调用就能遍历链表里的每一个节点。
 *
 * 节点的 pvOwner 指向"包含该节点的对象"（调度器里就是 TCB），
 * 因此节点与其宿主之间形成了双向链接。
 *
 * @param pxTCB  输出：被置为"下一个节点的宿主"的地址（通常是 TCB）
 * @param pxList 要遍历的链表
 */
#if ( configNUMBER_OF_CORES == 1 )
    #define listGET_OWNER_OF_NEXT_ENTRY( pxTCB, pxList )                                       \
    do {                                                                                       \
        List_t * const pxConstList = ( pxList );                                               \
        /* 游标前移到下一个节点并返回它，同时保证不返回链表尾部的哨兵标记。 */                 \
        ( pxConstList )->pxIndex = ( pxConstList )->pxIndex->pxNext;                           \
        if( ( void * ) ( pxConstList )->pxIndex == ( void * ) &( ( pxConstList )->xListEnd ) ) \
        {                                                                                      \
            ( pxConstList )->pxIndex = ( pxConstList )->xListEnd.pxNext;                       \
        }                                                                                      \
        ( pxTCB ) = ( pxConstList )->pxIndex->pvOwner;                                         \
    } while( 0 )
#else /* #if ( configNUMBER_OF_CORES == 1 ) */

/* 多核（SMP）用不到这个功能：FreeRTOS SMP 调度器不使用 pxIndex，
 * 它应当始终指向 xListEnd。因此这里故意不定义该宏，以防有人误更新 pxIndex。
 */
#endif /* #if ( configNUMBER_OF_CORES == 1 ) */

/*
 * uxListRemove() 的"不返回值"版本，做成内联宏是为了让 xTaskIncrementTick() 稍微快一点。
 *
 * 从链表中删除一个节点。节点自己记录了它所在的链表，
 * 所以只需要把节点传进来即可。
 *
 * @param pxItemToRemove 要删除的节点：它会把自己从 pxContainer 指向的链表中移除
 * （删除后链表里剩余的节点个数不再返回）
 */
#define listREMOVE_ITEM( pxItemToRemove ) \
    do {                                  \
        /* 节点知道自己挂在哪个链表上，直接取出来。 */                                              \
        List_t * const pxList = ( pxItemToRemove )->pxContainer;                                    \
                                                                                                    \
        ( pxItemToRemove )->pxNext->pxPrevious = ( pxItemToRemove )->pxPrevious;                    \
        ( pxItemToRemove )->pxPrevious->pxNext = ( pxItemToRemove )->pxNext;                        \
        /* 保证 pxIndex 仍然指向一个有效节点。 */                                                   \
        if( pxList->pxIndex == ( pxItemToRemove ) )                                                 \
        {                                                                                           \
            pxList->pxIndex = ( pxItemToRemove )->pxPrevious;                                       \
        }                                                                                           \
                                                                                                    \
        ( pxItemToRemove )->pxContainer = NULL;                                                     \
        ( ( pxList )->uxNumberOfItems ) = ( UBaseType_t ) ( ( ( pxList )->uxNumberOfItems ) - 1U ); \
    } while( 0 )

/*
 * vListInsertEnd() 的内联版本，用于让 xTaskIncrementTick() 稍微快一点。
 *
 * 把节点插入链表。插入位置使得它"在多次调用 listGET_OWNER_OF_NEXT_ENTRY 时
 * 最后一个被取到"。
 *
 * 链表成员 pxIndex 用来遍历：调用 listGET_OWNER_OF_NEXT_ENTRY 会把 pxIndex 前移。
 * 用 vListInsertEnd 插入，实际就是把节点放在 pxIndex 当前指向的位置上 ——
 * 这意味着链表里其它每个节点都会先被 listGET_OWNER_OF_NEXT_ENTRY 取到，
 * 然后才轮到刚插入的这个节点。
 *
 * @param pxList        要插入的链表
 * @param pxNewListItem 要插入的新节点
 */
#define listINSERT_END( pxList, pxNewListItem )           \
    do {                                                  \
        ListItem_t * const pxIndex = ( pxList )->pxIndex; \
                                                          \
        /* 只有定义了 configASSERT() 时这些检查才有效，用于发现链表结构被内存写坏； \
         * 查不出"配置错误"或"API 用法错误"导致的问题。 */                                      \
        listTEST_LIST_INTEGRITY( ( pxList ) );                                  \
        listTEST_LIST_ITEM_INTEGRITY( ( pxNewListItem ) );                      \
                                                                                \
        /* 把新节点插入 ( pxList )：不排序，而是让它成为 \
         * "下一次 listGET_OWNER_OF_NEXT_ENTRY() 最后被取到"的那个节点。 */                       \
        ( pxNewListItem )->pxNext = pxIndex;                                                        \
        ( pxNewListItem )->pxPrevious = pxIndex->pxPrevious;                                        \
                                                                                                    \
        pxIndex->pxPrevious->pxNext = ( pxNewListItem );                                            \
        pxIndex->pxPrevious = ( pxNewListItem );                                                    \
                                                                                                    \
        /* 记住这个节点挂在哪个链表上。 */                                                          \
        ( pxNewListItem )->pxContainer = ( pxList );                                                \
                                                                                                    \
        ( ( pxList )->uxNumberOfItems ) = ( UBaseType_t ) ( ( ( pxList )->uxNumberOfItems ) + 1U ); \
    } while( 0 )

/*
 * 取"链表头节点的宿主"（链表通常按值升序排列，所以头节点就是最小的那个）。
 *
 * 返回头节点的 pvOwner（通常是 TCB）。
 *
 * @param pxList 要查询的链表
 */
#define listGET_OWNER_OF_HEAD_ENTRY( pxList )            ( ( &( ( pxList )->xListEnd ) )->pxNext->pvOwner )

/*
 * 检查某个节点是否在指定链表中：节点里维护了 pxContainer，
 * 本宏只是比较"节点的 container"和"给定链表"是否一致。
 *
 * @param pxList     要检查的链表
 * @param pxListItem 要检查的节点
 * @return 在链表中返回 pdTRUE，否则 pdFALSE
 */
#define listIS_CONTAINED_WITHIN( pxList, pxListItem )    ( ( ( pxListItem )->pxContainer == ( pxList ) ) ? ( pdTRUE ) : ( pdFALSE ) )

/*
 * 返回节点所在的链表（即它的 pxContainer）。
 *
 * @param pxListItem 要查询的节点
 * @return 指向"引用该节点的 List_t 对象"的指针
 */
#define listLIST_ITEM_CONTAINER( pxListItem )            ( ( pxListItem )->pxContainer )

/*
 * 粗略判断"链表是否已被初始化"：
 * vListInitialise() 会把 pxList->xListEnd.xItemValue 设为 portMAX_DELAY。
 */
#define listLIST_IS_INITIALISED( pxList )                ( ( pxList )->xListEnd.xItemValue == portMAX_DELAY )

/*
 * 使用链表之前必须调用！它初始化链表结构的所有成员，
 * 并把 xListEnd 节点插入链表，作为"结尾标记"。
 *
 * @param pxList 要初始化的链表
 */
void vListInitialise( List_t * const pxList ) PRIVILEGED_FUNCTION;

/*
 * 使用节点之前必须调用！它把节点的 container 置空，
 * 避免节点"误以为"自己已经在某个链表里了。
 *
 * @param pxItem 要初始化的节点
 */
void vListInitialiseItem( ListItem_t * const pxItem ) PRIVILEGED_FUNCTION;

/*
 * 把节点插入链表，插入位置由它的 xItemValue 决定（按值升序）。
 *
 * @param pxList        要插入的链表
 * @param pxNewListItem 要插入的节点
 */
void vListInsert( List_t * const pxList,
                  ListItem_t * const pxNewListItem ) PRIVILEGED_FUNCTION;

/*
 * 把节点插入链表，位置使得它"在多次调用 listGET_OWNER_OF_NEXT_ENTRY 时最后一个被取到"。
 *
 * 链表成员 pxIndex 用来遍历：调用 listGET_OWNER_OF_NEXT_ENTRY 会把 pxIndex 前移。
 * 用 vListInsertEnd 插入，实际就是把节点放在 pxIndex 当前指向的位置 ——
 * 这意味着链表里其它每个节点都会先被取到，然后才轮到刚插入的节点。
 *
 * @param pxList        要插入的链表
 * @param pxNewListItem 要插入的节点
 */
void vListInsertEnd( List_t * const pxList,
                     ListItem_t * const pxNewListItem ) PRIVILEGED_FUNCTION;

/*
 * 从链表中删除一个节点。节点自己记录了它所在的链表，
 * 所以只需要把节点传进来即可。
 *
 * @param pxItemToRemove 要删除的节点：它会把自己从 pxContainer 指向的链表中移除
 * @return 删除后链表里剩余的节点个数
 */
UBaseType_t uxListRemove( ListItem_t * const pxItemToRemove ) PRIVILEGED_FUNCTION;

/* *INDENT-OFF* */
#ifdef __cplusplus
    }
#endif
/* *INDENT-ON* */

#endif /* ifndef LIST_H */
