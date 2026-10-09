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
 * list.c —— 链表的实现（FreeRTOS 内核的"地基"）
 *
 * 内核里所有"排队"的东西都是用这个链表做的：
 *   - 就绪链表：按优先级排队等待运行的任务
 *   - 延时链表：按"唤醒时刻"排队正在延时的任务
 *   - 事件链表：等待某个事件（队列/信号量）的任务
 *
 * 链表是【环形双向链表 + 哨兵节点(xListEnd)】：
 *   最后一个真节点的 pxNext 指向哨兵，哨兵的 pxNext 指向第一个真节点，
 *   所以链表中没有 NULL，判断"走到头"的方式是"回到了哨兵"。
 */

#include <stdlib.h>

/* 定义 MPU_WRAPPERS_INCLUDED_FROM_API_FILE 是为了阻止 task.h 把下面所有 API
 * 重新定义成 MPU 包装版本——这个宏只应在"应用程序包含 task.h"时生效。
 * （本芯片没有 MPU，这一段对 F103 无实际影响，保持原样即可） */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "FreeRTOS.h"
#include "list.h"

/* 上面两个头文件需要 MPU_WRAPPERS_INCLUDED_FROM_API_FILE 处于定义状态，
 * 但本文件自身不需要，所以在这里取消定义。 */
#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

/*-----------------------------------------------------------
* 链表的公开 API（详细说明见 list.h）
*----------------------------------------------------------*/

/**
 * @brief  初始化一个链表（把它变成"空链表"）
 * @param  pxList 要初始化的链表
 *
 * 做法：把哨兵 xListEnd 挂成链表里唯一的节点，并让它自己指向自己，
 *       这样"空链表"的判断就是 xListEnd.pxNext == &xListEnd。
 */
void vListInitialise( List_t * const pxList )
{
    traceENTER_vListInitialise( pxList );

    /* 链表结构里有一个"哨兵节点"，用来标记链表的结尾。
     * 初始化时把它作为唯一的节点插进去（此时链表是空的）。 */
    pxList->pxIndex = ( ListItem_t * ) &( pxList->xListEnd );

    listSET_FIRST_LIST_ITEM_INTEGRITY_CHECK_VALUE( &( pxList->xListEnd ) );

    /* 哨兵的排序值是"能取到的最大值"，保证它永远排在链表最后面。 */
    pxList->xListEnd.xItemValue = portMAX_DELAY;

    /* 哨兵的前后指针都指向自己 —— 这样我们就知道链表是空的。 */
    pxList->xListEnd.pxNext = ( ListItem_t * ) &( pxList->xListEnd );
    pxList->xListEnd.pxPrevious = ( ListItem_t * ) &( pxList->xListEnd );

    /* 当哨兵是完整的 ListItem_t 类型时，初始化它剩下的成员
     * （configUSE_MINI_LIST_ITEM 为 1 时哨兵是"迷你节点"，没有这些成员） */
    #if ( configUSE_MINI_LIST_ITEM == 0 )
    {
        pxList->xListEnd.pvOwner = NULL;
        pxList->xListEnd.pxContainer = NULL;
        listSET_SECOND_LIST_ITEM_INTEGRITY_CHECK_VALUE( &( pxList->xListEnd ) );
    }
    #endif

    /* 链表里的节点数 = 0 */
    pxList->uxNumberOfItems = ( UBaseType_t ) 0U;

    /* 如果打开了"链表数据完整性检查"（configUSE_LIST_DATA_INTEGRITY_CHECK_BYTES=1），
     * 就往链表里写入已知值，用于检测数据被踩（调试用） */
    listSET_LIST_INTEGRITY_CHECK_1_VALUE( pxList );
    listSET_LIST_INTEGRITY_CHECK_2_VALUE( pxList );

    traceRETURN_vListInitialise();
}
/*-----------------------------------------------------------*/

/**
 * @brief  初始化一个链表节点
 * @param  pxItem 要初始化的节点（通常是任务的状态节点、队列的等待节点）
 *
 * 只做一件事：把节点的 pxContainer 清空 —— 表示"这个节点目前不在任何链表上"。
 */
void vListInitialiseItem( ListItem_t * const pxItem )
{
    traceENTER_vListInitialiseItem( pxItem );

    /* 确保这个节点没有被记录成"在某个链表上"。 */
    pxItem->pxContainer = NULL;

    /* 若开启完整性检查，写入已知值（调试用，检测内存被踩）。 */
    listSET_FIRST_LIST_ITEM_INTEGRITY_CHECK_VALUE( pxItem );
    listSET_SECOND_LIST_ITEM_INTEGRITY_CHECK_VALUE( pxItem );

    traceRETURN_vListInitialiseItem();
}
/*-----------------------------------------------------------*/

/**
 * @brief  把节点插入链表"末尾"（不排序）
 * @param  pxList        目标链表
 * @param  pxNewListItem 要插入的新节点
 *
 * 注意：这里的"末尾"不是排序意义的末尾，而是"插在 pxIndex 之前"。
 * 内核用它实现"同优先级任务轮转"：新任务加到队尾，
 * 于是 listGET_OWNER_OF_NEXT_ENTRY() 能依次取到每个任务 → 这就是时间片的基础。
 */
void vListInsertEnd( List_t * const pxList,
                     ListItem_t * const pxNewListItem )
{
    ListItem_t * const pxIndex = pxList->pxIndex;

    traceENTER_vListInsertEnd( pxList, pxNewListItem );

    /* 下面这些检查只有在定义了 configASSERT() 时才有实际作用，
     * 用于发现"链表数据结构被内存写坏"的情况。
     * 它们查不出"配置错误"或"API 用法错误"导致的逻辑问题。 */
    listTEST_LIST_INTEGRITY( pxList );
    listTEST_LIST_ITEM_INTEGRITY( pxNewListItem );

    /* 把新节点插到 pxList 里：不做排序，
     * 而是让新节点成为"下一次调用 listGET_OWNER_OF_NEXT_ENTRY() 时最后被取到"的那个。 */
    pxNewListItem->pxNext = pxIndex;
    pxNewListItem->pxPrevious = pxIndex->pxPrevious;

    /* 仅在"判定覆盖测试"时使用（正常编译为空操作）。 */
    mtCOVERAGE_TEST_DELAY();

    pxIndex->pxPrevious->pxNext = pxNewListItem;
    pxIndex->pxPrevious = pxNewListItem;

    /* 记住这个节点挂在哪个链表上（方便以后快速删除）。 */
    pxNewListItem->pxContainer = pxList;

    /* 节点数 +1 */
    ( pxList->uxNumberOfItems ) = ( UBaseType_t ) ( pxList->uxNumberOfItems + 1U );

    traceRETURN_vListInsertEnd();
}
/*-----------------------------------------------------------*/

/**
 * @brief  把节点按 xItemValue 升序插入链表（插入排序）；相同值插在已有相同值之后
 * @param  pxList        目标链表
 * @param  pxNewListItem 要插入的新节点
 *
 * 内核用它实现：
 *   - 延时链表：按"唤醒时刻"排序 → 内核每个 tick 只需看链头就知道谁该醒了
 *   - 事件链表：按"优先级"排序 → 取链头就是最高优先级的等待者
 */
void vListInsert( List_t * const pxList,
                  ListItem_t * const pxNewListItem )
{
    ListItem_t * pxIterator;
    const TickType_t xValueOfInsertion = pxNewListItem->xItemValue;

    traceENTER_vListInsert( pxList, pxNewListItem );

    /* 完整性检查（仅定义 configASSERT() 时有效，用于发现内存被踩）。 */
    listTEST_LIST_INTEGRITY( pxList );
    listTEST_LIST_ITEM_INTEGRITY( pxNewListItem );

    /* 把新节点按 xItemValue 升序插入。
     *
     * 如果链表中已经有相同 xItemValue 的节点，新节点要插在它【后面】。
     * （这样同一优先级就绪链表里的多个任务能轮流分到 CPU。）
     * 但如果 xItemValue 恰好等于哨兵的排序值，下面的循环会永远走不完，
     * 所以先单独判断这种情况。 */
    if( xValueOfInsertion == portMAX_DELAY )
    {
        /* 值最大 → 直接插到哨兵前面（即链表最后） */
        pxIterator = pxList->xListEnd.pxPrevious;
    }
    else
    {
        /* *** 注意 ***********************************************************
        *  如果你的程序在这里崩溃，常见原因如下（也可参考 FreeRTOS 官网 FAQ）：
        *
        *   1) 栈溢出
        *   2) 中断优先级配置错误 —— 尤其在 Cortex-M 上，中断优先级"数值越大
        *      实际级别越低"，很容易反直觉。参见 configMAX_SYSCALL_INTERRUPT_PRIORITY
        *   3) 在临界区里、或调度器挂起时调用 API；
        *      或在中断里调用了名字不带 "FromISR" 的 API
        *   4) 队列/信号量还没初始化就使用，或调度器还没启动就使用
        *      （是不是在 vTaskStartScheduler() 之前就有中断在跑了？）
        *   5) 支持中断嵌套的移植层里，节拍中断的优先级必须 ≤
        *      configMAX_SYSCALL_INTERRUPT_PRIORITY
        **********************************************************************/

        /* 从哨兵下一个节点开始，只要"下一个节点的值 ≤ 待插入的值"就一直往后走，
         * 走到的位置就是新节点应该插入的位置（保证升序）。 */
        for( pxIterator = ( ListItem_t * ) &( pxList->xListEnd ); pxIterator->pxNext->xItemValue <= xValueOfInsertion; pxIterator = pxIterator->pxNext )
        {
            /* 循环体里什么都不做，只是"往后走"找插入位置。 */
        }
    }

    /* 改四个指针完成插入（这就是"链表插入不用搬数据"的原因） */
    pxNewListItem->pxNext = pxIterator->pxNext;
    pxNewListItem->pxNext->pxPrevious = pxNewListItem;
    pxNewListItem->pxPrevious = pxIterator;
    pxIterator->pxNext = pxNewListItem;

    /* 记住这个节点在哪个链表上，以后可以快速删除。 */
    pxNewListItem->pxContainer = pxList;

    /* 节点数 +1 */
    ( pxList->uxNumberOfItems ) = ( UBaseType_t ) ( pxList->uxNumberOfItems + 1U );

    traceRETURN_vListInsert();
}
/*-----------------------------------------------------------*/


/**
 * @brief  把节点从它所在的链表中摘除
 * @param  pxItemToRemove 要摘除的节点
 * @return 摘除后链表里剩余的节点个数
 *
 * 注意：节点自己知道自己挂在哪个链表上（pxContainer），所以不用传链表进来。
 * 内核靠它把任务从"就绪链表"摘下来放进"延时链表"。
 */
UBaseType_t uxListRemove( ListItem_t * const pxItemToRemove )
{
    /* 节点里记录了它所在的链表，直接取出来。 */
    List_t * const pxList = pxItemToRemove->pxContainer;

    traceENTER_uxListRemove( pxItemToRemove );

    /* 让"前一个"和"后一个"直接连起来，从而跳过自己（改两个指针即可） */
    pxItemToRemove->pxNext->pxPrevious = pxItemToRemove->pxPrevious;
    pxItemToRemove->pxPrevious->pxNext = pxItemToRemove->pxNext;

    /* 仅在"判定覆盖测试"时使用。 */
    mtCOVERAGE_TEST_DELAY();

    /* 保证 pxIndex 仍然指向一个有效的节点（如果刚才删掉的正是它指向的节点，
     * 就让它前移一个）。 */
    if( pxList->pxIndex == pxItemToRemove )
    {
        pxList->pxIndex = pxItemToRemove->pxPrevious;
    }
    else
    {
        mtCOVERAGE_TEST_MARKER();
    }

    /* 节点已经不在任何链表上了 */
    pxItemToRemove->pxContainer = NULL;
    /* 节点数 -1 */
    ( pxList->uxNumberOfItems ) = ( UBaseType_t ) ( pxList->uxNumberOfItems - 1U );

    traceRETURN_uxListRemove( pxList->uxNumberOfItems );

    /* 返回剩余节点数（可以据此判断"链表是不是空了"） */
    return pxList->uxNumberOfItems;
}
/*-----------------------------------------------------------*/
