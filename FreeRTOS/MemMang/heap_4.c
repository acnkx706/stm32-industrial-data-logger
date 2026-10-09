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
 * heap_4.c —— 内存管理（本工程使用的方案）
 *
 * 它是 pvPortMalloc() 和 vPortFree() 的一种实现：
 *   在释放内存块时，会把【相邻】的空闲块合并（coalesce）成一块，
 *   从而尽量抑制内存碎片。
 *
 * 对比其它方案（想换只需把工程里的 heap_4.c 换成对应文件）：
 *   heap_1.c  只能分配、不能释放（最简单，最省代码）
 *   heap_2.c  能释放，但【不能合并】相邻空闲块 → 容易碎片化（已淘汰）
 *   heap_3.c  直接用标准库的 malloc/free（需要自己在启动文件里加大堆）
 *   heap_4.c  能释放 + 能合并（最常用，本工程用它）
 *   heap_5.c  同 heap_4，但支持【多个不连续】的内存区域
 *
 * 核心数据结构：把堆（一个大数组 ucHeap）切成若干块，
 * 空闲块用"地址从小到大"的单向链表串起来（BlockLink_t）。
 * 分配 = 在链表里找足够大的块（可能需要切分）；
 * 释放 = 把块插回链表，并尝试与前后相邻的块合并。
 */
#include <stdlib.h>
#include <string.h>

/* 定义 MPU_WRAPPERS_INCLUDED_FROM_API_FILE 是为了阻止 task.h 把 API 重新定义成 MPU 包装版。
 * 该宏只应在"应用程序包含 task.h"时生效。 */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "FreeRTOS.h"
#include "task.h"

#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#if ( configSUPPORT_DYNAMIC_ALLOCATION == 0 )
    #error This file must not be used if configSUPPORT_DYNAMIC_ALLOCATION is 0
#endif

/* 释放时是否把内存清零（默认关闭；开启可帮助发现"用了已释放内存"的问题） */
#ifndef configHEAP_CLEAR_MEMORY_ON_FREE
    #define configHEAP_CLEAR_MEMORY_ON_FREE    0
#endif

/* 一个块被切分后，剩下的部分不能太小（至少要能装下块头 + 一点数据） */
#define heapMINIMUM_BLOCK_SIZE    ( ( size_t ) ( xHeapStructSize << 1 ) )

/* 假定 1 字节 = 8 位 */
#define heapBITS_PER_BYTE         ( ( size_t ) 8 )

/* size_t 能表示的最大值 */
#define heapSIZE_MAX              ( ~( ( size_t ) 0 ) )

/* 检查 a*b 是否会溢出 */
#define heapMULTIPLY_WILL_OVERFLOW( a, b )     ( ( ( a ) > 0 ) && ( ( b ) > ( heapSIZE_MAX / ( a ) ) ) )

/* 检查 a+b 是否会溢出 */
#define heapADD_WILL_OVERFLOW( a, b )          ( ( a ) > ( heapSIZE_MAX - ( b ) ) )

/* 检查 a-b 是否会下溢 */
#define heapSUBTRACT_WILL_UNDERFLOW( a, b )    ( ( a ) < ( b ) )

/* BlockLink_t 结构里 xBlockSize 的最高位（MSB）被借用来标记"这个块是否已分配"：
 *   MSB = 1 → 该块已经分配给应用（在用）
 *   MSB = 0 → 该块还是空闲的（在空闲链表里）
 * 这样就不用额外增加一个成员变量来记录分配状态，省内存。 */
#define heapBLOCK_ALLOCATED_BITMASK    ( ( ( size_t ) 1 ) << ( ( sizeof( size_t ) * heapBITS_PER_BYTE ) - 1 ) )
#define heapBLOCK_SIZE_IS_VALID( xBlockSize )    ( ( ( xBlockSize ) & heapBLOCK_ALLOCATED_BITMASK ) == 0 )
#define heapBLOCK_IS_ALLOCATED( pxBlock )        ( ( ( pxBlock->xBlockSize ) & heapBLOCK_ALLOCATED_BITMASK ) != 0 )
#define heapALLOCATE_BLOCK( pxBlock )            ( ( pxBlock->xBlockSize ) |= heapBLOCK_ALLOCATED_BITMASK )
#define heapFREE_BLOCK( pxBlock )                ( ( pxBlock->xBlockSize ) &= ~heapBLOCK_ALLOCATED_BITMASK )

/*-----------------------------------------------------------*/

/* 堆的内存本体（就是一个大数组） */
#if ( configAPPLICATION_ALLOCATED_HEAP == 1 )

/* 应用层已经自己定义了 RTOS 堆数组 —— 通常是想把它放到特定段或指定地址。 */
    extern uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];
#else
    /* 默认：内核自己定义这个大数组，大小 = configTOTAL_HEAP_SIZE（本工程 10KB） */
    PRIVILEGED_DATA static uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];
#endif /* configAPPLICATION_ALLOCATED_HEAP */

/* 块头结构：用来把"空闲块"按内存地址顺序串成链表。 */
typedef struct A_BLOCK_LINK
{
    struct A_BLOCK_LINK * pxNextFreeBlock; /**< 指向下一个空闲块 */
    size_t xBlockSize;                     /**< 本块的大小（最高位另作"是否已分配"标记） */
} BlockLink_t;

/* 把 configENABLE_HEAP_PROTECTOR 设为 1 可以开启"堆指针保护"：
 * 用应用提供的随机数（金丝雀值）对块指针做异或，一旦发生堆溢出，
 * 指针就会变成不可预测的值，从而被 heapVALIDATE_BLOCK_POINTER 断言抓住。 */
#if ( configENABLE_HEAP_PROTECTOR == 1 )

/**
 * @brief 由应用提供的函数：返回一个随机值作为"金丝雀"（用于保护堆指针）
 *
 * @param pxHeapCanary [输出] 返回金丝雀值
 */
    extern void vApplicationGetRandomHeapCanary( portPOINTER_SIZE_TYPE * pxHeapCanary );

/* 用于保护内部堆指针的金丝雀值 */
    PRIVILEGED_DATA static portPOINTER_SIZE_TYPE xHeapCanary;

/* 读写 BlockLink_t 指针的宏：把指针与随机金丝雀异或后再存/取。
 * 这样一旦发生堆溢出，指针值会变得随机不可预测，从而被断言抓到。 */
    #define heapPROTECT_BLOCK_POINTER( pxBlock )    ( ( BlockLink_t * ) ( ( ( portPOINTER_SIZE_TYPE ) ( pxBlock ) ) ^ xHeapCanary ) )
#else

    #define heapPROTECT_BLOCK_POINTER( pxBlock )    ( pxBlock )

#endif /* configENABLE_HEAP_PROTECTOR */

/* 断言：检查堆块指针是否落在堆的有效范围内 */
#define heapVALIDATE_BLOCK_POINTER( pxBlock )                          \
    configASSERT( ( ( uint8_t * ) ( pxBlock ) >= &( ucHeap[ 0 ] ) ) && \
                  ( ( uint8_t * ) ( pxBlock ) <= &( ucHeap[ configTOTAL_HEAP_SIZE - 1 ] ) ) )

/*-----------------------------------------------------------*/

/*
 * 把一个"刚被释放"的块插入空闲链表的正确位置（按地址排序）。
 * 如果它与前一块或后一块的内存地址相邻，就把它们合并成一块。
 */
static void prvInsertBlockIntoFreeList( BlockLink_t * pxBlockToInsert ) PRIVILEGED_FUNCTION;

/*
 * 第一次调用 pvPortMalloc() 时自动调用，用来初始化堆结构。
 */
static void prvHeapInit( void ) PRIVILEGED_FUNCTION;

/*-----------------------------------------------------------*/

/* 每个已分配内存块的开头都会放一个 BlockLink_t 结构，它的长度必须按字节对齐 */
static const size_t xHeapStructSize = ( sizeof( BlockLink_t ) + ( ( size_t ) ( portBYTE_ALIGNMENT - 1 ) ) ) & ~( ( size_t ) portBYTE_ALIGNMENT_MASK );

/* 链表头和链表尾的标记（xStart 是头，pxEnd 指向堆末尾的结束标记） */
PRIVILEGED_DATA static BlockLink_t xStart;
PRIVILEGED_DATA static BlockLink_t * pxEnd = NULL;

/* 统计数据：分别记录"分配/释放次数"和"剩余空闲字节数"（但不反映碎片情况） */
PRIVILEGED_DATA static size_t xFreeBytesRemaining = ( size_t ) 0U;
PRIVILEGED_DATA static size_t xMinimumEverFreeBytesRemaining = ( size_t ) 0U;
PRIVILEGED_DATA static size_t xNumberOfSuccessfulAllocations = ( size_t ) 0U;
PRIVILEGED_DATA static size_t xNumberOfSuccessfulFrees = ( size_t ) 0U;

/*-----------------------------------------------------------*/

/**
 * @brief  从 RTOS 堆里分配内存（相当于内核版的 malloc）
 * @param  xWantedSize 需要的字节数
 * @return 分配到的内存地址；失败（堆不够）返回 NULL
 *
 * 流程：
 *   1) 把申请大小"加上块头大小"并做字节对齐（实际占用会比申请的大）
 *   2) 挂起调度器（保护堆链表：注意这里用的是"挂起调度器"，不是关中断，
 *      因为查找空闲块可能耗时，关中断会影响实时性）
 *   3) 从空闲链表里找第一个足够大的块；若剩余部分够大，就把它切成两块
 *   4) 标记该块为"已分配"，返回"块头之后"的地址给调用者
 */
void * pvPortMalloc( size_t xWantedSize )
{
    BlockLink_t * pxBlock;
    BlockLink_t * pxPreviousBlock;
    BlockLink_t * pxNewBlockLink;
    void * pvReturn = NULL;
    size_t xAdditionalRequiredSize;

    if( xWantedSize > 0 )
    {
        /* 申请的大小要增加 —— 因为除了数据本身，还要在开头放一个 BlockLink_t 结构。 */
        if( heapADD_WILL_OVERFLOW( xWantedSize, xHeapStructSize ) == 0 )
        {
            xWantedSize += xHeapStructSize;

            /* 确保块始终按要求的字节数对齐。 */
            if( ( xWantedSize & portBYTE_ALIGNMENT_MASK ) != 0x00 )
            {
                /* 需要补齐字节对齐。 */
                xAdditionalRequiredSize = portBYTE_ALIGNMENT - ( xWantedSize & portBYTE_ALIGNMENT_MASK );

                if( heapADD_WILL_OVERFLOW( xWantedSize, xAdditionalRequiredSize ) == 0 )
                {
                    xWantedSize += xAdditionalRequiredSize;
                }
                else
                {
                    xWantedSize = 0;
                }
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }
        }
        else
        {
            xWantedSize = 0;
        }
    }
    else
    {
        mtCOVERAGE_TEST_MARKER();
    }

    vTaskSuspendAll();
    {
        /* 如果是第一次调用 malloc，需要初始化堆（建立空闲块链表）。 */
        if( pxEnd == NULL )
        {
            prvHeapInit();
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }

        /* 检查要分配的块大小是否大到把最高位用上了。
         * 块大小的最高位被用来标记"块归谁所有"（应用还是内核），所以必须是 0。 */
        if( heapBLOCK_SIZE_IS_VALID( xWantedSize ) != 0 )
        {
            if( ( xWantedSize > 0 ) && ( xWantedSize <= xFreeBytesRemaining ) )
            {
                /* 从链表头（地址最低的块）开始遍历，直到找到一个足够大的块。 */
                pxPreviousBlock = &xStart;
                pxBlock = heapPROTECT_BLOCK_POINTER( xStart.pxNextFreeBlock );
                heapVALIDATE_BLOCK_POINTER( pxBlock );

                while( ( pxBlock->xBlockSize < xWantedSize ) && ( pxBlock->pxNextFreeBlock != heapPROTECT_BLOCK_POINTER( NULL ) ) )
                {
                    pxPreviousBlock = pxBlock;
                    pxBlock = heapPROTECT_BLOCK_POINTER( pxBlock->pxNextFreeBlock );
                    heapVALIDATE_BLOCK_POINTER( pxBlock );
                }

                /* 如果走到了结束标记，说明没找到足够大的块。 */
                if( pxBlock != pxEnd )
                {
                    /* 返回可用的内存地址 —— 要跳过开头的 BlockLink_t 结构。 */
                    pvReturn = ( void * ) ( ( ( uint8_t * ) heapPROTECT_BLOCK_POINTER( pxPreviousBlock->pxNextFreeBlock ) ) + xHeapStructSize );
                    heapVALIDATE_BLOCK_POINTER( pvReturn );

                    /* 这个块要交给应用使用了，所以必须从空闲链表中摘掉。 */
                    pxPreviousBlock->pxNextFreeBlock = pxBlock->pxNextFreeBlock;

                    /* 如果块比需要的更大，就把它切成两块。 */
                    configASSERT( heapSUBTRACT_WILL_UNDERFLOW( pxBlock->xBlockSize, xWantedSize ) == 0 );

                    if( ( pxBlock->xBlockSize - xWantedSize ) > heapMINIMUM_BLOCK_SIZE )
                    {
                        /* 把这个块切成两块：在"申请大小"之后创建新的空闲块。
                         * 这里的 void 强制转换是为了避免编译器报字节对齐警告。 */
                        pxNewBlockLink = ( void * ) ( ( ( uint8_t * ) pxBlock ) + xWantedSize );
                        configASSERT( ( ( ( size_t ) pxNewBlockLink ) & portBYTE_ALIGNMENT_MASK ) == 0 );

                        /* 计算切分后两块各自的大小。 */
                        pxNewBlockLink->xBlockSize = pxBlock->xBlockSize - xWantedSize;
                        pxBlock->xBlockSize = xWantedSize;

                        /* 把新切出来的空闲块插进空闲链表。 */
                        pxNewBlockLink->pxNextFreeBlock = pxPreviousBlock->pxNextFreeBlock;
                        pxPreviousBlock->pxNextFreeBlock = heapPROTECT_BLOCK_POINTER( pxNewBlockLink );
                    }
                    else
                    {
                        mtCOVERAGE_TEST_MARKER();
                    }

                    xFreeBytesRemaining -= pxBlock->xBlockSize;

                    if( xFreeBytesRemaining < xMinimumEverFreeBytesRemaining )
                    {
                        xMinimumEverFreeBytesRemaining = xFreeBytesRemaining;
                    }
                    else
                    {
                        mtCOVERAGE_TEST_MARKER();
                    }

                    /* 这个块要返回给调用者了 —— 标记为"已分配"，且它没有"下一个空闲块"了。 */
                    heapALLOCATE_BLOCK( pxBlock );
                    pxBlock->pxNextFreeBlock = NULL;
                    xNumberOfSuccessfulAllocations++;
                }
                else
                {
                    mtCOVERAGE_TEST_MARKER();
                }
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }

        traceMALLOC( pvReturn, xWantedSize );
    }
    ( void ) xTaskResumeAll();

    #if ( configUSE_MALLOC_FAILED_HOOK == 1 )
    {
        if( pvReturn == NULL )
        {
            vApplicationMallocFailedHook();     /* 分配失败钩子：方便定位"堆不够" */
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }
    }
    #endif /* if ( configUSE_MALLOC_FAILED_HOOK == 1 ) */

    configASSERT( ( ( ( size_t ) pvReturn ) & ( size_t ) portBYTE_ALIGNMENT_MASK ) == 0 );
    return pvReturn;
}
/*-----------------------------------------------------------*/

/**
 * @brief  把内存还给 RTOS 堆（相当于内核版的 free）
 * @param  pv 之前 pvPortMalloc() 返回的地址
 *
 * 流程：
 *   1) 退回到块头（BlockLink_t 就在数据前面）
 *   2) 标记为"空闲"
 *   3) 挂起调度器，把块插回空闲链表 —— 这一步会自动尝试与相邻空闲块【合并】
 */
void vPortFree( void * pv )
{
    uint8_t * puc = ( uint8_t * ) pv;
    BlockLink_t * pxLink;

    if( pv != NULL )
    {
        /* 要释放的内存前面紧挨着一个 BlockLink_t 结构。 */
        puc -= xHeapStructSize;

        /* 这个强制转换是为了避免编译器报警告。 */
        pxLink = ( void * ) puc;

        heapVALIDATE_BLOCK_POINTER( pxLink );
        configASSERT( heapBLOCK_IS_ALLOCATED( pxLink ) != 0 );
        configASSERT( pxLink->pxNextFreeBlock == NULL );

        if( heapBLOCK_IS_ALLOCATED( pxLink ) != 0 )
        {
            if( pxLink->pxNextFreeBlock == NULL )
            {
                /* 这个块要还给堆了 —— 不再是"已分配"状态。 */
                heapFREE_BLOCK( pxLink );
                #if ( configHEAP_CLEAR_MEMORY_ON_FREE == 1 )
                {
                    /* 检查下溢：如果 xBlockSize 被踩坏了就可能发生。 */
                    if( heapSUBTRACT_WILL_UNDERFLOW( pxLink->xBlockSize, xHeapStructSize ) == 0 )
                    {
                        ( void ) memset( puc + xHeapStructSize, 0, pxLink->xBlockSize - xHeapStructSize );
                    }
                }
                #endif

                vTaskSuspendAll();
                {
                    /* 把这个块加入空闲链表（内部会尝试合并相邻空闲块）。 */
                    xFreeBytesRemaining += pxLink->xBlockSize;
                    traceFREE( pv, pxLink->xBlockSize );
                    prvInsertBlockIntoFreeList( ( ( BlockLink_t * ) pxLink ) );
                    xNumberOfSuccessfulFrees++;
                }
                ( void ) xTaskResumeAll();
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }
    }
}
/*-----------------------------------------------------------*/

/**
 * @brief 查询堆里当前还剩多少空闲字节
 */
size_t xPortGetFreeHeapSize( void )
{
    return xFreeBytesRemaining;
}
/*-----------------------------------------------------------*/

/**
 * @brief 查询"历史最小剩余空闲字节数"（用来判断堆是否曾接近耗尽）
 */
size_t xPortGetMinimumEverFreeHeapSize( void )
{
    return xMinimumEverFreeBytesRemaining;
}
/*-----------------------------------------------------------*/

/**
 * @brief 空函数：只是为了让链接器安静（某些工具链会引用它）
 */
void vPortInitialiseBlocks( void )
{
    /* 本函数只是为了满足链接器的引用需求。 */
}
/*-----------------------------------------------------------*/

/**
 * @brief  分配内存并清零（内核版的 calloc）
 * @param  xNum  元素个数
 * @param  xSize 每个元素的字节数
 * @return 分配到的内存地址（已清零）；失败返回 NULL
 */
void * pvPortCalloc( size_t xNum,
                     size_t xSize )
{
    void * pv = NULL;

    if( heapMULTIPLY_WILL_OVERFLOW( xNum, xSize ) == 0 )
    {
        pv = pvPortMalloc( xNum * xSize );

        if( pv != NULL )
        {
            ( void ) memset( pv, 0, xNum * xSize );
        }
    }

    return pv;
}
/*-----------------------------------------------------------*/

/**
 * @brief 初始化堆：把整个 ucHeap 数组变"一块大的空闲块 + 一个结束标记"
 *
 * 这是理解 heap_4 的关键：堆一开始是【一整块】空闲内存，
 * 之后每次分配都会把它切小，每次释放都会尝试把小块合回大块。
 */
static void prvHeapInit( void ) /* PRIVILEGED_FUNCTION */
{
    BlockLink_t * pxFirstFreeBlock;
    portPOINTER_SIZE_TYPE uxStartAddress, uxEndAddress;
    size_t xTotalHeapSize = configTOTAL_HEAP_SIZE;

    /* 确保堆从一个正确对齐的地址开始。 */
    uxStartAddress = ( portPOINTER_SIZE_TYPE ) ucHeap;

    if( ( uxStartAddress & portBYTE_ALIGNMENT_MASK ) != 0 )
    {
        uxStartAddress += ( portBYTE_ALIGNMENT - 1 );
        uxStartAddress &= ~( ( portPOINTER_SIZE_TYPE ) portBYTE_ALIGNMENT_MASK );
        xTotalHeapSize -= ( size_t ) ( uxStartAddress - ( portPOINTER_SIZE_TYPE ) ucHeap );
    }

    #if ( configENABLE_HEAP_PROTECTOR == 1 )
    {
        vApplicationGetRandomHeapCanary( &( xHeapCanary ) );
    }
    #endif

    /* xStart 用来保存"空闲链表第一个块"的指针（void 转换是为了避免编译器警告）。 */
    xStart.pxNextFreeBlock = ( void * ) heapPROTECT_BLOCK_POINTER( uxStartAddress );
    xStart.xBlockSize = ( size_t ) 0;

    /* pxEnd 用来标记"空闲块的结尾"，它被放在堆空间的末尾。 */
    uxEndAddress = uxStartAddress + ( portPOINTER_SIZE_TYPE ) xTotalHeapSize;
    uxEndAddress -= ( portPOINTER_SIZE_TYPE ) xHeapStructSize;
    uxEndAddress &= ~( ( portPOINTER_SIZE_TYPE ) portBYTE_ALIGNMENT_MASK );
    pxEnd = ( BlockLink_t * ) uxEndAddress;
    pxEnd->xBlockSize = 0;
    pxEnd->pxNextFreeBlock = heapPROTECT_BLOCK_POINTER( NULL );

    /* 一开始只有一块空闲块，大小 = 整个堆空间减去 pxEnd 占用的部分。 */
    pxFirstFreeBlock = ( BlockLink_t * ) uxStartAddress;
    pxFirstFreeBlock->xBlockSize = ( size_t ) ( uxEndAddress - ( portPOINTER_SIZE_TYPE ) pxFirstFreeBlock );
    pxFirstFreeBlock->pxNextFreeBlock = heapPROTECT_BLOCK_POINTER( pxEnd );

    /* 只有一块，且覆盖整个可用堆空间。 */
    xMinimumEverFreeBytesRemaining = pxFirstFreeBlock->xBlockSize;
    xFreeBytesRemaining = pxFirstFreeBlock->xBlockSize;
}
/*-----------------------------------------------------------*/

/**
 * @brief 把一个空闲块按地址顺序插回空闲链表，并【合并相邻的空闲块】
 *
 * 合并是 heap_4 比 heap_2 强的地方：
 *   - 如果"前一个块"紧挨着它 → 合并到前一块里
 *   - 如果"后一个块"紧挨着它 → 再把后一块也并进来
 * 于是反复分配/释放也不会把堆切得越来越碎。
 */
static void prvInsertBlockIntoFreeList( BlockLink_t * pxBlockToInsert ) /* PRIVILEGED_FUNCTION */
{
    BlockLink_t * pxIterator;
    uint8_t * puc;

    /* 沿着链表走，直到找到一个"地址比待插入块更高"的块。 */
    for( pxIterator = &xStart; heapPROTECT_BLOCK_POINTER( pxIterator->pxNextFreeBlock ) < pxBlockToInsert; pxIterator = heapPROTECT_BLOCK_POINTER( pxIterator->pxNextFreeBlock ) )
    {
        /* 循环体什么都不做，只是走到正确的位置。 */
    }

    if( pxIterator != &xStart )
    {
        heapVALIDATE_BLOCK_POINTER( pxIterator );
    }

    /* 待插入块与它"前面那块"是不是地址连续（能拼成一块）？ */
    puc = ( uint8_t * ) pxIterator;

    if( ( puc + pxIterator->xBlockSize ) == ( uint8_t * ) pxBlockToInsert )
    {
        pxIterator->xBlockSize += pxBlockToInsert->xBlockSize;
        pxBlockToInsert = pxIterator;
    }
    else
    {
        mtCOVERAGE_TEST_MARKER();
    }

    /* 待插入块与它"后面那块"是不是地址连续（能拼成一块）？ */
    puc = ( uint8_t * ) pxBlockToInsert;

    if( ( puc + pxBlockToInsert->xBlockSize ) == ( uint8_t * ) heapPROTECT_BLOCK_POINTER( pxIterator->pxNextFreeBlock ) )
    {
        if( heapPROTECT_BLOCK_POINTER( pxIterator->pxNextFreeBlock ) != pxEnd )
        {
            /* 把两块合成一块大的。 */
            pxBlockToInsert->xBlockSize += heapPROTECT_BLOCK_POINTER( pxIterator->pxNextFreeBlock )->xBlockSize;
            pxBlockToInsert->pxNextFreeBlock = heapPROTECT_BLOCK_POINTER( pxIterator->pxNextFreeBlock )->pxNextFreeBlock;
        }
        else
        {
            pxBlockToInsert->pxNextFreeBlock = heapPROTECT_BLOCK_POINTER( pxEnd );
        }
    }
    else
    {
        pxBlockToInsert->pxNextFreeBlock = pxIterator->pxNextFreeBlock;
    }

    /* 如果待插入块"填上了空隙"，也就是同时与前后两块都合并了，
     * 那么它的 pxNextFreeBlock 已经在上面设好了，这里不能再设，
     * 否则会让它指向自己。 */
    if( pxIterator != pxBlockToInsert )
    {
        pxIterator->pxNextFreeBlock = heapPROTECT_BLOCK_POINTER( pxBlockToInsert );
    }
    else
    {
        mtCOVERAGE_TEST_MARKER();
    }
}
/*-----------------------------------------------------------*/

/**
 * @brief 获取堆的统计信息（空闲块数量、最大/最小空闲块、历史最低剩余等）
 * @param pxHeapStats 输出结构体
 */
void vPortGetHeapStats( HeapStats_t * pxHeapStats )
{
    BlockLink_t * pxBlock;
    size_t xBlocks = 0, xMaxSize = 0, xMinSize = portMAX_DELAY; /* 用 portMAX_DELAY 当作"可移植的最大值" */

    vTaskSuspendAll();
    {
        pxBlock = heapPROTECT_BLOCK_POINTER( xStart.pxNextFreeBlock );

        /* 如果堆还没初始化，pxBlock 会是 NULL（堆会在第一次分配时自动初始化）。 */
        if( pxBlock != NULL )
        {
            while( pxBlock != pxEnd )
            {
                /* 统计块数，并记录目前为止看到的最大块。 */
                xBlocks++;

                if( pxBlock->xBlockSize > xMaxSize )
                {
                    xMaxSize = pxBlock->xBlockSize;
                }

                if( pxBlock->xBlockSize < xMinSize )
                {
                    xMinSize = pxBlock->xBlockSize;
                }

                /* 走到链表里的下一块，直到最后一块。 */
                pxBlock = heapPROTECT_BLOCK_POINTER( pxBlock->pxNextFreeBlock );
            }
        }
    }
    ( void ) xTaskResumeAll();

    pxHeapStats->xSizeOfLargestFreeBlockInBytes = xMaxSize;
    pxHeapStats->xSizeOfSmallestFreeBlockInBytes = xMinSize;
    pxHeapStats->xNumberOfFreeBlocks = xBlocks;

    taskENTER_CRITICAL();
    {
        pxHeapStats->xAvailableHeapSpaceInBytes = xFreeBytesRemaining;
        pxHeapStats->xNumberOfSuccessfulAllocations = xNumberOfSuccessfulAllocations;
        pxHeapStats->xNumberOfSuccessfulFrees = xNumberOfSuccessfulFrees;
        pxHeapStats->xMinimumEverFreeBytesRemaining = xMinimumEverFreeBytesRemaining;
    }
    taskEXIT_CRITICAL();
}
/*-----------------------------------------------------------*/

/*
 * 重置本文件里的状态（这些状态通常在启动时初始化）。
 * 应用若要在"重启调度器"之前调用它，必须先调用本函数。
 */
void vPortHeapResetState( void )
{
    pxEnd = NULL;

    xFreeBytesRemaining = ( size_t ) 0U;
    xMinimumEverFreeBytesRemaining = ( size_t ) 0U;
    xNumberOfSuccessfulAllocations = ( size_t ) 0U;
    xNumberOfSuccessfulFrees = ( size_t ) 0U;
}
/*-----------------------------------------------------------*/
