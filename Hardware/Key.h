#ifndef __KEY_H
#define __KEY_H

/*
 * 按键驱动接口（非阻塞版）
 *
 * 设计约定：
 *  1. 驱动层零延时：消抖靠任务以 20ms 周期调 Key_Scan()，本文件内禁止出现
 *     vTaskDelay / Delay_ms / while 死等——延时混进驱动 = 调用任务被绑架。
 *  2. 事件位图：一次扫描把两个键都扫完，用一个字节报告"这次扫描发生了什么"。
 *     调用方只有一处，结构上杜绝"轮询每个键函数、漏扫/重复消费"的老坑。
 *  3. 驱动只报事件不做业务：谁按下、什么含义，由任务层决定（freertos_demo.c）。
 */

/* 事件位定义（Key_Scan 返回值用） */
#define KEY1_EVENT      0x01    /* PB1 确认按下（消抖后只报一次，松手前不再报） */
#define KEY11_EVENT     0x02    /* PB11 确认按下 */

/* 两个键同时确认按下时的返回值 = KEY1_EVENT | KEY11_EVENT = 0x03 */

void Key_Init(void);
uint8_t Key_Scan(void);

#endif
