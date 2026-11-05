#ifndef __EXP02_H
#define __EXP02_H

#include "stm32f10x.h"
#include "ProbeConfig.h"

/*==============================================================================
 *  EXP-02 测试钩子（SD 延迟分布 + 停摆注入）
 *
 *  做了什么：提供延迟打点、丢条打点、停摆注入时长查询、测试任务入口
 *  为什么：250ms 一直是设计假设，需要用实测分布判定；缓冲链先丢哪一类也要注入验证
 *  影响什么：TEST_EXP02=0 时本头文件只展开成空宏，正式固件零开销、零 Flash
 *  怎么验证：TEST_EXP02=0 编译后 Code 大小与改动前一致；=1 时串口出现 EXP02 行
 *============================================================================*/

#if (TEST_EXP02 != 0)

/* op 取值：0 = f_write，1 = f_sync（两个直方图分开统计） */
void     Exp02_Init(void);
void     Exp02_Lat(uint8_t op, uint32_t us);
void     Exp02_NoteLoss(uint8_t reason);
uint32_t Exp02_StallSdMs(void);         /* 返回待注入的毫秒数，取走后自动清零 */
uint32_t Exp02_StallProcMs(void);       /* 同上，POS 侧控制组用 */
void     Exp02_Task(void *pvParameters);

#else

#define Exp02_Init()            do { } while (0)
#define Exp02_Lat(op, us)       do { } while (0)
#define Exp02_NoteLoss(r)       do { } while (0)
#define Exp02_StallSdMs()       0u
#define Exp02_StallProcMs()     0u

#endif

#endif
