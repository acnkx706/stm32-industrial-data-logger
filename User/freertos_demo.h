#ifndef __FREERTOS_DEMO_H
#define __FREERTOS_DEMO_H

void freertos_demo(void);

/* ★ IWDG 独立看门狗（2026-10-12 加，阶段 3 最后一块）
 *   ★ 必须在所有耗时初始化之后、调度器启动之前调用
 *     —— IWDG 一旦启动不能停，而启动阶段 SD 初始化可能耗好几秒。 */
void WD_Init(void);

#endif
