/* DWT_Delay.h -- 主机侧桩头文件
 *
 * 做了什么：把 12us 的忙等换成空操作
 * 为什么：真实现靠 DWT 周期计数，x86 上没有这个外设
 * 影响什么：只影响触发脉冲的宽度，不影响测距换算
 * 怎么验证：编译通过且 Start() 能返回
 */
#ifndef STUB_DWT_DELAY_H
#define STUB_DWT_DELAY_H

#include <stdint.h>

static void DWT_Delay_us(uint32_t us) { (void)us; }

#endif
