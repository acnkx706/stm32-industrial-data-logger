#ifndef __AD_H
#define __AD_H

/* 两个通道的原始值（12 位，0~4095）：由 DMA 在后台连续刷新，任务里直接读、零阻塞
 *   AD_Value[0] = PA1 = ADC1_IN1 = 光敏 AO（模拟量）
 *   AD_Value[1] = PB0 = ADC1_IN8 = 电池分压（20k+10k → 实际电压 = 读数 × VDDA/4096 × 3） */
extern volatile uint16_t AD_Value[2];
void AD_TestVdd(void);      /* ★ 补 void（2026-10-11）：原来写 AD_TestVdd() ✗
                             *   在 C 里表示"参数未指定"✗ → 触发 #1295-D 警告 ✓ */
void AD_Init(void);

/* ★★ 新增声明（2026-10-11 独立审查 F23）✗
 *   原来 SD_SPI.c 里用【局部 extern】声明它 ✗
 *   → 那是独立于 AD.h 的【第二份签名】✓
 *   → AD.c 哪天改了返回值类型/参数 ✗ SD_SPI.c 那行不会跟着变 ✗ 编译器也不报错 ✓
 *     （两个编译单元各信自己的声明 ✓ 只有链接期/运行期才炸 ✓）
 *   ★ 挪进头文件 = 单一事实来源 ✓（和 Crc16.h 立的规矩同一条 ✓）
 *   ⚠️ 另注（同条审查 F23 的问题 2）：AD_ReadVrefint 内部是 for(200000) 等 JEOC 的【忙等】✗
 *     SD_SPI.c 的 5 处调用点全在【卡初始化失败/写失败重试】的路径上 ✗
 *     → 卡出问题时 task_SD（最低优先级）会额外卡住几十~几百 ms 纯忙等 ✓
 *     ★ 当前可接受（它本就跑在故障路径上 ✓ 不影响采集链 ✓）
 *     ★★ 若将来这个诊断要长期保留 → 应改成"带超时 + 错误时跳过"✗ 而不是死等 ✓ */
uint16_t AD_ReadVrefint(void);

#endif
