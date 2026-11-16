/* test_sr04.c -- HC-SR04 驱动的单元测试（主机侧，gcc 编译）
 *
 * 做了什么：把真实的 HC_SR04.c 直接 include 进来，用桩替代 STM32 寄存器，
 *           逐条验证量程窗口、脉宽换算、边沿状态机、16 位回绕。
 * 为什么：改的是一个可能影响所有历史数据的常量，必须证明"除了放大上限，
 *         其他行为一个字都没变"。
 * 影响什么：只读源码、只在主机上运行，不改固件、不碰硬件。
 * 怎么验证：gcc 编出来跑，全部 PASS 即通过。
 *
 * 编译：
 *   gcc -std=c99 -Wall -I. -I../../Hardware -o t.exe test_sr04.c && ./t.exe
 */

#include <stdio.h>
#include <stdint.h>

#include "stm32f10x.h"
#include "HC_SR04.h"

/* ---- 桩需要的全局量 ---- */
DWT_TypeDef g_stub_dwt;
uint32_t    SystemCoreClock = 72000000u;
uint16_t    g_stub_capture  = 0u;
uint8_t     g_stub_it       = 0u;
uint16_t    g_stub_polarity = 0xFFFFu;

/* 故意直接包含源文件：这样才能够到 static 的状态变量做黑盒+白盒混合测试 */
#include "HC_SR04.c"

static int g_fail = 0;
static int g_total = 0;

#define CHECK(cond, fmt, ...) do {                       \
    g_total++;                                           \
    if (cond) { printf("  PASS  " fmt "\n", __VA_ARGS__); } \
    else { g_fail++; printf("  FAIL  " fmt "\n", __VA_ARGS__); } \
} while (0)

/* 把脉宽塞进状态机，跑一次 Poll，返回结果 */
static uint8_t poll_with_pulse(uint16_t us, uint16_t *mm)
{
    *mm = 0xEEEEu;
    s_state = 3u;
    s_us    = us;
    return HC_SR04_Poll(mm);
}

int main(void)
{
    uint16_t mm;
    uint8_t  r;

    printf("=== 1. 量程窗口边界 ===\n");

    r = poll_with_pulse(HC_PULSE_MIN_US, &mm);
    CHECK(r == 1u, "脉宽 %u（下限）应成功 → r=%u mm=%u", HC_PULSE_MIN_US, r, mm);

    r = poll_with_pulse((uint16_t)(HC_PULSE_MIN_US - 1u), &mm);
    CHECK(r == 2u, "脉宽 %u（下限-1）应被拒 → r=%u", HC_PULSE_MIN_US - 1u, r);

    r = poll_with_pulse(HC_PULSE_MAX_US, &mm);
    CHECK(r == 1u, "脉宽 %u（上限）应成功 → r=%u mm=%u", HC_PULSE_MAX_US, r, mm);

    r = poll_with_pulse((uint16_t)(HC_PULSE_MAX_US + 1u), &mm);
    CHECK(r == 2u, "脉宽 %u（上限+1）应被拒 → r=%u", HC_PULSE_MAX_US + 1u, r);

    r = poll_with_pulse(65535u, &mm);
    CHECK(r == 2u, "脉宽 65535（无回波满载）应被拒 → r=%u", r);

    printf("\n=== 2. 脉宽 → 毫米 换算 ===\n");
    {
        static const struct { uint16_t us; uint16_t exp; const char *what; } T[] = {
            {  117u,     20u, "2cm"   },
            { 2400u,    412u, "旧上限 2400us" },
            { 2330u,    400u, "旧注释误写的 4m" },
            { 5831u,   1002u, "1m"    },
            { 11662u,  2005u, "2m"    },
            { 23300u,  4007u, "4m"    },
        };
        for (unsigned i = 0u; i < sizeof(T)/sizeof(T[0]); ++i)
        {
            r = poll_with_pulse(T[i].us, &mm);
            CHECK(r == 1u && mm == T[i].exp,
                  "%-18s us=%5u → r=%u mm=%4u（期望 %u）",
                  T[i].what, T[i].us, r, mm, T[i].exp);
        }
    }

    printf("\n=== 3. 回归：2400us 以前会被拒，现在必须通过 ===\n");
    r = poll_with_pulse(2400u, &mm);
    CHECK(r == 1u, "2400us 现在应成功（旧逻辑此值已是边界）→ r=%u mm=%u", r, mm);
    r = poll_with_pulse(2410u, &mm);
    CHECK(r == 1u, "2410us 现在应成功（旧逻辑必丢）→ r=%u mm=%u", r, mm);
    r = poll_with_pulse(4000u, &mm);
    CHECK(r == 1u, "4000us=686mm 现在应成功 → r=%u mm=%u", r, mm);

    printf("\n=== 4. 溢出安全 ===\n");
    {
        uint32_t v = (uint32_t)HC_PULSE_MAX_US * 172u;
        CHECK(v == 4007600u, "上限乘法 %u*172 = %u（uint32 内，无溢出）",
              HC_PULSE_MAX_US, v);
        uint32_t mm32 = v / 1000u;
        CHECK(mm32 == 4007u && mm32 <= 65535u,
              "换算结果 %u mm 放得进 uint16", mm32);
    }

    printf("\n=== 5. 边沿状态机 + 16 位回绕 ===\n");
    {
        /* 完整走一遍：Start → 上升沿 → 下降沿 → Poll */
        HC_SR04_Start();
        CHECK(s_state == 1u, "Start 后状态应为 1（等上升沿），实际 %u", s_state);
        CHECK(g_stub_polarity == TIM_ICPolarity_Rising,
              "Start 后极性应为上升沿，实际 %u", g_stub_polarity);

        g_stub_it      = SET;
        g_stub_capture = 60000u;       /* 上升沿 */
        TIM4_IRQHandler();
        CHECK(s_state == 2u, "上升沿后状态应为 2，实际 %u", s_state);
        CHECK(g_stub_polarity == TIM_ICPolarity_Falling,
              "上升沿后极性应翻成下降沿，实际 %u", g_stub_polarity);

        g_stub_capture = 2000u;        /* 下降沿：65536+2000-60000 = 7536us */
        TIM4_IRQHandler();
        CHECK(s_state == 3u, "下降沿后状态应为 3，实际 %u", s_state);
        CHECK(s_us == 7536u, "回绕相减应为 7536us，实际 %u", s_us);

        r = HC_SR04_Poll(&mm);
        CHECK(r == 1u && mm == 1296u,
              "Poll 应成功 1296mm → r=%u mm=%u", r, mm);
        CHECK(s_state == 0u, "Poll 后应回到空闲，实际 %u", s_state);
    }

    printf("\n=== 6. 无回波超时 ===\n");
    {
        HC_SR04_Start();
        /* 让 CYCCNT 跨过 45ms：45ms × 72MHz = 3,240,000 周期 */
        g_stub_dwt.CYCCNT = 4000000u;
        s_t0_cyc = 0u;
        r = HC_SR04_Poll(&mm);
        CHECK(r == 2u, "超时后应返回 2 → r=%u", r);
        CHECK(s_state == 0u, "超时后状态应回 0，实际 %u", s_state);
    }

    printf("\n========================================\n");
    printf("%d 项检查，%d 项失败\n", g_total, g_fail);
    return g_fail ? 1 : 0;
}
