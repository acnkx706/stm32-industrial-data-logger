/*
 * model_infer.c -- 端侧事件分类器推理实现（纯 C，无 STM32 依赖）
 *
 * ★ 性能优化历程（每一步都单独可回退）
 *
 * ① 权重转置           407us -> 252us
 *    · 旧：model_W1[k * MODEL_H1 + j]，内层每步跳 32 个 float（128 字节）
 *    · 新：model_W1T[j * NIN + k]，内层连续，Flash 预取能命中
 *
 * ② 乘倒数代替除法      （与 ① 同批）
 *    · 旧：140 次 VDIV，每次 14 周期 ⇒ 1960 周期
 *    · 新：140 次 VMUL
 *
 * ③ 4 路累加器展开      （与 ① 同批）
 *    · 单累加器形成依赖链，FPU 要等上一次结果
 *
 * ④ 权重搬进 CCM RAM    252us -> ?
 *    · 根因：F407 在 168MHz 下 Flash 有 5 个等待周期，
 *      而本次实测 8.3 周期/乘加里，约 6 个周期花在等 Flash
 *    · CCM RAM 在 0x10000000，64KB，CPU 访问零等待（DMA 到不了，但这里不需要）
 *    · 权重合计 5108 个 float = 20.4KB，装得下
 *    · Model_LoadWeights(addr) 在启动时搬一次，之后推理全走 RAM
 *    · 不调它也能跑（默认仍指向 Flash 里的常量），PC 验证不受影响
 */

#include <math.h>
#include <string.h>

/* ★ 允许编译器把 s += a*b 融合成一条 VMLA
 * · 不开启时生成 VMUL + VADD 两条，VADD 要等 VMUL 的 3 周期延迟
 * · M4F 的 FPv4-SP 支持 VMLA.F32，单条完成乘加
 * · 怎么验证：反汇编里 VMLA 计数应大于 0 */
#pragma STDC FP_CONTRACT ON
#include "model_weights.h"

static const char *const MODEL_CLASS_NAME[MODEL_OUT] = {
    "normal", "light_step", "impact", "tilt"
};

#define MODEL_ALPHA   (1.0f / (float)MODEL_TAU)
#define NIN           (MODEL_W * MODEL_IN)

/* ★ 权重指针：默认指向 Flash 里的常量，Model_LoadWeights 之后指向 RAM
 * · 用指针而不是直接用数组名，是为了"可选地搬到 RAM"这件事
 * · 代价：多一次指针解引用，但换来的是 5 个等待周期不再发生 */
static const float *s_W1 = model_W1T;
static const float *s_W2 = model_W2T;
static const float *s_W3 = model_W3T;

/* EMA 基线：每通道一个 float */
static float s_base[MODEL_IN];
static int   s_inited = 0;


void Model_LoadWeights(void *ram_base)
{
    float *p = (float *)ram_base;
    int n;

    n = MODEL_H1 * NIN;
    memcpy(p, model_W1T, (size_t)n * sizeof(float));
    s_W1 = p;
    p += n;

    n = MODEL_H2 * MODEL_H1;
    memcpy(p, model_W2T, (size_t)n * sizeof(float));
    s_W2 = p;
    p += n;

    n = MODEL_OUT * MODEL_H2;
    memcpy(p, model_W3T, (size_t)n * sizeof(float));
    s_W3 = p;
}


int Model_WeightsBytes(void)
{
    return (MODEL_H1 * NIN + MODEL_H2 * MODEL_H1 + MODEL_OUT * MODEL_H2) * (int)sizeof(float);
}


void Model_Init(const float *first_sample)
{
    int i;
    for (i = 0; i < MODEL_IN; i++) {
        s_base[i] = first_sample[i];
    }
    s_inited = 1;
}


void Model_Update(float *feat, const float *raw)
{
    int i;

    if (!s_inited) {
        Model_Init(raw);
    }
    for (i = 0; i < MODEL_IN; i++) {
        s_base[i] += MODEL_ALPHA * (raw[i] - s_base[i]);
        feat[i] = raw[i] - s_base[i];
    }
}


/* ★ 4 路累加器点积：w 连续，n 为 4 的倍数时最快 */
static float dot4(const float *a, const float *w, int n)
{
    float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f, s3 = 0.0f;
    int k = 0;

    for (; k + 4 <= n; k += 4) {
        s0 += a[k]     * w[k];
        s1 += a[k + 1] * w[k + 1];
        s2 += a[k + 2] * w[k + 2];
        s3 += a[k + 3] * w[k + 3];
    }
    for (; k < n; k++) {
        s0 += a[k] * w[k];
    }
    return (s0 + s1) + (s2 + s3);
}


void Model_Logits(const float win[MODEL_W][MODEL_IN], float *logits)
{
    float a[NIN];
    float h1[MODEL_H1];
    float h2[MODEL_H2];
    const float *flat = &win[0][0];
    int   k, j;

    /* ① 展平 + 标准化（乘法代替除法） */
    for (k = 0; k < NIN; k++) {
        a[k] = (flat[k] - model_in_mean[k]) * model_in_inv_scale[k];
    }

    /* ② 第一层：NIN -> H1，ReLU */
    for (j = 0; j < MODEL_H1; j++) {
        float s = model_b1[j] + dot4(a, &s_W1[j * NIN], NIN);
        h1[j] = (s > 0.0f) ? s : 0.0f;
    }

    /* ③ 第二层：H1 -> H2，ReLU */
    for (j = 0; j < MODEL_H2; j++) {
        float s = model_b2[j] + dot4(h1, &s_W2[j * MODEL_H1], MODEL_H1);
        h2[j] = (s > 0.0f) ? s : 0.0f;
    }

    /* ④ 输出层：H2 -> OUT */
    for (j = 0; j < MODEL_OUT; j++) {
        logits[j] = model_b3[j] + dot4(h2, &s_W3[j * MODEL_H2], MODEL_H2);
    }
}


int Model_Predict(const float win[MODEL_W][MODEL_IN])
{
    float logits[MODEL_OUT];
    int   best = 0;
    int   j;

    Model_Logits(win, logits);
    for (j = 1; j < MODEL_OUT; j++) {
        if (logits[j] > logits[best]) {
            best = j;
        }
    }
    return best;
}
