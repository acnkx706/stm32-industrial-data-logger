/*==============================================================================
 *  exp02.c -- EXP-02：SD 延迟直方图 + 停摆注入
 *
 *  做了什么：在 f_write / f_sync 上打点累加 1ms 桶直方图；记录第一次丢条的时刻与原因；
 *            用一个低优先级任务按矩阵注入停摆并逐次打印一行 CSV。
 *  为什么：250ms 是设计假设、没有测量出处；缓冲链先丢哪一类也只有推导没有实测。
 *  影响什么：TEST_EXP02=0 时整个文件不参与编译，正式固件零开销。
 *  怎么验证：=0 时 Code 与改动前一致；=1 时串口出现 LAT / TOP / EXP02 行。
 *============================================================================*/

#include "ProbeConfig.h"

#if (TEST_EXP02 != 0)

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stm32f10x.h"

#include "exp02.h"
#include "task_sd.h"
#include "Serial.h"
#include "DWT_Delay.h"

/* ---- 直方图：256 个 1ms 桶 + 1 个溢出桶，写/同步各一套 ---------------- */
#define LAT_BINS      256
#define LAT_DUMP_STEP 1000u     /* 每累计这么多样本，请求一次 dump */

static uint16_t s_hist[2][LAT_BINS];
static uint16_t s_ovf[2];
static uint32_t s_n[2];
static uint32_t s_sum_us[2];
static uint32_t s_max_us[2];
static volatile uint8_t  s_dump_req;

/* ---- top-32 最慢样本（线性插入，32 个元素足够快）---------------------- */
#define TOP_N 32
static uint32_t s_top_us[TOP_N];
static uint32_t s_top_tick[TOP_N];
static uint8_t  s_top_op[TOP_N];
static uint16_t s_top_cnt;

/* ---- 停摆注入 --------------------------------------------------------- */
static volatile uint32_t s_stall_sd;    /* 非 0 = 下次 f_write 前停这么久 */
static volatile uint32_t s_stall_proc;  /* 非 0 = 下次出队后停这么久 */
static volatile uint8_t  s_armed;       /* 1 = 正在统计"第一次丢条" */
static volatile uint32_t s_arm_tick;
static volatile uint32_t s_first_tick;
static volatile uint8_t  s_first_reason;

/* ---- 计数器快照 ------------------------------------------------------- */
typedef struct {
    uint32_t acq;
    uint32_t qfull;
    uint32_t qmax;
    uint32_t loss[LOSS_N];
} Snap_t;

extern Stat_t g_stat;                    /* freertos_demo.c 定义 */

static void snap_take(Snap_t *s)
{
    uint32_t tot[LOSS_N];
    uint8_t i;
    taskENTER_CRITICAL();
    s->acq   = g_stat.acq_count;
    s->qfull = g_stat.queue_full;
    s->qmax  = g_stat.q_max_depth;
    Loss_GetTotals(tot);
    for (i = 0u; i < LOSS_N; i++) { s->loss[i] = tot[i]; }
    taskEXIT_CRITICAL();
}

/*==============================================================================
 *  对外接口
 *============================================================================*/
void Exp02_Init(void)
{
    uint8_t i;
    for (i = 0u; i < 2u; i++)
    {
        uint16_t b;
        for (b = 0u; b < LAT_BINS; b++) { s_hist[i][b] = 0u; }
        s_ovf[i] = 0u; s_n[i] = 0u; s_sum_us[i] = 0u; s_max_us[i] = 0u;
    }
    s_top_cnt = 0u;
    s_stall_sd = 0u;
    s_stall_proc = 0u;
    s_armed = 0u;
}

void Exp02_Lat(uint8_t op, uint32_t us)
{
    uint16_t ms;
    uint16_t i, worst;

    if (op > 1u) { return; }

    s_n[op]++;
    s_sum_us[op] += us;
    if (us > s_max_us[op]) { s_max_us[op] = us; }

    ms = (uint16_t)(us / 1000u);
    if (ms >= LAT_BINS) { s_ovf[op]++; }
    else if (s_hist[op][ms] < 0xFFFFu) { s_hist[op][ms]++; }

    /* top-32：只在与最小值比较后插入，避免每次都挪数组 */
    if (s_top_cnt < TOP_N)
    {
        s_top_us[s_top_cnt] = us;
        s_top_tick[s_top_cnt] = xTaskGetTickCount();
        s_top_op[s_top_cnt] = op;
        s_top_cnt++;
    }
    else
    {
        worst = 0u;
        for (i = 1u; i < TOP_N; i++) { if (s_top_us[i] < s_top_us[worst]) { worst = i; } }
        if (us > s_top_us[worst])
        {
            s_top_us[worst] = us;
            s_top_tick[worst] = xTaskGetTickCount();
            s_top_op[worst] = op;
        }
    }

    if (((s_n[0] + s_n[1]) % LAT_DUMP_STEP) == 0u) { s_dump_req = 1u; }
}

void Exp02_NoteLoss(uint8_t reason)
{
    if ((s_armed != 0u) && (s_first_tick == 0u))
    {
        s_first_tick = xTaskGetTickCount();
        s_first_reason = reason;
    }
}

uint32_t Exp02_StallSdMs(void)
{
    uint32_t v = s_stall_sd;
    s_stall_sd = 0u;                     /* 一次停摆只生效一次 */
    return v;
}

uint32_t Exp02_StallProcMs(void)
{
    uint32_t v = s_stall_proc;
    s_stall_proc = 0u;
    return v;
}

/*==============================================================================
 *  dump：把直方图与 top-32 打成串口 CSV（不在 SD 路径里调用）
 *============================================================================*/
static void exp02_dump(void)
{
    uint8_t op;
    uint16_t b, i;

    for (op = 0u; op < 2u; op++)
    {
        Serial_Printf("LAT,SUM,%u,%lu,%lu,%lu,%u\r\n",
                      (unsigned)op,
                      (unsigned long)s_n[op],
                      (unsigned long)((s_n[op] != 0u) ? (s_sum_us[op] / s_n[op]) : 0u),
                      (unsigned long)s_max_us[op],
                      (unsigned)s_ovf[op]);
        for (b = 0u; b < LAT_BINS; b++)
        {
            if (s_hist[op][b] != 0u)
            {
                Serial_Printf("LAT,%u,%u,%u\r\n", (unsigned)op, (unsigned)b,
                              (unsigned)s_hist[op][b]);
            }
        }
    }
    for (i = 0u; i < s_top_cnt; i++)
    {
        Serial_Printf("TOP,%u,%lu,%lu\r\n", (unsigned)s_top_op[i],
                      (unsigned long)s_top_us[i], (unsigned long)s_top_tick[i]);
    }
    s_dump_req = 0u;
}

/*==============================================================================
 *  测试任务：停摆矩阵（仅在 TEST_SD_STALL / TEST_PROC_STALL 时跑）
 *============================================================================*/
#if (TEST_SD_STALL != 0) || (TEST_PROC_STALL != 0)

static const uint32_t k_stall_ms[] = { 100u, 250u, 500u, 1000u, 2000u };
#define STALL_LEVELS (sizeof(k_stall_ms) / sizeof(k_stall_ms[0]))
#define TRIALS_PER_LEVEL 10u

static void exp02_one_trial(uint8_t kind, uint32_t ms, uint8_t qlen, uint8_t trial)
{
    Snap_t a, b;
    uint32_t first_ms = 0u;

    /* 基线：先恢复 3 秒，让队列/缓冲回到稳定态 */
    s_stall_sd = 0u;
    s_stall_proc = 0u;
    s_armed = 0u;
    vTaskDelay(pdMS_TO_TICKS(3000));

    snap_take(&a);

    s_first_tick = 0u;
    s_first_reason = 0xFFu;
    s_arm_tick = xTaskGetTickCount();
    s_armed = 1u;

    if (kind == 0u) { s_stall_sd = ms; }
    else            { s_stall_proc = ms; }

    /* 停摆 + 恢复观察窗 */
    vTaskDelay(pdMS_TO_TICKS(ms + 5000u));
    s_armed = 0u;

    snap_take(&b);

    if (s_first_tick != 0u)
    {
        first_ms = (uint32_t)(s_first_tick - s_arm_tick) * (uint32_t)portTICK_PERIOD_MS;
    }

    Serial_Printf("EXP02,%u,%lu,%u,%u,%lu,%lu,%lu,%lu,%lu,%lu,%u,%lu,%lu,%lu,%u\r\n",
                  (unsigned)kind, (unsigned long)ms, (unsigned)qlen, (unsigned)trial,
                  (unsigned long)b.qmax,
                  (unsigned long)(b.qfull - a.qfull),
                  (unsigned long)(b.loss[LOSS_Q_FULL] - a.loss[LOSS_Q_FULL]),
                  (unsigned long)(b.loss[LOSS_NO_BUF] - a.loss[LOSS_NO_BUF]),
                  (unsigned long)(b.loss[LOSS_ENQ_FAIL] - a.loss[LOSS_ENQ_FAIL]),
                  (unsigned long)(b.loss[LOSS_ACQ_LATE] - a.loss[LOSS_ACQ_LATE]),
                  (unsigned)s_first_reason, (unsigned long)first_ms,
                  (unsigned long)a.acq, (unsigned long)b.acq,
                  (unsigned)(g_sd_error));
}

#endif

void Exp02_Task(void *pvParameters)
{
    (void)pvParameters;

    Exp02_Init();
    Serial_Printf("EXP02,START,lat=%u,stallSD=%u,stallPROC=%u,qlen=%u\r\n",
                  (unsigned)TEST_SD_LAT, (unsigned)TEST_SD_STALL,
                  (unsigned)TEST_PROC_STALL, (unsigned)REC_QUEUE_LEN);

#if (TEST_SD_STALL != 0) || (TEST_PROC_STALL != 0)
    {
        uint32_t lv;
        uint8_t kind;

        /* kind 0 = 停 SD 任务，1 = 停 PROC 任务（控制组） */
        for (kind = 0u; kind < 2u; kind++)
        {
#if (TEST_SD_STALL == 0)
            if (kind == 0u) { continue; }
#endif
#if (TEST_PROC_STALL == 0)
            if (kind == 1u) { continue; }
#endif
            for (lv = 0u; lv < STALL_LEVELS; lv++)
            {
                uint8_t t;
                for (t = 1u; t <= TRIALS_PER_LEVEL; t++)
                {
                    exp02_one_trial(kind, k_stall_ms[lv],
                                    (uint8_t)REC_QUEUE_LEN, t);
                }
            }
        }
        Serial_Printf("EXP02,DONE\r\n");
    }
#endif

    for (;;)
    {
        if (s_dump_req != 0u) { exp02_dump(); }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

#endif  /* TEST_EXP02 */
