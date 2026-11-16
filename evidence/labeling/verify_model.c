/*
 * verify_model.c -- PC 侧验证：C 推理 vs sklearn 预测逐条比对
 *
 * 做了什么：读 verify_windows.csv（含 sklearn 的预测），
 *           用 model_infer.c 的同一份代码再算一遍，比对类别
 * 为什么：手写 C 最常见的错是权重下标布局搞反。
 *         如果不比对，在 MCU 上只会表现为"准确率很低"，
 *         分不清是模型弱还是代码错。
 * 影响什么：只在 PC 上跑，不进 Keil 工程
 * 怎么验证：输出 agree == total 即为通过
 *
 * 编译运行：
 *   gcc -O2 -o verify_model.exe verify_model.c
 *   ./verify_model.exe verify_windows.csv
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "model_infer.c"

int main(int argc, char **argv)
{
    const char *path = (argc > 1) ? argv[1] : "verify_windows.csv";
    FILE *fp = fopen(path, "r");
    static char line[65536];
    int total = 0, agree = 0, shown = 0;
    int hist[MODEL_OUT][MODEL_OUT];

    memset(hist, 0, sizeof(hist));

    if (fp == NULL) {
        printf("打不开 %s\n", path);
        return 2;
    }

    if (fgets(line, (int)sizeof(line), fp) == NULL) {
        printf("文件为空\n");
        fclose(fp);
        return 2;
    }

    while (fgets(line, (int)sizeof(line), fp) != NULL) {
        float win[MODEL_W][MODEL_IN];
        char *tok;
        int k, sk, got;

        tok = strtok(line, ",");
        if (tok == NULL) {
            continue;
        }
        sk = atoi(tok);

        for (k = 0; k < MODEL_W * MODEL_IN; k++) {
            tok = strtok(NULL, ",");
            if (tok == NULL) {
                break;
            }
            win[k / MODEL_IN][k % MODEL_IN] = (float)atof(tok);
        }
        if (k != MODEL_W * MODEL_IN) {
            continue;
        }

        got = Model_Predict(win);
        total++;
        if (got == sk) {
            agree++;
        } else if (shown < 5) {
            printf("  不一致 #%d: C=%d(%s)  sklearn=%d\n",
                   total, got, MODEL_CLASS_NAME[got], sk);
            shown++;
        }
        if (sk >= 0 && sk < MODEL_OUT && got >= 0 && got < MODEL_OUT) {
            hist[sk][got]++;
        }
    }
    fclose(fp);

    printf("\n总窗口 %d，一致 %d，不一致 %d\n", total, agree, total - agree);
    printf("一致率 %.2f%%\n", total ? 100.0 * agree / total : 0.0);
    printf("\n混淆矩阵（行=sklearn 列=C）：\n     ");
    {
        int j;
        for (j = 0; j < MODEL_OUT; j++) {
            printf("%12s", MODEL_CLASS_NAME[j]);
        }
        printf("\n");
    }
    {
        int i, j;
        for (i = 0; i < MODEL_OUT; i++) {
            printf("%-12s", MODEL_CLASS_NAME[i]);
            for (j = 0; j < MODEL_OUT; j++) {
                printf("%12d", hist[i][j]);
            }
            printf("\n");
        }
    }
    return (total > 0 && agree == total) ? 0 : 1;
}
