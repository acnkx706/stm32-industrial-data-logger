/*
 * model_infer.h -- 端侧事件分类器推理接口
 *
 * 做了什么：定义 EMA 基线更新 + 窗口推理两个接口
 * 为什么：把"特征工程"和"网络推理"分开，
 *         因为 EMA 基线要每一拍更新一次，推理只在窗口凑满时做一次
 * 影响什么：调用者要维护一个 7 通道的采样环形缓冲（20 拍）
 * 怎么验证：PC 上用 verify_model.c 跑一批真实窗口，
 *           输出必须和 sklearn 的 predict 完全一致
 */

#ifndef __MODEL_INFER_H
#define __MODEL_INFER_H

#include "model_weights.h"

/* 类别名，顺序必须和训练时的 CLASSES 一致 */
extern const char *const MODEL_CLASS_NAME[MODEL_OUT];

/* 做什么：上电时用第一拍原始读数初始化 EMA 基线
 * 为什么：不初始化的话基线从 0 开始，前几分钟的特征全是垃圾
 * 怎么验证：初始化和不初始化各跑一遍，前 60 秒的预测应该不同 */
void Model_Init(const float *first_sample);

/* 做什么：喂一拍原始采样，更新基线并输出高差特征
 * 为什么：这一步是"去漂移"的核心，必须在切窗口之前做
 *         如果先切窗口再高差，窗口内的绝对电平会被抹掉，
 *         "遮光"这种状态型事件就检不出来
 * 影响什么：每拍只花 7 次乘加
 * 怎么验证：同一段数据跑一遍，light 通道在遮住时特征应稳定在 +800 左右 */
void Model_Update(float *feat, const float *raw);

/* 做什么：对一个已填满的窗口做推理，返回类别下标
 * 为什么：窗口是 feat 的二维数组，不是原始值——
 *         调用者必须把 Model_Update 的输出存进环形缓冲
 * 怎么验证：PC 与 MCU 对同一窗口必须给出同一个类别 */
int Model_Predict(const float win[MODEL_W][MODEL_IN]);

/* 做什么：只算 logits，不给类别（调试用）
 * 为什么：看各类得分能判断"差一点"还是"完全不沾边"
 * 怎么验证：argmax(logits) 必须等于 Model_Predict 的返回值 */
void Model_Logits(const float win[MODEL_W][MODEL_IN], float *logits);

#endif
