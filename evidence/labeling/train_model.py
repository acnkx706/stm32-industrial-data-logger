#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
训练端侧事件分类器 v2 —— 改用"相对慢基线"特征

做了什么
    1. 对每一路通道先做 EMA 慢基线高差：feat = x[t] - baseline[t]
       其中 baseline 是时间常数 60 秒的一阶低通
    2. 再切 2 秒窗口、标准化、训练小 MLP
    3. 导出权重成 C 头文件（含基线更新常数）

为什么必须这么做
    · v1 直接用绝对值，训练集 97% 但跨会话只剩 82%
    · 根因：两个会话的 light 基线差 350 个计数，
      绝对特征让模型把"换了一天"当成"换了一个类别"
    · 基线高差能同时满足两个相反的需求：
        - 状态型（遮光，持续 20 秒）：基线跟不上，偏移保留
        - 慢漂移（昼夜，小时级）：被基线完全吸收
    · 在 MCU 上只要 7 个 float，每个采样一次乘加

怎么验证
    · 跨会话测试应该明显高于 v1 的 0.819
    · light_step 的召回不能是 0

用法
    python train_model.py
"""

import argparse
import csv
import os
import sys

import numpy as np

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

COLS = ["seq", "tick", "press", "rtc", "ax", "ay", "az", "gx", "gy", "gz",
        "temp_ds", "temp_chip", "humi", "light", "vbat", "sonar_mm", "crc"]

CHANNELS = ["light", "ax", "ay", "az", "gx", "gy", "gz"]

CLASSES = ["normal", "light_step", "impact", "tilt"]
LABEL_MAP = {
    "normal":      "normal",
    "light_down":  "light_step",
    "light_block": "light_step",
    "shake":       "impact",
    "tap_board":   "impact",
    "tilt":        "tilt",
}

HERE = os.path.dirname(os.path.abspath(__file__))

# ★ 基线时间常数：60 秒。10Hz ⇒ alpha = 1/(60*10) = 1/600
TAU_SAMPLES = 1800


def load_session(path):
    A = np.load(path).astype(np.float32)
    return {c: A[:, i] for i, c in enumerate(COLS)}


def ema_highpass(x, tau=TAU_SAMPLES):
    """一阶低通做基线，返回 x - baseline。纯前向递归，O(1) 内存。"""
    alpha = 1.0 / tau
    out = np.empty_like(x, dtype=np.float32)
    b = float(x[0])
    for i in range(len(x)):
        b += alpha * (float(x[i]) - b)
        out[i] = float(x[i]) - b
    return out, float(x[0])


def build_masks(t, labels, n):
    guard = 30
    lab = {c: np.zeros(n, bool) for c in CLASSES}
    labeled = np.zeros(n, bool)
    for r in labels:
        raw = r["label"]
        if raw not in LABEL_MAP:
            continue
        cls = LABEL_MAP[raw]
        ts, te = int(r["tick_start"]), int(r["tick_end"])
        if te <= t[0] or ts >= t[-1]:
            continue
        m = (t >= ts) & (t <= te)
        if cls == "light_step":
            i0 = int(np.argmax(t >= ts)) + 10
            i1 = int(np.argmax(t >= te)) - 10
            if i1 <= i0:
                continue
            lab[cls][i0:i1] = True
            labeled[i0:i1] = True
        else:
            lab[cls] |= m
            labeled |= m
    near = labeled.copy()
    for k in range(1, guard + 1):
        near[k:] |= labeled[:-k]
        near[:-k] |= labeled[k:]
    lab["normal"] = ~near
    return lab


def cut_windows(F, mask, W, stride):
    """F: (N, C) 特征矩阵；在 mask 全真的连续段上滑窗。"""
    starts, run = [], []
    for i, ok in enumerate(mask):
        if ok:
            run.append(i)
        else:
            if run:
                starts.extend(range(run[0], max(run[0] + 1, run[-1] - W + 2), stride))
                run = []
    if run:
        starts.extend(range(run[0], max(run[0] + 1, run[-1] - W + 2), stride))
    out = [F[s:s + W] for s in starts if s + W <= len(mask) and mask[s:s + W].all()]
    return np.array(out, dtype=np.float32) if out else np.zeros((0, W, F.shape[1]), np.float32)


def cut_events(F, t, labels, want, W):
    out = []
    for r in labels:
        if r["label"] not in want:
            continue
        ts = int(r["tick_start"])
        if ts < t[0] or ts > t[-1]:
            continue
        i0 = int(np.argmax(t >= ts)) - W // 4
        i0 = max(0, min(len(t) - W, i0))
        out.append(F[i0:i0 + W])
    return np.array(out, dtype=np.float32) if out else np.zeros((0, W, F.shape[1]), np.float32)


def export_c_header(mean, scale, W1, b1, W2, b2, W3, b3, path, W, base0, tau):
    """导出转置后的权重 + 倒数 scale

    为什么转置
        sklearn 的 coefs_ 形状是 (输入, 输出)。推理时内层循环沿"输入维"展开，
        原布局 model_W1[k][j] 每走一步要跳 H1 个 float（20 维就是 80 字节）。
        F407 在 168MHz 下 Flash 有 5 个等待周期，跳跃访问完全用不上预取
        ⇒ 实测 13.4 周期 / 次乘加，比理论值慢 13 倍。
        转置成 W1T[j][k] 后内层连续，预取能命中。

    为什么要 inv_scale
        标准化原来写 (x - mean) / scale，每维一次除法。
        M4F 的 VDIV 要 14 个周期，140 维就是 1960 周期，
        改成乘倒数后只需 140 次乘法。
    """
    def arr(name, a):
        flat = np.asarray(a).reshape(-1)
        body = ", ".join("%.8ff" % float(v) for v in flat)
        return "static const float %s[%d] = {%s};\n" % (name, flat.size, body)

    inv_scale = 1.0 / np.asarray(scale, dtype=np.float64)
    s = []
    s.append("/* model_weights.h -- 自动生成，勿手改\n")
    s.append(" * 特征：EMA 慢基线高差，tau = %d 采样（%.0f 秒 @10Hz）\n" % (tau, tau / 10.0))
    s.append(" * 窗口 %d 拍（%.1f 秒），通道 %d\n" % (W, W * 0.1, len(CHANNELS)))
    s.append(" * 类别：%s\n" % (" / ".join(CLASSES)))
    s.append(" *\n * 权重已转置：W1T 形状 (%d, %d)，W2T (%d, %d)，W3T (%d, %d)\n" %
             (W1.shape[1], W1.shape[0], W2.shape[1], W2.shape[0], W3.shape[1], W3.shape[0]))
    s.append(" * 标准化因子已转成倒数（乘代替除）\n */\n\n")
    s.append("#ifndef __MODEL_WEIGHTS_H\n#define __MODEL_WEIGHTS_H\n\n")
    s.append("#define MODEL_W     %d\n" % W)
    s.append("#define MODEL_IN    %d\n" % len(CHANNELS))
    s.append("#define MODEL_H1    %d\n" % W1.shape[1])
    s.append("#define MODEL_H2    %d\n" % W2.shape[1])
    s.append("#define MODEL_OUT   %d\n" % W3.shape[1])
    s.append("#define MODEL_TAU   %d\n" % tau)
    s.append("\n/* EMA 基线初值（上电时用实时读数初始化，这里只是参考） */\n")
    s.append(arr("model_base0", base0))
    s.append("\n")
    s.append(arr("model_in_mean", mean))
    s.append(arr("model_in_inv_scale", inv_scale))
    s.append("\n/* 转置后的权重 */\n")
    s.append(arr("model_W1T", np.asarray(W1).T))
    s.append(arr("model_b1", b1))
    s.append(arr("model_W2T", np.asarray(W2).T))
    s.append(arr("model_b2", b2))
    s.append(arr("model_W3T", np.asarray(W3).T))
    s.append(arr("model_b3", b3))
    s.append("\n#endif\n")
    with open(path, "w", encoding="utf-8") as f:
        f.write("".join(s))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--window", type=int, default=20)
    ap.add_argument("--hidden", type=int, default=16)
    ap.add_argument("--stride", type=int, default=5)
    ap.add_argument("--normal-ratio", type=float, default=3.0,
                    help="normal 降采样到其他类总样本的这个倍数")
    ap.add_argument("--out", default=os.path.join(HERE, "model_weights.h"))
    a = ap.parse_args()
    W = a.window

    labels = list(csv.DictReader(open(os.path.join(HERE, "labels.csv"), encoding="utf-8")))

    Xs, ys, srcs = [], [], []
    for si, name in enumerate(("session.npy", "session2.npy")):
        p = os.path.join(HERE, name)
        if not os.path.isfile(p):
            continue
        d = load_session(p)
        t = d["tick"].astype(np.int64)
        # ★ 2026-11-13 修：两个会话的 tick 都从接近 0 开始，范围重叠。
        #   只按 tick 找标签会把另一段会话的标签错配到本段。
        #   ⇒ 用 rtc（真实 Unix 秒，两段不重叠）再筛一层。
        rlo = int(d["rtc"].min())
        rhi = int(d["rtc"].max())
        slabel = [r for r in labels
                  if rlo - 5 <= int(r["rtc_start"] or 0) <= rhi + 5]
        # ★ 先做基线高差，再切窗口
        F = np.stack([ema_highpass(d[c])[0] for c in CHANNELS], axis=1)
        base0 = np.array([float(d[c][0]) for c in CHANNELS], np.float32)
        if len(Xs) == 0:
            FIRST_BASE0 = base0
        lab = build_masks(t, slabel, len(t))
        print("[%s] 特征已高差化" % name)
        for ci, cls in enumerate(CLASSES):
            if cls == "impact":
                X = cut_events(F, t, slabel, {"shake", "tap_board"}, W)
            elif cls == "tilt":
                X = cut_events(F, t, slabel, {"tilt"}, W)
            else:
                X = cut_windows(F, lab[cls], W, a.stride)
            if len(X) == 0:
                continue
            Xs.append(X); ys.append(np.full(len(X), ci)); srcs.append(np.full(len(X), si))
            print("   %-12s %4d 窗口" % (cls, len(X)))

    X = np.concatenate(Xs).reshape(-1, W * len(CHANNELS))
    y = np.concatenate(ys)
    src = np.concatenate(srcs)
    print("\n总样本 %d，特征维 %d" % (len(X), X.shape[1]))

    # ★ normal 降采样，避免类别极度不平衡
    rng = np.random.default_rng(0)
    ni = CLASSES.index("normal")
    idx_n = np.where(y == ni)[0]
    idx_o = np.where(y != ni)[0]
    keep = min(len(idx_n), int(len(idx_o) * a.normal_ratio))
    sel = np.concatenate([rng.choice(idx_n, keep, replace=False), idx_o])
    sel.sort()
    X, y, src = X[sel], y[sel], src[sel]
    print("normal 降采样到 %d（其他类 %d）" % (keep, len(idx_o)))

    # ★ 主评估：分层随机 80/20；副评估：跨会话
    tr_idx, te_idx = [], []
    for c in range(len(CLASSES)):
        ii = np.where(y == c)[0]
        rng.shuffle(ii)
        cut = max(1, int(len(ii) * 0.8))
        tr_idx.extend(ii[:cut]); te_idx.extend(ii[cut:])
    tr_idx = np.array(tr_idx); te_idx = np.array(te_idx)

    mean = X[tr_idx].mean(axis=0)
    scale = X[tr_idx].std(axis=0)
    scale[scale < 1e-6] = 1.0
    Xn = (X - mean) / scale

    from sklearn.neural_network import MLPClassifier
    from sklearn.metrics import confusion_matrix
    clf = MLPClassifier(hidden_layer_sizes=(a.hidden, 16), activation="relu",
                        solver="adam", max_iter=4000, random_state=0)
    clf.fit(Xn[tr_idx], y[tr_idx])

    def report(idx, name):
        if len(idx) == 0:
            return 1.0
        pred = clf.predict(Xn[idx])
        acc = float((pred == y[idx]).mean())
        print("\n=== %s  准确率 %.3f  (%d 样本) ===" % (name, acc, len(idx)))
        print(confusion_matrix(y[idx], pred, labels=range(len(CLASSES))))
        print("  行=真值 列=预测  顺序: " + " ".join(CLASSES))
        for i, c in enumerate(CLASSES):
            n = int((y[idx] == i).sum())
            if n:
                print("    %-12s n=%4d  召回 %.2f" % (c, n, float((pred[y[idx] == i] == i).mean())))
        return acc

    acc_test = report(te_idx, "测试集（分层随机 20%）")
    report(tr_idx, "训练集")
    if len(np.unique(src)) >= 2:
        for s in np.unique(src):
            report(np.where(src == s)[0], "会话 %d 全体" % s)

    # ★ 2026-11-13 加：跨会话检验（真正的泛化指标）
    #   为什么必须做：随机切分会把同一段时间切到训练和测试两边，
    #   得到的 98% 是"记住这一段"，不是"学会这个概念"
    #   跨会话才是部署后的真实场景（换了一天、环境变了）
    print("\n" + "=" * 62)
    print("跨会话检验：训练一个会话，测试另一个会话")
    print("=" * 62)
    for holdout in np.unique(src):
        tr2 = src != holdout
        te2 = src == holdout
        if te2.sum() < 5 or tr2.sum() < 50:
            continue
        m2 = X[tr2].mean(axis=0)
        s2 = X[tr2].std(axis=0)
        s2[s2 < 1e-6] = 1.0
        clf2 = MLPClassifier(hidden_layer_sizes=(a.hidden, 16), activation="relu",
                             solver="adam", max_iter=4000, random_state=0)
        clf2.fit((X[tr2] - m2) / s2, y[tr2])
        pred = clf2.predict((X[te2] - m2) / s2)
        acc2 = float((pred == y[te2]).mean())
        print("\n训练=会话%d(%d)  →  测试=会话%d(%d)   准确率 %.3f"
              % (1 - holdout if len(np.unique(src)) == 2 else -1, tr2.sum(),
                 holdout, te2.sum(), acc2))
        print(confusion_matrix(y[te2], pred, labels=range(len(CLASSES))))
        print("  行=真值 列=预测  顺序: " + " ".join(CLASSES))
        for i, c in enumerate(CLASSES):
            n = int((y[te2] == i).sum())
            if n:
                print("    %-12s n=%4d  召回 %.2f" % (c, n, float((pred[y[te2] == i] == i).mean())))
    # ★ 2026-11-13 加：导出测试窗口 + sklearn 预测，供 C 实现逐条比对
    #   为什么要：手写的 C 推理如果有下标错位（比如权重布局搞反），
    #             在 MCU 上表现为"准确率很低"，很难判断是模型问题还是代码问题
    #             PC 上逐条对数能把"代码错"和"模型弱"彻底分开
    dump = os.path.join(HERE, "verify_windows.csv")
    sel_te = te_idx[:200]
    pred_te = clf.predict(Xn[sel_te])
    with open(dump, "w", encoding="utf-8", newline="") as fh:
        wr = csv.writer(fh)
        wr.writerow(["sk_pred"] + ["x%d" % i for i in range(X.shape[1])])
        for i, p in zip(sel_te, pred_te):
            wr.writerow([int(p)] + ["%.8g" % v for v in X[i]])
    print("\n已导出 %s（%d 个窗口，含 sklearn 预测）" % (dump, len(sel_te)))
    W1, b1 = clf.coefs_[0], clf.intercepts_[0]
    W2, b2 = clf.coefs_[1], clf.intercepts_[1]
    W3, b3 = clf.coefs_[2], clf.intercepts_[2]
    export_c_header(mean, scale, W1, b1, W2, b2, W3, b3, a.out, W, FIRST_BASE0, TAU_SAMPLES)
    nparam = sum(x.size for x in (W1, b1, W2, b2, W3, b3))
    print("\n已导出 %s" % a.out)
    print("  参数 %d（float32 ≈ %.1f KB）" % (nparam, nparam * 4 / 1024.0))
    print("  每次推理 %d MAC → F407 保守估 %.2f ms" % (nparam, nparam / 42e6 * 1000))
    return 0


if __name__ == "__main__":
    sys.exit(main())





