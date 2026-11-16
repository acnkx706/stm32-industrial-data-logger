# probe_data.py -- 记录仪原始数据的"信息量体检"
#
# 做了什么：流式扫描一天 17 列的 CSV，算每列的量纲、变异系数、变化速率、
#           自相关；再算 11 维物理量的相关矩阵与 PCA 方差解释。
# 为什么：模型的识别上限由传感器数据的信息量决定，而这个信息量可以先算出来，
#         不必先训模型。散得太开 = 对异常不敏感；相关性太强 = 维度虚高。
# 影响什么：只读原始 CSV，不修改任何文件；输出到 stdout 和 --out。
# 怎么验证：tick 相邻差恒为 100ms（证实 10Hz）；humi 的 65535 计数应为 DHT22 读失败。
#
# 用法：
#   python probe_data.py --file <CSV> [--rows 2000000] [--out report.md]

import argparse
import io
import sys
from collections import Counter

import numpy as np

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

COLS = ["seq", "tick", "press", "rtc", "ax", "ay", "az", "gx", "gy", "gz",
        "temp_ds", "temp_chip", "humi", "light", "vbat", "sonar_mm", "crc"]

# 物理量列（去掉元数据）
PHYS = ["press", "ax", "ay", "az", "gx", "gy", "gz",
        "temp_ds", "temp_chip", "humi", "light", "vbat", "sonar_mm"]

# DHT22 读失败时驱动常回 0xFFFF
INVALID = {"humi": 65535.0}


def stream(path, max_rows):
    """逐行解析，跳过注释与表头，列数不符的行计数后丢弃。"""
    buf, bad, n = [], 0, 0
    with open(path, "r", encoding="latin-1") as f:
        for line in f:
            s = line.rstrip("\r\n")
            if not s or s[0] == "#" or s.startswith("seq,"):
                continue
            p = s.split(",")
            if len(p) != 17:
                bad += 1
                continue
            buf.append(p)
            n += 1
            if n >= max_rows:
                break
    arr = np.array(buf, dtype=np.float64)
    return arr, bad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--file", required=True)
    ap.add_argument("--rows", type=int, default=2_000_000)
    ap.add_argument("--out", default="")
    a = ap.parse_args()

    out = []

    def w(s=""):
        out.append(s)
        print(s)

    arr, bad = stream(a.file, a.rows)
    w(f"# 数据体检报告")
    w()
    w(f"- 文件：`{a.file}`")
    w(f"- 有效行：{len(arr):,}   坏行（列数不符）：{bad:,}")
    if len(arr) < 100:
        w("\n数据太少，终止。")
        return

    df = {c: arr[:, i] for i, c in enumerate(COLS)}

    # ---- 0. 采样率验证 ----
    dt = np.diff(df["tick"])
    w(f"- tick 相邻差：中位数 {np.median(dt):.0f} ms，众数 {Counter(dt).most_common(3)}")
    w(f"- 推算采样率：{1000.0 / np.median(dt):.2f} Hz")

    # ---- 1. 量纲与异常值 ----
    w()
    w("## 1. 每列量纲与分布")
    w()
    w("| 列 | min | p1 | p50 | p99 | max | 均值 | 标准差 | 无效值 |")
    w("|---|---|---|---|---|---|---|---|---|")
    for c in COLS:
        v = df[c]
        inv = INVALID.get(c)
        n_inv = int(np.sum(v == inv)) if inv is not None else 0
        vv = v[v != inv] if inv is not None else v
        if len(vv) == 0:
            w(f"| {c} | — | — | — | — | — | — | — | {n_inv} |")
            continue
        w("| {} | {:.0f} | {:.0f} | {:.0f} | {:.0f} | {:.0f} | {:.1f} | {:.1f} | {} |".format(
            c, vv.min(), np.percentile(vv, 1), np.median(vv), np.percentile(vv, 99),
            vv.max(), vv.mean(), vv.std(), n_inv))

    # ---- 2. 信息量：变异系数 / 变化速率 / 自相关 ----
    w()
    w("## 2. 每列的信息量")
    w()
    w("变异系数 = 标准差 / |均值|，太小说明这列几乎不动，对判别没贡献。")
    w("变化速率 = |一阶差分| 的中位数，衡量 100ms 内这列能变多少。")
    w("自相关(1) 接近 1 说明是慢变量（趋势型），接近 0 说明逐拍跳动。")
    w()
    w("| 列 | 变异系数 | 变化速率 | 自相关(1) | 唯一值 | 判读 |")
    w("|---|---|---|---|---|---|")
    for c in PHYS:
        v = df[c]
        inv = INVALID.get(c)
        if inv is not None:
            v = v[v != inv]
        if len(v) < 100:
            w(f"| {c} | — | — | — | — | 有效值太少 |")
            continue
        m, s = v.mean(), v.std()
        cv = abs(s / m) if abs(m) > 1e-12 else float("inf")
        rate = np.median(np.abs(np.diff(v)))
        if s > 1e-12:
            ac = np.corrcoef(v[:-1], v[1:])[0, 1]
        else:
            ac = float("nan")
        uniq = len(np.unique(v))
        if cv < 0.001:
            verdict = "几乎不动，无判别力"
        elif uniq < 10:
            verdict = "取值太少，近似离散"
        elif ac > 0.995:
            verdict = "极慢，只能看趋势"
        elif ac < 0.2:
            verdict = "逐拍跳动，噪声占比高"
        else:
            verdict = "可用"
        w(f"| {c} | {cv:.4f} | {rate:.3f} | {ac:.3f} | {uniq} | {verdict} |")

    # ---- 3. 相关矩阵 ----
    w()
    w("## 3. 相关矩阵（|r| > 0.7 的才算强相关）")
    w()
    cols = [c for c in PHYS if np.std(df[c]) > 1e-9]
    mat = np.array([df[c] for c in cols])
    # 无效值处理：把整列含 65535 的位置剔除
    keep = np.ones(mat.shape[1], dtype=bool)
    for i, c in enumerate(cols):
        if c in INVALID:
            keep &= (mat[i] != INVALID[c])
    mat = mat[:, keep]
    C = np.corrcoef(mat)
    w("```")
    w("            " + " ".join(f"{c[:8]:>8}" for c in cols))
    for i, c in enumerate(cols):
        row = " ".join(f"{C[i, j]:8.2f}" for j in range(len(cols)))
        w(f"{c[:11]:<11} {row}")
    w("```")
    pairs = []
    for i in range(len(cols)):
        for j in range(i + 1, len(cols)):
            if abs(C[i, j]) > 0.7:
                pairs.append(f"{cols[i]} ~ {cols[j]}: r = {C[i, j]:.2f}")
    w()
    if pairs:
        w("强相关对（其中一个是冗余的）：")
        for p in pairs:
            w(f"- {p}")
    else:
        w("没有 |r| > 0.7 的强相关对，维度基本独立。")

    # ---- 4. PCA：正常数据散不散 ----
    w()
    w("## 4. 正常数据的联合离散度（PCA）")
    w()
    X = mat.T.astype(np.float64)
    X = (X - X.mean(axis=0)) / (X.std(axis=0) + 1e-12)
    cov = np.cov(X, rowvar=False)
    ev = np.sort(np.linalg.eigvalsh(cov))[::-1]
    ev = np.clip(ev, 0, None)
    ratio = ev / ev.sum()
    cum = np.cumsum(ratio)
    w("| 主成分 | 解释方差 | 累计 |")
    w("|---|---|---|")
    for i in range(min(8, len(ratio))):
        w(f"| PC{i+1} | {ratio[i]*100:.1f}% | {cum[i]*100:.1f}% |")
    w()
    n90 = int(np.searchsorted(cum, 0.90) + 1)
    w(f"达到 90% 方差需要 **{n90}** 个主成分（总共 {len(cols)} 维）。")
    if n90 >= len(cols) - 1:
        w()
        w("判读：**维度几乎全独立，没有冗余**，说明每一列都在提供不同信息。"
          "好处是特征不浪费；坏处是「正常」区域是个高维球，边界模糊。")
    elif n90 <= 3:
        w()
        w("判读：**大部分方差集中在前几个主成分**，说明数据本质是低维的，"
          "其余列基本是噪声，可以裁掉。")

    # ---- 5. 10 秒窗口的样本散度（真正决定能不能检异常） ----
    w()
    w("## 5. 10 秒窗口（100 拍）的样本间距离")
    w()
    w("把每 100 个点组成一个「样本」，算样本之间的欧氏距离（标准化后）。")
    w("距离分布越窄，说明「正常」越稳定，异常越好区分；越宽，越难。")
    w()
    W = 100
    nwin = len(X) // W
    if nwin < 10:
        w("窗口数太少，跳过。")
    else:
        feat = X[:nwin * W].reshape(nwin, W, -1)
        # 每个窗口取 均值 + 标准差 + 差分均值，三类统计量
        f_mean = feat.mean(axis=1)
        f_std = feat.std(axis=1)
        f_rate = np.abs(np.diff(feat, axis=1)).mean(axis=1)
        F = np.concatenate([f_mean, f_std, f_rate], axis=1)
        ctr = F.mean(axis=0)
        d = np.linalg.norm(F - ctr, axis=1)
        w(f"- 窗口数：{nwin}")
        w(f"- 到中心的距离：p50 = {np.median(d):.2f}, p95 = {np.percentile(d,95):.2f}, "
          f"max = {d.max():.2f}")
        sp = np.percentile(d, 95) / max(np.median(d), 1e-9)
        w(f"- 离散比 p95/p50 = **{sp:.2f}**")
        w()
        if sp < 1.5:
            w("判读：正常样本高度聚集，异常有较大空间被检出。**数据可用。**")
        elif sp < 3.0:
            w("判读：正常样本有一定离散，需要较宽的判据，"
              "或者用自编码器学分布而不是用距离。")
        else:
            w("判读：正常样本散得很开，单靠距离判据不可行，"
              "必须做特征工程（差分、中值滤波、温补）。")

    if a.out:
        with open(a.out, "w", encoding="utf-8") as f:
            f.write("\n".join(out))
        print(f"\n已写入 {a.out}")


if __name__ == "__main__":
    main()
