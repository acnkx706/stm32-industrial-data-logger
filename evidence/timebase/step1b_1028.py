# step1b_1028.py -- 台阶定位（渐变稳健版）
#
# 做了什么：
#   1) 反证检查修正：10 Hz 下同一秒有 10 行，所以 Δrtc 只应取 0 或 1（原来用 ≠1 判，
#      把 90% 的正常行都算成"跳变"，是错的）；
#   2) 用 5 分钟窗口 / 60 秒步长的滚动回归出 ppm 曲线，逐点打印过渡段；
#   3) 台阶用"平台 + 过中点 + 10%~90% 宽度"描述，不强行套两段直线的瞬时阶跃模型；
#   4) 两段回归只作为交叉验证，并明确它的局限（过渡 20 分钟时它会把断点拉偏）。
# 为什么：方案 §3 要求"若过渡 >30 min 就记渐变，不要硬写阶跃"。两段回归在渐变上
#         会把断点定到别处、斜率也算歪，必须换口径，否则结论不可复现。
# 影响什么：只读 CSV。
# 怎么验证：平台值应稳定（±30 ppm）；过中点时刻应与曲线目视一致；宽度应可重复。
#
# 用法：python step1b_1028.py
import sys

import numpy as np
import pandas as pd

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

STAGE = r"..\longrun\_stage"
NAME = "26102801.CSV"


def main():
    print("读 %s ..." % NAME)
    df = pd.read_csv("%s/%s" % (STAGE, NAME),
                     usecols=["seq", "tick", "rtc", "temp_ds", "temp_chip"],
                     on_bad_lines="skip", engine="python").dropna()
    print("行数 %d，rtc %d..%d\n" % (len(df), df.rtc.min(), df.rtc.max()))

    # ---- 反证（修正版）----
    d_rtc = np.diff(df.rtc.values)
    d_tick = np.diff(df.tick.values)
    d_seq = np.diff(df.seq.values)
    bad_rtc = int(((d_rtc != 0) & (d_rtc != 1)).sum())
    vc = pd.Series(d_tick).value_counts().head(6)
    print("[反证] seq 回落 %d 次 | Δrtc 不等于 0/1 的行 %d 次 | Δtick 非 100 的行 %d 次"
          % (int((d_seq < 0).sum()), bad_rtc, int((d_tick != 100).sum())))
    print("[反证] Δtick 最常见取值: %s"
          % ", ".join("%d×%d" % (k, v) for k, v in vc.items()))
    print("[温度] temp_ds %.1f~%.1f ℃   temp_chip %.1f~%.1f ℃\n"
          % (df.temp_ds.min() / 100, df.temp_ds.max() / 100,
             df.temp_chip.min() / 100, df.temp_chip.max() / 100))

    # ---- 逐秒 ----
    g = df.groupby("rtc", sort=True)["tick"].agg(["mean", "count"])
    g = g[g["count"] >= 5]
    x = g.index.values.astype(np.int64)
    y = g["mean"].values

    def ppm_series(win, step=60):
        xs, ps = [], []
        pos = x[0]
        while pos + win <= x[-1]:
            m = (x >= pos) & (x < pos + win)
            if m.sum() > 100:
                a = np.polyfit(x[m], y[m], 1)[0]
                xs.append(pos + win / 2)
                ps.append((a / 1000.0 - 1.0) * 1e6)
            pos += step
        return np.array(xs), np.array(ps)

    xx, pp5 = ppm_series(300)                    # 5 分钟窗
    x10, pp10 = ppm_series(600)                  # 10 分钟窗

    day0 = x[0]
    def hhmm(t):
        return pd.to_datetime(int(t), unit="s", utc=True).strftime("%H:%M")

    pre = np.median(pp5[(xx >= day0 + 17 * 3600) & (xx < day0 + 19 * 3600)])
    post = np.median(pp5[(xx >= day0 + 22 * 3600) & (xx <= day0 + 24 * 3600 - 60)])
    print("平台值（5 min 窗）：pre(17:00-19:00) = %.0f ppm，post(22:00-23:59) = %.0f ppm"
          % (pre, post))

    mid = (pre + post) / 2.0
    nz = np.where((xx >= day0 + 18 * 3600) & (xx <= day0 + 22 * 3600))[0]
    cross = None
    for k in nz[1:]:
        if (pp5[k] - mid) * (pp5[k - 1] - mid) <= 0:
            cross = xx[k]
            break
    print("过中点（%.0f ppm）时刻 = %s UTC" % (mid, hhmm(cross) if cross else "未找到"))

    lo_t = pre + 0.1 * (post - pre)
    hi_t = pre + 0.9 * (post - pre)
    band = np.where((xx >= day0 + 18 * 3600) & (xx <= day0 + 22 * 3600) &
                    (pp5 >= min(lo_t, hi_t)) & (pp5 <= max(lo_t, hi_t)))[0]
    if len(band) > 1:
        print("过渡 10%%~90%% = %s ~ %s UTC，宽度 %.0f min"
              % (hhmm(xx[band[0]]), hhmm(xx[band[-1]]), (xx[band[-1]] - xx[band[0]]) / 60.0))

    print("\n过渡段逐点（10 min 窗，每 2 min 一个点）：")
    sel = (x10 >= day0 + 19 * 3600) & (x10 <= day0 + 21 * 3600)
    for t, p in zip(x10[sel], pp10[sel]):
        if int(t) % 120 == 0:
            print("  %s  %8.0f ppm" % (hhmm(t), p))

    # ---- 交叉验证：两段回归（只在 19:00~21:30 内，且报告它的局限）----
    lo, hi = day0 + 19 * 3600, day0 + 21 * 3600 + 1800
    m = (x >= lo) & (x <= hi)
    xs, ys = x[m].astype(float), y[m]
    n = len(xs)
    c = np.concatenate(([0.0], np.cumsum(np.ones(n))))
    sx = np.concatenate(([0.0], np.cumsum(xs)))
    sy = np.concatenate(([0.0], np.cumsum(ys)))
    sxx = np.concatenate(([0.0], np.cumsum(xs * xs)))
    sxy = np.concatenate(([0.0], np.cumsum(xs * ys)))
    syy = np.concatenate(([0.0], np.cumsum(ys * ys)))

    def rss(i, j):
        nn = c[j] - c[i]
        if nn < 30:
            return np.inf
        Sx, Sy = sx[j] - sx[i], sy[j] - sy[i]
        Sxx, Sxy, Syy = sxx[j] - sxx[i], sxy[j] - sxy[i], syy[j] - syy[i]
        var = Sxx - Sx * Sx / nn
        if var <= 0:
            return np.inf
        cov = Sxy - Sx * Sy / nn
        return (Syy - Sy * Sy / nn) - cov * cov / var

    best = (np.inf, None)
    for k in range(100, n - 100):
        r = rss(0, k) + rss(k, n)
        if r < best[0]:
            best = (r, k)
    k = best[1]
    a1 = np.polyfit(xs[:k], ys[:k], 1)[0]
    a2 = np.polyfit(xs[k:], ys[k:], 1)[0]
    print("\n[交叉验证] 两段回归断点 = %s UTC（前 %.0f / 后 %.0f ppm）"
          % (hhmm(xs[k]), (a1 / 1000 - 1) * 1e6, (a2 / 1000 - 1) * 1e6))
    print("  注意：过渡是渐变时，两段直线模型会把断点拉向过渡段中部、并把前段斜率算浅，"
          "\n        所以它只能当量级校验，不能当断点时刻。")


if __name__ == "__main__":
    main()
