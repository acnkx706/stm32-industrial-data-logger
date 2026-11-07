# step1_1028.py -- 用现有 CSV 把 10-28 的 tick/rtc 台阶定位到分钟
#
# 做了什么：读 261027/261028/261029 三个 CSV，按 rtc 分桶取 tick 均值，
#           先做 10 分钟滚动回归看 ppm 曲线，再在 18:00~21:00 UTC 内用两段回归
#           找残差平方和最小的断点，最后量 10%~90% 的过渡宽度。
# 为什么：日平均比率只能说明"台阶存在"，定不到分钟；而且 tick 和 rtc 谁是变量
#           靠比率分不开，得先把时间点钉死，第二步 24h 才有对照。
# 影响什么：只读 CSV，不改固件、不碰板子。
# 怎么验证：断点前后斜率差应 >1000 ppm；滚动过零点与两段回归断点应相差 <=5 min。
#
# 用法：python step1_1028.py
import io
import sys

import numpy as np
import pandas as pd

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

STAGE = r"..\longrun\_stage"
OUT = "step_1028.csv"


def load(name):
    p = "%s/%s" % (STAGE, name)
    df = pd.read_csv(p, usecols=["seq", "tick", "rtc", "temp_ds", "temp_chip"],
                     on_bad_lines="skip", engine="python")
    df = df.dropna()
    print("%s: %d 行, rtc %d..%d" % (name, len(df), df.rtc.min(), df.rtc.max()))
    return df


def sanity(df, name):
    d_rtc = np.diff(df.rtc.values)
    d_tick = np.diff(df.tick.values)
    d_seq = np.diff(df.seq.values)
    print("  [反证] seq 回落 %d 次 | rtc 跳变(≠1) %d 次 | tick 跳变(≠100) %d 次"
          % (int((d_seq < 0).sum()), int((d_rtc != 1).sum()), int((d_tick != 100).sum())))
    print("  [温度] temp_ds %.1f~%.1f  temp_chip %.1f~%.1f"
          % (df.temp_ds.min(), df.temp_ds.max(), df.temp_chip.min(), df.temp_chip.max()))


def per_second(df):
    g = df.groupby("rtc", sort=True)["tick"].agg(["mean", "count"])
    g = g[g["count"] >= 5]                     # 少于 5 行的秒桶不要，边界不完整
    return g.index.values.astype(np.int64), g["mean"].values


def rolling_ppm(x, y, win=600, step=60):
    """x=rtc(s), y=tick_avg(ms)。返回 (窗口中心 x, ppm)。"""
    out_x, out_ppm = [], []
    x0, x1 = x[0], x[-1]
    pos = x0
    while pos + win <= x1:
        m = (x >= pos) & (x < pos + win)
        if m.sum() > 100:
            a = np.polyfit(x[m], y[m], 1)[0]        # ms per s
            out_x.append(pos + win / 2)
            out_ppm.append((a / 1000.0 - 1.0) * 1e6)
        pos += step
    return np.array(out_x), np.array(out_ppm)


def two_segment_break(x, y, lo, hi, step=10):
    """在 [lo,hi] 内找两段直线拟合残差平方和最小的断点。"""
    m = (x >= lo) & (x <= hi)
    xs = x[m].astype(np.float64)
    ys = y[m]
    n = len(xs)
    if n < 200:
        return None
    # 累积量，O(1) 算每段 RSS
    c1 = np.concatenate(([0.0], np.cumsum(np.ones(n))))
    sx = np.concatenate(([0.0], np.cumsum(xs)))
    sy = np.concatenate(([0.0], np.cumsum(ys)))
    sxx = np.concatenate(([0.0], np.cumsum(xs * xs)))
    sxy = np.concatenate(([0.0], np.cumsum(xs * ys)))
    syy = np.concatenate(([0.0], np.cumsum(ys * ys)))

    def rss(i, j):
        nn = c1[j] - c1[i]
        if nn < 30:
            return np.inf
        Sx = sx[j] - sx[i]
        Sy = sy[j] - sy[i]
        Sxx = sxx[j] - sxx[i]
        Sxy = sxy[j] - sxy[i]
        Syy = syy[j] - syy[i]
        var = Sxx - Sx * Sx / nn
        if var <= 0:
            return np.inf
        cov = Sxy - Sx * Sy / nn
        ss_tot = Syy - Sy * Sy / nn
        return ss_tot - cov * cov / var

    best = (np.inf, None)
    idx = np.where((xs >= lo) & (xs <= lo + (hi - lo)))[0]
    for k in range(60, n - 60, max(1, int(step))):
        r = rss(0, k) + rss(k, n)
        if r < best[0]:
            best = (r, k)
    k = best[1]
    if k is None:
        return None
    a1 = np.polyfit(xs[:k], ys[:k], 1)[0]
    a2 = np.polyfit(xs[k:], ys[k:], 1)[0]
    return xs[k], (a1 / 1000 - 1) * 1e6, (a2 / 1000 - 1) * 1e6


def main():
    df27 = load("26102701.CSV"); sanity(df27, "10-27")
    df28 = load("26102801.CSV"); sanity(df28, "10-28")
    df29 = load("26102901.CSV"); sanity(df29, "10-29")

    for df, nm in ((df27, "10-27"), (df28, "10-28"), (df29, "10-29")):
        x, y = per_second(df)
        xx, pp = rolling_ppm(x, y)
        print("%s 滚动 ppm: 前段中位 %.0f, 后段中位 %.0f"
              % (nm, np.median(pp[:len(pp)//3]), np.median(pp[-len(pp)//3:])))

    x, y = per_second(df28)
    res = y - 1000.0 * x
    print("\n10-28 残差 res_ms: 起 %.0f, 止 %.0f" % (res[0], res[-1]))

    xx, pp = rolling_ppm(x, y)
    sign = np.sign(pp)
    cross = None
    for k in range(1, len(pp)):
        if sign[k] != sign[k - 1]:
            cross = xx[k]
            break
    print("滚动过零点 rtc = %s" % cross)

    # 18:00~21:00 UTC = 断点搜索窗
    day0 = x[0] - (x[0] % 86400)
    lo = day0 + 18 * 3600
    hi = day0 + 21 * 3600
    bp = two_segment_break(x, y, lo, hi)
    if bp is None:
        print("两段回归失败")
        return
    bx, p1, p2 = bp
    print("两段回归断点 rtc = %.0f，前 %.0f ppm，后 %.0f ppm" % (bx, p1, p2))
    print("  折合 UTC: %s" % pd.to_datetime(int(bx), unit="s", utc=True))

    # 过渡宽度：ppm 从 10% 到 90%
    span = p2 - p1
    if abs(span) > 1:
        t10 = p1 + 0.1 * span
        t90 = p1 + 0.9 * span
        m = np.where((pp >= min(t10, t90)) & (pp <= max(t10, t90)))[0]
        if len(m) > 1:
            print("过渡宽度 10%%->90%% = %.0f min（%s ~ %s UTC）"
                  % ((xx[m[-1]] - xx[m[0]]) / 60.0,
                     pd.to_datetime(int(xx[m[0]]), unit="s", utc=True).strftime("%H:%M"),
                     pd.to_datetime(int(xx[m[-1]]), unit="s", utc=True).strftime("%H:%M")))

    with io.open(OUT, "w", encoding="utf-8", newline="") as f:
        f.write("rtc,tick_avg,res_ms,ppm_10min\n")
        j = 0
        for i in range(len(x)):
            ppm = ""
            if j < len(xx) and abs(x[i] - xx[j]) < 30:
                ppm = "%.0f" % pp[j]
                if i > 0 and x[i] - xx[j] > 30:
                    j += 1
            f.write("%d,%.1f,%.1f,%s\n" % (x[i], y[i], res[i], ppm))
    print("\n输出 -> %s（%d 秒桶）" % (OUT, len(x)))


if __name__ == "__main__":
    main()
