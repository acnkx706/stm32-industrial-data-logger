# step1d_1028.py -- 相位误差法：直接对 r = rtc − 0.1·i 做两段拟合
#
# 做了什么：
#   每个样本的序号 i 就是"第 i 个 0.1 秒"。若 RTC 和采样时钟同速，则 rtc = 0.1·i + 常数；
#   把 r = rtc − 0.1·i 画出来，r 的斜率就是两者的相对速率，台阶表现为 r 的折线拐点。
#   先对 r 做 60 秒滑动平均（消掉 1 秒量化的锯齿），再在 19:00~21:00 内做两段直线拟合找拐点。
# 为什么：按秒取均值会让"某秒 10 条 / 11 条"变成 100 ms 的系统跳变，进而造出 ±250 ppm 的
#         假周期；而 5 分钟窗口的斜率误差本身就有 ±330 ppm。相位误差法用上了每一个样本，
#         精度只取决于拟合区间长度（2 小时 ≈ ±140 ppm，用来定拐点足够）。
# 影响什么：只读 CSV。
# 怎么验证：台阶前斜率应约 −1800 ppm、台阶后应约 0；拐点两侧线性度好（残差里看不到周期）。
#
# 用法：python step1d_1028.py
import sys

import numpy as np
import pandas as pd

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

STAGE = r"..\longrun\_stage"


def hhmm(t):
    return pd.to_datetime(int(t), unit="s", utc=True).strftime("%H:%M:%S")


def load(name):
    df = pd.read_csv("%s/%s" % (STAGE, name),
                     usecols=["seq", "tick", "rtc"],
                     on_bad_lines="skip", engine="python").dropna()
    i = np.arange(len(df), dtype=np.float64)
    rtc = df.rtc.values.astype(np.float64)
    r = rtc - 0.1 * i                     # 相位误差（秒）
    return df, i, rtc, r


def ppm_of(slope):
    """slope = d(r)/d(i)，r 单位 s，i 单位样本。"""
    return (slope / 0.1) * 1e6


def seg_ppm(i, r, m):
    a = np.polyfit(i[m], r[m], 1)[0]
    return ppm_of(a)


def main():
    for nm in ("26102701.CSV", "26102901.CSV"):
        df, i, rtc, r = load(nm)
        day0 = rtc[0]
        m = (rtc >= day0 + 17 * 3600) & (rtc < day0 + 23 * 3600)
        print("%s: 全天斜率 %+.0f ppm（17:00~23:00 段）" % (nm, seg_ppm(i, r, m)))

    print("")
    df, i, rtc, r = load("26102801.CSV")
    day0 = rtc[0]
    print("26102801.CSV: %d 行" % len(df))

    m_pre = (rtc >= day0 + 17 * 3600) & (rtc < day0 + 19 * 3600)
    m_post = rtc >= day0 + 22 * 3600
    pre = seg_ppm(i, r, m_pre)
    post = seg_ppm(i, r, m_post)
    print("  pre  17:00-19:00 = %+.0f ppm" % pre)
    print("  post 22:00-24:00 = %+.0f ppm" % post)

    # 60 秒滑动平均（600 样本）消锯齿
    w = 600
    ker = np.ones(w) / w
    rs = np.convolve(r, ker, mode="same")
    edge = w
    rs[:edge] = np.nan
    rs[-edge:] = np.nan

    # 两段拟合：在 18:30~21:30 内扫拐点
    lo = np.searchsorted(rtc, day0 + 18 * 3600 + 1800)
    hi = np.searchsorted(rtc, day0 + 21 * 3600 + 1800)
    ii = i[lo:hi]
    rr = rs[lo:hi]
    ok = ~np.isnan(rr)
    ii, rr = ii[ok], rr[ok]
    n = len(ii)
    c = np.concatenate(([0.0], np.cumsum(np.ones(n))))
    sx = np.concatenate(([0.0], np.cumsum(ii)))
    sy = np.concatenate(([0.0], np.cumsum(rr)))
    sxx = np.concatenate(([0.0], np.cumsum(ii * ii)))
    sxy = np.concatenate(([0.0], np.cumsum(ii * rr)))
    syy = np.concatenate(([0.0], np.cumsum(rr * rr)))

    def rss(a, b):
        nn = c[b] - c[a]
        if nn < 200:
            return np.inf
        Sx, Sy = sx[b] - sx[a], sy[b] - sy[a]
        Sxx, Sxy, Syy = sxx[b] - sxx[a], sxy[b] - sxy[a], syy[b] - syy[a]
        var = Sxx - Sx * Sx / nn
        if var <= 0:
            return np.inf
        cov = Sxy - Sx * Sy / nn
        return (Syy - Sy * Sy / nn) - cov * cov / var

    best = (np.inf, None)
    for k in range(3000, n - 3000, 50):
        v = rss(0, k) + rss(k, n)
        if v < best[0]:
            best = (v, k)
    k = best[1]
    a1 = np.polyfit(ii[:k], rr[:k], 1)[0]
    a2 = np.polyfit(ii[k:], rr[k:], 1)[0]
    bp_rtc = day0 + 18 * 3600 + 1800 + (ii[k] - ii[0]) * 0.1
    print("  两段拟合拐点 = %s UTC（前 %+.0f ppm / 后 %+.0f ppm）"
          % (hhmm(bp_rtc), ppm_of(a1), ppm_of(a2)))

    # 过渡宽度：用 5 分钟（3000 样本）滑窗的局部斜率
    print("\n  局部斜率（10 分钟窗、1 分钟步长）：")
    for s in range(np.searchsorted(rtc, day0 + 19 * 3600),
                   np.searchsorted(rtc, day0 + 20 * 3600 + 1800), 600):
        m = slice(s, s + 6000)
        a = np.polyfit(i[m], r[m], 1)[0]
        print("    %s  %+8.0f ppm" % (hhmm(rtc[s + 3000]), ppm_of(a)))


if __name__ == "__main__":
    main()
