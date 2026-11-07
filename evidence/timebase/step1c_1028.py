# step1c_1028.py -- 用"样本序号"当横轴，彻底避开按秒分桶的量化伪影
#
# 做了什么：不按 rtc 分桶，而是把每一行当成"第 i 个样本"（10 Hz ≈ 每样本 0.1 s），
#           直接拟合 rtc = a·i + b。RTC 正常时 a = 0.1，偏离多少就是多少 ppm。
# 为什么：按秒取 tick 均值会引入系统性偏差 —— 一秒里 10 条还是 11 条，
#           均值会跳 100 ms，而相邻桶相关，回归斜率就被带出 ±250 ppm 的假起伏。
#           用样本序号当横轴，横轴是干净的等距序列，读出的就是纯 RTC 速率。
# 影响什么：只读 CSV。
# 怎么验证：台阶前应得约 -1800 ppm、台阶后应得约 0 ppm，且台阶后不再有周期性起伏。
#
# 用法：python step1c_1028.py
import sys

import numpy as np
import pandas as pd

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

STAGE = r"..\longrun\_stage"


def hhmm(t):
    return pd.to_datetime(int(t), unit="s", utc=True).strftime("%H:%M:%S")


def analyse(name, show_window=None):
    df = pd.read_csv("%s/%s" % (STAGE, name),
                     usecols=["seq", "tick", "rtc"],
                     on_bad_lines="skip", engine="python").dropna()
    n = len(df)
    i = np.arange(n, dtype=np.float64)
    rtc = df.rtc.values.astype(np.float64)
    tick = df.tick.values.astype(np.float64)
    # 样本间距（tick 域）应恒为 100 ms
    dt = np.diff(tick)
    print("%s: %d 行 | Δtick 中位 %.0f ms | 起始 rtc %d 结束 %d"
          % (name, n, np.median(dt), rtc[0], rtc[-1]))

    def seg_ppm(m):
        a = np.polyfit(i[m], rtc[m], 1)[0]      # s / sample
        return (a / 0.1 - 1.0) * 1e6

    day0 = rtc[0]
    idx_day0 = 0
    pre = (rtc >= day0 + 17 * 3600) & (rtc < day0 + 19 * 3600)
    post = (rtc >= day0 + 22 * 3600)
    print("  pre (17:00-19:00) = %+.0f ppm    post (22:00-24:00) = %+.0f ppm"
          % (seg_ppm(pre), seg_ppm(post)))

    # 滚动：5 分钟窗（30 000 样本），30 秒步长
    win = 30000
    step = 3000
    xs, ps = [], []
    for s in range(0, n - win, step):
        m = slice(s, s + win)
        a = np.polyfit(i[m], rtc[m], 1)[0]
        xs.append(rtc[s + win // 2])
        ps.append((a / 0.1 - 1.0) * 1e6)
    xs = np.array(xs); ps = np.array(ps)
    print("  滚动 ppm 范围 %.0f ~ %.0f（假起伏应该消失）" % (ps.min(), ps.max()))

    # 中点穿越
    mid = (pre_ppm + post_ppm) / 2.0 if False else None
    m_pre = np.median(ps[(xs >= day0 + 17 * 3600) & (xs < day0 + 19 * 3600)])
    m_post = np.median(ps[xs >= day0 + 22 * 3600])
    mid = (m_pre + m_post) / 2.0
    zone = np.where((xs >= day0 + 18 * 3600) & (xs <= day0 + 22 * 3600))[0]
    cross = None
    for k in zone[1:]:
        if (ps[k] - mid) * (ps[k - 1] - mid) <= 0:
            cross = xs[k]; break
    print("  中点 %.0f ppm 穿越时刻 = %s UTC" % (mid, hhmm(cross) if cross else "未找到"))

    if cross is not None:
        lo_t = m_pre + 0.1 * (m_post - m_pre)
        hi_t = m_pre + 0.9 * (m_post - m_pre)
        band = [k for k in zone if min(lo_t, hi_t) <= ps[k] <= max(lo_t, hi_t)]
        if band:
            print("  过渡 10%%~90%% = %s ~ %s，宽度 %.1f min"
                  % (hhmm(xs[band[0]]), hhmm(xs[band[-1]]),
                     (xs[band[-1]] - xs[band[0]]) / 60.0))

    if show_window:
        a, b = show_window
        print("  逐点（30 秒步长）：")
        for t, p in zip(xs, ps):
            tt = (t - day0) / 3600.0
            if a <= tt <= b:
                print("    %s  %+8.0f ppm" % (hhmm(t), p))
    return xs, ps


if __name__ == "__main__":
    for nm in ("26102701.CSV", "26102801.CSV", "26102901.CSV"):
        analyse(nm, show_window=(19.5, 21.0) if nm.startswith("261028") else None)
        print("")
