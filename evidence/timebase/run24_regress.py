# run24_regress.py -- 用卡上 CSV 做全段回归，并看 tick 偏差是不是在漂
#
# 做了什么：
#   1) 读 26110601.CSV + 26110701.CSV，按 rtc 切出 13 小时那段（起点 seq=1747）；
#   2) 验证跨文件、跨小时 seq 连续（有没有重启或漏拍）；
#   3) 整段回归 tick vs rtc 得 ratio；再按 3 小时滑窗看 ratio 稳不稳；
#   4) 拟合相位误差 r = rtc − 0.1·i 的二次项，给出"漂移率 ppm/小时"。
# 为什么：串口两点估计只能用首尾，噪声大且看不到漂移。CSV 有 46 万行，
#         用全部样本能把 ratio 压到几 ppm，并且能回答"这个偏差是固定的还是在变"——
#         这一条决定了它像"器件规格内偏差"还是"随温度漂"。
# 影响什么：只读 CSV。
# 怎么验证：整段 ratio 应与串口两点算出的 -1217 ppm 一致（±10 ppm 内）。
#
# 用法：python run24_regress.py
import sys

import numpy as np
import pandas as pd

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

FILES = ("run24_out/26110601.CSV", "run24_out/26110701.CSV")
RTC0, RTC1 = 1793968683, 1794015678       # 起点 / 终点（串口记录）
SEQ0 = 1747


def main():
    parts = []
    for f in FILES:
        d = pd.read_csv(f, usecols=["seq", "tick", "rtc"],
                        on_bad_lines="skip", engine="python").dropna()
        d = d[(d.rtc >= RTC0) & (d.rtc <= RTC1)]
        print("%-28s 段内 %d 行  seq %d..%d" % (f, len(d), d.seq.min(), d.seq.max()))
        parts.append(d)
    df = pd.concat(parts, ignore_index=True).sort_values("seq").reset_index(drop=True)
    n = len(df)
    print("\n合并后 %d 行，seq %d..%d，rtc %d..%d"
          % (n, df.seq.min(), df.seq.max(), df.rtc.min(), df.rtc.max()))

    d_seq = np.diff(df.seq.values)
    d_tick = np.diff(df.tick.values)
    print("[验证] seq 非 +1 的行 %d 处 | tick 回落 %d 处 | 最大 seq 步长 %d"
          % (int((d_seq != 1).sum()), int((d_tick < 0).sum()), int(d_seq.max())))

    i = np.arange(n, dtype=np.float64)
    rtc = df.rtc.values.astype(np.float64)
    tick = df.tick.values.astype(np.float64)

    # 整段
    a = np.polyfit(i, rtc, 1)[0]          # s/sample
    b = np.polyfit(i, tick, 1)[0]         # ms/sample
    ratio = (b / (a * 1000.0) - 1.0) * 1e6
    print("\n整段 ratio = %+.1f ppm（%d 个样本回归）" % (ratio, n))

    # 3 小时滑窗
    print("\n3 小时窗口（起点 UTC）        ratio ppm")
    for k in range(0, n - 108000, 108000 // 2):
        m = slice(k, k + 108000)
        if k + 108000 > n:
            break
        a2 = np.polyfit(i[m], rtc[m], 1)[0]
        b2 = np.polyfit(i[m], tick[m], 1)[0]
        t0 = pd.to_datetime(int(rtc[k]), unit="s", utc=True)
        print("  %s              %+8.1f" % (t0.strftime("%m-%d %H:%M"), (b2 / (a2 * 1000) - 1) * 1e6))

    # 二次项：漂移率
    r = rtc - 0.1 * i
    c2, c1, c0 = np.polyfit(i, r, 2)
    per_hour = 36000  # 每小时 36000 个样本
    d0 = (2 * c2 * 0 + c1) / 0.1 * 1e6
    dN = (2 * c2 * n + c1) / 0.1 * 1e6
    print("\n相位误差二次拟合：起点 %+.1f ppm，终点 %+.1f ppm，13 小时变化 %+.1f ppm"
          % (d0, dN, dN - d0))
    print("折合漂移率 %+.2f ppm/小时" % ((dN - d0) / 13.05))


if __name__ == "__main__":
    main()
