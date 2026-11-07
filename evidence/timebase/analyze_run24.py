# analyze_run24.py -- 24 小时绝对速率：把 tick/rtc 比率拆成 tick 和 RTC 两个绝对速率
#
# 做了什么：读 24 小时那段 CSV，先验证"没有重启"（seq 单调、tick 单调、Δrtc 只取 0/1），
#           再用首尾两点 + 线性回归两种办法算三个量：
#             ratio_ppm = Δtick/(Δrtc×1000) − 1   （板内比率，最准）
#             ppm_tick  = Δtick/(Δpc×1000)  − 1   （要靠 PC 时钟）
#             ppm_rtc   = Δrtc /Δpc         − 1   （要靠 PC 时钟）
# 为什么：比率只能说明"两者相对变没变"；要判断是 tick 变了还是 RTC 变了，
#           必须有一个第三方时钟（PC 的 NTP）。PC 时钟的误差直接进 ppm_tick/ppm_rtc，
#           所以脚本会把 PC 的同步状态一起打进结论。
# 影响什么：只读。
# 怎么验证：三个量必须满足 (1+ppm_tick) = (1+ppm_rtc)(1+ratio_ppm)，误差 <2 ppm。
#
# 用法：
#   python analyze_run24.py <csv> <pc_epoch_start> <pc_epoch_end>
#   其中 csv 的第一行/最后一行就是起止；pc_epoch 从 run24_record.csv 里抄。
import sys

import numpy as np
import pandas as pd

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

CSV = sys.argv[1] if len(sys.argv) > 1 else "run24.CSV"
PC0 = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
PC1 = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0


def main():
    df = pd.read_csv(CSV, usecols=["seq", "tick", "rtc"],
                     on_bad_lines="skip", engine="python").dropna()
    n = len(df)
    print("行数 %d，rtc %d..%d，tick %d..%d" % (n, df.rtc.min(), df.rtc.max(),
                                                df.tick.min(), df.tick.max()))

    # ---- 没有重启吗 ----
    d_seq = np.diff(df.seq.values)
    d_tick = np.diff(df.tick.values)
    d_rtc = np.diff(df.rtc.values)
    reb = int((d_seq < 0).sum() + (d_tick < 0).sum())
    print("[验证] seq 回落 %d 次 | tick 回落 %d 次 | Δrtc 不等于 0/1 的行 %d 次"
          % (int((d_seq < 0).sum()), int((d_tick < 0).sum()),
             int(((d_rtc != 0) & (d_rtc != 1)).sum())))
    if reb:
        print("!! 出现重启迹象，这一段不能用于 24h 速率，先分段")
    print("[验证] Δtick 中位 %.0f ms，非 100 的行 %d（应只有 ±1 的抖动）"
          % (np.median(d_tick), int((d_tick != 100).sum())))

    i = np.arange(n, dtype=np.float64)
    tick = df.tick.values.astype(np.float64)
    rtc = df.rtc.values.astype(np.float64)

    # ---- 板内比率（不依赖 PC）----
    dtick = tick[-1] - tick[0]
    drtc = rtc[-1] - rtc[0]
    ratio = (dtick / (drtc * 1000.0) - 1.0) * 1e6
    a = np.polyfit(i, tick, 1)[0]          # ms / sample
    b = np.polyfit(i, rtc, 1)[0]           # s / sample
    ratio_reg = (a / (b * 1000.0) - 1.0) * 1e6
    print("\n板内比率 ratio_ppm = %+.1f（首尾）/ %+.1f（回归）" % (ratio, ratio_reg))

    if PC0 and PC1:
        dpc = PC1 - PC0
        ptick = (dtick / (dpc * 1000.0) - 1.0) * 1e6
        prtc = (drtc / dpc - 1.0) * 1e6
        print("PC 时间跨度 %.1f s（%.2f h）" % (dpc, dpc / 3600.0))
        print("ppm_tick = %+.1f    ppm_rtc = %+.1f" % (ptick, prtc))
        chk = ((1 + ptick * 1e-6) - (1 + prtc * 1e-6) * (1 + ratio * 1e-6)) * 1e6
        print("自洽检查：(1+tick) − (1+rtc)(1+ratio) = %.2f ppm（应≈0，偏差说明 PC 端时间戳有误）"
              % chk)

        print("\n判读（按方案 §4 表）：")
        def near(v, t, tol):
            return abs(v - t) <= tol
        if near(ptick, 0, 300) and prtc < -1000:
            print("  -> RTC 慢，tick 正常")
        elif ptick < -1000 and near(prtc, 0, 300):
            print("  -> tick 慢，RTC 正常")
        elif near(ptick, 0, 300) and near(prtc, 0, 300):
            print("  -> 两者都接近 0（post 状态）")
        elif ptick < -1000 and prtc < -1000:
            print("  -> 两者都慢，单靠两路分不开")
        else:
            print("  -> 新状态，照实记录")

        print("\n注意：PC 端未重新对时（无管理员权限），上次同步 17:44、根散度 4.28 s，"
              "\n      这带来约 ±%.0f ppm 的绝对误差；比率 ratio_ppm 不受影响。" % (4.28 / dpc * 1e6 * 86400 if dpc else 0))


if __name__ == "__main__":
    main()
