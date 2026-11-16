#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
超声测距的温度补偿 —— 理论、实现、以及对真实数据的应用

做什么
    1. 把固件按 21°C 标定的 sonar_mm 换算成温度补偿后的距离
    2. 在真实 CSV 上统计"补偿量有多大、分布如何"
    3. 外推到储罐液位的典型距离（500/1000/2000/3000mm），给出补偿收益

为什么
    固件 Hardware/HC_SR04.c 写死：mm = s_us * 172 / 1000
    ⇒ 换算常数 0.172 mm/us
    ⇒ 反推声速 c0 = 0.172 × 2000 = 344 m/s
    ⇒ 等价于 (344 - 331.3)/0.606 ≈ 21.0 °C 标定

    而声速随温度变：c(T) = 331.3 + 0.606·T

    固件的读数 = 真实距离 × (c0 / c(T))
    所以温度每偏离 21°C 一度，读数系统性偏约 0.176%

    补偿公式：真实距离 = 读数 × c(T) / c0

怎么验证
    补偿后，同一个固定目标在温度变化下的读数应该变平（斜率趋于 0）。
    ⚠️ 现有数据只有 ~100mm 距离，那里补偿量 <1mm，低于超声波 ±2mm 噪声
       ⇒ 测不出效果。需要 ≥1000mm 且 ≥10°C 跨度的数据才能验证。

用法
    python sonar_tcomp.py --file <CSV> [--rows N]
    python sonar_tcomp.py --demo
"""

import argparse
import glob
import os
import sys

import numpy as np

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

COLS = ["seq", "tick", "press", "rtc", "ax", "ay", "az", "gx", "gy", "gz",
        "temp_ds", "temp_chip", "humi", "light", "vbat", "sonar_mm", "crc"]

# ★ 固件里的常数：0.172 mm/us ⇒ 声速 344 m/s（等价 21.0°C 标定）
C_FIRMWARE = 0.172 * 2000.0          # 344.0 m/s


def c_of_t(temp_c):
    """声速（m/s），20°C 附近常用近似"""
    return 331.3 + 0.606 * np.asarray(temp_c, dtype=np.float64)


def compensate(mm_raw, temp_c):
    """把按 344m/s 标定的读数，换算成当前温度下的真实距离。

    固件：读数 = 真实 × (344 / c(T))
    所以： 真实 = 读数 × c(T) / 344
    """
    return np.asarray(mm_raw, dtype=np.float64) * c_of_t(temp_c) / C_FIRMWARE


def load_csv(path, max_rows=500000):
    rows = []
    with open(path, encoding="latin-1") as f:
        for line in f:
            s = line.rstrip()
            if not s or s[0] == "#" or s.startswith("seq,"):
                continue
            p = s.split(",")
            if len(p) != 17:
                continue
            rows.append(p)
            if len(rows) >= max_rows:
                break
    if not rows:
        return None
    A = np.array(rows, dtype=np.float64)
    return {c: A[:, i] for i, c in enumerate(COLS)}


def report_table():
    """理论表：不同液位、不同温度下的补偿量"""
    print("=" * 72)
    print("  一、补偿量速查表（读数 → 补偿后，单位 mm）")
    print("=" * 72)
    print()
    temps = [0, 5, 10, 15, 21, 25, 30, 35, 40]
    print("  读数\\温度  " + "".join("%8d" % t for t in temps))
    print("  " + "-" * 68)
    for raw in [100, 500, 1000, 2000, 4000]:
        line = "  %6d mm " % raw
        for t in temps:
            corr = compensate(raw, t) - raw
            line += "%+8.1f" % corr
        print(line)
    print()
    print("  正数 = 补偿后读数变大（原读数偏小），负数 = 补偿后变小")
    print()


def report_real(path, d):
    """在真实数据上统计补偿量"""
    print("=" * 72)
    print("  二、应用到真实数据：%s" % os.path.basename(path))
    print("=" * 72)
    print()

    temp = np.where(d["temp_ds"] == -32768, np.nan, d["temp_ds"] / 100.0)
    sonar = d["sonar_mm"].astype(np.float64)

    ok = (~np.isnan(temp)) & (sonar > 0) & (sonar < 5000)
    n_ok = int(ok.sum())
    print("  总行数 %d，sonar 有效 %d（%.1f%%）" % (len(sonar), n_ok, 100.0 * n_ok / max(len(sonar), 1)))
    if n_ok < 100:
        print("  !! 有效样本太少，跳过")
        print()
        return

    t = temp[ok]
    s = sonar[ok]
    s_comp = compensate(s, t)
    corr = s_comp - s

    print("  温度范围: %.2f ~ %.2f °C （跨度 %.2f）" % (t.min(), t.max(), t.max() - t.min()))
    print("  读数范围: %.0f ~ %.0f mm （中位 %.0f）" % (s.min(), s.max(), np.median(s)))
    print()
    print("  补偿量统计：")
    print("    中位数 %+7.3f mm" % np.median(corr))
    print("    最小   %+7.3f mm" % corr.min())
    print("    最大   %+7.3f mm" % corr.max())
    print("    范围   %7.3f mm" % (corr.max() - corr.min()))
    print()
    print("  读数标准差  补偿前 %.3f mm  补偿后 %.3f mm" % (s.std(), s_comp.std()))
    print()

    # 判断这段数据能不能看出补偿效果
    corr_span = corr.max() - corr.min()
    if corr_span < 2.0:
        print("  判读：补偿量跨度 %.2f mm ＜ 超声波噪声 ±2mm" % corr_span)
        print("        ⇒ 这份数据【看不出】补偿效果，符合预期（距离太近）")
    else:
        print("  判读：补偿量跨度 %.2f mm ＞ 噪声，理论上可观测" % corr_span)
    print()

    # 同一目标、不同温度下，补偿前后的散布
    print("  按温度分箱（只列样本≥50 的箱）：")
    print("    温度(°C)  样本   补偿前中位   补偿后中位   补偿量")
    print("    " + "-" * 58)
    tb = np.round(t)
    for u in sorted(set(tb)):
        m = (tb == u)
        if m.sum() < 50:
            continue
        print("    %6.1f  %6d  %10.1f  %11.1f  %+8.2f"
              % (u, m.sum(), np.median(s[m]),
                 np.median(compensate(s[m], t[m])), np.median(corr[m])))
    print()


def report_scenario():
    """外推：储罐典型距离上，补偿能带来什么"""
    print("=" * 72)
    print("  三、外推：储罐液位场景下的补偿收益")
    print("=" * 72)
    print()
    print("  假设房间温度从 15°C 变化到 30°C（跨度 15°C，很常见）")
    print()
    print("  真实液位   补偿前读数(15°C)  补偿前(30°C)  不补偿的误差跨度")
    print("  " + "-" * 66)
    for D in [200, 500, 1000, 2000, 3000]:
        r15 = D * C_FIRMWARE / c_of_t(15)
        r30 = D * C_FIRMWARE / c_of_t(30)
        print("  %6d mm  %14.1f  %12.1f  %14.1f"
              % (D, r15, r30, abs(r30 - r15)))
    print()
    print("  补偿后：两种情况都被还原成真实液位，误差跨度 = 0（理论上）")
    print("  实际会剩下传感器的自身误差（标称 ±3mm）和噪声（实测 ±2mm）")
    print()
    # ★ 从上面那张表现算，不写死数字
    print("  储罐液位典型精度要求 ±10mm ⇒")
    for D in [200, 500, 1000, 2000, 3000]:
        span = abs(D * C_FIRMWARE / c_of_t(30) - D * C_FIRMWARE / c_of_t(15))
        if span <= 10.0:
            verdict = "不补偿也够"
        elif span <= 20.0:
            verdict = "勉强，接近超标"
        else:
            verdict = "必须补偿"
        print("    · %4d mm 液位：误差跨度 %5.1f mm  → %s" % (D, span, verdict))
    print()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--file", default=None, help="CSV 文件或目录（目录取最大的那个）")
    ap.add_argument("--rows", type=int, default=500000)
    ap.add_argument("--demo", action="store_true", help="只跑理论表和场景外推")
    a = ap.parse_args()

    report_table()

    if a.file:
        path = a.file
        if os.path.isdir(path):
            cands = sorted(glob.glob(os.path.join(path, "*.CSV")), key=os.path.getsize)
            if not cands:
                print("目录里没有 CSV")
                return 1
            path = cands[-1]
        d = load_csv(path, a.rows)
        if d is None:
            print("读不到数据")
            return 1
        report_real(path, d)

    report_scenario()

    print("=" * 72)
    print("  怎么验证补偿真的有效（等有了 1000mm 数据再跑这一步）")
    print("=" * 72)
    print()
    print("  标准：同一个固定目标，温度变化时")
    print("    · 补偿前：读数随温度漂移，斜率 ≈ -1.76 mm/°C @ 1000mm")
    print("    · 补偿后：斜率应降到 0 附近（剩下的就是噪声）")
    print()
    print("  验收数字：1000mm 处、10°C 跨度")
    print("    补偿前漂移 ≈ 17.6 mm")
    print("    补偿后漂移应 ≤ 5 mm（噪声量级）")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
