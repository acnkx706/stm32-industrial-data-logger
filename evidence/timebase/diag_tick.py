# -*- coding: utf-8 -*-
"""
把"tick 不准"拆成两件事：
  A. 时钟真的偏了 —— tick 走 1 秒，真实时间过了 1/0.99818 秒
  B. 循环跑偏了   —— tick 是准的，但每 100 tick 多花了 0.18 tick 去干活

判据：
  - tick_span / rtc_span  → 时钟误差（和采样循环无关，tick 就是个 1 kHz 计数器）
  - 行数 / rtc_span       → 采样率误差（受循环影响）
  - 逐行 Δtick 直方图     → 全是 100 ⇒ 循环没漂；出现 101/102 ⇒ 循环在漂
若 A 成立，两个 ppm 相等；若 B 成立，只有"行数"那个 ppm 偏，tick/rtc 是 0。
"""
import sys, json, collections

def scan(path):
    n = 0
    prev_tick = None
    prev_rtc = None
    tick_span = 0
    rtc_span = 0
    hist = collections.Counter()
    bad = 0
    first_rtc = None
    last_rtc = None

    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            if not line or line[0] == '#' or line[0] == '\r' or line[0] == '\n':
                continue
            p = line.split(',')
            if len(p) < 4:
                bad += 1
                continue
            try:
                tick = int(p[1]); rtc = int(p[3])
            except ValueError:
                bad += 1
                continue
            if first_rtc is None:
                first_rtc = rtc
            last_rtc = rtc
            n += 1
            if prev_tick is not None:
                d = tick - prev_tick
                dr = rtc - prev_rtc
                if d > 0 and dr >= 0:
                    hist[d] += 1
                    tick_span += d
                    rtc_span += dr
            prev_tick = tick
            prev_rtc = rtc

    out = {
        "rows": n,
        "bad": bad,
        "rtc_span_s": rtc_span,
        "tick_span_ms": tick_span,
    }
    if rtc_span > 0:
        out["ppm_tick_vs_rtc"] = round(((tick_span / 1000.0) / rtc_span - 1.0) * 1e6, 1)
        out["rows_per_s"] = round(n / float(rtc_span), 4)
        out["ppm_rows_vs_10hz"] = round((n / float(rtc_span) / 10.0 - 1.0) * 1e6, 1)
    # 只留 90~130 那些真正表示"一步"的桶，超过 130 单独汇总
    steps = {k: v for k, v in hist.items() if 90 <= k <= 130}
    big = sum(v for k, v in hist.items() if k > 130)
    out["steps_90_130"] = dict(sorted(steps.items()))
    out["steps_gt130"] = big
    if steps:
        tot = sum(steps.values())
        out["pct_exactly_100"] = round(100.0 * steps.get(100, 0) / tot, 2)
        out["pct_101"] = round(100.0 * steps.get(101, 0) / tot, 2)
        out["pct_99"] = round(100.0 * steps.get(99, 0) / tot, 2)
    return out

if __name__ == "__main__":
    res = {}
    for p in sys.argv[1:]:
        try:
            res[p] = scan(p)
        except Exception as e:
            res[p] = {"error": str(e)}
    print(json.dumps(res, ensure_ascii=False, indent=2))
