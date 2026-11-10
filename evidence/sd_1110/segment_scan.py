# -*- coding: utf-8 -*-
"""
分段扫描：把回写拼接切掉，只对"干净段"算 tick/rtc 速率。
判据（写死）：段内必须 seq 步长 = 1、tick 递增、rtc 不倒退；否则断开。
输出：每文件的分段表 + 只统计 >= MIN_SEG 秒的干净段得到的加权 ppm。
"""
import sys, os, json

MIN_SEG = 600.0     # 只采信 10 分钟以上的干净段


def scan(path):
    segs = []
    cuts = 0
    rows = 0
    prev = None
    cur = None

    def close(c):
        if c and c["n"] > 1 and c["rtc"] > 0:
            c["ppm"] = round((c["tick"] / 1000.0 / c["rtc"] - 1.0) * 1e6, 1)
            segs.append(c)

    with open(path, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            if not line or line[0] in '#\r\n':
                continue
            p = line.split(',')
            if len(p) < 4:
                continue
            try:
                seq = int(p[0]); tick = int(p[1]); rtc = int(p[3])
            except ValueError:
                continue
            rows += 1
            if prev is None:
                cur = {"n": 1, "tick": 0, "rtc": 0,
                       "rtc0": rtc, "rtc1": rtc, "seq0": seq, "seq1": seq}
            else:
                ds = seq - prev[0]; dt = tick - prev[1]; dr = rtc - prev[2]
                if ds == 1 and dt > 0 and dr >= 0:
                    cur["n"] += 1
                    cur["tick"] += dt
                    cur["rtc"] += dr
                    cur["rtc1"] = rtc
                    cur["seq1"] = seq
                else:
                    cuts += 1
                    close(cur)
                    cur = {"n": 1, "tick": 0, "rtc": 0,
                           "rtc0": rtc, "rtc1": rtc, "seq0": seq, "seq1": seq,
                           "cut_ds": ds, "cut_dr": dr}
            prev = (seq, tick, rtc)
    close(cur)

    good = [s for s in segs if s["rtc"] >= MIN_SEG]
    tot_tick = sum(s["tick"] for s in good)
    tot_rtc = sum(s["rtc"] for s in good)
    out = {
        "file": os.path.basename(path),
        "rows": rows,
        "segments": len(segs),
        "cuts": cuts,
        "good_segments": len(good),
        "good_rtc_s": tot_rtc,
        "good_tick_s": round(tot_tick / 1000.0, 1),
        "good_cover_pct": round(100.0 * tot_rtc / sum(max(s["rtc"], 1) for s in segs), 1) if segs else 0,
    }
    if tot_rtc > 0:
        out["ppm_tick_vs_rtc"] = round((tot_tick / 1000.0 / tot_rtc - 1.0) * 1e6, 1)
    # 最长的 8 段
    top = sorted(good, key=lambda s: -s["rtc"])[:8]
    out["longest"] = [
        {"rtc0": s["rtc0"], "rtc1": s["rtc1"], "span_s": s["rtc"],
         "rows": s["n"], "ppm": s["ppm"], "seq": "%d-%d" % (s["seq0"], s["seq1"])}
        for s in top
    ]
    # 回写块：以"seq 回跳"开头的段
    back = [s for s in segs if s.get("cut_ds", 0) < 0]
    out["backward_segments"] = len(back)
    out["backward_rows"] = sum(s["n"] for s in back)
    return out


if __name__ == "__main__":
    res = []
    for p in sys.argv[1:]:
        sys.stderr.write("scanning %s\n" % p); sys.stderr.flush()
        try:
            res.append(scan(p))
        except Exception as e:
            res.append({"file": os.path.basename(p), "error": str(e)})
    print(json.dumps(res, ensure_ascii=False, indent=2))
