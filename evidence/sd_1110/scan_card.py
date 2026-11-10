# -*- coding: utf-8 -*-
"""
扫描 SD 卡上 11-06 ~ 11-10 的 CSV：逐行 CRC + seq 连续性 + 节拍直方图 + tick/rtc 速率。
只读 D:\ 之外的 E:\，只写同目录 json。

判据（写死在代码里，改之前先想清楚）：
  - CRC 覆盖"最后一个逗号之前"的全部字节，CRC-16/KERMIT（poly 0x8408, init 0, 反射）
  - Δtick 应为 100；99/101 是 10Hz 与 1ms 栅格对齐的抖动；>=200 才是漏拍
  - ppm_tick_vs_rtc = (ΔΣtick/1000 / ΔΣrtc - 1) * 1e6
"""
import sys, os, json, collections

TBL = []
for _i in range(256):
    _c = _i
    for _ in range(8):
        _c = (_c >> 1) ^ 0x8408 if (_c & 1) else (_c >> 1)
    TBL.append(_c)


def crc16(buf):
    c = 0
    for b in buf:
        c = (c >> 8) ^ TBL[(c ^ b) & 0xFF]
    return c & 0xFFFF


def scan(path):
    n = 0
    bad_crc = 0
    bad_field = 0
    nonmono_rtc = 0
    seq_gap = 0
    seq_gap_max = 0
    seq_restart = 0
    hist = collections.Counter()
    tick_span = 0
    rtc_span = 0
    pseq = ptick = prtc = None
    first = last = None
    crc_samples = []

    with open(path, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            if not line or line[0] in '#\r\n':
                continue
            line = line.rstrip('\r\n')
            if not line:
                continue
            cut = line.rfind(',')
            if cut < 0:
                bad_field += 1
                continue
            head = line[:cut]
            parts = head.split(',')
            if len(parts) != 16:
                bad_field += 1
                continue
            try:
                seq = int(parts[0]); tick = int(parts[1]); rtc = int(parts[3])
                stored = int(line[cut + 1:])
            except ValueError:
                bad_field += 1
                continue

            n += 1
            if first is None:
                first = (seq, tick, rtc)
            last = (seq, tick, rtc)

            # CRC 全量校验
            if crc16(head.encode('ascii', 'ignore')) != stored:
                bad_crc += 1
                if len(crc_samples) < 5:
                    crc_samples.append(line[:120])

            if prtc is not None:
                d = tick - ptick
                dr = rtc - prtc
                ds = seq - pseq
                if ds != 1:
                    seq_gap += 1
                    if ds > seq_gap_max:
                        seq_gap_max = ds
                    if ds < 0:
                        seq_restart += 1
                if d > 0 and dr >= 0:
                    hist[d] += 1
                    tick_span += d
                    rtc_span += dr
                if dr < 0:
                    nonmono_rtc += 1
            pseq, ptick, prtc = seq, tick, rtc

    out = {
        "file": os.path.basename(path),
        "rows": n,
        "bad_field": bad_field,
        "bad_crc": bad_crc,
        "seq_restart": seq_restart,
        "seq_gap": seq_gap,
        "seq_gap_max": seq_gap_max,
        "rtc_backward": nonmono_rtc,
        "rtc_first": first[2] if first else None,
        "rtc_last": last[2] if last else None,
        "rtc_span_s": rtc_span,
        "tick_span_ms": tick_span,
    }
    if rtc_span > 0:
        out["ppm_tick_vs_rtc"] = round((tick_span / 1000.0 / rtc_span - 1.0) * 1e6, 1)
        out["rows_per_s"] = round(n / float(rtc_span), 4)
        out["ppm_rows_vs_10hz"] = round((n / float(rtc_span) / 10.0 - 1.0) * 1e6, 1)
    steps = {int(k): v for k, v in hist.items() if 90 <= k <= 130}
    out["dtick_90_130"] = dict(sorted(steps.items()))
    out["dtick_gt130"] = sum(v for k, v in hist.items() if k > 130)
    tot = sum(steps.values()) or 1
    out["pct_100"] = round(100.0 * steps.get(100, 0) / tot, 2)
    out["pct_99"] = round(100.0 * steps.get(99, 0) / tot, 2)
    out["pct_101"] = round(100.0 * steps.get(101, 0) / tot, 2)
    if crc_samples:
        out["crc_sample_lines"] = crc_samples
    return out


if __name__ == "__main__":
    res = []
    for p in sys.argv[1:]:
        sys.stderr.write("scanning %s ...\n" % p)
        sys.stderr.flush()
        try:
            res.append(scan(p))
        except Exception as e:
            res.append({"file": os.path.basename(p), "error": str(e)})
    print(json.dumps(res, ensure_ascii=False, indent=2))
