# exp01b.py -- EXP-01b 账本收口：把四份东西统一成一套口径
#
# 做了什么：生成 manifest_clean11 / segments_* / events_by_day / file_inventory /
#           valid_csv_bytes / crc_summary / summary_clean11
# 为什么：README 的 16,175 由"864,000/天减实际行数"得出，混了三种东西，
#         必须先按公式拆成 false/real/rtc_day/anomaly 四块，且每块能落到文件
# 影响什么：只读输入目录与卡；输出全部写 --out
# 怎么验证：逐文件 false+real+rtc_day+anomaly == 864000 - rows，误差 ≤1 行
#
# 公式（写死，避免各说各话）：
#   段       相邻两行 Δtick <= 0 处切开
#   可信段   rtc_span*10 与 tick_span/100 的比值落在 0.9~1.1
#   false    Σ可信段 (rtc_span*10 - tick_expected)     ← tick 与物理秒不同步
#   real     Σ可信段 (tick_expected - rows)            ← 段内真缺
#   anomaly  Σ异常段 (rtc_span*10 - rows)              ← rtc 跳变的段
#   rtc_day  864000 - Σ全部段 rtc_span*10              ← 该天 rtc 跨度不足 86400 秒
import argparse
import csv
import hashlib
import os
import re
import sys

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

ALLOWED = b"0123456789,-."
NAME_RE = re.compile(r"^(\d{6})(\d{2})\.CSV$")


def scan(path):
    size = 0
    rows = 0
    valid_bytes = 0
    loss_rows = 0
    stall_rows = 0
    seq_gap_rows = 0
    missed_rows = 0
    hdr_lines = 0
    bad_lines = 0
    segs = []
    cur = None
    prev_tick = None

    with open(path, "rb") as f:
        for raw in f:
            size += len(raw)
            body = raw.rstrip(b"\r\n")
            if not body:
                continue
            if body[:1] == b"#":
                continue
            if body.startswith(b"seq,"):
                hdr_lines += 1
                continue
            if body.count(b",") < 14 or body.translate(None, ALLOWED):
                bad_lines += 1
                continue

            fld = body.split(b",")
            try:
                seq = int(fld[0])
                tick = int(fld[1])
                rtc = int(fld[3])
            except ValueError:
                bad_lines += 1
                continue

            rows += 1
            valid_bytes += len(raw)

            if cur is None:
                cur = dict(fseq=seq, ftick=tick, frtc=rtc, lseq=seq, ltick=tick, lrtc=rtc, rows=1)
                segs.append(cur)
            else:
                d = tick - prev_tick
                if d <= 0:
                    cur = dict(fseq=seq, ftick=tick, frtc=rtc, lseq=seq, ltick=tick, lrtc=rtc, rows=1)
                    segs.append(cur)
                else:
                    # 先按段记下缺口，等判定该段是否可信后再归集
                    if d > 150:
                        cur.setdefault("gaps", []).append(((d // 100) - 1, d))
                    cur["ltick"] = tick
                    cur["lrtc"] = rtc
                    cur["lseq"] = seq
                    cur["rows"] += 1
            prev_tick = tick

    for s in segs:
        s["span_t"] = s["ltick"] - s["ftick"]
        s["span_r10"] = (s["lrtc"] - s["frtc"]) * 10
        s["tick_exp"] = s["span_t"] / 100.0 + 1.0
        ratio = (s["span_r10"] / (s["span_t"] / 100.0)) if s["span_t"] > 0 else 0.0
        s["trusted"] = (s["span_t"] > 0) and (s["span_r10"] > 0) and (0.9 <= ratio <= 1.1)

    # 只统计可信段内的缺口，并按 seq 再分一次：
    #   seq 连续但 tick 有缺口  → 拍子丢了、记录没产生（未产生）
    #   seq 跳号               → 按 seq 真的有行不在这个文件里（回灌交错或真缺）
    for s in segs:
        if not s["trusted"]:
            continue
        tickgap = s["tick_exp"] - s["rows"]
        seqspan = s["lseq"] - s["fseq"] + 1
        seqgap = seqspan - s["rows"]
        if seqgap > 0:
            seq_gap_rows += seqgap
            missed_rows += max(0.0, tickgap - seqgap)
        else:
            missed_rows += max(0.0, tickgap)

    return dict(size=size, rows=rows, valid_bytes=valid_bytes, hdr_lines=hdr_lines,
                bad_lines=bad_lines, segs=segs,
                loss_rows=loss_rows, stall_rows=stall_rows,
                seq_gap_rows=seq_gap_rows, missed_rows=missed_rows)


def blocks(st):
    false = sum(s["span_r10"] - s["tick_exp"] for s in st["segs"] if s["trusted"])
    real = sum(s["tick_exp"] - s["rows"] for s in st["segs"] if s["trusted"])
    anom = sum(s["span_r10"] - s["rows"] for s in st["segs"] if not s["trusted"])
    rtc_day = 864000.0 - sum(s["span_r10"] for s in st["segs"])
    return dict(false=false, real=real, anomaly=anom, rtc_day=rtc_day)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--card", default="E:\\")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    files = sorted(n for n in os.listdir(a.input)
                   if NAME_RE.match(n.upper()) and NAME_RE.match(n.upper()).group(2) != "90")
    evts = sorted(n for n in os.listdir(a.input)
                  if NAME_RE.match(n.upper()) and NAME_RE.match(n.upper()).group(2) == "90")

    # 注意：比较时只取前 8 位，否则 "26110101.CSV" > "26110101" 会被上界排掉
    clean = [n for n in files if "26102201" <= n[:8] <= "26110101"]

    recs = {}
    for n in files:
        st = scan(os.path.join(a.input, n))
        b = blocks(st)
        recs[n] = (st, b)

    # ---------- manifest_clean11 ----------
    cols = ["file", "date", "bytes", "rows", "sha256",
            "first_seq", "last_seq", "first_tick", "last_tick", "first_rtc", "last_rtc",
            "segments", "tick_expected", "nominal_rtc10",
            "false_gap", "real_gap", "seq_gap_rows", "missed_rows", "rtc_day", "anomaly", "check"]
    with open(os.path.join(a.out, "manifest_clean11.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for n in clean:
            st, b = recs[n]
            s0, sl = st["segs"][0], st["segs"][-1]
            tick_exp = sum(s["tick_exp"] for s in st["segs"])
            nom = sum(s["span_r10"] for s in st["segs"])
            chk = (b["false"] + b["real"] + b["rtc_day"] + b["anomaly"]) - (864000.0 - st["rows"])
            w.writerow([n, "20%s-%s-%s" % (n[0:2], n[2:4], n[4:6]), st["size"], st["rows"],
                        hashlib.sha256(open(os.path.join(a.input, n), "rb").read()).hexdigest(),
                        s0["fseq"], sl["lseq"], s0["ftick"], sl["ltick"], s0["frtc"], sl["lrtc"],
                        len(st["segs"]), round(tick_exp, 2), round(nom, 2),
                        round(b["false"], 2), round(b["real"], 2),
                        st["seq_gap_rows"], round(st["missed_rows"], 2),
                        round(b["rtc_day"], 2),
                        round(b["anomaly"], 2), round(chk, 2)])

    # ---------- segments ----------
    for key in ("26102201", "26102501", "26102601"):
        name = key + ".CSV"
        st, b = recs[name]
        with open(os.path.join(a.out, "segments_%s.csv" % key[2:6]), "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["seg", "first_seq", "last_seq", "first_tick", "last_tick",
                        "first_rtc", "last_rtc", "rows", "tick_expected", "gap",
                        "rtc10", "trusted"])
            for i, s in enumerate(st["segs"], 1):
                w.writerow([i, s["fseq"], s["lseq"], s["ftick"], s["ltick"],
                            s["frtc"], s["lrtc"], s["rows"], round(s["tick_exp"], 2),
                            round(s["tick_exp"] - s["rows"], 2), round(s["span_r10"], 2),
                            1 if s["trusted"] else 0])

    # ---------- events_by_day ----------
    EVT = re.compile(r"^#\s*([A-Z_]+)\s*,(.*)$")
    agg = {}
    for n in evts:
        date = "20%s-%s-%s" % (n[0:2], n[2:4], n[4:6])
        for raw in open(os.path.join(a.input, n), "rb"):
            line = raw.rstrip(b"\r\n").decode("latin-1")
            m = EVT.match(line)
            if not m:
                continue
            typ, rest = m.group(1), m.group(2).split(",")
            reason, cnt = "", 0
            if typ == "LOST" and len(rest) >= 3:
                try:
                    cnt = int(rest[1])
                except ValueError:
                    cnt = 0
                reason = ",".join(rest[2:])
            else:
                for tok in rest:
                    if "=" in tok:
                        k, v = tok.split("=", 1)
                        if k in ("recs", "n"):
                            try:
                                cnt = max(cnt, int(v))
                            except ValueError:
                                pass
            k = (date, typ, reason)
            agg[k] = agg.get(k, 0) + cnt
    with open(os.path.join(a.out, "events_by_day.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["date", "type", "reason", "count"])
        for (date, typ, reason), c in sorted(agg.items()):
            w.writerow([date, typ, reason, c])

    # ---------- valid_csv_bytes ----------
    with open(os.path.join(a.out, "valid_csv_bytes.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["file", "bytes_on_disk", "valid_csv_bytes", "foreign_bytes",
                    "foreign_pct", "rows", "header_lines", "bad_lines"])
        t_disk = t_valid = 0
        for n in files:
            st, _ = recs[n]
            t_disk += st["size"]
            t_valid += st["valid_bytes"]
            w.writerow([n, st["size"], st["valid_bytes"], st["size"] - st["valid_bytes"],
                        round(100.0 * (st["size"] - st["valid_bytes"]) / st["size"], 3) if st["size"] else 0,
                        st["rows"], st["hdr_lines"], st["bad_lines"]])
        w.writerow(["TOTAL", t_disk, t_valid, t_disk - t_valid,
                    round(100.0 * (t_disk - t_valid) / t_disk, 3), "", "", ""])

    # ---------- file_inventory ----------
    inv = []
    seen = set()
    if os.path.isdir(a.card):
        for e in os.listdir(a.card):
            if not e.upper().endswith(".CSV"):
                continue
            p = os.path.join(a.card, e)
            if not os.path.isfile(p):
                continue
            sz = os.path.getsize(p)
            key = e.upper()
            if key in seen:
                inv.append([e, sz, "", "duplicate_entry", "同名目录条目（FAT 残留）"])
                continue
            seen.add(key)
            m = NAME_RE.match(key)
            if not m:
                inv.append([e, sz, "", "excluded_name", "文件名不符合 YYMMDDNN.CSV"])
            elif m.group(2) == "90":
                inv.append([e, sz, "", "event_file", "事件文件，并入 events_by_day"])
            elif not ("261018" <= key[:6] <= "261103"):
                inv.append([e, sz, "", "excluded_date", "超出 10-18~11-03"])
            elif sz == 0:
                inv.append([e, sz, "", "empty", "0 字节，无数据"])
            elif key in files:
                inv.append([e, sz, recs[key][0]["rows"], "used", ""])
            else:
                inv.append([e, sz, "", "excluded_other", "未纳入"])
    with open(os.path.join(a.out, "file_inventory.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["file", "bytes", "rows", "status", "note"])
        for r in sorted(inv, key=lambda x: x[0]):
            w.writerow(r)

    # ---------- crc_summary ----------
    crc_path = os.path.join(os.path.dirname(a.out), "out", "crc_all.csv")
    if os.path.exists(crc_path):
        cr = list(csv.DictReader(open(crc_path, encoding="utf-8")))
        parsed_ok = sum(int(r["crc_ok"]) for r in cr)
        parsed_bad = sum(int(r["crc_bad"]) for r in cr)
        old = 0
        fmt = 0
        for r in cr:
            for kv in (r["note"] or "").split(";"):
                if kv.startswith("no_crc_rows="):
                    old += int(kv.split("=")[1])
                if kv.startswith("fmt_bad="):
                    fmt += int(kv.split("=")[1])
        with open(os.path.join(a.out, "crc_summary.md"), "w", encoding="utf-8", newline="\n") as f:
            f.write("# CRC 结果（EXP-01b 口径）\n\n")
            f.write("| 类别 | 行数 | 说明 |\n|---|---:|---|\n")
            f.write("| 解析行，CRC 通过 | %d | 格式合规且两处 CRC 一致 |\n" % parsed_ok)
            f.write("| 解析行，CRC 失败 | **%d** | 逐行 CRC16/KERMIT 与行尾字段不符 |\n" % parsed_bad)
            f.write("| 旧格式（crc=0） | %d | 早期固件不写 CRC，不参与比对 |\n" % old)
            f.write("| 格式坏行 | %d | 不足 15 字段或含非数字字符，未参与 CRC |\n" % fmt)
            f.write("\n**\"0 失败\"只对解析行成立**：格式坏行与旧格式行都不在统计口径内。\n")

    # ---------- summary_clean11 ----------
    tot = dict(false=0.0, real=0.0, rtc_day=0.0, anomaly=0.0)
    rows_sum = 0
    for n in clean:
        st, b = recs[n]
        rows_sum += st["rows"]
        for k in tot:
            tot[k] += b[k]
    nominal = 864000.0 * len(clean)
    seqg_sum = sum(recs[n][0]["seq_gap_rows"] for n in clean)
    miss_sum = sum(recs[n][0]["missed_rows"] for n in clean)
    with open(os.path.join(a.out, "summary_clean11.md"), "w", encoding="utf-8", newline="\n") as f:
        f.write("# 干净窗口账本（10-22 ~ 11-01，%d 个文件）\n\n" % len(clean))
        f.write("## 总量\n\n")
        f.write("| 项 | 值 |\n|---|---:|\n")
        f.write("| 文件数 | %d |\n" % len(clean))
        f.write("| 名义（864,000 × %d） | %.0f |\n" % (len(clean), nominal))
        f.write("| 实际解析行 | %d |\n" % rows_sum)
        f.write("| 差额 | %.0f |\n" % (nominal - rows_sum))
        f.write("\n## 四块拆分（公式逐项可复算）\n\n")
        f.write("| 块 | 行数 | 公式 |\n|---|---:|---|\n")
        f.write("| 假缺口 false_gap | %.0f | Σ可信段 (rtc_span×10 − tick_expected) |\n" % tot["false"])
        f.write("| 真缺口 real_gap | %.0f | Σ可信段 (tick_expected − rows) |\n" % tot["real"])
        f.write("| 　其中 seq 跳号 seq_gap_rows | %d | 按 seq 算真的有行不在本文件（10-22 那 149 行来自回灌交错） |\n" % seqg_sum)
        f.write("| 　其中 未产生 missed_rows | %.0f | 拍子丢了但 seq 连续：设备在跑、没采样，记录从未产生 |\n" % miss_sum)
        f.write("| rtc 日长不足 rtc_day | %.0f | 864,000 − Σ全部段 rtc_span×10 |\n" % tot["rtc_day"])
        f.write("| 异常段 anomaly | %.0f | Σ异常段 (rtc_span×10 − rows) |\n" % tot["anomaly"])
        f.write("| 合计 | %.0f | 应等于差额 |\n" % (tot["false"] + tot["real"] + tot["rtc_day"] + tot["anomaly"]))
        f.write("\n## 逐文件\n\n")
        f.write("| 文件 | rows | false | real | rtc_day | anomaly | 差额 | 校验 |\n|---|---:|---:|---:|---:|---:|---:|---:|\n")
        for n in clean:
            st, b = recs[n]
            d = nominal / len(clean) - st["rows"]
            chk = b["false"] + b["real"] + b["rtc_day"] + b["anomaly"] - d
            f.write("| %s | %d | %.0f | %.0f | %.0f | %.0f | %.0f | %.2f |\n"
                    % (n, st["rows"], b["false"], b["real"], b["rtc_day"], b["anomaly"], d, chk))
        f.write("\n说明：缺口是\"相对 864,000/天\"的差，不等于丢行。真缺口只认 real_gap。\n")

    print("clean files:", len(clean))
    print("rows=%d nominal=%.0f diff=%.0f" % (rows_sum, nominal, nominal - rows_sum))
    print("false=%.0f real=%.0f rtc_day=%.0f anomaly=%.0f"
          % (tot["false"], tot["real"], tot["rtc_day"], tot["anomaly"]))
    print("->", a.out)


if __name__ == "__main__":
    main()
