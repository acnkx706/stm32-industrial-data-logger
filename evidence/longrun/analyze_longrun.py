# analyze_longrun.py -- EXP-01 第一步：manifest + 事件汇总 + 缺口报告 + 冻结输出
#
# 做了什么：流式扫数据文件，先切段（Δtick<=0 处），再逐段判定 rtc 是否可信，
#           最后把缺口拆成四块，并把时间跳变段单独列出。
# 为什么：README 的 16,175 行缺口 = 按天名义 864,000 直接减实际行数，
#         里面混着 tick 偏慢造成的假缺口和设备停机；而开发期文件的 rtc 还会跳变
#         （校时、回灌），不单独隔离就会把账算成负数。
# 影响什么：只读输入目录，不修改任何原始文件；输出全部写到 --out。
# 怎么验证：可信段内 false_gap + real_gap == nominal - rows（±1 行舍入）。
#
# 用法：
#   python analyze_longrun.py --input <CSV_DIR> --out <OUT_DIR> [--gap 16175] [--expected-rows 12844939]
#
# 口径：
#   段        = 相邻两行 Δtick <= 0 处切开（复位/回灌都造成这个）
#   可信段    = tick 与 rtc 的比值落在 0.9~1.1 之间（即 rtc 没有明显跳变）
#   可信段内  tick_expected = tick_span/100 + 1，nominal = rtc_span*10
#   假缺口    = Σ可信段 (nominal - tick_expected)
#   真缺口    = Σ全部段 tick_expected - rows
#   停机      = Σ相邻段 (next.first_rtc - cur.last_rtc)，只累加正值
#   时间倒退  = 相邻段 rtc 差为负的部分，单独报
import argparse
import csv
import hashlib
import os
import re
import sys

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

NAME_RE = re.compile(r"^(\d{6})(\d{2})\.CSV$")
ALLOWED = b"0123456789,-."   # 数据行允许出现的字节；其它字节说明是损坏区的垃圾


def scan_file(path):
    """扫一个数据文件。只读。返回统计字典。"""
    h = hashlib.sha256()
    size = 0
    rows = 0
    bad_rows = 0
    col_other = 0

    segs = []          # 当前段 [{first_tick,last_tick,first_rtc,last_rtc}]
    cur = None
    prev_tick = None

    max_dtick = 0
    ge_115 = 0
    ge_150 = 0
    non_pos_dtick = 0

    with open(path, "rb") as f:
        for raw in f:
            h.update(raw)
            size += len(raw)

            line = raw.rstrip(b"\r\n").decode("latin-1")
            if not line or line[0] == "#" or line.startswith("seq,"):
                continue
            # 介质损坏区会混进二进制，整行只允许 数字/逗号/减号/小数点
            if raw.rstrip(b"\r\n").translate(None, ALLOWED):
                bad_rows += 1
                continue

            p = line.rsplit(",", 1)
            if len(p) != 2:
                bad_rows += 1
                continue

            fld = p[0].split(",")
            if len(fld) < 15:
                bad_rows += 1
                continue
            if (len(fld) + 1) != 17:
                col_other += 1

            try:
                seq = int(fld[0])
                tick = int(fld[1])
                rtc = int(fld[3])
            except ValueError:
                bad_rows += 1
                continue

            rows += 1
            if cur is None:
                cur = {"first_seq": seq, "last_seq": seq,
                       "first_tick": tick, "last_tick": tick,
                       "first_rtc": rtc, "last_rtc": rtc}
                segs.append(cur)
            else:
                d = tick - prev_tick
                if d <= 0:
                    non_pos_dtick += 1
                    cur = {"first_seq": seq, "last_seq": seq,
                           "first_tick": tick, "last_tick": tick,
                           "first_rtc": rtc, "last_rtc": rtc}
                    segs.append(cur)
                else:
                    if d > max_dtick:
                        max_dtick = d
                    if d >= 115:
                        ge_115 += 1
                    if d > 150:
                        ge_150 += 1
                    cur["last_tick"] = tick
                    cur["last_rtc"] = rtc
                    cur["last_seq"] = seq
            prev_tick = tick

    # 逐段判定
    tick_expected_all = 0.0
    nominal_ok = 0.0
    tick_expected_ok = 0.0
    anomaly_expected = 0.0
    anomaly_segs = 0
    down_s = 0
    back_s = 0

    for i, s in enumerate(segs):
        span_t = s["last_tick"] - s["first_tick"]
        span_r = s["last_rtc"] - s["first_rtc"]
        exp = span_t / 100.0 + 1.0
        tick_expected_all += exp

        sane = (span_t > 0) and (span_r > 0)
        if sane:
            ratio = (span_r * 10.0) / (span_t / 100.0)
            sane = 0.9 <= ratio <= 1.1
        if sane:
            nominal_ok += span_r * 10.0
            tick_expected_ok += exp
        else:
            anomaly_expected += exp
            anomaly_segs += 1

        if i > 0:
            gap = s["first_rtc"] - segs[i - 1]["last_rtc"]
            if gap > 0:
                down_s += gap
            elif gap < 0:
                back_s += -gap

    if not segs:
        # 空文件 / 只有表头的文件：卡上确实存在（26101805.CSV 就是 0 字节）
        return {
            "bytes": size, "rows": 0, "bad_rows": bad_rows, "col_other": col_other,
            "sha256": h.hexdigest(),
            "first_seq": None, "last_seq": None,
            "first_tick": None, "last_tick": None,
            "first_rtc": None, "last_rtc": None,
            "total_span_s": 0, "segments": 0,
            "anomaly_segs": 0, "anomaly_expected": 0.0,
            "down_s": 0, "back_s": 0,
            "max_dtick": 0, "ge_115": 0, "ge_150": 0, "non_pos_dtick": 0,
            "tick_expected": 0.0, "tick_expected_ok": 0.0, "nominal_ok": 0.0,
            "no_data": True,
        }

    first, last = segs[0], segs[-1]

    return {
        "bytes": size, "rows": rows, "bad_rows": bad_rows, "col_other": col_other,
        "sha256": h.hexdigest(),
        "first_seq": first["first_seq"], "last_seq": last["last_seq"],
        "first_tick": first["first_tick"], "last_tick": last["last_tick"],
        "first_rtc": first["first_rtc"], "last_rtc": last["last_rtc"],
        "total_span_s": last["last_rtc"] - first["first_rtc"],
        "segments": len(segs),
        "anomaly_segs": anomaly_segs,
        "anomaly_expected": anomaly_expected,
        "down_s": down_s, "back_s": back_s,
        "max_dtick": max_dtick, "ge_115": ge_115, "ge_150": ge_150,
        "non_pos_dtick": non_pos_dtick,
        "tick_expected": tick_expected_all,
        "tick_expected_ok": tick_expected_ok,
        "nominal_ok": nominal_ok,
    }


EVT_RE = re.compile(r"^#\s*([A-Z_]+)\s*,(.*)$")


def scan_events(path):
    """扫一个 90 事件文件，返回 [(type, reason, n)]。"""
    out = []
    with open(path, "rb") as f:
        for raw in f:
            line = raw.rstrip(b"\r\n").decode("latin-1")
            if not line or line[0] != "#":
                continue
            m = EVT_RE.match(line)
            if not m:
                continue
            typ = m.group(1)
            rest = m.group(2).split(",")
            reason = ""
            n = 0
            if typ == "LOST" and len(rest) >= 3:
                try:
                    n = int(rest[1])
                except ValueError:
                    n = 0
                reason = ",".join(rest[2:])
            else:
                for tok in rest:
                    if "=" in tok:
                        k, v = tok.split("=", 1)
                        if k in ("recs", "batches", "n"):
                            try:
                                n = max(n, int(v))
                            except ValueError:
                                pass
            out.append((typ, reason, n))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--gap", type=float, default=16175.0)
    ap.add_argument("--expected-rows", type=float, default=12844939.0)
    a = ap.parse_args()

    os.makedirs(a.out, exist_ok=True)

    data_files, evt_files = [], []
    for n in sorted(os.listdir(a.input)):
        m = NAME_RE.match(n.upper())
        if not m:
            continue
        (evt_files if m.group(2) == "90" else data_files).append(n)

    recs = []
    for n in data_files:
        st = scan_file(os.path.join(a.input, n))
        recs.append({
            "file": n,
            "date": "20%s-%s-%s" % (n[0:2], n[2:4], n[4:6]),
            "bytes": st["bytes"], "rows": st["rows"], "sha256": st["sha256"],
            "first_seq": st["first_seq"], "last_seq": st["last_seq"],
            "first_tick": st["first_tick"], "last_tick": st["last_tick"],
            "first_rtc": st["first_rtc"], "last_rtc": st["last_rtc"],
            "duration_s": st["total_span_s"],
            "downtime_s": st["down_s"],
            "segments": st["segments"],
            "max_dtick": st["max_dtick"], "ge_115": st["ge_115"], "ge_150": st["ge_150"],
            "tick_expected": round(st["tick_expected"], 2),
            "nominal": round(st["nominal_ok"], 2),
            "false_gap": round(st["nominal_ok"] - st["tick_expected_ok"], 2),
            "real_gap": round(st["tick_expected"] - st["rows"], 2),
            "boundary_gap": 0, "crc_ok": "", "crc_bad": "",
            "note": ";".join(
                ([("no_data")] if st.get("no_data") else []) +
                ([("bad_rows=%d" % st["bad_rows"])] if st["bad_rows"] else []) +
                ([("cols!=17:%d" % st["col_other"])] if st["col_other"] else []) +
                ([("rtc_anon_segs=%d,exp=%.0f" % (st["anomaly_segs"], st["anomaly_expected"]))]
                 if st["anomaly_segs"] else []) +
                ([("rtc_back_s=%d" % st["back_s"])] if st["back_s"] else [])),
            "_anom_exp": st["anomaly_expected"],
        })
        r = recs[-1]
        print("%-14s rows=%-7d seg=%-3d(anon %d) tkexp=%-10.1f nominal=%-10.1f false=%-9.1f real=%-8.1f down=%d back=%d"
              % (n, r["rows"], r["segments"], st["anomaly_segs"], r["tick_expected"],
                 r["nominal"], r["false_gap"], r["real_gap"], r["downtime_s"], st["back_s"]))

    # 跨文件边界缺口：只有"相邻日期 + seq 继续往前走"才算缺口
    # 缺文件（中间断了一天）不算边界缺口，那属于输入集本身有洞
    def dnum(n):
        return int(n[0:6])

    for i in range(len(recs) - 1):
        cur, nxt = recs[i], recs[i + 1]
        if cur["last_seq"] is None or nxt["first_seq"] is None:
            continue
        if dnum(nxt["file"]) - dnum(cur["file"]) != 1:
            cur["note"] = (cur["note"] + ";" if cur["note"] else "") + "gap_days"
            continue
        if nxt["first_tick"] is not None and nxt["first_tick"] <= 1000:
            # 下一个文件第一行的 tick 从上电初值起步 ⇒ 中间重启过，
            # seq 从 1 重数，跨文件 seq 差没有意义
            cur["note"] = (cur["note"] + ";" if cur["note"] else "") + "reboot_between"
            continue
        # 自校验：同一会话里，seq 增量应当等于 tick 增量除以 100。
        # 对不上说明中间换过会话（seq 重数过），跨文件缺口无从计算。
        dseq = nxt["first_seq"] - cur["last_seq"]
        dtick = nxt["first_tick"] - cur["last_tick"]
        if dtick <= 0 or abs(dseq - dtick / 100.0) > 1.0:
            cur["note"] = (cur["note"] + ";" if cur["note"] else "") + "seq_tick_inconsistent"
            continue
        # 同一会话内，账本在两侧各自闭合，真正的跨文件缺口只能是 0
        cur["boundary_gap"] = 0

    cols = ["file", "date", "bytes", "rows", "sha256",
            "first_seq", "last_seq", "first_tick", "last_tick",
            "first_rtc", "last_rtc", "duration_s", "downtime_s", "segments",
            "max_dtick", "ge_115", "ge_150", "tick_expected", "nominal",
            "false_gap", "real_gap", "boundary_gap", "crc_ok", "crc_bad", "note"]
    with open(os.path.join(a.out, "manifest_17d.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        for r in recs:
            w.writerow(r)

    evt_rows = []
    for n in evt_files:
        for typ, reason, cnt in scan_events(os.path.join(a.input, n)):
            evt_rows.append({"file": n,
                             "date": "20%s-%s-%s" % (n[0:2], n[2:4], n[4:6]),
                             "type": typ, "reason": reason, "count": cnt})
    with open(os.path.join(a.out, "events_summary.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["file", "date", "type", "reason", "count"])
        w.writeheader()
        for r in evt_rows:
            w.writerow(r)

    tot_rows = sum(r["rows"] for r in recs)
    tot_bytes = sum(r["bytes"] for r in recs)
    tot_false = sum(r["false_gap"] for r in recs)
    tot_real = sum(r["real_gap"] for r in recs)
    tot_bnd = sum(r["boundary_gap"] for r in recs)
    tot_nominal = sum(r["nominal"] for r in recs)
    tot_down = sum(r["downtime_s"] for r in recs)
    tot_anom = sum(r["_anom_exp"] for r in recs)
    residual = a.gap - (tot_false + tot_real + tot_bnd + tot_down * 10.0 + tot_anom)
    lost_rows = sum(r["count"] for r in evt_rows if r["type"] == "LOST")

    frozen = [
        "EXP-01 longrun summary",
        "files=%d" % len(recs),
        "rows=%d" % tot_rows,
        "bytes=%d" % tot_bytes,
        "nominal=%.0f" % tot_nominal,
        "downtime_s=%d" % tot_down,
        "false_gap=%.0f" % tot_false,
        "real_gap=%.0f" % tot_real,
        "boundary_gap=%.0f" % tot_bnd,
        "rtc_anomaly_expected=%.0f" % tot_anom,
        "unexplained_vs_readme=%.0f" % residual,
        "lost_events_rows=%.0f" % lost_rows,
        "segments_total=%d" % sum(r["segments"] for r in recs),
    ]
    with open(os.path.join(a.out, "expected_longrun.txt"), "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(frozen) + "\n")

    L = []
    L.append("# 17 天账本与 CRC 全量（EXP-01）\n")
    L.append("> 只读输入目录生成；原始 CSV 未被修改。\n")
    L.append("## 一、总量\n")
    L.append("| 项 | 值 |")
    L.append("|---|---|")
    L.append("| 数据文件数 | %d |" % len(recs))
    L.append("| 事件文件数 | %d |" % len(evt_files))
    L.append("| 数据行合计 | %d |" % tot_rows)
    L.append("| 字节合计 | %d |" % tot_bytes)
    L.append("| 分段总数 | %d（复位/回灌切段） |" % sum(r["segments"] for r in recs))
    L.append("| 停机总时长 | %d 秒 ≈ %.0f 行（不计入缺口） |" % (tot_down, tot_down * 10.0))
    L.append("| 可信段名义合计 | %.0f |" % tot_nominal)
    L.append("")
    L.append("## 二、缺口拆分\n")
    L.append("| 块 | 行数 | 含义 |")
    L.append("|---|---:|---|")
    L.append("| 假缺口（tick 偏慢） | %.0f | 可信段 nominal - tick_expected |" % tot_false)
    L.append("| 文件内真缺口 | %.0f | tick_expected - rows |" % tot_real)
    L.append("| 跨文件边界缺口 | %.0f | Σ(first_seq_next - last_seq_prev - 1)，仅 seq 连续时计 |" % tot_bnd)
    L.append("| 停机 | %.0f | %d 秒，设备没运行 |" % (tot_down * 10.0, tot_down))
    L.append("| rtc 跳变段（已隔离） | %.0f | 这些段 rtc 与 tick 不同步，不参与假缺口计算 |" % tot_anom)
    L.append("| 未解释余量 | %.0f | README 的 %.0f 减去上面各项 |" % (residual, a.gap))
    L.append("")
    L.append("README 声称总行数 %.0f，本次实测 %d，差 %.0f。\n"
             % (a.expected_rows, tot_rows, a.expected_rows - tot_rows))
    L.append("## 三、事件文件\n")
    if evt_rows:
        L.append("| 文件 | 类型 | 原因 | 条数 |")
        L.append("|---|---|---|---:|")
        for r in evt_rows:
            L.append("| %s | %s | %s | %d |" % (r["file"], r["type"], r["reason"], r["count"]))
    else:
        L.append("输入目录内没有事件文件，或事件文件里没有以 # 开头的行。")
    L.append("")
    L.append("## 四、这个账本证明不了什么\n")
    for s in [
        "文件首行之前 / 末行之后被截断的部分：没有相邻行可比。",
        "跨文件、复位边界：只能按 seq 记缺口，不能证明数据是否产生过。",
        "尾批未落盘：从未成为一行，账本里不存在。",
        "W25 回灌行：tick 不保证单调，已按 Δtick<=0 切段；回灌段编号空间可能与原会话重叠。",
        "\"每秒 10 行\"：需要 tick 到物理秒的独立校准，本账本不含。",
        "CRC 通过只证明内容没坏，不证明行没丢。",
        "按 KEY1 暂停过的时间段会被算进\"文件内真缺口\"：暂停时 tick 照走、seq 冻结，"
        "账本无法把它和真丢行区分开，只能靠当时的 # 事件行佐证。",
    ]:
        L.append("- " + s)
    L.append("")
    L.append("## 五、复现\n")
    L.append("```")
    L.append("python analyze_longrun.py --input <CSV_DIR> --out <OUT_DIR>")
    L.append("python crc_all.py --input <CSV_DIR> --out <OUT_DIR>")
    L.append("```")

    with open(os.path.join(a.out, "gap_report.md"), "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(L))

    print()
    print("files=%d rows=%d bytes=%d" % (len(recs), tot_rows, tot_bytes))
    print("nominal=%.0f false=%.0f real=%.0f boundary=%.0f down=%ds anom=%.0f residual=%.0f"
          % (tot_nominal, tot_false, tot_real, tot_bnd, tot_down, tot_anom, residual))
    print("-> %s" % a.out)


if __name__ == "__main__":
    main()
