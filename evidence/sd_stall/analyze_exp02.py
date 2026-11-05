# analyze_exp02.py -- 解析 EXP02 行，生成 stall_raw.csv / stall_summary.csv
#
# 做了什么：从串口原始日志里抽 EXP02 行，拆成字段；按 (kind, stall_ms, qlen) 聚合
# 为什么：验收要求 first_reason 10/10 一致、d_qfull=0、first_loss_ms 有 min/mean/max
# 影响什么：只读日志，输出 CSV
# 怎么验证：stall_raw 行数 = 日志里 EXP02 数据行数；summary 的 n 与之一致
import csv
import os
import sys

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

LOG = sys.argv[1] if len(sys.argv) > 1 else "serial_stall8.log"
OUT = sys.argv[2] if len(sys.argv) > 2 else "."

COLS = ["kind", "stall_ms", "qlen", "trial", "qmax", "d_qfull", "d_qfull_loss",
        "d_nobuf", "d_enqfail", "d_acq_late", "first_reason", "first_loss_ms",
        "acq_before", "acq_after", "sd_err"]

RA = {0: "LOSS_Q_FULL", 1: "LOSS_NO_BUF", 2: "LOSS_ENQ_FAIL", 3: "LOSS_ROW_BIG",
      4: "LOSS_SD_WRITE", 5: "LOSS_ACQ_LATE", 0xFF: "none"}

rows = []
for line in open(LOG, "rb").read().decode("latin-1").replace("\r", "\n").split("\n"):
    if not line.startswith("EXP02,"):
        continue
    p = line.split(",")
    if len(p) != len(COLS) + 1:
        continue
    try:
        vals = [int(x) for x in p[1:]]
    except ValueError:
        continue
    rows.append(dict(zip(COLS, vals)))

os.makedirs(OUT, exist_ok=True)
with open(os.path.join(OUT, "stall_raw.csv"), "w", newline="", encoding="utf-8") as f:
    w = csv.DictWriter(f, fieldnames=COLS)
    w.writeheader()
    for r in rows:
        w.writerow(r)

# 聚合
grp = {}
for r in rows:
    k = (r["kind"], r["stall_ms"], r["qlen"])
    grp.setdefault(k, []).append(r)

summary = []
for k in sorted(grp):
    rs = grp[k]
    reasons = {}
    for r in rs:
        nm = RA.get(r["first_reason"], str(r["first_reason"]))
        reasons[nm] = reasons.get(nm, 0) + 1
    fl = [r["first_loss_ms"] for r in rs if r["first_loss_ms"] > 0]
    summary.append({
        "kind": "SD" if k[0] == 0 else "PROC",
        "stall_ms": k[1],
        "qlen": k[2],
        "n": len(rs),
        "first_reason": "/".join("%s:%d" % (a, b) for a, b in sorted(reasons.items())),
        "first_loss_ms_min": min(fl) if fl else "",
        "first_loss_ms_mean": round(sum(fl) / len(fl), 1) if fl else "",
        "first_loss_ms_max": max(fl) if fl else "",
        "d_qfull_sum": sum(r["d_qfull"] for r in rs),
        "d_nobuf_sum": sum(r["d_nobuf"] for r in rs),
        "d_enqfail_sum": sum(r["d_enqfail"] for r in rs),
        "d_acq_late_sum": sum(r["d_acq_late"] for r in rs),
        "qmax_max": max(r["qmax"] for r in rs),
        "sd_err_sum": sum(r["sd_err"] for r in rs),
    })

cols = ["kind", "stall_ms", "qlen", "n", "first_reason", "first_loss_ms_min",
        "first_loss_ms_mean", "first_loss_ms_max", "d_qfull_sum", "d_nobuf_sum",
        "d_enqfail_sum", "d_acq_late_sum", "qmax_max", "sd_err_sum"]
with open(os.path.join(OUT, "stall_summary.csv"), "w", newline="", encoding="utf-8") as f:
    w = csv.DictWriter(f, fieldnames=cols)
    w.writeheader()
    for s in summary:
        w.writerow(s)

print("trials =", len(rows))
for s in summary:
    print("%-4s %5dms qlen=%d n=%2d  first=%-28s loss_ms=%s/%s/%s  dqfull=%d dnobuf=%d qmax=%d"
          % (s["kind"], s["stall_ms"], s["qlen"], s["n"], s["first_reason"],
             s["first_loss_ms_min"], s["first_loss_ms_mean"], s["first_loss_ms_max"],
             s["d_qfull_sum"], s["d_nobuf_sum"], s["qmax_max"]))
