# analyze_lat.py -- 解析 LAT/TOP 行，生成延迟三张表
#
# 做了什么：把串口里的 LAT 块切成"每次 dump 一块"，取最后一块（计数是累计的），
#           算 p50/p90/p99/max 与 ge100ms/ge250ms，另存 top-32
# 为什么：验收要求每个条件有 p50/p90/p99/max，且直方图计数和 = 样本数
# 影响什么：只读日志，输出 CSV
# 怎么验证：各桶计数之和应等于 SUM 行的 n
import csv
import os
import re
import sys

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

LOG = sys.argv[1] if len(sys.argv) > 1 else "serial_lat.log"
OUT = sys.argv[2] if len(sys.argv) > 2 else "lat"

MAXB = 256
RE_SUM = re.compile(r"^LAT,SUM,(\d+),(\d+),(\d+),(\d+),(\d+)$")
RE_BIN = re.compile(r"^LAT,(\d+),(\d+),(\d+)$")
RE_TOP = re.compile(r"^TOP,(\d+),(\d+),(\d+)$")


def new_block():
    return {"n": [0, 0], "avg": [0, 0], "max": [0, 0], "ovf": [0, 0],
            "hist": [[0] * MAXB, [0] * MAXB], "top": []}


blocks = []
cur = new_block()
blocks.append(cur)

for line in open(LOG, "rb").read().decode("latin-1").replace("\r", "\n").split("\n"):
    m = RE_SUM.match(line)
    if m:
        op, n, avg, mx, ovf = (int(g) for g in m.groups())
        if op == 0:
            cur = new_block()
            blocks.append(cur)
        cur["n"][op] = n
        cur["avg"][op] = avg
        cur["max"][op] = mx
        cur["ovf"][op] = ovf
        continue

    m = RE_BIN.match(line)
    if m:
        op, b, c = (int(g) for g in m.groups())
        if 0 <= op <= 1 and 0 <= b < MAXB:
            cur["hist"][op][b] = c
        continue

    m = RE_TOP.match(line)
    if m:
        op, us, tick = (int(g) for g in m.groups())
        cur["top"].append((op, us, tick))

last = blocks[-1]
os.makedirs(OUT, exist_ok=True)


def pct(hist, n, q):
    if n == 0:
        return 0
    need = n * q
    acc = 0
    for b in range(MAXB):
        acc += hist[b]
        if acc >= need:
            return b
    return MAXB


summary = []
with open(os.path.join(OUT, "latency_hist.csv"), "w", newline="", encoding="utf-8") as f:
    w = csv.writer(f)
    w.writerow(["op", "bin_ms", "count"])
    for op in (0, 1):
        for b in range(MAXB):
            if last["hist"][op][b]:
                w.writerow([("write" if op == 0 else "sync"), b, last["hist"][op][b]])

with open(os.path.join(OUT, "latency_top.csv"), "w", newline="", encoding="utf-8") as f:
    w = csv.writer(f)
    w.writerow(["op", "us", "tick"])
    for op, us, tick in sorted(last["top"], key=lambda x: -x[1]):
        w.writerow([("write" if op == 0 else "sync"), us, tick])

for op in (0, 1):
    h = last["hist"][op]
    n = last["n"][op]
    ge100 = sum(h[100:]) + last["ovf"][op]
    ge250 = sum(h[250:]) + last["ovf"][op]
    summary.append({
        "op": "write" if op == 0 else "sync",
        "n": n,
        "avg_us": last["avg"][op],
        "p50_ms": pct(h, n, 0.50),
        "p90_ms": pct(h, n, 0.90),
        "p99_ms": pct(h, n, 0.99),
        "max_us": last["max"][op],
        "ge100ms": ge100,
        "ge250ms": ge250,
        "ovf": last["ovf"][op],
        "hist_sum": sum(h),
    })

with open(os.path.join(OUT, "latency_summary.csv"), "w", newline="", encoding="utf-8") as f:
    w = csv.DictWriter(f, fieldnames=list(summary[0].keys()))
    w.writeheader()
    for s in summary:
        w.writerow(s)

print("dumps =", len(blocks) - 1)
for s in summary:
    print("%-5s n=%-6d avg=%6.2fms p50=%d p90=%d p99=%d max=%7.2fms ge100=%d ge250=%d 桶和=%d"
          % (s["op"], s["n"], s["avg_us"] / 1000.0, s["p50_ms"], s["p90_ms"], s["p99_ms"],
             s["max_us"] / 1000.0, s["ge100ms"], s["ge250ms"], s["hist_sum"]))
