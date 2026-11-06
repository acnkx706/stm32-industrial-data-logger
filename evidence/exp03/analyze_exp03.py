# analyze_exp03.py -- 把 EXP-03 的串口日志拆成"一次事件一行"的操作表
#
# 做了什么：逐行扫串口日志，用 `REC,seq<...>,tick<...>` 里的 tick 当时间基准，
#           把 SD 侧的事件（ERR / recovered / replay done / drained / LOST / BOOT）
#           打上相对时间，再按"一次拔卡 = 从第一个 ERR 到下一次 replay done"归并成一行。
# 为什么：EXP-03 要靠串口日志重建每一次拔插的时间线；手填操作表容易错、
#           而且 `pc_epoch` 和板子 tick 对不齐。板子自己打的 tick 才是可靠时间轴。
# 影响什么：只读日志，不碰板子、不碰卡。
# 怎么验证：输出的 cycles 行数应等于实际拔卡次数；event_csv 里每条事件都有 tick。
#
# 用法：
#   python analyze_exp03.py exp03_A5.log out_A5
import io
import re
import sys

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

LOG = sys.argv[1] if len(sys.argv) > 1 else "exp03_A5.log"
OUT = sys.argv[2] if len(sys.argv) > 2 else "out"

RE_TICK = re.compile(r"tick<(\d+)>")
RE_SEQ = re.compile(r"seq<(\d+)>")
RE_ERR = re.compile(r"SD: ERR fr=(\d+) site=(\d+)")
RE_RECOVERED = re.compile(r"SD: recovered after (\d+)s \(fr was (\d+)\)")
RE_REPLAY = re.compile(r"SD: replay done (\d+) batches / (\d+) recs")
RE_DRAIN = re.compile(r"SD: drained 1 batch \(stored (\d+), ram=(\d+) recs\) pending=(\d+)")
RE_CACHE = re.compile(r"SD: cache restored 1 batch \(left=(\d+)\)")
RE_LOG = re.compile(r"SD: log = (0:\S+)  size=(\d+)")

events = []
cur_tick = 0
cur_seq = 0

with io.open(LOG, encoding="utf-8", errors="replace") as f:
    for raw in f:
        line = raw.strip()
        if not line:
            continue
        m = RE_TICK.search(line)
        if m:
            cur_tick = int(m.group(1))
        m = RE_SEQ.search(line)
        if m:
            cur_seq = int(m.group(1))

        mt = None
        if RE_ERR.search(line):
            mt = "err"
            fr, site = RE_ERR.search(line).groups()
            events.append((cur_tick, cur_seq, "err", "fr=%s site=%s" % (fr, site)))
        elif RE_RECOVERED.search(line):
            sec, fr = RE_RECOVERED.search(line).groups()
            events.append((cur_tick, cur_seq, "recovered", "%ss fr=%s" % (sec, fr)))
        elif RE_REPLAY.search(line):
            b, r = RE_REPLAY.search(line).groups()
            events.append((cur_tick, cur_seq, "replay_done", "%s batches / %s recs" % (b, r)))
        elif RE_DRAIN.search(line):
            stored, ram, pend = RE_DRAIN.search(line).groups()
            events.append((cur_tick, cur_seq, "drain", "pending=%s ram=%s" % (pend, ram)))
        elif RE_CACHE.search(line):
            events.append((cur_tick, cur_seq, "cache_restore", "left=%s" % RE_CACHE.search(line).group(1)))
        elif RE_LOG.search(line):
            nm, sz = RE_LOG.search(line).groups()
            events.append((cur_tick, cur_seq, "open", "%s size=%s" % (nm, sz)))
        elif "=== BOOT ===" in line:
            events.append((cur_tick, cur_seq, "BOOT", ""))
        elif "SD: LOST" in line:
            events.append((cur_tick, cur_seq, "LOST_SERIAL", line[:60]))

# 归并：一次"拔卡"= 连续的 err 段 + 其后的 recovered/replay
cycles = []
i = 0
while i < len(events):
    tick, seq, kind, det = events[i]
    if kind == "err":
        start = tick
        nerr = 0
        t_last = tick
        while i < len(events) and events[i][2] == "err":
            nerr += 1
            t_last = events[i][0]
            i += 1
        # 往后找 recovered / replay
        rec = None
        rep = None
        limit = i + 400
        j = i
        while j < len(events) and j < limit:
            if events[j][2] == "recovered" and rec is None:
                rec = events[j]
            if events[j][2] == "replay_done" and rec is not None:
                rep = events[j]
                break
            if events[j][2] == "err":
                break
            j += 1
        cycles.append({
            "pull_tick": start,
            "errs": nerr,
            "err_span_ms": t_last - start,
            "recovered_s": (rec[3] if rec else ""),
            "recover_tick": (rec[0] if rec else ""),
            "gap_ms": (rec[0] - start if rec else ""),
            "replay": (rep[3] if rep else ""),
            "replay_tick": (rep[0] if rep else ""),
        })
        i = j + 1 if rep else j
    else:
        i += 1

print("事件总数: %d" % len(events))
print("拔卡循环: %d 次" % len(cycles))
print("")
print("| # | 拔卡 tick | ERR 条数 | ERR 持续 ms | recovered | 从拔到恢复 ms | replay |")
print("|---:|---:|---:|---:|---|---:|---|")
for k, c in enumerate(cycles, 1):
    print("| %d | %s | %d | %s | %s | %s | %s |"
          % (k, c["pull_tick"], c["errs"], c["err_span_ms"],
             c["recovered_s"], c["gap_ms"], c["replay"]))

# 明细表
with io.open("%s_events.csv" % OUT, "w", encoding="utf-8", newline="") as f:
    f.write("tick_ms,seq,kind,detail\n")
    for tick, seq, kind, det in events:
        f.write("%d,%d,%s,\"%s\"\n" % (tick, seq, kind, det))
print("")
print("事件明细 -> %s_events.csv" % OUT)

boots = sum(1 for e in events if e[2] == "BOOT")
lost = sum(1 for e in events if e[2] == "LOST_SERIAL")
print("=== BOOT === 次数: %d （非预期复位判据，应为 0）" % boots)
print("SD: LOST 串口行数: %d" % lost)
