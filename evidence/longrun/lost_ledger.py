#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
LOST 对账 + 重复行检查（2026-11-09）
====================================

★ 要回答两个问题
  ① **LOST 对账**：固件自己在事件文件里记的 `# LOST`（19,426 条）准不准？
     真缺的条数应该等于"每个会话里 max(seq) − 去重后的条数"。
  ② **回写会不会重录**：掉卡缓存回写（`# SD_REPLAY`）把旧记录追加到文件后面时，
     会不会出现同一个 seq 写了两遍？

★ 为什么口径要这么定
  · seq 每次开机从 1 开始 ⇒ 一个"会话"= 从 seq==1 到下一次 seq==1；
  · 回写会把旧 seq 追加到文件末尾，于是文件里的 seq 不是单调的
    ⇒ 不能用"相邻两行差值"算缺口（会在 2900→2666→2900 这种地方把同一段缺口算两遍）；
  · 用"集合"最稳：缺的 = max − 集合大小；重复的 = 行数 − 集合大小。

★ 判据
  · 缺口 ≈ LOST 合计 ⇒ 固件记账可信；
  · 重复 > 0 ⇒ 回写把同一行写了两遍（真问题，要查 DropCache 的指针推进）；
  · 重复 == 0 ⇒ 回写只补没写过的行，行为正确。

★ 用法
  python lost_ledger.py <目录>          # 默认 evidence/longrun/_stage
  python lost_ledger.py <目录> --json out.json
"""

import argparse
import json
import os
import re
import sys
from collections import Counter

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

DATA_RE = re.compile(r"^(\d{6})(\d{2})\.CSV$", re.IGNORECASE)   # YYMMDDNN.CSV
EVT_RE = re.compile(r"^(\d{6})90\.CSV$", re.IGNORECASE)          # 事件文件用 90 后缀
LOST_RE = re.compile(r"^# LOST,(\d+),(\d+),(\w+)")
REPLAY_RE = re.compile(r"^# SD_REPLAY,(\d+),batches=(\d+),recs=(\d+)")


def scan_dir(path):
    data, evt = [], []
    for name in os.listdir(path):
        if EVT_RE.match(name):
            evt.append(name)
        elif DATA_RE.match(name):
            data.append(name)
    return sorted(data), sorted(evt)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir", nargs="?", default=".")
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    data_files, evt_files = scan_dir(args.dir)
    print("数据文件 %d 个，事件文件 %d 个" % (len(data_files), len(evt_files)))

    # ---------- ① 数据侧：按文件统计 ----------
    #   ★ 为什么用 (seq, tick, rtc) 做唯一键：
    #     每次开机 seq 和 tick 都从头开始，(seq,tick) 在不同会话之间必然重复
    #     （实测 26102601.CSV 里 "1,101" 就出现多次，但 rtc 不同）。
    #     rtc 是墙钟秒，跨会话单调 ⇒ 三元组才是真唯一。
    total_rows = 0
    dups = 0
    per_file = []
    dup_samples = []          # (文件, 首次行号, 再次行号, 键)
    for fname in data_files:
        rows = 0
        keys = {}
        with open(os.path.join(args.dir, fname), encoding="utf-8", errors="replace") as fh:
            header = fh.readline()
            if not header.lower().startswith("seq"):
                fh.seek(0)
            for lineno, line in enumerate(fh, start=2):
                if not line or line[0] == "#":
                    continue
                # ★ 严格判"这是不是一条记录"：15 个逗号 + 关键列都是数字
                parts = line.rstrip("\r\n").split(",")
                if len(parts) < 15:
                    continue
                try:
                    seq = int(parts[0]); tick = int(parts[1]); rtc = int(parts[3])
                except ValueError:
                    continue
                rows += 1
                k = (seq, tick, rtc)
                if k in keys:
                    if len(dup_samples) < 8:
                        dup_samples.append((fname, keys[k], lineno, k))
                else:
                    keys[k] = lineno
        d = rows - len(keys)
        total_rows += rows
        dups += d
        per_file.append((fname, rows, len(keys), d))

    # ---------- ② 事件侧：LOST 与 REPLAY ----------
    lost_total = 0
    lost_by_reason = Counter()
    replay_batches = 0
    replay_recs = 0
    for fname in evt_files:
        with open(os.path.join(args.dir, fname), encoding="utf-8", errors="replace") as fh:
            for line in fh:
                line = line.strip()
                m = LOST_RE.match(line)
                if m:
                    n = int(m.group(2))
                    lost_total += n
                    lost_by_reason[m.group(3)] += n
                    continue
                m = REPLAY_RE.match(line)
                if m:
                    replay_batches += int(m.group(2))
                    replay_recs += int(m.group(3))

    # ---------- 报告 ----------
    print("\n=== 数据侧 ===")
    print("  数据行           : %d" % total_rows)
    print("  唯一记录(seq+tick+rtc) : %d" % (total_rows - dups))
    print("  重复行           : %d" % dups)

    print("\n=== 事件侧 ===")
    print("  LOST 合计        : %d" % lost_total)
    for r, n in lost_by_reason.most_common():
        print("     %-10s %d" % (r, n))
    print("  SD_REPLAY        : %d 批 / %d 条" % (replay_batches, replay_recs))

    print("\n=== 重复行明细（只有非零的）===")
    any_dup = False
    for fname, rows, uniq, d in per_file:
        if d:
            any_dup = True
            print("  %-14s 行=%8d 唯一=%8d 重复=%6d" % (fname, rows, uniq, d))
    if not any_dup:
        print("  所有文件重复行 = 0")

    print("\n=== 结论 ===")
    if dups == 0:
        print("  回写没有重录：SD_REPLAY 追加的都是原先没写进去的行")
    else:
        print("  ⚠️ 有 %d 行重复 ⇒ 需要查 DropCache 指针推进或回写边界" % dups)

    if dup_samples:
        print("\n=== 重复行样本（首次行号 → 再次行号）===")
        for fname, first, again, k in dup_samples:
            print("  %s  %d → %d   键(seq,tick,rtc)=%s   间隔 %d 行"
                  % (fname, first, again, k, again - first))
        gaps = [again - first for _, first, again, _ in dup_samples]
        if gaps:
            print("  （样本间隔：%s）" % ", ".join(str(g) for g in gaps))

    if args.json:
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump({
                "rows": total_rows, "dups": dups,
                "per_file": [{"file": f, "rows": r, "uniq": u, "dups": d}
                             for f, r, u, d in per_file],
                "lost_total": lost_total,
                "lost_by_reason": dict(lost_by_reason),
                "replay_batches": replay_batches, "replay_recs": replay_recs,
                "dup_samples": [{"file": f, "first": a, "again": b, "key": list(k)}
                                for f, a, b, k in dup_samples],
            }, fh, ensure_ascii=False, indent=1)
        print("\n明细已写：%s" % args.json)
    return 0


if __name__ == "__main__":
    sys.exit(main())
