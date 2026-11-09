#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
定位 17 天数据里那 71 行 CRC 不符（2026-11-09）
==============================================

★ 要回答什么
  17 天账本重算后，可解析 12,722,839 行里有 71 行 CRC 对不上。
  "71 行"这个数本身没有诊断价值，要回答的是：
    · 是**系统性**（某类行永远算错，例如含负数/含特殊字符），还是**随机**（介质坏块）？
    · 集中在哪些文件/时段？（是否与拔卡实验、掉卡缓存回写同时发生？）
    · 那些行的 seq/tick 还能解析吗？（能=内容完整只是 CRC 错；不能=结构性损坏）

★ 判据
  · 若坏行的 (stored - computed) 差值全相同，或全落在同一类值上 ⇒ 系统性
  · 若坏行零散分布、差值随机 ⇒ 随机坏块（介质/写入时序）
  · 若坏行 seq 连续、前后行都正常 ⇒ 单行级损坏，不是整段丢

★ 怎么用
  python crc_bad_locate.py <目录>            # 默认 evidence/longrun/_stage
  python crc_bad_locate.py <目录> --json out.json
"""

import argparse
import json
import os
import re
import sys

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

REC_RE = re.compile(r"^\d{8}\.CSV$", re.IGNORECASE)


def crc16_kermit(data):
    """与固件 Crc16_Kermit 同参数（反射 0x8408 / 初值 0 / 无输出异或）。"""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc >> 1) ^ 0x8408) if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


def field(parts, name, names):
    """按表头取字段（表头缺失时按固定下标）。"""
    try:
        i = names.index(name)
    except ValueError:
        return None
    if i >= len(parts):
        return None
    try:
        return int(parts[i])
    except ValueError:
        return None


def scan(path):
    bad = []
    total = 0
    names = []
    with open(path, encoding="utf-8", errors="replace") as fh:
        header = fh.readline().strip().split(",")
        if header and header[0].lower() in ("seq", "tick"):
            names = header
        else:
            fh.seek(0)
        for lineno, line in enumerate(fh, start=2):
            line = line.rstrip("\r\n")
            if not line or line.startswith("#"):
                continue
            cut = line.rfind(",")
            if cut < 0:
                continue
            prefix, crc_text = line[:cut], line[cut + 1:]
            try:
                stored = int(crc_text)
            except ValueError:
                continue
            total += 1
            got = crc16_kermit(prefix.encode("ascii", errors="replace"))
            if got != stored:
                parts = prefix.split(",")
                bad.append({
                    "line": lineno,
                    "stored": stored,
                    "computed": got,
                    "diff": (stored - got) & 0xFFFF,
                    "seq": field(parts, "seq", names),
                    "tick": field(parts, "tick", names),
                    "rtc": field(parts, "rtc", names),
                    "len": len(line),
                    "text": line[:100],
                })
    return total, bad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir", nargs="?", default=".")
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    files = sorted(f for f in os.listdir(args.dir) if REC_RE.match(f))
    if not files:
        print("目录里没有 YYMMDDNN.CSV：%s" % args.dir)
        return 2

    all_bad = []
    total_rows = 0
    per_file = []
    for f in files:
        n, bad = scan(os.path.join(args.dir, f))
        total_rows += n
        if bad:
            per_file.append((f, len(bad), n, bad))
            for b in bad:
                b["file"] = f
            all_bad.extend(bad)

    print("扫描 %d 个文件，%d 行；CRC 不符 %d 行" % (len(files), total_rows, len(all_bad)))
    print("\n--- 按文件 ---")
    for f, k, n, bad in per_file:
        first, last = bad[0], bad[-1]
        print("  %-14s %3d 行坏 / %8d 行   首行 %6d (seq=%s rtc=%s)  末行 %6d"
              % (f, k, n, first["line"], first["seq"], first["rtc"], last["line"]))

    if not all_bad:
        print("没有坏行。")
        return 0

    print("\n--- 差值分布（stored - computed）---")
    from collections import Counter
    dc = Counter(b["diff"] for b in all_bad)
    for d, k in dc.most_common(8):
        print("  0x%04X × %d" % (d, k))
    if len(dc) == 1:
        print("  ⇒ 差值全相同：**系统性**（同一处算法/参数问题）")
    elif max(dc.values()) > len(all_bad) * 0.5:
        print("  ⇒ 差值高度集中：偏系统性")
    else:
        print("  ⇒ 差值随机：偏**介质/写入时序**损坏")

    print("\n--- 结构完整性 ---")
    nulls = sum(1 for b in all_bad if b["seq"] is None)
    print("  seq 解析失败：%d / %d" % (nulls, len(all_bad)))
    lens = Counter(b["len"] for b in all_bad)
    print("  行长分布（前 5）：", ", ".join("%dB×%d" % (l, k) for l, k in lens.most_common(5)))

    print("\n--- 时间分布（按 rtc 的小时桶）---")
    hc = Counter()
    for b in all_bad:
        if b["rtc"]:
            hc[b["rtc"] // 3600] += 1
    for h, k in sorted(hc.items()):
        import time as _t
        print("  %s  ×%d" % (_t.strftime("%Y-%m-%d %H:00", _t.localtime(h * 3600)), k))

    print("\n--- 坏行样本（前 8 条）---")
    for b in all_bad[:8]:
        print("  %s:%d  stored=%5d computed=%5d  seq=%s"
              % (b["file"], b["line"], b["stored"], b["computed"], b["seq"]))
        print("      %s" % b["text"])

    if args.json:
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump(all_bad, fh, ensure_ascii=False, indent=1)
        print("\n明细已写：%s" % args.json)
    return 0


if __name__ == "__main__":
    sys.exit(main())
