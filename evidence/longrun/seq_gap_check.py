# seq_gap_check.py -- 核查 10-22 那 149 个 seq 跳号到底是"回灌/重复"还是"真缺"
#
# 做了什么：在 26102201.CSV 里找出所有"段内 seq 向前跳号"的区间，
#           把每个缺失的 seq 拿到整个文件里反查是否存在；不存在的才算真缺。
# 为什么：EXP-01b 把 10-22 的 1,730 拆成 1,581（未产生）+ 149（seq 跳号），
#           但"seq 跳号"不等于"数据丢失"——回灌会打乱书写顺序，造成同样的跳号。
# 影响什么：只读；输出 seq_gap_check.csv 与一行 storage_loss 结论
# 怎么验证：若每个缺失 seq 都能在同一文件的其它位置找到，则 storage_loss = 0
import csv
import os
import sys

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

ALLOWED = b"0123456789,-."
SRC = r"C:\Users\acnkx\Desktop\stm32\项目1：工业记录仪\工业记录仪\evidence\longrun\_stage\26102201.CSV"
OUT = r"C:\Users\acnkx\Desktop\stm32\项目1：工业记录仪\工业记录仪\evidence\longrun\out\seq_gap_check.csv"

# 第一遍：按行号收集 (seq, tick, rtc, lineno)
rows = []
with open(SRC, "rb") as f:
    for lineno, raw in enumerate(f, 1):
        body = raw.rstrip(b"\r\n")
        if not body or body[:1] == b"#" or body.startswith(b"seq,"):
            continue
        if body.count(b",") < 14 or body.translate(None, ALLOWED):
            continue
        fld = body.split(b",")
        try:
            rows.append((int(fld[0]), int(fld[1]), int(fld[3]), lineno))
        except ValueError:
            pass

# 建索引：seq -> [行号...]（回灌/重复会让同一 seq 出现多次）
index = {}
for seq, tick, rtc, ln in rows:
    index.setdefault(seq, []).append((ln, tick, rtc))

# 第二遍：切段并找段内向前跳号
prev = None
prev_ln = None
gap_no = 0
out_rows = []
missing_total = 0
found_total = 0

for seq, tick, rtc, ln in rows:
    if prev is not None and seq > prev + 1:
        gap_no += 1
        for miss in range(prev + 1, seq):
            missing_total += 1
            hits = index.get(miss, [])
            # 排除"就是这两行自己"的情况：命中行号若等于相邻行不算
            others = [(l, t, r) for (l, t, r) in hits if l != ln and l != prev_ln]
            found = len(others) > 0
            if found:
                found_total += 1
                l0 = others[0]
                out_rows.append([os.path.basename(SRC), gap_no, miss, prev, seq,
                                 1, l0[0], l0[1], l0[2], "found_elsewhere"])
            else:
                out_rows.append([os.path.basename(SRC), gap_no, miss, prev, seq,
                                 0, "", "", "", "truly_missing"])
    prev = seq
    prev_ln = ln

os.makedirs(os.path.dirname(OUT), exist_ok=True)
with open(OUT, "w", newline="", encoding="utf-8") as f:
    w = csv.writer(f)
    w.writerow(["file", "gap_no", "missing_seq", "prev_seq", "next_seq",
                "found_elsewhere", "found_line_no", "found_tick", "found_rtc", "verdict"])
    w.writerows(out_rows)

storage_loss = missing_total - found_total
print("行数=%d  段内跳号区间=%d  缺失 seq=%d  能在他处找到=%d  真缺=%d"
      % (len(rows), gap_no, missing_total, found_total, storage_loss))
print("storage_loss =", storage_loss)
print("->", OUT)
