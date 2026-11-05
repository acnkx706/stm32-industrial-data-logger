# crc_all.py -- EXP-01 第二步：全量逐行 CRC 校验（慢，可后台跑）
#
# 做了什么：对每个数据文件，逐行取"最后一个逗号之前"的文本算 CRC16/KERMIT，
#           和行尾十进制 crc 比对，统计通过/失败/无 CRC。
# 为什么：README 里"CRC 零不匹配"只覆盖过两个文件，不是全量；且它是内容完整性证据，
#         不等于行完整性，必须单独出一张表，不能和缺口账本混为一谈。
# 影响什么：只读输入目录；输出 crc_all.csv。
# 怎么验证：启动时先跑锚点 CRC16/KERMIT("123456789") == 0x2189，不通过直接退出。
#
# 用法：
#   python crc_all.py --input <CSV_DIR> --out <OUT_DIR>
import argparse
import csv
import os
import re
import sys

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

NAME_RE = re.compile(r"^(\d{6})(\d{2})\.CSV$")
ANCHOR = 0x2189
ALLOWED = b"0123456789,-."   # 数据行允许出现的字节；其它字节说明是损坏区的垃圾


def crc16_kermit(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
    return crc & 0xFFFF


def check_file(path):
    rows = 0
    ok = 0
    bad = 0
    zero = 0
    first_bad = None
    sample = ""
    fmt_bad = 0
    lineno = 0

    with open(path, "rb") as f:
        for raw in f:
            lineno += 1
            line = raw.rstrip(b"\r\n")
            if not line:
                continue
            if line[:1] == b"#":
                continue
            if line.startswith(b"seq,"):
                continue

            p = line.rsplit(b",", 1)
            if len(p) != 2:
                fmt_bad += 1
                continue
            # 与 analyze_longrun.py 用同一套规则：至少 15 个字段才当数据行；
            # 并且整行只允许 数字/逗号/减号/小数点，出现其它字节说明是介质损坏区的垃圾。
            if p[0].count(b",") < 14 or p[0].translate(None, ALLOWED):
                fmt_bad += 1
                continue
            try:
                want = int(p[1])
            except ValueError:
                fmt_bad += 1
                continue

            rows += 1
            if want == 0:
                zero += 1
                continue

            got = crc16_kermit(p[0])
            if got == want:
                ok += 1
            else:
                bad += 1
                if first_bad is None:
                    first_bad = lineno
                    sample = line.decode("latin-1")[:120]

    note = []
    if zero:
        note.append("no_crc_rows=%d" % zero)
    if fmt_bad:
        note.append("fmt_bad=%d" % fmt_bad)

    return {
        "rows": rows, "crc_ok": ok, "crc_bad": bad,
        "first_bad_line": first_bad if first_bad is not None else "",
        "sample": sample,
        "note": ";".join(note),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    v = crc16_kermit(b"123456789")
    if v != ANCHOR:
        print("!! CRC 自检失败：得到 0x%04X，期望 0x%04X —— 拒绝继续" % (v, ANCHOR))
        raise SystemExit(2)
    print("CRC 自检 OK (0x2189)")

    os.makedirs(a.out, exist_ok=True)

    files = sorted(n for n in os.listdir(a.input)
                   if NAME_RE.match(n.upper()) and NAME_RE.match(n.upper()).group(2) != "90")

    out_csv = os.path.join(a.out, "crc_all.csv")
    tot_rows = tot_ok = tot_bad = 0
    with open(out_csv, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["file", "rows", "crc_ok", "crc_bad", "first_bad_line", "note", "sample"])
        for n in files:
            r = check_file(os.path.join(a.input, n))
            tot_rows += r["rows"]
            tot_ok += r["crc_ok"]
            tot_bad += r["crc_bad"]
            w.writerow([n, r["rows"], r["crc_ok"], r["crc_bad"],
                        r["first_bad_line"], r["note"], r["sample"]])
            print("%-14s rows=%-7d ok=%-7d bad=%-3d %s"
                  % (n, r["rows"], r["crc_ok"], r["crc_bad"], r["note"]))

    rate = (100.0 * tot_ok / tot_rows) if tot_rows else 0.0
    print()
    print("files=%d rows=%d ok=%d bad=%d pass=%.4f%%" % (len(files), tot_rows, tot_ok, tot_bad, rate))
    print("-> %s" % out_csv)


if __name__ == "__main__":
    main()
