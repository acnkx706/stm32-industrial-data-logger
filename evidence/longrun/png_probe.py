# png_probe.py -- 追查 PNG 头为什么出现在 CSV 里
#
# 做了什么：在文件里找出所有非 CSV 字节区间，统计长度、起始偏移的对齐情况、PNG 签名个数
# 为什么：如果垃圾区按 512B 扇区或 32KB 簇对齐，那是 FAT 簇链串了文件；
#           如果是字节级零散插入，那是写入过程中混入，成因完全不同
# 影响什么：只读
# 怎么验证：对照 png 签名个数与完整 IHDR 宽高，能判断是不是一张真图片
import sys

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

ALLOWED = set(b"0123456789,-.\r\n")
PNG_SIG = b"\x89PNG\r\n\x1a\n"


def probe(path):
    data = open(path, "rb").read()
    n = len(data)

    sigs = []
    i = data.find(PNG_SIG)
    while i != -1:
        sigs.append(i)
        i = data.find(PNG_SIG, i + 1)

    ihdr = data.find(b"IHDR")
    wh = None
    if ihdr != -1:
        w = int.from_bytes(data[ihdr + 4:ihdr + 8], "big")
        h = int.from_bytes(data[ihdr + 8:ihdr + 12], "big")
        wh = (w, h)

    # 找连续的非 CSV 区间
    runs = []
    start = None
    for i in range(n):
        ok = data[i] in ALLOWED
        if not ok and start is None:
            start = i
        elif ok and start is not None:
            runs.append((start, i - start))
            start = None
    if start is not None:
        runs.append((start, n - start))

    print("file      :", path)
    print("bytes     :", n)
    print("png_sig   :", len(sigs), sigs[:5])
    print("IHDR size :", wh)
    print("noncsv_runs:", len(runs))
    if runs:
        lens = [r[1] for r in runs]
        print("  run len  : min=%d max=%d 总非CSV字节=%d" % (min(lens), max(lens), sum(lens)))
        # 对齐检查
        a512 = sum(1 for r in runs if r[0] % 512 == 0)
        a4k = sum(1 for r in runs if r[0] % 4096 == 0)
        print("  起点%512==0: %d/%d   起点%4096==0: %d/%d" % (a512, len(runs), a4k, len(runs)))
        print("  前 8 段 (offset, len, off%512, off%32768):")
        for st, ln in runs[:8]:
            print("    %10d %8d  %5d %6d" % (st, ln, st % 512, st % 32768))


probe(sys.argv[1])
