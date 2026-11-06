# backup_card.py -- 把卡上所有文件全量备份到 PC，并出一份带 SHA256 的清单
#
# 做了什么：逐个文件复制，复制时算"源"的 SHA256，复制完再算"目标"的 SHA256，
#           两边一致才写进清单；最后打印总文件数/总字节。
# 为什么：EXP-01 的 1.36 GB 原始数据只有这一份，任何写满/格式化之前必须全量留底；
#         而且不能只信"复制没报错"，要两边哈希对上才算数。
# 影响什么：只读卡，不写卡。目标盘上会新建一个备份目录。
# 怎么验证：清单里每一行 src_sha256 == dst_sha256，且行数等于卡上文件数。
#
# 用法：
#   python backup_card.py --from E --to "D:\_card_backup_20261106b"
#   python backup_card.py --from E --to ... --force     # 目标目录非空也继续
import argparse
import csv
import hashlib
import io
import os
import shutil
import sys
import time

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

CHUNK = 4 << 20


def sha256_of(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(CHUNK), b""):
            h.update(b)
    return h.hexdigest()


def fmt_mtime(path):
    """时间戳可能坏掉：卡上有 1601-01-01 的非法时间，localtime 会抛 OSError。
    这种文件的时间戳本身没有信息量，记成 invalid 比让整个备份脚本崩掉好。"""
    try:
        return time.strftime("%Y-%m-%d %H:%M:%S",
                             time.localtime(os.path.getmtime(path)))
    except (OSError, OverflowError, ValueError):
        return "invalid"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--from", dest="src", default="E")
    ap.add_argument("--to", dest="dst", required=True)
    ap.add_argument("--force", action="store_true")
    a = ap.parse_args()

    src = a.src.rstrip(":\\") + ":\\"
    dst = os.path.abspath(a.dst)
    if not os.path.isdir(src):
        print("源不存在:", src)
        raise SystemExit(2)
    if os.path.isdir(dst) and os.listdir(dst):
        if not a.force:
            print("目标目录非空，加 --force 才继续:", dst)
            raise SystemExit(2)
    os.makedirs(dst, exist_ok=True)

    files = []
    for name in sorted(os.listdir(src)):
        p = os.path.join(src, name)
        if os.path.isfile(p):
            files.append(name)
    print("卡上根目录文件数:", len(files))

    rows = []
    total = 0
    t0 = time.time()
    for i, name in enumerate(files, 1):
        s = os.path.join(src, name)
        d = os.path.join(dst, name)
        hs = sha256_of(s)
        # 用 copyfile 不用 copy2：copy2 会复制时间戳，而卡上有 1601 年的非法时间，
        # 设置它会抛 OSError，把备份中断在一个无关紧要的元数据上
        shutil.copyfile(s, d)
        hd = sha256_of(d)
        size = os.path.getsize(s)
        total += size
        ok = (hs == hd) and (os.path.getsize(d) == size)
        rows.append({
            "file": name,
            "bytes": size,
            "sha256": hs,
            "dst_sha256": hd,
            "match": "OK" if ok else "MISMATCH",
            "src_mtime": fmt_mtime(s),
        })
        print("  [%2d/%2d] %-22s %12d B  %s  %s"
              % (i, len(files), name, size, hs[:16], "OK" if ok else "MISMATCH"),
              flush=True)

    man = os.path.join(dst, "manifest_backup.csv")
    with io.open(man, "w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    bad = [r for r in rows if r["match"] != "OK"]
    print("")
    print("合计: %d 个文件 / %d B (%.3f GB) / 用时 %.1f s"
          % (len(rows), total, total / 1e9, time.time() - t0))
    print("清单:", man)
    if bad:
        print("!! %d 个文件哈希不一致，先别做任何破坏性操作" % len(bad))
        for r in bad:
            print("   ", r["file"])
        raise SystemExit(3)
    print("全部一致，可以进入下一步")


if __name__ == "__main__":
    main()
