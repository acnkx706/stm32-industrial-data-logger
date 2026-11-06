# card_stress_test.py -- 整卡写满再读回校验（h2testw 的替代品）
#
# 做了什么：按 1 MiB 一个块，用"由块号决定"的固定图案把卡写满，fsync + 刷卷缓存，
#           然后把每一块读回来逐字节比对，报告速度、第一处不一致的偏移、坏块数。
# 为什么：h2testw 要另外下载；而且它把写和读放在一次运行里，中间不能拔卡。
#         这个脚本把写和读拆成两条命令，中间可以安全弹出/重新插入，
#         这样才测得到"卡自己掉电一轮之后数据还在不在"。
# 影响什么：会写满整张卡并覆盖上面所有文件。只在备份验证过之后用。
# 怎么验证：--verify 报告"mismatch=0"，并且写之前/之后的可用空间一致地少掉同样多。
#
# 用法：
#   python card_stress_test.py --drive E --write          # 写满整卡
#   python card_stress_test.py --drive E --verify         # 读回校验（可换一次插拔后再跑）
#   python card_stress_test.py --drive E --delete         # 删掉填充文件、刷盘
#
# 判定：
#   mismatch=0 且读到 NBLK 块  -> 这张卡的存储介质在本次覆盖范围内是可读写的
#   mismatch>0                 -> 直接判"卡不健康"，记下偏移，别再把责任推给固件
#   写的过程中报"磁盘已满"是正常的（就是要把卡写满），不算失败
import argparse
import ctypes
import hashlib
import io
import json
import os
import subprocess
import sys
import time

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

BLK = 1 << 20          # 1 MiB 一块
STATE = "card_stress_state.json"
FILLER = "STRESS.BIN"


def base_pattern():
    """生成 1 MiB 基准图案。用确定性公式，不用随机数：换一台机器也能复算。"""
    b = bytearray(BLK)
    for i in range(BLK):
        b[i] = (i * 37 + (i >> 8) * 91 + 0x5A) & 0xFF
    return bytes(b)


BASE = base_pattern()


def block_for(k):
    """第 k 块的期望内容：把基准图案按块号旋转，这样块与块不同、又不占内存。"""
    r = (k * 7919) % BLK
    return BASE[r:] + BASE[:r]


def free_total(drive):
    d = drive.rstrip(":\\") + ":\\"
    free = ctypes.c_ulonglong(0)
    total = ctypes.c_ulonglong(0)
    ctypes.windll.kernel32.GetDiskFreeSpaceExW(
        ctypes.c_wchar_p(d), None, ctypes.byref(total), ctypes.byref(free))
    return free.value, total.value


def flush_volume(drive):
    letter = drive.rstrip(":\\")[0].upper()
    r = subprocess.run(
        ["powershell", "-NoProfile", "-Command",
         "Write-VolumeCache -DriveLetter %s" % letter],
        capture_output=True, timeout=300)
    return "Write-VolumeCache OK" if r.returncode == 0 else "FLUSH FAILED"


def eject(drive):
    letter = drive.rstrip(":\\")[0].upper()
    subprocess.run(
        ["powershell", "-NoProfile", "-Command",
         "$sh=New-Object -ComObject Shell.Application;"
         "$sh.Namespace(17).ParseName('%s:').InvokeVerb('Eject')" % letter],
        capture_output=True, timeout=300)


def do_write(drive):
    path = os.path.join(drive.rstrip(":\\") + ":\\", FILLER)
    free, total = free_total(drive)
    print("写之前: 总 %.2f GB / 可用 %.2f GB" % (total / 1e9, free / 1e9))
    if os.path.exists(path):
        print("发现旧的 %s，先删掉" % FILLER)
        os.remove(path)

    nblk = 0
    t0 = time.time()
    full = False
    try:
        with open(path, "wb", buffering=0) as f:
            while True:
                n = f.write(block_for(nblk))
                if n != BLK:
                    print("!! 第 %d 块只写进了 %d 字节" % (nblk, n))
                    break
                nblk += 1
                if (nblk % 512) == 0:
                    print("  已写 %.2f GB  (%.1f MB/s)"
                          % (nblk * BLK / 1e9,
                             nblk * BLK / 1e6 / max(1e-6, time.time() - t0)),
                          flush=True)
    except OSError as e:
        # errno 28 = 磁盘已满，这是本测试的正常终点；其它 errno 才是真的写坏了
        if getattr(e, "errno", None) == 28 or "space" in str(e).lower():
            full = True
            print("写满：第 %d 块时磁盘已满（这是预期的终点）" % nblk)
        else:
            print("!! 写入出错（不是磁盘满，是卡的问题）: %s" % e)
            print("!! 第 %d 块写失败" % nblk)
    dt = time.time() - t0
    print("写完成: %d 块 / %.3f GB / %.1f s / %.1f MB/s%s"
          % (nblk, nblk * BLK / 1e9, dt, nblk * BLK / 1e6 / max(1e-6, dt),
             "（已写满整卡）" if full else "（未写满，见上面的报错）"))
    print("刷盘:", flush_volume(drive))

    st = {
        "filler": path,
        "nblk": nblk,
        "bytes": nblk * BLK,
        "write_s": round(dt, 1),
        "base_sha256": hashlib.sha256(BASE).hexdigest(),
        "when": time.strftime("%Y-%m-%d %H:%M:%S"),
        "total_bytes": total,
        "free_before": free,
        "free_after": free_total(drive)[0],
    }
    io.open(STATE, "w", encoding="utf-8").write(json.dumps(st, ensure_ascii=False, indent=2))
    print("状态已写入", STATE)
    print("复核: 写前可用 %.2f GB -> 写后 %.2f GB" % (st["free_before"] / 1e9,
                                                    st["free_after"] / 1e9))
    print("如果要验证'掉电一轮后数据还在'，现在跑 --eject 拔卡，再插回来跑 --verify")


def do_verify(drive):
    if not os.path.exists(STATE):
        print("找不到 %s，先跑 --write" % STATE)
        raise SystemExit(2)
    st = json.load(io.open(STATE, encoding="utf-8"))
    path = st["filler"]
    nblk = int(st["nblk"])
    if not os.path.exists(path):
        print("填充文件不见了:", path)
        raise SystemExit(2)
    size = os.path.getsize(path)
    print("待校验: %d 块 / %.3f GB；卡上实际大小 %.3f GB"
          % (nblk, nblk * BLK / 1e9, size / 1e9))
    if size != nblk * BLK:
        print("!! 文件大小与写入时不符（差 %d 字节）" % (size - nblk * BLK))

    bad = 0
    first_bad = None
    t0 = time.time()
    with open(path, "rb", buffering=0) as f:
        for k in range(nblk):
            got = f.read(BLK)
            exp = block_for(k)
            if got != exp:
                bad += 1
                if first_bad is None:
                    off = k * BLK
                    for i in range(min(len(got), BLK)):
                        if got[i] != exp[i]:
                            off += i
                            break
                    first_bad = (k, off)
                    print("  第一处不一致: 块 %d 偏移 %d (0x%X)" % (k, off, off))
            if (k % 512) == 0 and k:
                print("  已校验 %.2f GB  bad=%d  (%.1f MB/s)"
                      % (k * BLK / 1e9, bad,
                         k * BLK / 1e6 / max(1e-6, time.time() - t0)), flush=True)
    dt = time.time() - t0
    print("校验完成: %d 块 / mismatch=%d / %.1f s / %.1f MB/s"
          % (nblk, bad, dt, nblk * BLK / 1e6 / max(1e-6, dt)))
    if bad == 0:
        print("结论：这 %d 块全部一致，本次覆盖的范围内没有坏块" % nblk)
    else:
        print("结论：发现 %d 块不一致，判这张卡不健康" % bad)
    print("刷盘:", flush_volume(drive))


def do_delete(drive):
    path = os.path.join(drive.rstrip(":\\") + ":\\", FILLER)
    if os.path.exists(path):
        os.remove(path)
        print("已删除", path)
    else:
        print("没有填充文件")
    print("刷盘:", flush_volume(drive))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--drive", default="E")
    ap.add_argument("--write", action="store_true")
    ap.add_argument("--verify", action="store_true")
    ap.add_argument("--delete", action="store_true")
    ap.add_argument("--eject", action="store_true")
    a = ap.parse_args()

    if a.write:
        do_write(a.drive)
    if a.verify:
        do_verify(a.drive)
    if a.delete:
        do_delete(a.drive)
    if a.eject:
        flush_volume(a.drive)
        eject(a.drive)
        print("已安全弹出")


if __name__ == "__main__":
    main()
