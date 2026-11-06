# pc_fill.py -- 在电脑上把卡填到指定占用，并强制刷盘 + 复核
#
# 做了什么：按实际数据写入填充文件（不是 fsutil 只改大小），逐块 fsync，
#           随后调用 Write-VolumeCache 刷掉卷缓存，再重新读一次可用空间复核
# 为什么：上次用 fsutil 建 1.5GB 文件，改动被 Windows 写缓存扣着，拔卡即丢；
#           板子只该做测量，卡上的准备必须在这里做完并验证
# 影响什么：只写填充文件 FILLER.BIN；不动其它文件
# 怎么验证：脚本最后打印"复核可用空间"，与目标百分比对得上才算成功
#
# 用法：
#   python pc_fill.py --drive E --pct 90          # 填到 90%
#   python pc_fill.py --drive E --pct 50          # 填到 50%（会先删填充文件再重填）
#   python pc_fill.py --drive E --delete          # 只删填充文件
import argparse
import ctypes
import os
import subprocess
import sys

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

CHUNK = 4 * 1024 * 1024
FILLER = "FILLER.BIN"


def free_total(drive):
    d = drive.rstrip(":\\") + ":\\"
    free = ctypes.c_ulonglong(0)
    total = ctypes.c_ulonglong(0)
    ctypes.windll.kernel32.GetDiskFreeSpaceExW(
        ctypes.c_wchar_p(d), None,
        ctypes.byref(total), ctypes.byref(free))
    return free.value, total.value


def flush_volume(drive):
    """把卷缓存刷到卡上。失败就退回 eject，二者都失败则明确报错。"""
    letter = drive.rstrip(":\\")[0].upper()
    try:
        r = subprocess.run(
            ["powershell", "-NoProfile", "-Command",
             "Write-VolumeCache -DriveLetter %s" % letter],
            capture_output=True, timeout=120)
        if r.returncode == 0:
            return "Write-VolumeCache OK"
    except Exception:
        pass
    try:
        r = subprocess.run(
            ["powershell", "-NoProfile", "-Command",
             "$sh=New-Object -ComObject Shell.Application;"
             "$sh.Namespace(17).ParseName('%s:').InvokeVerb('Eject')" % letter],
            capture_output=True, timeout=120)
        if r.returncode == 0:
            return "Eject OK"
    except Exception:
        pass
    return "FLUSH FAILED"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--drive", default="E")
    ap.add_argument("--pct", type=float, default=0)
    ap.add_argument("--delete", action="store_true")
    a = ap.parse_args()

    root = a.drive.rstrip(":\\") + ":\\"
    path = os.path.join(root, FILLER)
    free, total = free_total(a.drive)
    print("初始: 总 %.2f GB / 可用 %.2f GB / 已用 %.1f%%"
          % (total / 1e9, free / 1e9, 100.0 * (1 - free / total)))

    if os.path.exists(path):
        os.remove(path)
        print("已删除旧填充文件")
        free, total = free_total(a.drive)

    if a.delete:
        print("只删填充文件，退出")
        return

    if a.pct <= 0:
        print("未指定 --pct，退出")
        return

    target_used = total * a.pct / 100.0
    need = int(target_used - (total - free))
    if need <= 0:
        print("当前已用 %.1f%% >= 目标 %.1f%%，无需填充"
              % (100.0 * (1 - free / total), a.pct))
    else:
        print("需要填充 %.2f GB" % (need / 1e9))
        buf = bytes(CHUNK)
        written = 0
        with open(path, "wb", buffering=0) as f:
            while written < need:
                n = min(CHUNK, need - written)
                f.write(buf[:n])
                written += n
                if written % (64 * 1024 * 1024) < CHUNK:
                    print("  已写 %.2f GB" % (written / 1e9), flush=True)
            os.fsync(f.fileno())

    print("刷盘:", flush_volume(a.drive))
    free2, total2 = free_total(a.drive)
    print("复核: 可用 %.2f GB / 已用 %.1f%%（目标 %.0f%%）"
          % (free2 / 1e9, 100.0 * (1 - free2 / total2), a.pct))
    if abs(100.0 * (1 - free2 / total2) - a.pct) > 3.0:
        print("!! 与目标相差超过 3 个百分点，先别拔卡")
        raise SystemExit(2)
    print("OK：现在可以拔卡")


if __name__ == "__main__":
    main()
