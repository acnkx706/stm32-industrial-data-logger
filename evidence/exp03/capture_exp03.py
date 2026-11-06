# capture_exp03.py -- 抓 EXP-03 串口，带断线重连与人工事件标记
#
# 做了什么：持续读串口原样落盘；串口断开（掉电那一瞬间 CH340 也会掉）就每隔 0.3 s 重试，
#           重连成功时往日志里写一行 `#CAP,<epoch>,reconnect`；
#           外部随时可以触发一行标记 `#CAP,<epoch>,<文字>`，用来对操作表。
# 为什么：EXP-03 的 C 组要断电 20 次，如果板子和 CH340 共用一根 USB，
#           每断一次 COM12 就没了。capture_exp02.py 一遇异常就整段抓包作废，
#           这里必须能自己爬起来，并且把"断在哪、什么时候回来的"记进同一份日志。
# 影响什么：只读串口；标记行以 `#CAP,` 开头，不影响 `REC,` / `STAT` / `SD:` 的解析。
# 怎么验证：拔掉板子电源 3 秒再上电，日志里应出现 reconnect 标记且前后数据都在。
#
# 用法：
#   python capture_exp03.py exp03_C.log 1800 COM12
#   python capture_exp03.py exp03_C.log 1800 COM12 --trigger _mark.txt --stop _stop.txt
#
# 打标记（另开一个窗口，或由主控脚本调用）：
#   Set-Content _mark.txt "pull A1   1793951900"
#   -- 内容有变化就算一次标记；capture 进程会在 0.1 s 内把它写进日志
import os
import sys
import time

import serial

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

OUT = sys.argv[1] if len(sys.argv) > 1 else "exp03.log"
SECS = int(sys.argv[2]) if len(sys.argv) > 2 else 1800
PORT = sys.argv[3] if len(sys.argv) > 3 else "COM12"

TRIG = None
STOP = None
for i, a in enumerate(sys.argv):
    if a == "--trigger" and i + 1 < len(sys.argv):
        TRIG = sys.argv[i + 1]
    if a == "--stop" and i + 1 < len(sys.argv):
        STOP = sys.argv[i + 1]

f = open(OUT, "wb")
t_start = time.time()
total = 0
gaps = []            # [(丢失起点 epoch, 恢复 epoch)]
at_line_start = True


def emit_marker(text, epoch=None):
    """把标记写成独立一行。先补齐换行，避免插进一条半截的 REC 行里。"""
    global at_line_start
    if epoch is None:
        epoch = int(time.time())
    prefix = b"" if at_line_start else b"\r\n"
    f.write(prefix + ("#CAP,%d,%s\r\n" % (epoch, text)).encode("utf-8"))
    f.flush()
    at_line_start = True


def pump(sp):
    global total, at_line_start
    chunk = sp.read(4096)
    if chunk:
        f.write(chunk)
        total += len(chunk)
        at_line_start = chunk.endswith(b"\n")
    return len(chunk)


emit_marker("start port=%s secs=%d" % (PORT, SECS))
sp = None
last_flush = 0.0
last_trig = None
down_since = None
next_try = 0.0

while time.time() - t_start < SECS:
    now = time.time()

    if STOP and os.path.exists(STOP):
        emit_marker("stop file")
        break

    # 人工标记。格式两种：
    #   "<epoch> <文字>"  -> 用调用方给的 epoch（推荐：epoch 在"动手那一刻"生成，
    #                        本进程什么时候读到的都不影响记录的时刻）
    #   "<文字>"          -> 用本进程读到它的时刻
    if TRIG and os.path.exists(TRIG):
        try:
            cur = open(TRIG, encoding="utf-8").read().strip()
        except OSError:
            cur = None
        if cur and cur != last_trig:
            last_trig = cur
            body = cur.lstrip("#").strip()
            head = body.split(" ", 1)
            if len(head) == 2 and head[0].isdigit():
                emit_marker(head[1], epoch=int(head[0]))
            else:
                emit_marker(body)
            try:
                os.remove(TRIG)
            except OSError:
                pass

    # 串口
    if sp is None:
        if now >= next_try:
            try:
                sp = serial.Serial(PORT, 115200, timeout=0.2)
                if down_since is not None:
                    gaps.append((down_since, int(now)))
                    emit_marker("reconnect after %.1fs" % (now - down_since))
                    down_since = None
            except Exception as e:
                sp = None
                if down_since is None:
                    down_since = now
                    emit_marker("port lost: %s" % str(e)[:60].replace("\n", " "))
                next_try = now + 0.3
                time.sleep(0.1)
                continue
        else:
            time.sleep(0.05)
            continue

    try:
        pump(sp)
    except Exception as e:
        try:
            sp.close()
        except Exception:
            pass
        sp = None
        next_try = time.time() + 0.3
        emit_marker("port error: %s" % str(e)[:60].replace("\n", " "))

    if now - last_flush >= 2.0:
        f.flush()
        last_flush = now

if sp is not None:
    try:
        sp.close()
    except Exception:
        pass

emit_marker("end bytes=%d gaps=%d" % (total, len(gaps)))
for a, b in gaps:
    print("断线区间: %d -> %d (%.1f s)" % (a, b, b - a))
f.close()
print("captured %d bytes -> %s（断线 %d 次）" % (total, OUT, len(gaps)))
