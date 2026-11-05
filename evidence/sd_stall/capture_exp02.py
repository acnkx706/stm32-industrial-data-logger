# capture_exp02.py -- 抓 EXP-02 测试输出
#
# 做了什么：烧录后连续读串口 SECS 秒，原样落盘
# 为什么：EXP02 行是逐 trial 打印的，必须完整保存原始 log 才能对账
# 影响什么：只读串口
# 怎么验证：日志里应有 EXP02,START 与 50 行 EXP02 数据行（5 档 x 10 次）
import sys
import time

import serial

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

OUT = sys.argv[1] if len(sys.argv) > 1 else r"exp02_stall8.log"
SECS = int(sys.argv[2]) if len(sys.argv) > 2 else 600
PORT = sys.argv[3] if len(sys.argv) > 3 else "COM12"

sp = serial.Serial(PORT, 115200, timeout=0.2)
f = open(OUT, "wb")
t0 = time.time()
n = 0
last = 0.0
while time.time() - t0 < SECS:
    chunk = sp.read(4096)
    if chunk:
        f.write(chunk)
        n += len(chunk)
    now = time.time() - t0
    if now - last >= 2.0:
        f.flush()
        last = now
f.close()
sp.close()
print("captured %d bytes -> %s" % (n, OUT))
