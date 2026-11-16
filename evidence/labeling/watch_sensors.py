#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
实时监视记录仪的几路传感器 —— 用来"找元件"

做什么
    把串口 REC 行里的 light / humi / temp_ds / sonar 抽出来，就地刷新显示。
    你拿手指逐个遮住板上的元件，哪一路数值动了，那个元件就是这个传感器。

为什么
    · 固件已经把值打在串口上了，不用万用表、不用改代码
    · 光敏电阻和 DHT22 外观很像，靠猜容易遮错

用法
    python watch_sensors.py
    python watch_sensors.py --port COM12

按 Ctrl+C 退出。

判断标准
    光敏电阻：遮住 → light 从 ~2300 掉到 500 以下（掉到 1000 以上都算没遮准）
    DHT22   ：哈气 3 秒 → 之后 10~60 秒内 humi 上升 2%RH 以上
              （它是慢传感器，当场不会动，要等一会儿）
"""

import argparse
import re
import sys
import time

try:
    import serial
except ImportError:
    print("缺少 pyserial：pip install pyserial")
    sys.exit(2)

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

FIELDS = ["light", "humi", "temp_ds", "sonar_mm", "vbat", "press"]
PAT = {f: re.compile(f + r"<(-?\d+)>") for f in FIELDS}

# 上一次的值，用来箭头提示变化方向
last = {}


def fmt(v, f):
    if v is None:
        return "   --   "
    if f == "humi":
        return "%6.1f%%" % (v / 10.0)
    if f == "temp_ds":
        return "%6.2fC" % (v / 100.0)
    if f == "light":
        return "%6d " % v
    if f == "sonar_mm":
        return "%6dmm" % v
    if f == "vbat":
        return "%6d " % v
    return "%6d " % v


def arrow(v, f):
    if v is None:
        return " "
    p = last.get(f)
    if p is None:
        return " "
    d = v - p
    if d > 0:
        return "↑"
    if d < 0:
        return "↓"
    return " "


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM12")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=0,
                    help="跑这么多秒后自动退出（0 = 一直跑，Ctrl+C 退出）")
    a = ap.parse_args()

    try:
        ser = serial.Serial(a.port, a.baud, timeout=0.05)
    except Exception as e:
        print(f"!! 打不开 {a.port}：{e}")
        print("   Keil 的 UART 窗口 / 串口助手 / 别的脚本是不是还开着？")
        return 2

    print(f"{a.port} @ {a.baud} 已打开")
    print("拿手指逐个遮住板上的元件，看哪一路数值跟着动\n")
    print("  light(PA1)    humi(DHT22)   temp(DS18B20)  sonar     vbat      press")
    print("  " + "-" * 72)

    buf = b""
    t_start = time.time()
    try:
        while True:
            if a.seconds > 0 and time.time() - t_start > a.seconds:
                break
            chunk = ser.read(512)
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                s = line.decode("latin-1", "replace")
                if not s.startswith("REC,"):
                    continue
                vals = {}
                for f in FIELDS:
                    m = PAT[f].search(s)
                    vals[f] = int(m.group(1)) if m else None
                row = " ".join(
                    fmt(vals[f], f) + arrow(vals[f], f) for f in FIELDS)
                print("\r  " + row, end="", flush=True)
                for f in FIELDS:
                    if vals[f] is not None:
                        last[f] = vals[f]
    except KeyboardInterrupt:
        print("\n\n退出")
    finally:
        ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

