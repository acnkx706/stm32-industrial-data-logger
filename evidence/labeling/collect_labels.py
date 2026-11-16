#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
采集带标签的异常样本 —— 串口抓 tick，自动和数据对齐

做了什么
    按屏幕/蜂鸣器提示做 4 类动作，脚本在一旁读串口的 REC 行，
    把每次动作的起止 tick 记进 labels.csv。

为什么用串口抓 tick 而不是靠手机时间
    · tick 是固件自己的毫秒计数，10Hz 采样 ⇒ 每 100ms 前进一次
    · 手机时间和板子 RTC 之间有几十秒误差，事后对不上
    · 用 tick 当标签的锚点，标签和数据是同一把尺子
    · 数据侧：SD 卡里的 CSV 每一行都有 tick 列 ⇒ 直接 join

影响什么
    · 只读串口，不改固件、不碰 SD 卡
    · 唯一的动作要求：SD 卡必须插在板子上（否则没有原始数据可标）

怎么验证
    · 采完看 labels.csv 的 tick_start/tick_end 是否单调递增
    · 拿 CSV 里 tick 落在区间内的行，数一下每类采了多少个样本

用法
    python collect_labels.py                 # 完整采集（约 10 分钟）
    python collect_labels.py --port COM12    # 指定串口
    python collect_labels.py --dry-run       # 只演练提示，不连串口
    python collect_labels.py --class humi_up # 只采某一类

★ 蜂鸣器含义（Windows 内置，无需硬件）
    · 高音两下        即将开始（准备）
    · 高音一长声      开始做动作
    · 低音一短声      停
    · 三连短音        这一类采完了
"""

import argparse
import csv
import os
import re
import sys
import threading
import time

try:
    import serial
except ImportError:
    print("缺少 pyserial：pip install pyserial")
    sys.exit(2)

try:
    import winsound
except ImportError:
    winsound = None

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

HERE = os.path.dirname(os.path.abspath(__file__))
LABELS_CSV = os.path.join(HERE, "labels.csv")

TICK_RE = re.compile(r"tick<(\d+)>")
RTC_RE = re.compile(r"rtc<(\d+)>")


# ============================================================================
#  提示音
# ============================================================================
def beep(kind):
    """kind: ready / go / stop / done"""
    if winsound is None:
        return
    try:
        if kind == "ready":
            winsound.Beep(1200, 120); winsound.Beep(1200, 120)
        elif kind == "go":
            winsound.Beep(1600, 400)
        elif kind == "stop":
            winsound.Beep(600, 250)
        elif kind == "done":
            for _ in range(3):
                winsound.Beep(1400, 90); time.sleep(0.06)
    except Exception:
        pass


# ============================================================================
#  串口后台读取线程：持续刷新最新 tick / rtc
# ============================================================================
class Reader(threading.Thread):
    def __init__(self, ser):
        super().__init__(daemon=True)
        self.ser = ser
        self.tick = None
        self.rtc = None
        self.lock = threading.Lock()
        self.alive = True
        self.sessions = 0          # 检测到 tick 回退 = 板子重启过

    def run(self):
        buf = b""
        while self.alive:
            try:
                chunk = self.ser.read(512)
            except Exception:
                break
            if chunk:
                buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                s = line.decode("latin-1", "replace")
                m = TICK_RE.search(s)
                if not m:
                    continue
                n = int(m.group(1))
                r = RTC_RE.search(s)
                with self.lock:
                    if self.tick is not None and n < self.tick:
                        self.sessions += 1
                    self.tick = n
                    if r:
                        self.rtc = int(r.group(1))

    def now(self):
        with self.lock:
            return self.tick, self.rtc

    def wait_fresh(self, timeout=3.0):
        """等到至少出现一次 tick 更新，说明串口活着。"""
        t0 = time.time()
        first = self.now()[0]
        while time.time() - t0 < timeout:
            cur = self.now()[0]
            if cur is not None and cur != first:
                return True
            time.sleep(0.05)
        return False

    def stop(self):
        self.alive = False


# ============================================================================
#  采集计划
# ============================================================================
PLAN = [
    # (label, action_s, rest_s, reps, 说明)
    ("normal",     60, 0,  1,  "保持不动，正常坐着就行"),
    ("humi_up",     3, 7, 15,  "对着 DHT22 哈气（就是白边那个网罩）"),
    ("light_down",  3, 3, 20,  "用手掌完全遮住光敏电阻"),
    ("shake",       2, 3, 20,  "拍桌面 / 轻敲板子边缘"),
    ("tilt",        2, 3, 20,  "把板子翻过来再放回去"),
    # ★ 2026-11-13 加：慢变信号必须用块状采集
    #   为什么：光敏被遮是"状态"，持续几十秒；
    #           3 秒脉冲式采集会让整段都处在同一状态，段内段前无对比
    #   影响什么：每块 20 秒，5 块 = 5 个独立样本，可各切 2 个 10 秒窗口
    ("light_block", 20, 20, 5, "用手掌完全遮住光敏电阻，遮住后保持不动"),
    # ★ 2026-11-13 加：直接碰板子（和"拍桌子"是两个物理事件）
    #   为什么分开采：拍桌子 = 振动经桌面传导，能量损耗大（实测 0.03g）；
    #               碰板子 = 直接冲击，预计大 10~30 倍
    #   影响什么：多一类 impact_direct，用来验证模型能否区分
    #             "振动传入" 和 "直接冲击"
    ("tap_board",   2, 3, 20, "用食指直接碰板子边缘（碰到板子，不是拍桌子）"),
]

# ★ 块状采集：动作段 20 秒，休息段也 20 秒 —— 足够把遮/松两次过渡都采到
BLOCK_CLASSES = ("light_block",)

FIELDNAMES = ["label", "rep", "tick_start", "tick_end", "rtc_start", "rtc_end", "note"]


def append_row(row):
    """立刻落盘：中途崩了也只丢最后一次。"""
    new = not os.path.isfile(LABELS_CSV)
    with open(LABELS_CSV, "a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=FIELDNAMES)
        if new:
            w.writeheader()
        w.writerow(row)
        f.flush()
        os.fsync(f.fileno())


def countdown(sec, label):
    for i in range(sec, 0, -1):
        print(f"\r  {label}  准备… {i} ", end="", flush=True)
        time.sleep(1)
    print("\r" + " " * 40 + "\r", end="")


def do_one(rd, label, action_s, note, rep, total, dry):
    t0, r0 = rd.now()
    beep("go")
    print(f"\r  ▶ [{rep}/{total}] 开始   tick={t0}", end="", flush=True)
    time.sleep(action_s)
    t1, r1 = rd.now()
    beep("stop")
    print(f"\r  ✓ [{rep}/{total}] 录音区间 tick {t0} → {t1}  "
          f"({(t1 - t0) if (t0 and t1) else 0} ms)          ", flush=True)
    if dry:
        return
    if t0 is None or t1 is None:
        print("     !! 没拿到 tick —— 板子没在发 REC 行？")
        return
    append_row({
        "label": label, "rep": rep,
        "tick_start": t0, "tick_end": t1,
        "rtc_start": r0 if r0 is not None else "",
        "rtc_end": r1 if r1 is not None else "",
        "note": note,
    })


def run_class(rd, label, action_s, rest_s, reps, desc, dry):
    print("\n" + "=" * 66)
    print(f"  类别：{label}      {reps} 次 × ({action_s}s 动作 + {rest_s}s 休息)")
    print(f"  动作：{desc}")
    print("=" * 66)
    input("  按回车开始这一类（先把动作准备好）...")
    beep("ready")
    time.sleep(0.6)
    for rep in range(1, reps + 1):
        do_one(rd, label, action_s, desc, rep, reps, dry)
        if rep < reps and rest_s:
            time.sleep(rest_s)
    beep("done")
    print(f"\n  ★ {label} 采完了")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM12")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--dry-run", action="store_true", help="不连串口，只走流程")
    ap.add_argument("--class", dest="only", default=None, help="只采这些类（逗号分隔）")
    a = ap.parse_args()

    ser = rd = None
    if not a.dry_run:
        print(f"打开 {a.port} @ {a.baud} ...")
        try:
            ser = serial.Serial(a.port, a.baud, timeout=0.05)
        except Exception as e:
            print(f"!! 打不开 {a.port}：{e}")
            print("   · Keil 的 UART 窗口 / 串口助手 / 别的脚本是不是还开着？")
            return 2
        rd = Reader(ser)
        rd.start()
        if not rd.wait_fresh(5.0):
            print("!! 5 秒内没看到板的 REC 行")
            print("   · 板子有没有通电？")
            print("   · 采集安静的时候 App 会不会不打 REC 了？")
            rd.stop(); ser.close()
            return 3
        print(f"串口 OK，当前 tick={rd.now()[0]}")
    else:
        print("[dry-run] 不连串口，tick 全是 None")

    if os.path.isfile(LABELS_CSV):
        ans = input(f"\n{LABELS_CSV} 已存在。追加还是覆盖？[a=追加 / o=覆盖] ")
        if ans.strip().lower().startswith("o"):
            os.remove(LABELS_CSV)
            print("已删除旧文件")

    print("\n" + "#" * 66)
    print("#  采集前确认三件事：")
    print("#    1. SD 卡插在板子上（否则没有原始数据可以标）")
    print("#    2. 板子放稳，别被线拽动")
    print("#    3. 关闭 Keil 的 UART 窗口和串口助手")
    print("#")
    print("#  提示音：高音长响=开始做   低音短响=停   三连响=这类完成")
    print("#" * 66)
    input("\n  都确认了，按回车开始总流程 ...")

    only_set = None
    if a.only:
        only_set = set(x.strip() for x in a.only.split(",") if x.strip())
        print(f"\n只采这些类：{sorted(only_set)}")
    for label, action_s, rest_s, reps, desc in PLAN:
        if only_set is not None and label not in only_set:
            continue
        run_class(rd, label, action_s, rest_s, reps, desc, a.dry_run)

    if rd:
        adds = rd.sessions
        rd.stop()
        time.sleep(0.2)
        ser.close()
        if adds:
            print(f"\n!! 采集中检测到 {adds} 次 tick 回退 —— 板子重启过，标签可能错位")

    print("\n" + "=" * 66)
    print("  全部采完")
    if os.path.isfile(LABELS_CSV):
        with open(LABELS_CSV, encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        from collections import Counter
        c = Counter(r["label"] for r in rows)
        print(f"  {LABELS_CSV}")
        for k, v in c.items():
            print(f"    {k:<12} {v} 段")
    print("  下一步：把 SD 卡给我，我按 tick 把数据切出来")
    print("=" * 66)
    return 0


if __name__ == "__main__":
    sys.exit(main())

