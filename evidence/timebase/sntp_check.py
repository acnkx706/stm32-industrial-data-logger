# sntp_check.py -- 量 PC 时钟相对 NTP 的偏差（不需要管理员）
#
# 做了什么：直接向几个 NTP 服务器发 SNTP 请求，算 PC 时钟与服务器时间的偏差（含往返补偿）。
# 为什么：`w32tm /resync` 要管理员权限；而 24h 实验里 entity 判定
#           "是 tick 变还是 RTC 变" 要靠 PC 当第三方时钟，PC 本身偏多少必须量出来。
#           本脚本**不改系统时钟**，只读数 —— 不扰动被测系统，两端各量一次还能看出 PC 的漂移。
# 影响什么：只发 UDP 包，不改任何配置。
# 怎么验证：多个服务器给出的一致值（差 <50 ms）即可信；把 rtt 大的丢掉。
#
# 用法：python sntp_check.py
import socket
import struct
import sys
import time

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

NTP_EPOCH = 2208988800          # 1900 -> 1970
SERVERS = ("time.windows.com", "pool.ntp.org", "ntp.aliyun.com", "time.google.com")


def query(host, timeout=3.0):
    pkt = b"\x1b" + 47 * b"\0"
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(timeout)
    t1 = time.time()
    try:
        s.sendto(pkt, (host, 123))
        data, _ = s.recvfrom(1024)
        t4 = time.time()
    except Exception as e:
        s.close()
        return None, str(e)
    s.close()
    if len(data) < 48:
        return None, "短包"
    u = struct.unpack("!12I", data[:48])
    server = (u[10] - NTP_EPOCH) + u[11] / 2.0 ** 32     # transmit timestamp
    rtt = t4 - t1
    return server - (t1 + rtt / 2.0), rtt


def main():
    good = []
    worst_rtt = 0.0
    for host in SERVERS:
        off, info = query(host)
        if off is None:
            print("%-20s 失败: %s" % (host, info))
            continue
        print("%-20s offset = %+.3f s   rtt = %.3f s" % (host, off, info))
        if info < 0.2:
            good.append(off)
            worst_rtt = max(worst_rtt, info)
    if not good:
        print("没有任何可用服务器")
        return
    mean = sum(good) / len(good)
    unc = worst_rtt / 2.0
    print("\n采用 %d 个（rtt<0.2s）均值得：%+.4f s" % (len(good), mean))
    print("定义：offset = 服务器时间 − PC 时间")
    print("含义：PC 时钟比真实时间%s %.4f s" % ("快" if mean < 0 else "慢", abs(mean)))
    print("修正公式：真实时刻 = pc_epoch + (%.4f)" % mean)
    print("不确定度：约 ±rtt/2 = ±%.0f ms；24h 上折合 ±%.1f ppm"
          % (unc * 1000, unc / 86400 * 1e6))
    print("PC epoch now = %d" % int(time.time()))


if __name__ == "__main__":
    main()
