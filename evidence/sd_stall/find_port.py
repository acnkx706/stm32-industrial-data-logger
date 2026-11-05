# find_port.py -- 找当前接了 CH340 的那个串口
#
# 做了什么：列出系统串口，并从 PnP 设备里筛选出状态为 OK 的 CH340/USB-SERIAL，取其中的 COMx
# 为什么：换 USB 口后 COM 号会变，写死 COM12 会让抓包/烧录直接失败
# 影响什么：只读系统信息
# 怎么验证：板子上电后运行，应打印一个 COM 号；没接则打印 NOT_FOUND
import re
import subprocess
import sys

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

# 1) 常见端口
try:
    import serial.tools.list_ports as lp
    ports = list(lp.comports())
except Exception:
    ports = []

for p in ports:
    print("PORT %-8s %s" % (p.device, p.description))

# 2) PnP 里状态 OK 的 CH340
try:
    r = subprocess.run(
        ["powershell", "-NoProfile", "-Command",
         "Get-PnpDevice -Class Ports | Where-Object {$_.Status -eq 'OK'} | "
         "Select-Object -ExpandProperty FriendlyName"],
        capture_output=True, timeout=20)
    # 控制台默认 GBK，中文名可能解码失败，按替换处理
    out = r.stdout.decode("gbk", "replace")
except Exception as e:
    out = ""
    print("pnp query failed:", e)

found = []
for line in out.splitlines():
    m = re.search(r"\((COM\d+)\)", line)
    if m and ("CH340" in line or "USB-SERIAL" in line or "USB 串行" in line):
        found.append(m.group(1))

if found:
    print("FOUND", found[0])
else:
    print("NOT_FOUND")
