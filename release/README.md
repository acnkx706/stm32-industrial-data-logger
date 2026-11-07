# Release v1.0（APP ver 74 + Boot 含 U3 暂存升级）

> 构建：2026-11-07 16:02 全量重编 App（含 BMP280 冷启动 / SD 重试预算 / I2C 锁范围修复）；Boot 13:44 构建（含 U3 暂存升级）
> 目标：STM32F103CBT6，Boot `0x08000000`（16 KB），App `0x08004000`（108 KB），参数区 `0x0801F000`+4 KB
> 固件横幅：`APP: ver 74 (2026-11-05 dsh: SD slow-write forensics)`

## 产物

| 文件 | 说明 | 大小 |
|---|---|---:|
| `APP.hex` | Intel HEX，基地址 `0x08004000`，可直接用 ST-Link Utility / Keil 烧 | 196,649 B |
| `APP.bin` | 原始二进制，与 `Objects/app.bin` 逐字节一致 | 69,892 B |
| `BOOT.hex` | Intel HEX，基地址 `0x08000000`，含 U3 暂存升级 | 30,876 B |
| `BOOT.bin` | 原始二进制，基地址 `0x08000000` | 10,960 B |
| `build.log` | App 全量构建，Code 65,888 / RO 3,932 / ZI 16,092，0 Error / 0 Warning；已脱敏 | 1,885 B |
| `SHA256SUMS` | 上面五个文件的 SHA256 | — |

## 烧录

1. 先烧 `BOOT.hex` 到 `0x08000000`（16 KB 区），再烧 `APP.hex` 到 `0x08004000`。
2. 串口 115200 应看到 `APP: ver 74`；`STAT` 里 `acq=proc`、`drop=0`。
3. 以后升级**不需要 ST-Link**：跑 `杂项文件\PC工具\flash_app.py`，走串口即可。

## 校验

```powershell
Get-FileHash .\APP.hex, .\APP.bin, .\BOOT.hex, .\BOOT.bin, .\build.log -Algorithm SHA256
Get-Content .\SHA256SUMS
```

两边的哈希应逐行一致。

## 边界

这版的能力与未做项见 `../docs/evidence-matrix.md` 和 `../docs/limitations.md`。
一句话：**拔卡/掉电之后还活着、丢多少、能恢复；不证明“永不丢/100 次 0 损坏”。**





