# Release v1.0（APP ver 74）

> 构建：2026-11-06 13:58（未重编；hex 由同一次构建的 `Objects/Project.axf` 转出）
> 目标：STM32F103CBT6，App 基地址 `0x08004000`，App 区 112 KB
> 固件横幅：`APP: ver 74 (2026-11-05 dsh: SD slow-write forensics)`

## 产物

| 文件 | 说明 | 大小 |
|---|---|---:|
| `APP.hex` | Intel HEX，基地址 `0x08004000`，可直接用 ST-Link Utility / Keil 烧 | 193,139 B |
| `APP.bin` | 原始二进制，与 `Objects/app.bin` 逐字节一致 | 68,644 B |
| `build.log` | 2026-11-06 13:58 生产构建，0 Error / 0 Warning；已脱敏 | 1,858 B |
| `SHA256SUMS` | 上面三个文件的 SHA256 | — |

## 烧录

1. 先确认 Bootloader 已在 `0x08000000`（16 KB），App 区从 `0x08004000` 开始。
2. 用 ST-Link Utility / Keil 打开 `APP.hex`，地址应为 `0x08004000`，烧录后复位。
3. 串口 115200 应看到 `APP: ver 74`；`STAT` 里 `acq=proc`、`drop=0`。

## 校验

```powershell
Get-FileHash .\APP.hex, .\APP.bin -Algorithm SHA256
Get-Content .\SHA256SUMS
```

两边的哈希应逐行一致。

## 边界

这版的能力与未做项见 `../docs/evidence-matrix.md` 和 `../docs/limitations.md`。
一句话：**拔卡/掉电之后还活着、丢多少、能恢复；不证明“永不丢/100 次 0 损坏”。**
