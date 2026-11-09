# Release v1.1（APP ver 75 / 镜像版本 6 + Boot 含 U3 暂存升级）

> 构建：2026-11-09 14:5x 全量重编（App 含镜像版本元数据 **ver=6**）；Boot 沿用 2026-11-08 10:23 那版
> （含 U3 暂存 + U5 防回滚 + 空闲自救 + 暂存区在 `0x7E0000`、自愈路径补抬防回滚下限）
> 目标：STM32F103CBT6，Boot `0x08000000`（16 KB），App `0x08004000`（108 KB），参数区 `0x0801F000`+4 KB
> 固件横幅：`APP: ver 75 (2026-11-09 dsh: SD loop fix + keys + OLED pages)`
>
> ⚠️ **这一版为什么把镜像版本从 5 提到 6**：2026-11-08 的实验把这块板子的
> `img_min_ver` 抬到了 6（`limitations.md` 第 37 条）。源码若仍是 5，
> 串口升级会被 `FAIL downgrade` 拦住 ⇒ 源码 / 板子 / 发布物三者必须一致。
> 旧的 v5 产物留在 `APP_v5_20261107.bin`，只作历史对照，**不要再用它走串口升级**。

## 这一版改了什么（相对 v5）

- **修死循环**：`SD_Init()` 的 CMD0 重试循环变量是 `uint8_t`，上限 300 ⇒ 永不退出，
  无卡时整机静默。改 `uint16_t` 后实测 `CMD0 NO RESPONSE after 300 tries` 正常返回。
- **重试退避**：无卡时重挂从"每 0.65 s 一次全量诊断"改成 24 轮 + 关键行，间隔 3.2 s；
  实测 `drop` 从 235 降到 2。
- **按键改造**：PB1 = 串口回显开关（关掉时连 IDLE/SONAR/DISP/周期 STAT 一起静默，
  手动统计 `STAT(key)` 仍可出）；PA0 短按 = 翻页，长按 = 安全关机；暂停功能删除。
- **OLED 多页**：数据 / 存储（文件名·大小·W25 待回写批数·剩余空间）/ 运行（队列与 I2C）。
- **OLED 保活**：每 5 秒重申一次显示配置，救"面板被复位后自动黑屏"；
  新增串口命令 `L1` = 整屏重新初始化。

## 产物

| 文件 | 说明 | 大小 |
|---|---|---:|
| `APP.hex` | Intel HEX，基地址 `0x08004000` | 见文件 |
| `APP.bin` | 原始二进制（镜像版本 6） | 71,216 B |
| `APP_v5_20261107.bin` | 上一版 v5 的二进制，仅作对照 | 69,956 B |
| `BOOT.hex` | Intel HEX，基地址 `0x08000000`，含 U3 暂存升级 | 32,480 B |
| `BOOT.bin` | 原始二进制，基地址 `0x08000000` | 11,528 B |
| `build.log` | App 构建，Code 67,188 / RO 3,948 / ZI 16,092，0 Error / 0 Warning | 见文件 |
| `build_boot.log` | Boot 构建，Code 11,184 / RO 296 / ZI 5,456，0 Error / 0 Warning | 847 B |
| `SHA256SUMS` | 上面几个文件的 SHA256 | — |

## 烧录

1. 先烧 `BOOT.hex` 到 `0x08000000`（16 KB 区），再烧 `APP.hex` 到 `0x08004000`。
2. 串口 115200 应看到 `APP: ver 75` 与 `APP: img ver=6 build=20261109`；
   `STAT` 里 `acq=proc`、`drop=0`。
3. 以后升级**不需要 ST-Link**：跑 `杂项文件\PC工具\flash_app.py`，走串口即可
   （串口升级会检查 `img_min_ver`，所以必须装 ≥6 的镜像）。

## 校验

```powershell
Get-FileHash .\APP.hex, .\APP.bin, .\BOOT.hex, .\BOOT.bin, .\build.log, .\build_boot.log -Algorithm SHA256
Get-Content .\SHA256SUMS
```

两边的哈希应逐行一致。

## 边界

这版的能力与未做项见 `../docs/evidence-matrix.md` 和 `../docs/limitations.md`。
一句话：**拔卡/掉电之后还活着、丢多少、能恢复；不证明“永不丢/100 次 0 损坏”。**








