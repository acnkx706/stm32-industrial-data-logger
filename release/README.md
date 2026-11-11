# Release v1.3（APP ver 77 / 镜像版本 8 + Boot 含 U3 暂存升级）

> 构建：2026-11-11 12:05 全量重编（App 含镜像版本元数据 **ver=8**）；Boot 沿用 2026-11-08 10:23 那版
> （含 U3 暂存 + U5 防回滚 + 空闲自救 + 暂存区在 `0x7E0000`、自愈路径补抬防回滚下限）
> 目标：STM32F103CBT6，Boot `0x08000000`（16 KB），App `0x08004000`（108 KB），参数区 `0x0801F000`+4 KB
> 固件横幅：`APP: ver 77 (2026-11-11 dsh: shutdown keeps tail batch + A1 diag)`
>
> ⚠️ **这一版为什么把镜像版本从 7 提到 8**：修了一个"关机丢最后一批"的缺陷并新增一条诊断命令。
> 板子用 ST-Link 烧时参数区不动，`img_min_ver` 仍是 6；
> 若之后走串口升级到 v8，下限会被抬到 8，届时**不能再用 ≤7 的旧 bin 升级**。

## 这一版改了什么（相对 v7，2026-11-11）

- **未挂载态关机丢最后一批（修）**：`task_sd.c` 的关机分支直接 `continue`，跳过了
  `Sd_DrainToCache()` / `Sd_DsFlush()`。卡不在位时按电源键关机，攒批缓冲 + 工作队列里的
  1~2 批（约 10~15 条）既不进 W25 也不进 SD —— 而 W25 缓存正是为无卡场景准备的。
  已在 `else`（未挂载）分支补这两句。串口提示同步改成
  `last batch cached in W25, card copy NOT saved`；屏幕仍报 `NOT SAVED`（卡上确实没有）。
  实测：断卡后连续关机 3 次，3 次都打出新提示。
- **新增串口命令 `A1` = OLED 运行期体检**：一次打印
  `idleSDA / idleSCL / puSDA / puSCL / ack88 / ack00 / ackAF / p3C / p3D`。
  用途：屏"看着亮但不更新"时分清是总线在丢还是面板自己黑。
  ⚠️ 判读要点：**必须先看开内上拉后的 `puSDA/puSCL`**——开漏松手直接读，
  "悬空"和"被拉死"都是 0，分不开；`p3C` 与 `p3D` 同时为 1 必然是异常。

## 上一版（v7 / ver 76，2026-11-10）改了什么

- **串口锁从来没生效（修）**：`Hardware/Serial.c` 用 `xSemaphoreCreateMutex()` 动态创建，
  而 `configTOTAL_HEAP_SIZE` 只有 64 B、控制块要约 80 B ⇒ 句柄恒为 NULL ⇒
  "整行原子打印"从未启用，多任务同时打印会互相插进字节流。
  改成 `xSemaphoreCreateMutexStatic(&s_ser_cb)`。实测：修前 18 秒内撕裂 2 次，
  修后 51,904 行 **0 次**。代价 ZI +80 B，Code −704 B。
- **FAT 文件时间戳写的是 UTC（修）**：`get_fattime()` 原来用 `MyRTC_GetDateTime()`（UTC），
  而文件名用本地日期 ⇒ 卡插到 PC 上文件名 `1110`、修改时间却是 `11-09 16:00`（差 8 小时）。
  改用 `MyRTC_GetLocalDateTime()`。实测修复后新写入文件的修改时间等于写入时刻。
  ⚠️ 卡上**旧文件的时间戳不会改变**。
- **装满刷不复位超时起点（修）**：`Sd_DsPush()` 装满 512 B 触发的刷没有清 `s_ds_t0`，
  下一条记录进来时起点仍是旧值 ⇒ 100 ms 后巡检判"已超 5 秒" ⇒ 单条成批，白占一个缓存槽。
  实测 17 次 `why=full` 全部紧跟一个 `n=1`。已补 `s_ds_t0 = 0u;`。
- **新增 `stall` 计数**：`SD: drained ... stall=N` 记录"一批不到 4 条"的次数，
  用于排查 2026-11-10 发现的"每批条数 3.67 vs 5.72"之谜（2026-11-11 复查后定性：
3.67 是唯一一次满环实测的口径，5.72 是同另一段 50 分钟窗口的分布，两者不能混用）。
- **串口命令 `M1` / `M0`**：运行期蜂鸣器静音开关（2026-11-10 加，台面调试用），
  状态存 `BKP_DR4`（备份域有电池）⇒ **复位 / 重烧都不丢**，开机打印
  `BUZZ: silent (restored from BKP_DR4)`。`M0` 恢复报警。
  ⚠️ 全片擦除（ST-Link Erase Full Chip）会把 DR4 一起清掉 ⇒ 恢复出厂"会响"。
- **`DISP: SLOW frame` 阈值 120 → 180 ms**：11-09 加多页显示 + 面板保活后，
  实测常态帧从 64~84 ms 涨到 117~124 ms，旧阈值落在噪声里（三分之一以上的帧都触发）。
  180 ms = 124 × 1.45。改后同负载 55 秒内触发 0 次（改前约 25 次）。

## 产物

| 文件 | 说明 | 大小 |
|---|---|---:|
| `APP.hex` | Intel HEX，基地址 `0x08004000` | 见文件 |
| `APP.bin` | 原始二进制（镜像版本 8） | 71,264 B |
| `APP_v5_20261107.bin` | **两版之前**的 v5（镜像版本 5），仅作历史对照；**不要拿它升级**，会被防回滚拒绝 | 69,956 B |
| `BOOT.hex` | Intel HEX，基地址 `0x08000000`，含 U3 暂存升级 | 32,480 B |
| `BOOT.bin` | 原始二进制，基地址 `0x08000000` | 11,528 B |
| `build.log` | App 构建，Code 66,976 / RO 4,208 / ZI 16,168，0 Error / 0 Warning | 见文件 |
| `build_boot.log` | Boot 构建，Code 11,184 / RO 296 / ZI 5,456，0 Error / 0 Warning | 847 B |
| `SHA256SUMS` | 上面几个文件的 SHA256 | — |

## 烧录

1. 先烧 `BOOT.hex` 到 `0x08000000`（16 KB 区），再烧 `APP.hex` 到 `0x08004000`。
2. 串口 115200 应看到 `APP: ver 77` 与 `APP: img ver=8 build=20261111`；
   `STAT` 里 `acq=proc`、`drop=0`。
3. 以后升级**不需要 ST-Link**：跑 `杂项文件\PC工具\flash_app.py`，走串口即可
   （串口升级会检查 `img_min_ver`；本版镜像版本 7，装完下限会被抬到 7，
   **之后不能再退回 5/6 的旧 bin**）。

## 校验

```powershell
Get-FileHash .\APP.hex, .\APP.bin, .\BOOT.hex, .\BOOT.bin, .\build.log, .\build_boot.log -Algorithm SHA256
Get-Content .\SHA256SUMS
```

两边的哈希应逐行一致。

## 边界

这版的能力与未做项见 `../docs/evidence-matrix.md` 和 `../docs/limitations.md`。
一句话：**拔卡/掉电之后还活着、丢多少、能恢复；不证明“永不丢/100 次 0 损坏”。**








