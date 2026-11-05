# EXP-02 执行记录（2026-11-06 凌晨完成）

> 状态：**已完成单卡最小版本**（方案 §12 的三件事全部做到，延迟样本还超额到 11k）
> 板子已恢复正式固件 `ver 74`，测试开关全 0，跑 20 秒确认 `drop=0 / sd_err=0 / 无 EXP02 输出`。

---

## 一、做了什么

| 步骤 | 结果 |
|---|---|
| 卡上 39 个条目完整备份 | ✅ `项目1：工业记录仪\_card_backup_20261106\`（38 文件 / 1,364,587,953 B） |
| 固件测试钩子 | ✅ 延迟直方图、top-32、首丢钩子、SD/PROC 停摆注入，全部 `#if TEST_EXP02` |
| SD 停摆矩阵（队列 8） | ✅ 50 次（49 条有效） |
| SD 停摆矩阵（队列 4） | ✅ 50 次（46 条有效） |
| PROC 停摆控制组（队列 8） | ✅ 40 次（37 条有效） |
| 延迟分布 | ✅ **11,000 次 `f_write` + 11,000 次 `f_sync`** |
| 恢复正式固件 | ✅ 开关全 0，`Code=64,600`，`STAT` 正常 |

## 二、三条结论

1. **H2 成立**：SD 停摆 1 s 时 10/10 次先涨 `LOSS_NO_BUF`，`Δqueue_full = 0`。
2. **H3 成立**：队列 8 vs 4 的第一计数器相同，`first_loss_ms` 均值 789 vs 772 ms，
   `qmax` 都是 1 —— 队列深度不是这条链的瓶颈，缓冲才是。
3. **控制组对照干净**：停 PROC 1 s 时 10/10 次先涨 `LOSS_Q_FULL`，`qmax` 冲到 8。
4. **250 ms 假设偏保守**：11,000 次写实测 max 126 ms、≥250 ms 零次；
   停摆实验里 250 ms 单次停摆 19/19 次零丢失，临界点在 500 ms 附近。

详见 [`stall_report.md`](../sd_stall/stall_report.md) 与 [`latency_report.md`](../sd_latency/latency_report.md)。

## 三、过程中踩的坑（都已修）

| # | 坑 | 后果 | 修法 |
|---|---|---|---|
| 1 | `MakeTask` 不认识 `Exp02_Task` | 任务建不起来；跨 `NULL` 句柄还会直接崩溃 → Boot 判定 App 无效并回退金副本 | 补静态栈/TCB 与映射，传真实句柄 |
| 2 | `#include "exp02.c"` 放在 `REC_QUEUE_LEN` / `Stat_t` 之前 | 测试版编译不过 | 挪到 `Stat_t g_stat;` 之后 |
| 3 | 探针开关块被插入两次 | 宏重定义 | 去重 |
| 4 | 测试固件会把 SD 错误接蜂鸣器 | 夜里会响 | `#if TEST_EXP02` 里屏蔽 SD 错误蜂鸣 |
| 5 | 串口 `REC,` 行与 `EXP02`/`LAT` 行交叠 | 每 50 次丢 1~4 条记录 | 解析器改严格正则，交叠行丢弃并计数 |
| 6 | 抓包到点停止时正巧切在 dump 中间 | 直方图桶和比样本数少 121 | 续抓一段，取完整块（11,000 次，桶和吻合） |
| 7 | USB 串口中途掉线（PermissionError 13） | 第一次矩阵只抓到 15 次 | 换口/重插后重跑；脚本改为可传端口 |

## 四、没做的

- 多卡对照（只有一张卡）、0% / 90% 填充对照。
- 队列 4 的 PROC 控制组。
- 连续多次停摆的累计效应。
- 拔卡/坏卡/掉电（EXP-03 的范围）。
- 延迟只观测到 126 ms，卡的 1/10 万量级长尾未覆盖。

## 五、复跑方式

```powershell
# 1) 打开测试开关（改 User/ProbeConfig.h）
#    TEST_EXP02 1, TEST_SD_STALL 1（或 TEST_PROC_STALL 1 / TEST_SD_LAT 1）
# 2) 编译并烧录
& D:\keil5\UV4\UV4.exe -r Project.uvprojx -j0 -o build_xxx.log
python 杂项文件\PC工具\flash_app.py --port COM12
# 3) 抓包与分析
python evidence\sd_stall\capture_exp02.py serial.log 700 COM12
python evidence\sd_stall\analyze_exp02.py serial.log out
python evidence\sd_latency\analyze_lat.py serial.log out
```

矩阵约 7.5 分钟（队列改动需改 `REC_QUEUE_LEN` 后重编）；延迟 10k 样本约 85 分钟。

## 六、原始日志

`serial_*.log` 与 `flash_*.log` 体积较大，**未入库**（见 `evidence/.gitignore`），
保留在本机 `evidence/sd_stall/` 下，可用上面的命令重新生成。
