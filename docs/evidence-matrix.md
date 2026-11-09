# 证据矩阵

> 日期：2026-11-08
> 用途：一张表回答“这个项目到底证明了什么、没证明什么”。
> 状态只有四档：`[实测]` 有原始样本且可复跑；`[设计目标]` 是设计取舍/核算，不是实测；
> `[未定案]` 有观测但不足以定论；`[未做]` 明确没做。
> 链接都相对仓库根目录。

---

## 1. 主张与证据

| # | 主张 | 状态 | 证据资产 | 边界 / 不能说的话 |
|---:|---|---|---|---|
| 1 | 10 Hz 采集，满量约 864,000 行/天 | `[实测]` | `evidence/longrun/结论-EXP01.md`（24 个数据文件 12,722,768 行）；日行数 864,004~864,049 | 精确抖动 σ=0.39 ms 的原始样本在备赛区，未入库；只能作“实验室台架”说 |
| 2 | 生产路径 `f_write` 11k：avg 10.07 ms / max 124.5 ms；`f_sync` avg 11.38 / max 126.1 ms | `[实测]` | `evidence/sd_latency/latency_report.md`、`latency_summary.csv` | **单卡、平静状态**；不是全局最坏；不覆盖拔卡/坏卡 |
| 3 | 停 SD 1 s：10/10 先涨 `LOSS_NO_BUF`，`queue_full` 增量 0 | `[实测]` | `evidence/sd_stall/stall_report.md`、`q8/` `q4/` `ctrl/` | 注入是 `vTaskDelay`，验证的是缓冲链归属，不是“卡内部 busy 1 s” |
| 4 | 队列 8 vs 4：第一计数器相同；500 ms 档 q8 3/8、q4 9/10 | `[实测]` | `evidence/sd_stall/stall_report.md` | 第一瓶颈是 512 B 缓冲，不是队列深度；不能写“队列深度完全无关” |
| 5 | 掉电 8/8 进 App、0 次回滚、每次丢 1 个在途批 2~6 行 | `[实测-有限]` | `evidence/exp03/EXP03-结果-20261106.md`、`card_out/26110601.CSV`、`26110690.CSV` | 8 次，不是 100 次；给不出概率；只测数据窗口，不测金副本保存窗口 |
| 6 | 运行中拔卡 6/6 不死机、6/6 恢复；代价 30 条 `no_buf` | `[实测-有限]` | `evidence/exp03/EXP03-结果-20261106.md`、`exp03_A_10pull.log` | 6 次；故障期缓存降采样到 1 条/秒，不是全分辨率；不给概率 |
| 7 | 无卡启动 89 s → 回写 89 条；exFAT 跨重启 → 回写 276 条 | `[实测]` | `evidence/exp03/EXP03-结果-20261106.md`、`exp03_B1_nocard.log`、`exp03_B2_recover.log` | “坏卡”只指文件系统不可挂载（exFAT），不是物理坏块 |
| 8 | 17 天账本：可解析 **12,722,839** 行，其中 CRC 通过 12,722,768 / **不匹配 71**；另有 **368,220 行二进制垃圾无法解析**；事件文件记 LOST **19,426** 条（`no_buf` 12,091 + `sd_write` 7,435）；0 次重启、跨文件 seq 缺口 0 | `[实测]` | `evidence/longrun/SD分析-17天.md`（2026-11-08 用 `sd_analyze.py` 逐行重算）、`out/crc_summary.md`、`out/summary_clean11.md` | 垃圾行集中在 10-18/10-19 三个文件（`26101801` 110k、`26101901` 254k、`26101903` 3.3k），那两天正是拔卡/恢复实验期，卡本身处于异常状态——**不能拿"17 天"整体说"零损坏"**；71 行是逐行重算出来的，和旧口径"0 失败"差在这；10-29 tick 台阶仍未定案；34 文件口径仍未证实 |
| 9 | 942 ms 单笔写只出现 1 次，20 万次未复现 | `[未定案]` | `evidence/EXP02b-结果-20261106.md`、`evidence/sd_latency/postburst/` | 不能写“已解决/已定位”；三个候选解释都没证明 |
| 10 | 掉卡缓存：全速率 63 min / 降采样 1 条/秒 ~10.6 h | `[设计目标]` + 核算 | `docs/架构总览.md`、`golden.h:162`、`task_sd.c` 的 `Sd_DsPush` | 只测到 89 s / 276 条，没灌满；容量不是实测 |
| 11 | 金副本：App 写、Boot 读；头最后写 | `[设计目标]` | `golden.c`、`Boot/`、`docs/架构总览.md` | 单槽保存窗口不是掉电安全；`app_ok` 失败后未重算；黑名单单槽；无回退标记 |
| 12 | 物理坏卡 / 坏块 / 卡寿命 / 高低温 / 72 h soak | `[未做]` | — | 不要写“100 次 0 损坏”“工业级验证” |
| 13 | 时间基：tick(HSE) 与 RTC(LSE) 独立；13.05 h 实测 tick −1,227 ppm、RTC −10 ppm；10-28 19:45 UTC 一次约 1,800 ppm 渐变；11 月初又回 −1,215 | `[实测-有限]` | `evidence/timebase/EXP04-结果-20261107.md`、`step1d_1028.py`、`run24_regress.py` | **13.05 h 不是 24 h**；原因未定案；第二次变化 11-03~11-06 未定位；不能写“换晶振/换板/重烧” |
| 14 | 参数区（4 KB @0x0801F000）双槽：正常启动 3 次 `seq` 不变（0 写入）；升级后进试用，4 次开机不确认则从金副本回退；`U`+`C` 取消后 `flags` 01→00 | `[实测]` | `../exp/U1-U2-测试-20261107.md` §12、`../exp/serial-u1u2.log`（10:39~11:01） | 单块板、单次跑通；**没做 10k 擦写寿命**、没做传输中途掉电；`img=0` 那一行原先靠 `seq` 2→4 推断，11-08 已直接抓到 `PARAM: img=0 boot=0 seq=66 flags=00 min=5` |
| 15 | 升级"先落盘再切换"：整包先写 W25 暂存区，传输期间内部 Flash 不动；装到一半断电，下次开机自动重装（`G: stage ok -> installing`） | `[实测]` | `../exp/logs/cut20_20261108.log`、`../exp/u3_cut20.py`、`../exp/u3_cut.log`、`Boot/boot_upgrade.c`、`Boot/boot_w25.c` | **2026-11-08 22:5x~23:2x 连做 20 轮，20/20 自愈**（每轮 `X` 后 1.2 s 复位，单轮 29.2 s，共 9.9 分钟）；此前另有 2 次手工注入（传 20 KB 中断、装到一半复位）；`--stage data` 那条还叠加了 `flags=01`，用 `C` 消歧；真实断电（拔电源）仍未做，全部用 ST-Link 复位模拟 |
| 16 | 防回滚：镜像带 version，参数区记 `img_min_ver`，升级时 `version < min` 直接拒绝（`FAIL downgrade ver=1 min=2`），不写任何 Flash | `[实测]` | `Boot/boot_upgrade.c`、`User/app_meta.h`、`docs/架构总览.md` §6 | 一次 v1→v2→v1 的验证；**只管安装，不管金副本恢复**；版本号靠人改，没有签名，改了元数据就绕过了 |
| 17 | 升级空闲自救：进入升级模式后传输断线 / 停住，Boot 数 15 次心跳（实测 25 秒，不是注释里写的 30 秒）后清标志跳回 App（`BOOT: idle, app still valid -> cancel & boot`） | `[实测]` | `../exp/u3_idle_cancel.py`、`../exp/u3_upgrade_cut_matrix.py`、`evidence/upgrade/U3-暂存升级-20261107.md` §6.4 | 2026-11-08 23:0x 复测：发 30% 数据块后复位，**25 秒**回 App，`flags` 由 01 清成 00（`seq` 119→120 就是那次写），随后复位开机 0.23 秒直达 App、无额外等待；**App 无效时不触发**（故意的，否则跳进空区）；真实拔电源未做 |
| 18 | U5 边界三连：同版本重装 `PASS`、v2→v5 跨版本 `PASS` 且 `min` 抬到 5、镜像无元数据 `FAIL nometa` | `[实测]` | `evidence/upgrade/U5-防回滚-20261107.md` | 三条各一次；"剥掉元数据就被拒"证明的是拒绝有效，不是防篡改 |
| 19 | 升级暂存区挪位（`0x7B0000`→`0x7E0000`）：原地址压在掉卡缓存事件槽 `0x7A2000~0x7DDFFF` 上 | `[实测]`（挪后升级通过）+ 算术推（损失量） | `evidence/upgrade/U3-暂存升级-20261107.md` §9、`docs/架构总览.md` §5.2 | "每次升级抹掉约 296/960 条事件"是**按地址和镜像大小算的**，修复前没留 W25 现场；事件槽装的是 `# LOST` 类诊断行，不是测量记录（记录在数据槽 `0x031000~0x7A0FFF`） |
| 20 | 自愈路径也抬防回滚下限：装到一半复位 ⇒ 开机 `G: stage installed` ⇒ `img_min_ver` 从 5 变 6 | `[实测]` | `../exp/logs/selfheal_minver_20261108.log`、`../exp/u5_selfheal_minver.py`、`Boot/boot_upgrade.c` 的 `Up_RecoverFromStage` | 2026-11-08 23:4x 用 v6 镜像（CRC16=0x9176）做一次：截断 1.2 s 后复位，`min` 5→6；此前只有代码级结论（同版本时 `SetMinVer` 早退看不见差别）。**跑完 v5 就装不回去了**，这是持续状态不是一次性演示；参数区备份 `../exp/logs/param_0801F000_before_exp3.bin` |
| 21 | 无卡时 App 仍能跑，但 SD 初始化被**无退避地反复重试**：每次约 0.65 秒、每 0.6 秒喷约 10 行日志 | `[实测]` | `docs/limitations.md` 第 39 条、`Hardware/SD_SPI.c` 的 `SD_DBG_INIT` 探针（默认 0） | 2026-11-09 上午实测：`try 0`→`try 100` 225 ms、`try 100`→`try 200` 218 ms（300 轮 ≈ 0.65 s）；`REC`/`IDLE`/`STAT` 同时正常输出，`sd_err=1 sd_last=3`。**"App 静默/屏黑"那条昨晚的结论部分是 ST-Link 把核心 halt 住造成的假象**（`-c SWD` 连接即复位+halt，`-Dump`/`-CoreReg` 后不自动恢复），已改正；屏黑与 `boot` 涨到 4 仍未复现，保留待查 |

---

## 2. 怎么用这张表

- **简历/面试只引 `[实测]`**；`[设计目标]` 必须带“按设计/按核算”；`[未定案]` 必须说“未定案”；`[未做]` 直接说没做。
- 每条 `[实测]` 都要能回答：**样本量、条件、单卡还是多卡、平静还是故障态**。
- 这条表不替代原始报告；有冲突时以原始报告为准，并回来改这张表。

---

## 3. 复跑入口

| 证据 | 命令 / 文件 |
|---|---|
| 10 Hz / 17 天账本 | `python evidence/longrun/crc_all.py`、`analyze_longrun.py`（先看 `结论-EXP01.md` 的“怎么复跑”） |
| SD 延迟 | `python evidence/sd_latency/analyze_lat.py` |
| SD 停摆 | `python evidence/sd_stall/analyze_exp02.py` |
| 拔卡/坏卡/掉电 | `python evidence/exp03/analyze_exp03.py exp03_A_10pull.log out_A`；CSV 用 `crc_all.py` 同款 CRC |
| 卡健康 | `python evidence/card_health/card_stress_test.py`（会写卡，先备份） |
| 升级：装到一半复位 | `python exp/u3_install_cut.py`（X 之后 1.2 s 用 ST-Link 复位，看 `G: stage ok -> installing`） |
| 升级：六个注入点矩阵 | `python exp/u3_upgrade_cut_matrix.py` |
| 升级：空闲 30 s 自救 | `python exp/u3_idle_cancel.py` |
| 升级：防回滚试用链 | `python exp/u5_v5_trial.py` |
| 17 天账本复核 | `python 杂项文件\PC工具\sd_analyze.py <项目>\evidence\longrun --md 报告.md`（12.7M 行约 13 分钟） |

---

## 4. 一句话

**这张表的价值不是“证明了很多”，而是把“哪些能说、哪些不能说”钉死。**
