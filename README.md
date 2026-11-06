# 工业设备状态记录终端（STM32F103C8T6 + FreeRTOS + FatFs）

多通道数据采集终端：姿态、温度、湿度、气压、光照、电池电压、超声波距离，10 Hz 同步采集，
写入 SD 卡 CSV。采集链设计为「永不阻塞」，目标是**现场任意时刻断电，已写入的数据不丢、文件不坏**。

> **当前状态：v1.0 可用，已连续实机运行 17 天**（2026-11-03，ver 52）
>
> 阶段 1~4 全部完成（采集链 / SD 落盘 / RTC + 记录级 CRC + 看门狗 / Bootloader 串口升级），
> 阶段 5（PC 上位机）已有一半：格式化、对时、升级、SD 日志分析四个脚本可用。
>
> 本文件所有数字都标注来源与状态：**✅ 实测 / 🔶 核算 / ⬜ 计划**。计划中的不会写成已完成。

---

## 1. 硬件与接线

| 模块 | 接口 | 引脚 | 备注 |
|---|---|---|---|
| MCU | STM32F103C8T6 | — | 72 MHz / 20 KB RAM / 64 KB Flash（Bootloader 占 16 KB，App 区 112 KB）|
| 调试串口 | USART1 | **PA9(TX) / PA10(RX)** | 115200-8-N-1；RX 用于对时、升级、命令 |
| OLED 0.96" | 软件 I2C | **PB10(SCL) / PB7(SDA)** | 2026-10-20 从 PB8/PB9 换过来（原插座 VCC/GND 打反，见 `OLED.c` 头注释）|
| MPU6050 | 软件 I2C | PA2(SCL) / PA3(SDA) | 三轴加速度 + 角速度 + 片内温度 |
| BMP280 | 软件 I2C（与 MPU6050 共用）| PA2 / PA3 | 气压 ×10 Pa；地址 0x76/0x77 |
| DS18B20 | 单总线 | PB14 | 需 4.7 kΩ 上拉 |
| **DHT22** | 单总线（开漏）| PB15 | 模块自带 10 kΩ 上拉 |
| **HC-SR04** | TRIG 输出 / ECHO 输入捕获 | PB5 / PB6 | ECHO 必须接 TIM4_CH1 |
| **W25Q64** | 软件 SPI | PA8(SCK) / PA11(MISO) / PA12(MOSI) / PB13(CS) | 8 MB，存金副本 + 掉卡缓存 |
| SD 卡模块 | 硬件 SPI1 + 软件 CS | PA4(CS) / PA5(SCK) / PA6(MISO) / PA7(MOSI) | 模块必须供 5 V；卡 ≤32 GB 且 FAT32 |
| ADC | ADC1 + DMA（循环）| PA1(光照) / PB0(电池分压 1/3) | 后台自动刷新 `AD_Value[]` |
| 蜂鸣器 | GPIO 输出 | PB12 | 低电平触发 |
| 电源键 | GPIO 输入 | PA0 | 长按 = 安全关机握手 |
| 调试器 | ST-Link | SWD | Keil `UV4.exe`；串口升级不需要它 |

引脚规划的教训：

- I2C 原本想放 PB6/PB7，被 OLED 模块物理遮挡，杜邦线插不进去
- 选引脚要三查：复用/JTAG 表 + 板上实物 + 已用引脚表
- PC14/PC15 是 LSE 晶振脚，动它们等于让 RTC 停摆（见 `OLED.c` 换脚注释）

## 2. 软件架构

```
task_ACQ (6) ──Record_t 44B──▶ [队列 深度 32] ──▶ task_PROC (2)
 100ms 硬节拍                                       │ 出队 → 拼 CSV 行
 绝不阻塞                                           │ 填 512B 乒乓缓冲（传指针，零拷贝）
                                                    ▼
                                          [缓冲指针队列] ──▶ task_SD (1) ──▶ FatFs ──▶ SD
task_KEY (2)    20ms  ──▶ 暂停/STAT/长按关机
task_DISP (1)  200ms  ──▶ OLED（唯一所有者）+ 超时黑屏
task_sensor (1) 1s    ──▶ DS18B20 + BMP280 + DHT22 + 超声波
task_GOLDEN (1) 一次性 ──▶ 把当前固件备份进 W25Q64（金副本）
```

| 任务 | 优先级 | 栈（字）| 周期 | 职责与约束 |
|---|---:|---:|---|---|
| ACQ | 6 | 128 | 100 ms | 采样并组装记录；`xQueueSend` 超时 0，绝不阻塞；漏拍会记账 |
| PROC | 2 | 320 | 队列驱动 | 拼 CSV、填乒乓缓冲；唯一的高频串口打印者 |
| KEY | 2 | 64 | 20 ms | 三态状态机消抖；禁止 printf（栈不够）|
| DISP | 1 | 224 | 200 ms | OLED 唯一所有者；心跳 + SD 错误码；30 s 无操作黑屏 |
| sensor | 1 | 320 | 1 s | 单总线温度 + 气压 + 湿度 + 超声波 |
| SD | 1 | 448 | 队列驱动 | **全工程唯一允许调用 `f_xxx` 的地方** |
| GOLDEN | 1 | 192 | 一次性 | 后台把内部 Flash 拷进 W25Q64 |

三条设计铁律：

1. **SD 的慢被压在低优先级后台**，ACQ→PROC 采集链按设计不等待 SD（实测生产路径 11k：`f_write` avg 10.07 ms、max 124.5 ms；0% 干净卡 10k p99 10 ms；不是“永不阻塞”，I2C 互斥仍用 `portMAX_DELAY`。一块 512 B 缓冲攒满 500 ms，最坏一笔约占 1/4）
2. **FatFs 只允许出现在 `task_sd.c`**（`_FS_REENTRANT=0`），从架构上消灭并发而不是加锁
3. **所有任务的内存都是静态分配的**（`.bss` 里的具名数组），堆只剩 64 B，`map` 能逐项对账

## 3. 存储与可靠性

### 文件命名

```
YYMMDDNN.CSV    数据文件，NN = 当天第几个（01 起）
YYMMDD90.CSV    事件文件（# 开头，PC 解析器跳过）
```

- **按本地日期切换**（`MyRTC_GetLocalDateTime`），本地 00:00 换新文件
- 一天约 864,000 行 ≈ 80 MB

### CSV 格式（17 列）

```csv
seq,tick,press,rtc,ax,ay,az,gx,gy,gz,temp_ds,temp_chip,humi,light,vbat,sonar_mm,crc
1,100,1021230,1793697276,-213,-1115,1669,1,-1,-1,-32768,2303,65535,2278,2088,52,2675
```

| 列 | 含义 | 无效值标记 |
|---|---|---|
| `rtc` | **真 Unix 秒（UTC）** | 0 = RTC 没走（见 §6）|
| `temp_ds` | DS18B20，×100 ℃ | `-32768` = 转换未完成 |
| `humi` | DHT22，×10 %RH | `65535` = 未读到 |
| `light` / `vbat` | ADC 原始值 0~4095 | — |
| `sonar_mm` | 超声波距离 mm | 0 = 无回波 |
| `crc` | 前 16 列文本的 CRC-16/KERMIT | — |

### 可靠性机制

| 机制 | 做法 |
|---|---|
| 写放大控制 | 512 B 攒块写（SD 最小写单位即扇区）|
| 落盘节奏 | 每批 `f_sync`，承诺「掉电最多丢最近一批」|
| 缓冲所有权 | 乒乓双缓冲，队列传指针；拿了必须交，否则泄漏停摆 |
| 掉卡缓存 | 卡不在时数据进 W25Q64，卡回来自动重放（`# SD_REPLAY`）|
| 写失败处理 | 连续 3 批失败 → 置 `g_sd_error` + 卸载卷，每 3 s 重试 |
| 安全关机 | 长按电源键 → 停采集 → 写完队列 → `f_close` → 屏显 `SAFE TO OFF` |
| 固件金副本 | 升级前把当前固件存进 W25Q64；Boot 可回退 |
| 丢数据记账 | 5 种丢失原因各一个计数器，冲进事件文件（`# LOST,<rtc>,<n>,<reason>`）|
| 可观测性 | 队列满、丢条、SD 错误码、栈水位、堆水位，全部可查。**丢东西可以，不能静默地丢** |

## 4. 时间（RTC）

三条规则，写在 `MyRTC.h` 里：

1. **CSV 的 `rtc` 列永远是 UTC**，不带时区。换时区/跨时区拷数据都不会歧义
2. **给人看的时间（文件名、屏显）加 8 小时**，走 `MyRTC_GetLocalDateTime()`
3. **「配过」和「对过」是两件事**：`BKP_DR1` 说有初始化过，`BKP_DR5` 说有人对过时

本板**已装 CR2032**，正常断电后 RTC 继续走（开机看到 `BKP magic hit -> keep counter` 就是证据）。
只有换电池、备份域被复位、或 LSE 起不来这几种情况，才会退回"用编译时刻当初值"。
已在跑但时间不对时，用串口 `T<unix>` 或 `time_sync.py` 对时。

## 5. 实测数据

### 采集与落盘

| 指标 | 值 | 状态 |
|---|---|---|
| 采集频率 | 10.001 Hz（按 tick 计，1 tick 名义 1 ms）| ✅ 实测 |
| 时间基 | 行内 `tick` 来自 SysTick（HSE 72 MHz 分频）；`rtc` 是独立的 LSE 秒。两者不是同一个时基，不能互相当秒用 | — |
| 周期抖动 | σ = 0.39 ms，94.5% 落在 100 ms ±1 ms | ✅ 实测 |
| 每天行数 | 864,004 ~ 864,049（满量段）；10-22~10-28 为 861,269 ~ 862,789 | ✅ 实测 |
| 每天体积 | 79 ~ 84 MB | ✅ 实测 |

### SD 延迟与停摆（2026-11-06，ver 74-EXP02，单卡）

| 项 | 值 | 状态 |
|---|---|---|
| 单次 `f_write`（生产路径） | 11,000 次：avg **10.07 ms**、p99 100 ms、max **124.5 ms** | ✅ 实测 |
| 单次 `f_sync`（生产路径） | 11,000 次：avg 11.38 ms、p99 101 ms、max 126.1 ms | ✅ 实测 |
| 全速 0% 干净卡 | 10,000 次：write avg 3.48 ms、p99 10 ms、max 27.98 ms；sync p99 13 ms、max 89.48 ms | ✅ 实测 |
| 全速 91.5% 脏卡 | 两臂各 100,000 次：write avg 3.63~3.64 ms、max 67.19~72.07 ms；≥256 ms 零次 | ✅ 实测 |
| ≥250 ms | 平静生产 11k **0**；91.5% 200k **0**；942 ms 仅 1 次，20 万次未复现 | ⚠️ 未定案 |
| 停 SD 任务 1 s | 10/10 次先涨 `LOSS_NO_BUF`，`Δqueue_full = 0`；`first_loss` 644~956 ms | ✅ 实测 |
| 停 PROC 任务 1 s（对照） | 10/10 次先涨 `LOSS_Q_FULL`，`qmax` 冲到 8 | ✅ 实测 |
| 队列 8 vs 4 | 第一计数器相同；500 ms 档 q8 3/8、q4 9/10；第一瓶颈是缓冲 | ✅ 实测 |
| 卡健康 | 格式化 + 3.956 GB 写满 + 拔插读回，mismatch=0；845 丢失簇已清；一次检查不等于完全健康 | ✅ 实测/⚠️ 一次 |
| 100k 分位数 | 直方图 `uint16` 饱和，p90/p99=256 是假数；只引 avg/max | ⚠️ 无效 |
| 故障注入（拔卡/坏卡/掉电） | 拔卡 6/6 无死机、丢条全是 `no_buf`（25 条）；无卡 89 s 回写 89 条；exFAT 跨重启回写 276 条；掉电 8/8 进 App、0 次回滚、尾损 7/8 为 0 行；CSV 9,212 行 0 坏行；chkdsk 干净 | ✅ 实测（样本 6/8，非概率） |
| 证据 | [延迟报告](evidence/sd_latency/latency_report.md) · [停摆报告](evidence/sd_stall/stall_report.md) · [EXP-02b 结果](evidence/EXP02b-结果-20261106.md) · [EXP-03 结果](evidence/exp03/EXP03-结果-20261106.md) · [卡健康](evidence/card_health/卡健康基线-20261106.md) |

### 17 天连续记录（2026-10-18 ~ 11-03，E 盘 SD 卡）

| 项 | 值 |
|---|---|
| 文件数 / 体积 | 34 个 / **1,189 MB** |
| 总行数 | 本账本实测 **12,722,768 行**（24 个数据文件）|
| 满量天数 | 10-29 起连续 6 天满量（误差 <100 行）|
| 10-22~11-02 相对 864,000 的差额 | **16,133 行**，四块拆分见下 |
| **CRC（全量）** | **24 个文件 12,722,768 行逐行校验，失败 0**；196 行是旧格式 crc=0，不参与 |
| 缺口账本 | [结论](evidence/longrun/结论-EXP01.md) · [干净窗口 11 天](evidence/longrun/out/summary_clean11.md) · [CRC 汇总](evidence/longrun/out/crc_summary.md) · [10-29 未定案](evidence/longrun/timeline_1029.md) · [26101801 取证](evidence/longrun/forensics_26101801.md) |

**16,133 行缺口的四块拆分**（README 早前按"864,000/天"口径写过 16,175，与本次相差 42 行，在容差内）：

| 块 | 行数 | 占比 | 说明 |
|---|---:|---:|---|
| 假缺口（tick 与物理秒不同步） | 10,095 | 63% | 不是丢数据，是"每天 864,000"这个假设与 tick 实际走的拍数不符 |
| 文件内真缺口 | 2,056 | 13% | 其中 1,730 集中在 10-22（当天 4 段/重启），其余多为个位数 |
| 其余（rtc 日长与异常段） | 3,982 | 25% | 未再细分 |
| 跨文件边界缺口 | 0 | — | 相邻文件 tick 连续且 seq 自洽 |

> 结论：**数据值（每行内容 + CRC）逐行校验通过；行数缺口里只有约 2,056 行是文件内真丢失。**
>
> 账本覆盖不了的：文件首尾截断、跨文件、复位、尾批未落盘、W25 回灌、以及 KEY1 暂停时段。
> 早期文件（10-18~10-20）另有介质层异常：26101801.CSV 里混入了二进制垃圾（含 PNG 块头），
> 154,605 行中有 55,600 行不是 CSV，详见 `evidence/longrun/findings.md`。

### 资源占用（2026-11-03，ver 52）

| 项 | 值 |
|---|---|
| Flash（Code + RO + RW）| 61,228 + 3,936 + 480 = **65,644 B**，App 区 112 KB ⇒ 余 49 KB |
| RAM（RW + ZI）| 480 + 14,976 = **15,456 B / 20,480** ⇒ 余 ~4.9 KB |
| 堆 | 64 B（名义值，静态分配后没有真正使用者）|
| 栈峰值（实测）| ACQ 69 字 / PROC 206 / KEY 23 / DISP 150 / SD 324 / GOLD — |

## 6. 串口升级（不需要 ST-Link）

PC 侧：`杂项文件\PC工具\flash_app.py`（或双击 `升级固件.bat`）

```text
python flash_app.py              # 完整升级：探针 → 请求复位 → 传 → 校验 → 跳转
python flash_app.py --check      # 只算体积和 CRC，不连串口
python flash_app.py --stage data # 只发数据不发 X（验证写 Flash 那一段）
```

协议（文本行，`\r\n` 结尾）：

```text
PC → MCU : E<字节数>        D<块号><hex>        X<整体CRC16>        U（请求进 Boot）
MCU → PC : E0/E1   K<块号>   N<块号>   PASS   FAIL <原因>   WAIT
```

**停等**：每块 128 字节，等 `K` 才发下一块。原因是擦写 Flash 时关中断，串口那段时间的字节会丢。

**升级前** Boot 会把当前固件存进 W25Q64 金副本，所以升坏了能回退。

## 7. 编译与运行

- 工具链：Keil MDK5 + **ARMCC V5.06u7（AC5）**，`-O1`（不要开 `-O3`）
- App 工程：`Project.uvprojx`；Boot 工程：`Boot\Project.uvprojx`
- 命令行编译（Keil 关着时可用）：

```powershell
UV4.exe -b Project.uvprojx -j0 -o build.log          # 增量
UV4.exe -r Boot\Project.uvprojx -j0 -o build_boot.log # 全量
```

- 注意：`MyRTC.c` 里用 `__DATE__` / `__TIME__` 给 RTC 设初值 ⇒ **同一份源码不同时刻编译，bin 的 CRC 会不同**，这不是代码变了
- SD 卡：先在 PC 上格式化为 FAT32（工程裁剪掉了 `f_mkfs`）

## 8. 目录结构

```
工业记录仪/
├─ User/          main.c（七步初始化）、freertos_demo.c（任务总装 + ACQ/PROC/KEY/DISP）
│                 task_sd.c/h（落盘任务 + 丢失记账）、golden.c/h（金副本）
├─ Boot/          Bootloader 工程（main / jump / flag / flash / upgrade / golden / w25）
│                 boot_config.h —— 分区地址的【唯一真源】，两边各放一份
├─ Hardware/      SD_SPI、MPU6050、BMP280、DS18B20、DHT22、OLED、AD、Key、Serial、MyRTC、W25Q64、HC_SR04
├─ system/        DWT_Delay（微秒延时；禁止用裸机 Delay.c，它会拆掉 SysTick）、Timer
├─ FatFs/         FatFs R0.12c + diskio_spi.c 移植层
├─ FreeRTOS/      FreeRTOS V11.1.0（heap_4）+ FreeRTOSConfig.h（静态分配、堆 64 B）
├─ Start/ Library/ ST 标准库 V3.5 与启动文件
└─ 阶段2-预置说明.md
```

## 9. 已知不足与 roadmap

| 项 | 状态 |
|---|---|
| Tickless idle（`configUSE_TICKLESS_IDLE`）| ⬜ 未启用：HAL/内核时基冲突，且实测大头是屏幕与 LDO，收益小 |
| STOP 模式 | 💡 候选：要真做电池供电才考虑 |
| SD 卡满后的处理 | ⬜ 未实现：按 80 MB/天，4 GB 卡约 25 天满 |
| P-MOS 断电重挂 SD（掉卡终极解法）| ⬜ 未实现：PB10 被 OLED 占用后计划挪到 PA15 |
| Modbus RTU 从机 / 上位机 GUI 增强 | 💡 候选 |

**已知卡上遗留**（不装电池时的历史产物）：

- `70010101.CSV`（8.7 MB）：LSE 没起振时 RTC 恒为 0，文件名退化成 1970；现在会在屏上报警
- `26101802 / 26101901 / 26101903`：开发期写坏的文件，尾部有二进制垃圾

## 10. 许可证

应用代码自制；FreeRTOS 与 FatFs 保持各自上游许可证，ST 标准库遵循其官方条款。
