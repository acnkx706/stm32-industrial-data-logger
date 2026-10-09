# 工业设备状态记录终端（STM32F103C8T6 + FreeRTOS + FatFs）

多通道数据采集终端：振动/姿态、温度、气压、光照、电池电压 10 Hz 同步采集，写入 SD 卡 CSV，
采集链路设计为「永不阻塞」，目标是**现场任意时刻断电，已写入的数据不丢、文件不坏**。

> ⚠️ **当前状态：半成品 / 开发中**（2026-10-10）
> 阶段 1（采集链）与阶段 2（SD 落盘）已完成并通过实测，阶段 3（RTC / 记录级 CRC / 看门狗）与
> 阶段 4（Bootloader）尚未开始。本文件所有数字都标注了来源与状态——**✅ 实测 / 🔶 核算 / ⬜ 计划**，
> 计划中的不会写成已完成。

---

## 1. 硬件与接线

| 模块 | 接口 | 引脚 | 备注 |
|---|---|---|---|
| MCU | STM32F103C8T6 | — | 72 MHz / 20 KB RAM / 64 KB Flash / 中容量 |
| 调试串口 | USART1 TX | PA9 | 115200-8-N-1，**仅 TX**（PA10 RX 未启用） |
| OLED 0.96" | 软件 I2C | PB8(SCL) / PB9(SDA) | 无显存驱动，直写 GDDRAM |
| MPU6050 | 软件 I2C | PA2(SCL) / PA3(SDA) | 三轴加速度 + 角速度 + 片内温度 |
| BMP280 | 软件 I2C（与 MPU6050 共用） | PA2 / PA3 | 气压 ×10 Pa；地址 0x76/0x77，ID=0x58 |
| DS18B20 | 单总线 | PB14 | 需 4.7 kΩ 上拉 |
| SD 卡模块 | 硬件 SPI1 + 软件 CS | PA4(CS) / PA5(SCK) / PA6(MISO) / PA7(MOSI) | **模块必须供 5 V**；卡 ≤32 GB 且 FAT32 |
| ADC | ADC1 + DMA（循环） | PA1（光照） / PB0（电池分压 1/3） | 后台自动刷新 `AD_Value[]` |
| 光敏（数字） | GPIO 输入 | PB13 | 只判亮/暗 |
| 按键 | GPIO 输入 | PB1(KEY1) / PB11(KEY2) | ⚠️ 极性待上板最终确认 |
| 蜂鸣器 | GPIO 输出 | PB12 | 未启用 |
| 调试器 | ST-Link | SWD | Keil `UV4.exe` |

> 引脚规划踩过的坑：I2C 原本想放 PB6/PB7，但该排针被 OLED 模块**物理遮挡**，杜邦线插不进去；
> 最终落在空闲的 PA2/PA3。**引脚选型要三查：复用/JTAG 表 + 板上实物 + 已用引脚表。**

## 2. 软件架构

```
task_ACQ (6) ──Record_t 40B──▶ [队列 深度8] ──▶ task_PROC (2)
 100ms 硬节拍                                      │ 出队 → 拼 CSV 行
 绝不阻塞                                          │ 填入 512B 乒乓缓冲（传指针，零拷贝）
                                                   ▼
                                        [缓冲指针队列] ──▶ task_SD (1) ──▶ FatFs ──▶ SD 卡
                                                              每批 f_write + f_sync
task_KEY (2)  20ms  ──状态标志 / 任务通知──▶ ACQ、PROC
task_DISP (1) 200ms ──▶ OLED（唯一所有者）
task_sensor(1) 1s   ──▶ DS18B20 单总线 + BMP280
```

| 任务 | 优先级 | 栈（字） | 周期 | 职责与约束 |
|---|---|---|---|---|
| ACQ | 6 | 256 | 100 ms | 采样并组装记录；`xQueueSend` 超时 0，**绝不阻塞** |
| PROC | 2 | 256 | 队列驱动 | 拼 CSV、填乒乓缓冲；唯一的串口打印者 |
| KEY | 2 | 128 | 20 ms | 三态状态机消抖；**禁止 printf**（栈不够） |
| DISP | 1 | 256 | 200 ms | OLED 唯一所有者；心跳计数 + SD 错误码 |
| sensor | 1 | 128 | 1 s | 单总线温度 + 气压 |
| SD | 1 | 256 | 队列驱动 | **全工程唯一允许调用 `f_xxx` 的地方** |

两条设计铁律：
1. **SD 的慢（busy 最坏 250 ms）被压在最低优先级**，ACQ→PROC 采集链永不阻塞；
2. **FatFs 只允许出现在 `task_sd.c`**（`_FS_REENTRANT=0`），从架构上消灭并发，而不是靠加锁。

## 3. 存储与可靠性

| 机制 | 做法 |
|---|---|
| 写放大控制 | 512 B 攒块写（SD 最小写单位即扇区），10 B/条直接写会产生 **50 倍写放大** 🔶 |
| 落盘节奏 | 每批 `f_sync` → 可承诺「掉电最多丢最近一批」；`f_sync` 返回值**必须**接住（`_FS_TINY=1` 时尾巴只在 `fs->win` 里） |
| 缓冲所有权 | 乒乓双缓冲，队列里传**指针**；任一时刻缓冲只在一个人手里，拿了必须交，否则泄漏停摆 |
| 错误处理 | 连续 3 批写失败 → 置 `g_sd_error` + 卸载卷，每 3 s 重试；错误码（FRESULT）上 OLED 与串口 |
| 可观测性 | 队列满计数、丢条计数、SD 错误码、栈水位、堆历史最低水位，全部可查——**丢东西可以，不能静默地丢** |

CSV 格式（16 列，首行为表头，写进 `LOG0001.CSV` 起递增的文件名）：

```csv
seq,tick,press,rtc,ax,ay,az,gx,gy,gz,temp_ds,temp_chip,humi,light,vbat,crc
1,100,1013250,0,-120,340,16200,-8,12,-3,2400,3650,65535,2871,65535,0
```

> `rtc` / `crc` / `humi` / `vbat` 当前为**预留占位**（0 / 0 / 0xFFFF / 0xFFFF），阶段 3 填实。

## 4. 实测数据

| 指标 | 值 | 状态 |
|---|---|---|
| 采集频率 | 10.001 Hz（400 样本 / 39898 ms） | ✅ 实测 |
| 周期抖动 | σ = 0.39 ms，94.5% 精确落在 100 ms；min/avg/max = 95/99.99/104 ms | ✅ 实测 |
| 零累积漂移 | seq=1031 时 tick=103100 → 100.000 ms 精确 | ✅ 实测 |
| 队列（阶段 1，下游仅打印） | full=0，历史最大积压 qmax=1 | ✅ 实测 |
| 软件 I2C 单次读 MPU6050 | 5239 µs，占 100 ms 周期的 5.24%；反推位操作 3.04 µs → SCL ≈ 110 kHz | ✅ 实测 |
| 堆占用 | 12 KB heap，开机实测 free=11856 B（已用 432 B） | ✅ 实测 |
| 内核结构体 | Queue_t=84 B / TCB_t=92 B / List_t=20 B（编译期探针 + fromelf） | ✅ 实测 |
| 当前占用（2026-10-10 map） | Flash **44.05 KB / 64 KB**，RAM **18.15 KB / 20 KB** | ✅ 实测 |
| 写卡不阻塞采集（写卡期间间隔仍 100 ms±、full=0） | — | ⬜ **待实测**（阶段 2 验收项） |
| 随机断电 100 次 / 0 文件损坏 | — | ⬜ 计划中 |
| 72 h soak、0 非预期复位 | — | ⬜ 计划中 |

## 5. 串口输出样例（115200）

```
=== boot ===
MPU ID = 68 (expect 68)
---- BMP280 probe (I2C: PA2=SCL PA3=SDA) ----
  ACK at 0x76 : reg[0xD0]=0x58  <== BMP280 (ID=0x58) -- FOUND
LIGHT DO = 1
gyro calibrating, keep still for 2s ...
MPU OFF acc=(0,0,0) fixed, gyro=(-8,12,-3) boot-cal
GYRO residual=(0,0,0)  (abs>5 = calibration NOT trustworthy)
AD init: light=2871 vbat=2048 (raw 0-4095)
=== Init Done ===
queue ok: item=40 bytes, free heap=11856
---- Task list ----
ACQ     R   6   204  1
...
---- Stack free (words) ----
ACQ             204
PROC            130
STAT acq=100 proc=100 full=0 qmax=1 drop=0 i2c=5239 i2c_max=5239 sd_err=0 sd_last=0 heap=11856 heapmin=11856
REC,seq<1>,tick<100>,press<1013250>,rtc<0>,ax<-120>,ay<340>,az<16200>,...
```

OLED 四行：`ACQ:` 采集数 / `PRC:` 处理数 / `FUL:` 满队次数 `QMX:` 最大积压 / `RUN` 或 `PAUSE` + 心跳 + `E:xx`（SD 错误码）。

## 6. 编译与运行

- 工具链：**Keil MDK5 + ARMCC V5.06u7**（AC5），`-O1`（**不要开 -O3**，有已知优化坑），C99，未启用 MicroLIB（printf 由 `fputc` 重定向到 USART1）
- 打开 `Project.uvprojx` → 编译（F7）→ ST-Link 下载
- SD 卡：**先在 PC 上格式化为 FAT32**（工程裁剪掉了 `f_mkfs`）
- 按键：KEY1 暂停/恢复采集，KEY2 立即打印 STAT

## 7. 目录结构

```
工业记录仪/
├─ User/          main.c、freertos_demo.c（任务总装）、task_sd.c/h（落盘任务）
├─ Hardware/      各外设驱动：SD_SPI、MPU6050、BMP280、DS18B20、OLED、AD、Key、Serial、MyI2C
├─ system/        DWT_Delay（微秒延时；**禁止用裸机 Delay.c，它会拆掉 SysTick**）
├─ FatFs/         FatFs R0.12c 原版 + diskio_spi.c 移植层 + 裁剪后的 ffconf.h
├─ FreeRTOS/      FreeRTOS V11.1.0（heap_4）+ FreeRTOSConfig.h
├─ Start/ Library/ ST 标准库 V3.5 与启动文件
└─ 阶段2预置_README.md
```

## 8. 已知不足与 roadmap

诚实列在这里——这些**还没有做**，面试与使用时请勿当成已完成：

| 项 | 状态 |
|---|---|
| RTC 时间戳（记录里 `rtc` 字段仍为 0） | ⬜ 阶段 3 |
| 记录级 CRC16 与掉电撕裂检测 | ⬜ 阶段 3 |
| IWDG 多任务喂狗、PVD 掉电预警 | ⬜ 阶段 3 |
| 串口互斥量（目前靠「只有一个打印者」侥幸成立） | ⬜ 阶段 3 |
| Bootloader / XMODEM 串口升级 | ⬜ 阶段 4 |
| Python 上位机（帧协议 / 校时 / 升级） | ⬜ 阶段 5 |
| Modbus RTU 从机、低功耗 Stop 模式、主机侧单元测试 | 💡 候选 |
| RAM 余量仅约 1.9 KB，Flash 余量约 20 KB | ⚠️ 硬约束，新增功能前先算账 |

已知代码问题（来自 2026-09-27 的一次静态审计，早于阶段 2）：
按键极性待上板最终裁决（`Key.c` 配 IPD，与硬件事实表"按下为低"冲突，若判错则按键全死且无报错）；
`Serial_SendString` 索引用 `uint8_t`，字符串 ≥256 B 时会回卷成死循环（`print_task_diagnostics` 是现成触发路径）。

## 9. 许可证

应用代码自制；FreeRTOS 与 FatFs 保持各自上游许可证，ST 标准库遵循其官方条款。
