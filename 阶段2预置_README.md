# 阶段 2 预置说明（SD 卡 + FatFs）

对应《28-数据记录仪项目-执行表》第四节（T2.1~T2.5）。本次**只新增文件**，未改动任何现有文件（含 .uvprojx/.uvoptx）——下面的 Keil 配置改动需要你手动做，每步都列了。

---

## 一、新增文件清单

| 文件 | 内容 | 对应执行表 |
|---|---|---|
| `system\DWT_Delay.c/h` | DWT 微秒/毫秒延时。**为什么不新增这个模块**：裸机 `system\Delay.c` 会重配并关闭 SysTick，调度器启动后调用 = 拆 FreeRTOS 心跳（已踩过的雷）。SD 驱动里的毫秒等待全部走 DWT | T1.5 延续 |
| `Hardware\SD_SPI.c/h` | SD 卡 SPI 模式完整驱动（硬件 SPI1）：CMD0/CMD8/ACMD41/CMD58/CMD16/CMD17/CMD18/CMD24/CMD25 全套 + 隐性必需的 CMD55（ACMD 前导）/CMD12（停多块读）/CMD59（关数据 CRC）/ACMD23（预擦除，可选优化）；R1 解析、CRC7 现算（已用规范已知值 CMD0=0x95、CMD8=0x87 验证）、0xFF 哑元、busy 等待带超时、读写各带 3 次重试、错误码 enum | T2.1 |
| `FatFs\ff.c / ff.h / diskio.h / integer.h` | FatFs **R0.12c** 原版，从 `ref\bootloader-sd\` 拷入，**一行未改**（版本号 68300 必须与 ffconf.h 配对，别换 R0.15） | T2.2 |
| `FatFs\ffconf.h` | 按执行表裁剪：`_FS_READONLY 0`、`_FS_MINIMIZE 2`、`_USE_STRFUNC 0`、`_USE_MKFS 0`（省 4~5KB）、`_CODE_PAGE 1`、`_USE_LFN 0`、`_FS_TINY 1`、`_FS_EXFAT 0`、`_FS_REENTRANT 0`、`_FS_NORTC 0`。**每一项都带"改了什么/为什么"注释** | T2.2 |
| `FatFs\diskio_spi.c` | diskio 5 函数对接 SD_SPI 驱动 + `get_fattime()`（阶段 2 返回固定 2026-1-1，**阶段 3 接 RTC 只改这一个函数**）；写失败"驱动层重试、本层上浮"的策略写在文件头注释 | T2.2 |
| `User\task_sd.c/h` | task_SD 任务**骨架**（prio 1 / 栈 256 字）：挂载态↔未挂载态状态机、乒乓缓冲指针队列交接、每批 f_sync、连续 3 批失败置 `g_sd_error` 丢批、插回自动续写。流程注释写全，关键决策点留 `/* TODO(你): */` | T2.3/T2.5 |
| `阶段2预置_README.md` | 本文件 | — |

**RAM 预算对账**（执行表第八节）：FATFS+FIL(TINY=1) ≈ 660B + 乒乓 512B×2，全部 static 放 heap 外；task_SD 栈 256 字（1KB）在 12KB heap 内 —— 与总账一致。

---

## 二、Keil 里加文件（逐步清单，只加 .c，头文件靠 Include Path）

1. 打开工程 → `Project → Options for Target → C/C++ → Include Paths`，在现有 7 条路径末尾**追加**：
   ```
   .\FatFs
   ```
   （不加这条，task_sd.c 找不到 ff.h。其余路径不用动。）
2. 左侧 Project 树，右键 **system** 组 → `Add Existing Files to Group 'system'` → 选 `system\DWT_Delay.c`
3. 右键 **Hardware** 组 → 添加 `Hardware\SD_SPI.c`
4. 新建组：右键工程根 → `Add Group...` → 命名 **FatFs** → 右键新组 → 添加两个文件：
   - `FatFs\ff.c`（FatFs 本体）
   - `FatFs\diskio_spi.c`（移植层）
5. 右键 **User** 组 → 添加 `User\task_sd.c`
6. 确认 Options → C/C++：`uC99` 保持勾选（工程现状即 C99 模式，ff.c/integer.h 需要）；Defines 保持 `USE_STDPERIPH_DRIVER`
7. 编译（F7）。预期 0 error 0 warning（新文件已单独用 armcc V5.06u7 逐一验证过，见第六节）

---

## 三、代码集成步骤（都是你手动改，共 5 处）

1. **heap 调 12KB**（T2.4）：`FreeRTOS\include\FreeRTOSConfig.h` 第 106 行
   ```c
   #define configTOTAL_HEAP_SIZE  ((size_t)(10*1024))   →  ((size_t)(12*1024))
   ```
   改完跑：`python "C:\Users\acnkx\.agents\skills\stm32c8t6-dev\scripts\check_freertos_config.py" "...\FreeRTOS\include\FreeRTOSConfig.h"`
2. **freertos_demo.c 头部**加 `#include "task_sd.h"`
3. **freertos_demo()** 里，`xQueueRec` 创建成功之后加一行：
   ```c
   task_sd_Init();     /* 建 SD 交接队列（必须在调度器启动前：task_SD 优先级低，PROC 可能抢先运行） */
   ```
4. **start_task()** 的临界区里、DISP 创建块后面，照抄一份创建块：
   ```c
   if (xTaskCreate(  (TaskFunction_t )   task_SD,
                     (char *         )   "SD",
                     (uint32_t       )   TASK_SD_STACK_SIZE,
                     (void *         )   NULL,
                     (UBaseType_t    )   TASK_SD_PRIO,
                     (TaskHandle_t * )   &task_sd_handler) != pdPASS)
   {
       fail_mask |= (1u << 4);     /* bit4 = SD 没建起来 */
   }
   ```
   （`task_sd_handler` 声明可加在 task_sd.h 或 freertos_demo.c 任务配置块，风格随你。）
5. **task_PROC()**：把"串口打印每条 REC"改成"拼 CSV 行进乒乓缓冲"。参考片段（填空处标了 TODO）：
   ```c
   /* 循环外：*/ static uint8_t *s_buf = NULL;  static uint16_t s_pos = 0;
   /* 收到 rec 后：*/
   if (s_buf == NULL && SD_BufAcquire(&s_buf) == pdPASS) { s_pos = 0; }
   if (s_buf != NULL)
   {
       /* TODO(你): 行格式自己定，但必须与 task_sd.c 里写的 CSV 表头一字不差 */
       int n = sprintf((char *)s_buf + s_pos, "%lu,%lu,%d,%d,...\r\n",
                       (unsigned long)rec.seq, (unsigned long)rec.tick, rec.ax, ...);
       s_pos += (uint16_t)n;
       if (s_pos >= SD_BUF_SIZE - 128)          /* 快满就交（留一行余量） */
       {
           SD_LogEnqueue(s_buf, s_pos);         /* pdFAIL 时缓冲已被还回，重拿即可 */
           s_buf = NULL;
       }
   }
   /* TODO(你): 空闲期兜底——暂停采集时手里攒的半块数据，用 1s 超时路径冲一次 SD_LogEnqueue */
   ```
   交接时序（task_sd.c 里也有）：`SD_BufAcquire 拿指针 → 填 → SD_LogEnqueue 交出 → task_SD 写完还回空闲队列`。指针任一时刻只在一个人手里，**拿了必须交**，否则缓冲泄漏、链路停摆。
6. （可选）**task_DISP()** 加一行显示 `g_sd_error`（如第 4 行显示 `SD:ERR`），对应验收项③的"错误标志上 OLED"。

铁律重申（执行表 T2.3）：**f_xxx 只允许出现在 task_sd.c**，其他任务一律走上面两个接口。

---

## 四、到货后的硬件验收（三态 + 四条验收）

接线（执行表 4.1）：CS→PA4，SCK→PA5，MISO→PA6，MOSI→PA7；带 AMS1117 稳压的模块**必须供 5V**（供 3.3V 有压差问题，卡时好时坏）；卡 ≤32GB 且在 PC 上格成 FAT32。

| 态 | 操作 | 预期 |
|---|---|---|
| ① 无卡 | 不插卡上电 | 系统正常跑，`g_sd_error=1`（OLED 有提示），ACQ/PROC 速率不受影响，不死机 |
| ② 坏卡 | 插 64G exFAT 卡 / 未格式化卡 / 接触不良卡 | SD_Init 失败或挂载失败 → 同①表现，task_SD 每 3 秒重试一次（串口可加打印观察） |
| ③ 正常卡 | 插 FAT32 卡 | 自动建 `LOG0001.CSV` 并写入；拔卡插 PC 数据完整可读；插回开发板 → 3 秒内自动续写、`g_sd_error` 清 0 |

执行表 T2.6 四条：① CSV 内容完整 ② **写卡期间 acq 间隔仍 100ms±、queue_full=0**（写卡不阻塞采集的实证）③ 拔卡不死机、插回恢复 ④ `xPortGetMinimumEverFreeHeapSize()` 最低水位 >2KB。

---

## 五、已知风险

1. **供电是头号嫌疑**：写卡瞬间电流 ~100mA，AMS1117 模块供 3.3V 会在写入时掉压 → 表现为"读没问题、写随机坏"。严格按模块要求供 5V。
2. **CMD59 关 CRC**：极个别老卡不认 CMD59（驱动返回 `SD_ERR_UNSUPPORTED`）。真遇到再说（补一套 CRC16），市面卡几乎不存在。
3. **MMC 卡不支持**（CMD1 没实现，执行表未要求）：插 MMC 会超时报 INIT_TIMEOUT。
4. **时间戳是假的**（固定 2026-1-1）：文件在 PC 上看修改时间不对，属预期；阶段 3 T3.1 接 RTC 后只改 `diskio_spi.c` 的 `get_fattime()`。
5. **拔卡时机**：正在写的最后一批会丢（每批 f_sync 已把损失限到 ≤512B），FAT 表不会脏——这是 T2.5 策略的设计行为。
6. **ff.c/ff.h/integer.h 是上游原版**，本次仅做编译冒烟、未逐行复查（也不该复查）；真出问题先怀疑 ffconf.h 和 diskio_spi.c。
7. **AC5 别开 -O3**（执行表已知优化坑），保持 -O1。
8. `SD_SPI.c` 非重入（共享 SPI1 总线 + 全局卡类型）：除 task_sd 经 FatFs 调用外，**不要**在别的任务/串口命令里直接调 SD_xxx。

---

## 六、本次验证方式（如实记录）

- 工程没加新文件前无法整体编译，改用 **armcc V5.06u7 单文件语法自检**（`--cpu Cortex-M3`，宏 `STM32F10X_MD + USE_STDPERIPH_DRIVER`，Include 路径含 Start/Library/User/system/Hardware/FreeRTOS/FatFs）：
  - `DWT_Delay.c`、`SD_SPI.c`：**--c90 严格模式**通过，0 error 0 warning
  - `diskio_spi.c`、`task_sd.c`、`ff.c`：**--c99**（工程实际模式，uC99=1）通过，0 error 0 warning
  - ff.c 编译通过同时验证了 ffconf.h 的裁剪组合自洽（配置非法会在 ff.c 里 `#error`）
- 自检中发现并已修复两处问题：① 工程 core_cm3.h 是老版 CMSIS，没有 DWT 结构体 → 在 `DWT_Delay.h` 加了 `#ifndef DWT` 保护的本地寄存器 shim（换新库自动失效，零冲突）；② 乒乓交接初版用计数信号量认不出"哪块"缓冲空闲，已改成空闲缓冲**指针队列**流转所有权（详见 task_sd.c 注释）。
- 未覆盖：链接期符号检查（Keil 加文件后整体编译）、硬件时序实测——这两步只能等你集成后做。
