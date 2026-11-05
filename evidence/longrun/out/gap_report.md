# 17 天账本与 CRC 全量（EXP-01）

> 只读输入目录生成；原始 CSV 未被修改。

## 一、总量

| 项 | 值 |
|---|---|
| 数据文件数 | 24 |
| 事件文件数 | 6 |
| 数据行合计 | 12722768 |
| 字节合计 | 1231916856 |
| 分段总数 | 871（复位/回灌切段） |
| 停机总时长 | 194927 秒 ≈ 1949270 行（不计入缺口） |
| 可信段名义合计 | 12615220 |

## 二、缺口拆分

| 块 | 行数 | 含义 |
|---|---:|---|
| 假缺口（tick 偏慢） | 11943 | 可信段 nominal - tick_expected |
| 文件内真缺口 | 1056787 | tick_expected - rows |
| 跨文件边界缺口 | 0 | Σ(first_seq_next - last_seq_prev - 1)，仅 seq 连续时计 |
| 停机 | 1949270 | 194927 秒，设备没运行 |
| rtc 跳变段（已隔离） | 1176278 | 这些段 rtc 与 tick 不同步，不参与假缺口计算 |
| 未解释余量 | -4178103 | README 的 16175 减去上面各项 |

README 声称总行数 12844939，本次实测 12722768，差 122171。

## 三、事件文件

| 文件 | 类型 | 原因 | 条数 |
|---|---|---|---:|
| 26101890.CSV | LOST | no_buf | 106 |
| 26101890.CSV | LOST | sd_write | 18 |
| 26101890.CSV | LOST | no_buf | 2 |
| 26101890.CSV | SD_RECOVER |  | 0 |
| 26101890.CSV | LOST | no_buf | 146 |
| 26101890.CSV | LOST | sd_write | 18 |
| 26101890.CSV | LOST | no_buf | 2 |
| 26101890.CSV | SD_RECOVER |  | 0 |
| 26101890.CSV | LOST | no_buf | 475 |
| 26101890.CSV | SD_REPLAY |  | 6 |
| 26101890.CSV | LOST | no_buf | 2 |
| 26101890.CSV | SD_RECOVER |  | 0 |
| 26101890.CSV | LOST | no_buf | 506 |
| 26101890.CSV | SD_REPLAY |  | 6 |
| 26101890.CSV | LOST | no_buf | 1 |
| 26101890.CSV | SD_RECOVER |  | 0 |
| 26101890.CSV | LOST | no_buf | 287 |
| 26101890.CSV | SD_REPLAY |  | 6 |
| 26101890.CSV | LOST | no_buf | 2 |
| 26101890.CSV | SD_RECOVER |  | 0 |
| 26101890.CSV | LOST | no_buf | 119 |
| 26101890.CSV | SD_REPLAY |  | 6 |
| 26101890.CSV | LOST | no_buf | 1 |
| 26101890.CSV | SD_RECOVER |  | 0 |
| 26101890.CSV | SD_REPLAY |  | 6 |
| 26101890.CSV | SD_RECOVER |  | 0 |
| 26101890.CSV | LOST | no_buf | 251 |
| 26101890.CSV | SD_REPLAY |  | 6 |
| 26101890.CSV | LOST | no_buf | 1 |
| 26101890.CSV | SD_RECOVER |  | 0 |
| 26101890.CSV | LOST | no_buf | 212 |
| 26101890.CSV | SD_RECOVER |  | 0 |
| 26101890.CSV | LOST | no_buf | 215 |
| 26101890.CSV | SD_REPLAY |  | 6 |
| 26101890.CSV | LOST | no_buf | 4 |
| 26101890.CSV | SD_RECOVER |  | 0 |
| 26101890.CSV | LOST | no_buf | 160 |
| 26101890.CSV | SD_REPLAY |  | 78 |
| 26101890.CSV | SD_RECOVER |  | 0 |
| 26101890.CSV | LOST | no_buf | 1 |
| 26101890.CSV | SD_REPLAY |  | 414 |
| 26101890.CSV | SD_RECOVER |  | 0 |
| 26101890.CSV | LOST | no_buf | 3 |
| 26101890.CSV | SD_RECOVER |  | 0 |
| 26101890.CSV | LOST | no_buf | 4 |
| 26101890.CSV | SD_REPLAY |  | 1441 |
| 26101890.CSV | SD_RECOVER |  | 0 |
| 26101890.CSV | LOST | no_buf | 6 |
| 26101890.CSV | LOST | no_buf | 6 |
| 26101890.CSV | SD_REPLAY |  | 630 |
| 26101890.CSV | SD_RECOVER |  | 0 |
| 26101890.CSV | LOST | no_buf | 9 |
| 26101890.CSV | SD_REPLAY |  | 559 |
| 26101890.CSV | SD_RECOVER |  | 0 |
| 26101890.CSV | LOST | no_buf | 13 |
| 26101890.CSV | SD_REPLAY |  | 606 |
| 26101890.CSV | SD_RECOVER |  | 0 |
| 26101890.CSV | LOST | no_buf | 39 |
| 26101890.CSV | SD_RECOVER |  | 0 |
| 26101890.CSV | LOST | no_buf | 27 |
| 26101890.CSV | SD_REPLAY |  | 804 |
| 26101890.CSV | SD_RECOVER |  | 0 |
| 26101890.CSV | LOST | no_buf | 4 |
| 26101890.CSV | SD_REPLAY |  | 876 |
| 26101890.CSV | SD_RECOVER |  | 0 |
| 26101890.CSV | SD_REPLAY |  | 78 |
| 26101890.CSV | SD_RECOVER |  | 0 |
| 26101890.CSV | SD_RECOVER |  | 0 |
| 26101890.CSV | SD_REPLAY |  | 258 |
| 26101890.CSV | SD_RECOVER |  | 0 |
| 26101890.CSV | SD_RECOVER |  | 0 |
| 26101890.CSV | SD_RECOVER |  | 0 |
| 26101890.CSV | SD_RECOVER |  | 0 |
| 26101890.CSV | SD_RECOVER |  | 0 |
| 26101890.CSV | LOST | no_buf | 1 |
| 26101890.CSV | SD_RECOVER |  | 0 |
| 26101890.CSV | SD_REPLAY |  | 954 |
| 26101890.CSV | SD_RECOVER |  | 0 |
| 26101890.CSV | SD_RECOVER |  | 0 |
| 26101890.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | LOST | no_buf | 172 |
| 26101990.CSV | SD_REPLAY |  | 6 |
| 26101990.CSV | LOST | no_buf | 2 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | LOST | no_buf | 9003 |
| 26101990.CSV | LOST | sd_write | 7396 |
| 26101990.CSV | LOST | no_buf | 5 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | LOST | no_buf | 3 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | LOST | no_buf | 9 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | LOST | no_buf | 3 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | LOST | no_buf | 6 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | LOST | no_buf | 3 |
| 26101990.CSV | SD_REPLAY |  | 74 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | LOST | no_buf | 6 |
| 26101990.CSV | SD_REPLAY |  | 70 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | SD_REPLAY |  | 43 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | LOST | no_buf | 12 |
| 26101990.CSV | LOST | no_buf | 9 |
| 26101990.CSV | LOST | sd_write | 1 |
| 26101990.CSV | LOST | sd_write | 1 |
| 26101990.CSV | LOST | sd_write | 1 |
| 26101990.CSV | SD_REPLAY |  | 168 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | LOST | no_buf | 16 |
| 26101990.CSV | SD_REPLAY |  | 168 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | LOST | no_buf | 2 |
| 26101990.CSV | SD_REPLAY |  | 70 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | SD_REPLAY |  | 15 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | SD_REPLAY |  | 4 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | SD_REPLAY |  | 4 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | SD_REPLAY |  | 32 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | SD_REPLAY |  | 1 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | SD_REPLAY |  | 37 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | LOST | no_buf | 9 |
| 26101990.CSV | SD_REPLAY |  | 13 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | SD_REPLAY |  | 5 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | SD_REPLAY |  | 14 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | SD_REPLAY |  | 8 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | SD_REPLAY |  | 5 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | LOST | no_buf | 2 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | LOST | no_buf | 2 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | SD_REPLAY |  | 70 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | LOST | no_buf | 1 |
| 26101990.CSV | SD_REPLAY |  | 5 |
| 26101990.CSV | SD_REPLAY |  | 66 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | SD_REPLAY |  | 42 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | SD_REPLAY |  | 12 |
| 26101990.CSV | SD_RECOVER |  | 0 |
| 26101990.CSV | SD_REPLAY |  | 146 |
| 26102090.CSV | SD_RECOVER |  | 0 |
| 26102090.CSV | LOST | no_buf | 16 |
| 26102090.CSV | SD_REPLAY |  | 193 |
| 26102090.CSV | SD_RECOVER |  | 0 |
| 26102090.CSV | LOST | no_buf | 12 |
| 26102090.CSV | SD_REPLAY |  | 67 |
| 26102090.CSV | SD_RECOVER |  | 0 |
| 26102090.CSV | LOST | no_buf | 6 |
| 26102090.CSV | SD_REPLAY |  | 32 |
| 26102090.CSV | SD_RECOVER |  | 0 |
| 26102090.CSV | SD_REPLAY |  | 64 |
| 26102090.CSV | SD_RECOVER |  | 0 |
| 26102090.CSV | LOST | no_buf | 25 |
| 26102090.CSV | SD_REPLAY |  | 418 |
| 26102090.CSV | SD_RECOVER |  | 0 |
| 26102090.CSV | SD_REPLAY |  | 70 |
| 26102090.CSV | SD_RECOVER |  | 0 |
| 26102090.CSV | SD_REPLAY |  | 47 |
| 26102090.CSV | SD_RECOVER |  | 0 |
| 26102090.CSV | SD_REPLAY |  | 41 |
| 26102090.CSV | SD_RECOVER |  | 0 |
| 26102090.CSV | LOST | no_buf | 6 |
| 26102090.CSV | SD_REPLAY |  | 40 |
| 26102090.CSV | SD_RECOVER |  | 0 |
| 26102090.CSV | LOST | no_buf | 4 |
| 26102090.CSV | SD_REPLAY |  | 51 |
| 26102290.CSV | SD_RECOVER |  | 0 |
| 26102290.CSV | SD_REPLAY |  | 11 |
| 26102590.CSV | SD_REPLAY |  | 14 |
| 26110390.CSV | SD_LOW |  | 0 |
| 26110390.CSV | SD_LOW |  | 0 |

## 四、这个账本证明不了什么

- 文件首行之前 / 末行之后被截断的部分：没有相邻行可比。
- 跨文件、复位边界：只能按 seq 记缺口，不能证明数据是否产生过。
- 尾批未落盘：从未成为一行，账本里不存在。
- W25 回灌行：tick 不保证单调，已按 Δtick<=0 切段；回灌段编号空间可能与原会话重叠。
- "每秒 10 行"：需要 tick 到物理秒的独立校准，本账本不含。
- CRC 通过只证明内容没坏，不证明行没丢。
- 按 KEY1 暂停过的时间段会被算进"文件内真缺口"：暂停时 tick 照走、seq 冻结，账本无法把它和真丢行区分开，只能靠当时的 # 事件行佐证。

## 五、复现

```
python analyze_longrun.py --input <CSV_DIR> --out <OUT_DIR>
python crc_all.py --input <CSV_DIR> --out <OUT_DIR>
```