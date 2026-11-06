# evidence/longrun —— EXP-01：17 天账本 + CRC 全量

> 目的：给 10-18 ~ 11-03 的数据文件建 manifest，把 README 的 16,175 行缺口拆成
> 「假缺口 / 文件内真缺口 / 跨文件边界 / 未解释余量」四块，并明确账本覆盖不了什么。

## 一键跑

```powershell
# 卡插在读卡器里，假设是 E:
.\run_exp01.ps1 -Src <卡>:\
```

脚本会先把 CSV 复制到本地 `_stage\`（规格要求，避免读卡器读一半掉线），再跑两个分析脚本。

也可以分开跑：

```powershell
python analyze_longrun.py --input <卡>:\ --out out
python crc_all.py        --input <卡>:\ --out out
```

## 产出

| 文件 | 内容 |
|---|---|
| `manifest_17d.csv` | 一行一个数据文件：行数/字节/SHA256/首末 seq-tick-rtc/分段/Δtick 分布/三段缺口 |
| `crc_all.csv` | 一行一个文件：rows / crc_ok / crc_bad / 首条坏行 / 备注 |
| `events_summary.csv` | 90 事件文件里的 `# LOST` 等事件，按类型与原因计数 |
| `gap_report.md` | 汇总、四块拆分、覆盖不了的边界 |
| `expected_longrun.txt` | 冻结的汇总数字，供 CI diff（不含路径与时间戳） |

## 口径（改之前先读）

```
段          相邻两行 Δtick <= 0 处切开（复位、W25 回灌都会造成）
可信段      tick 与 rtc 的比值落在 0.9~1.1；不在此区间记为 rtc 跳变段，单独统计
per 段      tick_expected = tick_span/100 + 1 ，  nominal = rtc_span*10
假缺口      Σ可信段 (nominal - tick_expected)       ← tick 时间基比物理秒快/慢造成的
真缺口      Σ全部段 tick_expected - rows            ← 时间基下仍然缺的行
停机        Σ相邻段 (next.first_rtc - cur.last_rtc)，只累加正值
边界缺口    仅当「日期相邻 + tick 连续 + seq 增量 == tick 增量/100」时才算，此时为 0
```

三条必须记住的：

1. **不要用 864,000/天 直接减实际行数就喊丢行**，先扣掉 tick 偏慢造成的假缺口与停机。
2. **不要用 Σ floor(Δtick/100)**，用 `Σ(tick_last - tick_first)/100` 或逐段公式。
3. **CRC 通过只证明内容没坏，不证明行没丢**，它与缺口账本是两件事。

## 这个账本证明不了什么

- 文件首行之前 / 末行之后的截断：没有相邻行可比。
- 跨文件、复位边界：只能按 seq 记，不能证明数据是否产生过。
- 尾批未落盘：从未成为一行。
- W25 回灌行：tick 不单调，已按 Δtick<=0 切段，但回灌段编号空间可能与原会话重叠。
- 「每秒 10 行」：需要 tick 到物理秒的独立校准。
- 按 KEY1 暂停的时间段会被算进「真缺口」：暂停时 tick 照走、seq 冻结，账本无法与真丢行区分。

## 验收（跑完自己核一遍）

1. CRC 自检 `CRC16/KERMIT("123456789") == 0x2189` 通过（`crc_all.py` 启动即查，不过直接退出）。
2. 数据文件数与总行数与 README 对得上，或差异有解释。
3. 逐文件 `false_gap + real_gap == nominal - rows`（±1 行舍入）。
4. 四块之和与 README 的 16,175 差 ≤ 50 行；超了写「未解释」，不硬凑。
5. `gap_report.md` 里六条「证明不了什么」原样保留。

