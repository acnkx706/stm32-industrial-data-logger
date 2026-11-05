# run_exp01.ps1 -- EXP-01 一键跑：复制到本地 SSD -> manifest -> 全量 CRC
#
# 做了什么：按日期范围挑数据/事件文件，去重后复制到本地暂存目录，再跑两个脚本
# 为什么：规格要求先复制到本地 SSD，避免读卡器读到一半掉线；
#         卡上存在"同名目录条目重复"（26101801.CSV 一份 23MB 一份 0 字节），必须去重
# 影响什么：只读源目录；暂存目录每次重建；结果写 out
# 怎么验证：打印复制文件数、跳过清单，末尾给 rows/false/real/boundary 与 CRC 通过率
param(
    [Parameter(Mandatory = $true)][string]$Src,
    [string]$DateFrom = "261018",
    [string]$DateTo   = "261103",
    [string]$Stage = "",
    [string]$Out = "",
    [double]$Gap = 16175,
    [double]$ExpectedRows = 12844939
)

$ErrorActionPreference = "Stop"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
if ([string]::IsNullOrWhiteSpace($Stage)) { $Stage = Join-Path $here "_stage" }
if ([string]::IsNullOrWhiteSpace($Out))   { $Out   = Join-Path $here "out" }

Write-Host "== 1/3 挑文件 + 复制到本地 SSD =="
if (Test-Path $Stage) { Remove-Item $Stage -Recurse -Force }
New-Item -ItemType Directory -Path $Stage -Force | Out-Null

$all = Get-ChildItem $Src -Filter "*.CSV" -Force |
       Where-Object { $_.Name -match '^\d{8}\.CSV$' } |
       Where-Object {
           $d = $_.Name.Substring(0, 6)
           ($d -ge $DateFrom) -and ($d -le $DateTo)
       }

# 同名条目去重：保留字节数最大的那份（FAT 里可能残留 0 字节残影）
$picked = @{}
$dropped = @()
foreach ($f in $all) {
    if ($picked.ContainsKey($f.Name)) {
        if ($f.Length -gt $picked[$f.Name].Length) {
            $dropped += ("{0} ({1} B, 被更大的同名条目替代)" -f $f.Name, $f.Length)
            $picked[$f.Name] = $f
        } else {
            $dropped += ("{0} ({1} B, 同名条目重复)" -f $f.Name, $f.Length)
        }
    } else {
        $picked[$f.Name] = $f
    }
}

$n = 0
foreach ($f in ($picked.Values | Sort-Object Name)) {
    Copy-Item $f.FullName -Destination $Stage -Force
    $n++
}
$bytes = ($picked.Values | Measure-Object Length -Sum).Sum
Write-Host ("   复制 {0} 个文件，{1:N0} 字节" -f $n, $bytes)
if ($dropped.Count -gt 0) {
    Write-Host "   跳过/合并："
    $dropped | ForEach-Object { Write-Host ("     - " + $_) }
}

Write-Host "== 2/3 manifest / 缺口 / 事件 =="
python (Join-Path $here "analyze_longrun.py") --input $Stage --out $Out --gap $Gap --expected-rows $ExpectedRows

Write-Host "== 3/3 全量 CRC（慢）=="
python (Join-Path $here "crc_all.py") --input $Stage --out $Out

Write-Host ""
Write-Host "结果目录：$Out"
Get-ChildItem $Out | Select-Object Name, Length | Format-Table -AutoSize
