# 录"满环回写"全过程：等到 replay done 或超时
# 用法：pwsh -File log_replay.ps1 -Seconds 12000 -Out xxx.log
param(
    [int]$Seconds = 12000,
    [string]$Out = "replay.log"
)

$ErrorActionPreference = 'Stop'

# 端口可能被上一个任务占着，重试 20 次
$port = $null
for ($i = 0; $i -lt 20; $i++) {
    try {
        $port = New-Object System.IO.Ports.SerialPort 'COM12', 115200, 'None', 8, 'One'
        $port.ReadTimeout = 500
        $port.Open()
        break
    } catch {
        Start-Sleep -Milliseconds 500
        $port = $null
    }
}
if ($null -eq $port) { Write-Output "FAILED: COM12 打不开"; exit 1 }

$fs = New-Object System.IO.StreamWriter($Out, $false, [System.Text.Encoding]::UTF8)
$fs.AutoFlush = $true
$fs.WriteLine("#### START $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')")

$sw = [System.Diagnostics.Stopwatch]::StartNew()
$n = 0
$lastLeft = -1
$done = $false
$t0 = Get-Date

while ($sw.Elapsed.TotalSeconds -lt $Seconds -and -not $done) {
    try {
        $line = $port.ReadLine()
        if ($line) {
            $t = $line.Trim()
            $fs.WriteLine((Get-Date).ToString('HH:mm:ss.fff') + ' ' + $t)
            $n++

            if ($t -match 'left=(\d+)') {
                $v = [int]$Matches[1]
                # 每 500 批记一次，避免刷屏
                if ($lastLeft -lt 0 -or [math]::Abs($v - $lastLeft) -ge 500) {
                    $fs.WriteLine("#### LEFT=$v at $((Get-Date).ToString('HH:mm:ss')) elapsed=$([int]$sw.Elapsed.TotalSeconds)s")
                    $lastLeft = $v
                }
            }
            if ($t -match 'replay done' -or $t -match 'BAD CACHE') {
                $fs.WriteLine("#### MARK: $t")
            }
            if ($t -match 'replay done') { $done = $true }
        }
    } catch { }
}

$fs.WriteLine("#### END rows=$n elapsed=$([int]$sw.Elapsed.TotalSeconds)s done=$done")
$fs.Close()
$port.Close()
