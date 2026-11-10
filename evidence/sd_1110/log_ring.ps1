# 盯串口，把每行带时间戳记下来，用于抓 "ovr 从 0 变 1" 的那一刻
# 用法：pwsh -File log_ring.ps1 <秒数> <输出文件>
param(
    [int]$Seconds = 2700,
    [string]$Out = "ring.log"
)

$ErrorActionPreference = 'Stop'
$port = New-Object System.IO.Ports.SerialPort 'COM12', 115200, 'None', 8, 'One'
$port.ReadTimeout = 300
$port.Open()

$fs = New-Object System.IO.StreamWriter($Out, $true, [System.Text.Encoding]::UTF8)
$fs.AutoFlush = $true

$sw = [System.Diagnostics.Stopwatch]::StartNew()
$n = 0
$lastOvr = -1
while ($sw.Elapsed.TotalSeconds -lt $Seconds) {
    try {
        $line = $port.ReadLine()
        if ($line) {
            $t = $line.Trim()
            $fs.WriteLine((Get-Date).ToString('HH:mm:ss.fff') + ' ' + $t)
            $n++
            if ($t -match 'ovr=(\d+)') {
                $v = [int]$Matches[1]
                if ($v -ne $lastOvr) {
                    $fs.WriteLine("#### OVR CHANGE -> $v at $((Get-Date).ToString('HH:mm:ss'))")
                    $lastOvr = $v
                }
            }
        }
    } catch { }
}
$fs.WriteLine("#### LOG END rows=$n elapsed=$([int]$sw.Elapsed.TotalSeconds)s")
$fs.Close()
$port.Close()
