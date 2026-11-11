# 串口常开：持续记录 + 通过文件下命令（不断口）
# 用法：pwsh -File mcu_console.ps1 -Seconds 7200 -Out console.log -CmdFile cmd.txt
param(
    [int]$Seconds = 7200,
    [string]$Out = "console.log",
    [string]$CmdFile = "cmd.txt"
)

$ErrorActionPreference = 'Stop'

$port = $null
for ($i = 0; $i -lt 20; $i++) {
    try {
        $port = New-Object System.IO.Ports.SerialPort 'COM12', 115200, 'None', 8, 'One'
        $port.ReadTimeout = 300
        $port.Open()
        break
    } catch {
        Start-Sleep -Milliseconds 500
        $port = $null
    }
}
if ($null -eq $port) { Write-Output "FAILED: COM12 打不开"; exit 1 }

$fs = New-Object System.IO.StreamWriter($Out, $true, [System.Text.Encoding]::UTF8)
$fs.AutoFlush = $true
$fs.WriteLine("#### CONSOLE START $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')")

$sw = [System.Diagnostics.Stopwatch]::StartNew()
$n = 0

while ($sw.Elapsed.TotalSeconds -lt $Seconds) {

    # ---- 命令通道：写文件即可下发，不用断口 ----
    if (Test-Path $CmdFile) {
        try {
            $c = (Get-Content -LiteralPath $CmdFile -Raw -ErrorAction Stop).Trim()
            if ($c.Length -gt 0) {
                $port.WriteLine($c)
                $fs.WriteLine("#### SENT -> $c  @$(Get-Date -Format 'HH:mm:ss')")
            }
            Remove-Item -LiteralPath $CmdFile -Force -ErrorAction SilentlyContinue
        } catch { }
    }

    # ---- 读串口 ----
    try {
        $line = $port.ReadLine()
        if ($line) {
            $fs.WriteLine((Get-Date).ToString('HH:mm:ss.fff') + ' ' + $line.Trim())
            $n++
        }
    } catch { }
}

$fs.WriteLine("#### CONSOLE END rows=$n elapsed=$([int]$sw.Elapsed.TotalSeconds)s")
$fs.Close()
$port.Close()
