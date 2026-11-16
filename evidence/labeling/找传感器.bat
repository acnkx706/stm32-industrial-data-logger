@echo off
chcp 65001 >nul
title Find sensors - live monitor
cd /d "%~dp0"

echo ============================================================
echo   Live sensor monitor
echo.
echo   Watch the numbers while you cover components one by one:
echo     light drops to under 500   -> that is the photoresistor
echo     humi rises after ~10s      -> that is the DHT22
echo.
echo   Ctrl+C to quit.
echo ============================================================
echo.

"C:\Users\acnkx\.dsh\dsh-runtimes\dsh-primary-runtime\dependencies\python\python.exe" watch_sensors.py %*

echo.
pause
