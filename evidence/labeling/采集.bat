@echo off
chcp 65001 >nul
title STM32 Recorder - Label Collection
cd /d "%~dp0"

echo ============================================================
echo   Label collection  (about 12 minutes)
echo.
echo   Before start, check:
echo     1. SD card is IN the board
echo     2. Keil UART window is CLOSED
echo     3. Serial assistant is CLOSED
echo     4. Board is steady, wires are not stretched
echo ============================================================
echo.

"C:\Users\acnkx\.dsh\dsh-runtimes\dsh-primary-runtime\dependencies\python\python.exe" collect_labels.py %*
set RC=%ERRORLEVEL%

echo.
echo ============================================================
echo   Exit code: %RC%
echo   (0 = ok, 2 = serial open failed, 3 = no REC line from board)
echo ============================================================
pause
