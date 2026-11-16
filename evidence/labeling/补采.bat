@echo off
chcp 65001 >nul
title Re-collect: LIGHT (block) + TAP BOARD   ~6 minutes
cd /d "%~dp0"

echo ============================================================
echo   Re-collect two classes   (about 6 minutes total)
echo.
echo   PART 1  light_block   5 rounds x (cover 20s + release 20s)
echo     - Put your whole palm right on top of the photoresistor
echo     - KEEP IT STILL for 20 seconds (do not wave)
echo     - Then take the hand AWAY and hold still for 20 seconds
echo.
echo   PART 2  tap_board     20 rounds x (tap 2s + rest 3s)
echo     - Tap the EDGE OF THE BOARD with your index finger
echo     - This is NOT the table. Touch the board itself.
echo     - About 3 taps per round is fine
echo.
echo   Beeps:  long high = START DOING   short low = STOP
echo   Do NOT click inside this window while it runs.
echo.
echo   Before start:
echo     1. SD card is IN the board
echo     2. Keil UART window is CLOSED
echo     3. Serial assistant is CLOSED
echo ============================================================
echo.

"C:\Users\acnkx\.dsh\dsh-runtimes\dsh-primary-runtime\dependencies\python\python.exe" collect_labels.py --class light_block,tap_board %*
set RC=%ERRORLEVEL%

echo.
echo ============================================================
echo   Exit code: %RC%
echo ============================================================
pause
