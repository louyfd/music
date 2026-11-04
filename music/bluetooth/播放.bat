@echo off
cd /d "%~dp0"
set PORT=COM16
echo Twinkle Twinkle -^> FPGA on %PORT%   (do=05 sol=09 la=0A)
echo.
python "%~dp0twinkle.py" %PORT%
echo.
pause
