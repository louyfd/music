@echo off
rem Rebuild SPP Assistant: onedir app folder + single-file exe
rem Pure ASCII + CRLF on purpose: cmd.exe reads the OEM codepage and expects CRLF.
setlocal
cd /d "%~dp0"

py -3.9 -c "import PyInstaller" 2>nul
if errorlevel 1 goto nopyinstaller

py -3.9 make_icon.py
if errorlevel 1 goto failed

py -3.9 -m PyInstaller --noconfirm --clean --onedir --windowed --name SPP_Assistant --icon "%~dp0spp.ico" --collect-submodules mido --distpath "%~dp0_out_onedir" --workpath "%~dp0_build_onedir" --specpath "%~dp0_build_onedir" "%~dp0spp_assistant.py"
if errorlevel 1 goto failed

py -3.9 -m PyInstaller --noconfirm --clean --onefile --windowed --name SPP_Assistant --icon "%~dp0spp.ico" --collect-submodules mido --distpath "%~dp0_out_onefile" --workpath "%~dp0_build_onefile" --specpath "%~dp0_build_onefile" "%~dp0spp_assistant.py"
if errorlevel 1 goto failed

if exist "SPP_Assistant" rmdir /s /q "SPP_Assistant"
move /y "_out_onedir\SPP_Assistant" "SPP_Assistant" >nul
if errorlevel 1 goto failed

move /y "_out_onefile\SPP_Assistant.exe" "SPP_Assistant_onefile.exe" >nul
if errorlevel 1 goto failed

rmdir /s /q "_out_onedir" 2>nul
rmdir /s /q "_out_onefile" 2>nul
rmdir /s /q "_build_onedir" 2>nul
rmdir /s /q "_build_onefile" 2>nul

echo.
echo BUILD OK
echo   app folder : SPP_Assistant\SPP_Assistant.exe
echo   single file: SPP_Assistant_onefile.exe
goto end

:nopyinstaller
echo PyInstaller missing. Run:  py -3.9 -m pip install pyinstaller
goto end

:failed
echo BUILD FAILED
goto end

:end
endlocal
pause
