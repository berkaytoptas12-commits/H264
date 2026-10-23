@echo off
rem Builds dist\h264_gui.exe (window) and dist\h264_send.exe (console) with
rem PyInstaller.  Needs 64-bit Python 3.8+ from python.org (Tkinter included).
rem On Linux: scripts/build_exe_wine.sh
cd /d "%~dp0"
python -m pip install --upgrade pyinstaller pyserial || goto :fail
python -m PyInstaller --noconfirm --clean --onefile --windowed --name h264_gui h264_gui.py || goto :fail
python -m PyInstaller --noconfirm --clean --onefile --console --name h264_send h264_send.py || goto :fail
echo.
echo Hazir: %~dp0dist\h264_gui.exe ve %~dp0dist\h264_send.exe
pause
exit /b 0
:fail
echo Derleme basarisiz.
pause
exit /b 1
