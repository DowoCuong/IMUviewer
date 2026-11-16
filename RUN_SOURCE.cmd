@echo off
setlocal
"%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe" -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0packaging\run_source.ps1" %*
set "imuExit=%errorlevel%"
if "%imuExit%"=="0" exit /b 0
echo IMU Viewer dung voi ma loi %imuExit%. Xem thong bao phia tren.
pause
exit /b %imuExit%
