@echo off
setlocal
if not exist "%~dp0dist\IMUJointViewer\IMUJointViewer.exe" (
  echo Khong tim thay portable dist\IMUJointViewer\IMUJointViewer.exe
  exit /b 1
)
start "" "%~dp0dist\IMUJointViewer\IMUJointViewer.exe" %*
