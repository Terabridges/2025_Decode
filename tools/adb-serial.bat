@echo off
setlocal
set "TARGET=%ANDROID_SERIAL%"
if "%TARGET%"=="" set "TARGET=192.168.43.1:5555"
adb -s %TARGET% %*
