@echo off
setlocal EnableExtensions

set "TARGET=%ANDROID_SERIAL%"
if "%TARGET%"=="" set "TARGET=192.168.43.1:5555"

set "ADB_EXE="

if not defined ADB_EXE if defined ANDROID_SDK_ROOT if exist "%ANDROID_SDK_ROOT%\platform-tools\adb.exe" set "ADB_EXE=%ANDROID_SDK_ROOT%\platform-tools\adb.exe"
if not defined ADB_EXE if defined ANDROID_HOME if exist "%ANDROID_HOME%\platform-tools\adb.exe" set "ADB_EXE=%ANDROID_HOME%\platform-tools\adb.exe"
if not defined ADB_EXE if exist "%LOCALAPPDATA%\Android\Sdk\platform-tools\adb.exe" set "ADB_EXE=%LOCALAPPDATA%\Android\Sdk\platform-tools\adb.exe"
if not defined ADB_EXE for %%I in (adb.exe) do set "ADB_EXE=%%~$PATH:I"

if not defined ADB_EXE (
	echo adb.exe not found. Install Android platform-tools or set ANDROID_SDK_ROOT/ANDROID_HOME.
	exit /b 1
)

"%ADB_EXE%" -s "%TARGET%" %*
exit /b %ERRORLEVEL%
