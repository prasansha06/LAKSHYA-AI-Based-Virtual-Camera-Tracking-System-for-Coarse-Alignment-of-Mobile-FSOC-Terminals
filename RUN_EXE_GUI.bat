@echo off
title ISRO FSOC Tracking System - Standalone Mission Control GUI
echo ================================================================================
echo   ISRO SIH 2026 - Problem Statement 26169
echo   Launching Standalone Mission Control Dashboard (Web GUI + WebSocket Bridge)
echo ================================================================================
echo.

set EXE_PATH=%~dp0dist\ISRO_FSOC_Tracker\ISRO_FSOC_Tracker.exe

if not exist "%EXE_PATH%" (
    echo [ERROR] Executable not found at: %EXE_PATH%
    pause
    exit /b 1
)

echo [INFO] Launching Mission Control Dashboard and WebSocket Bridge...
"%EXE_PATH%" --gui

pause
