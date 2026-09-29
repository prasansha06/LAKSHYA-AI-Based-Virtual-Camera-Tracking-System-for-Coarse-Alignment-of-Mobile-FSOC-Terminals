@echo off
title ISRO FSOC Tracking System - Standalone Executable
echo ================================================================================
echo   ISRO SIH 2026 - Problem Statement 26169
echo   AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC
echo   Launching Standalone Executable (Zero Python / Virtualenv Required)
echo ================================================================================
echo.

set EXE_PATH=%~dp0dist\ISRO_FSOC_Tracker\ISRO_FSOC_Tracker.exe

if not exist "%EXE_PATH%" (
    echo [ERROR] Executable not found at: %EXE_PATH%
    echo Please make sure the project dist folder is present.
    pause
    exit /b 1
)

echo [INFO] Starting Standalone Simulation Executable...
echo.
"%EXE_PATH%"

pause
