@echo off
title ISRO FSOC Tracking System - Standalone Benchmark Stage 1
echo ================================================================================
echo   ISRO SIH 2026 - Problem Statement 26169
echo   Running Automated Benchmark Stage 1 (Straight, Circular, Figure-8, Random)
echo ================================================================================
echo.

set EXE_PATH=%~dp0dist\ISRO_FSOC_Tracker\ISRO_FSOC_Tracker.exe

if not exist "%EXE_PATH%" (
    echo [ERROR] Executable not found at: %EXE_PATH%
    pause
    exit /b 1
)

echo [INFO] Running Stage 1 Automated Scenarios...
"%EXE_PATH%" --benchmark1

pause
