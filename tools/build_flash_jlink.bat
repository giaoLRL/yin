@echo off
setlocal

set "PROJECT_DIR=%~dp0.."
set "JLINKEXE=D:\Program Files\SEGGER\JLink\JLink.exe"
set "ELF=%PROJECT_DIR:\=/%/Debug/empty_cpp.out"
set "PROGRAM_COMMAND=loadfile %ELF% verify reset go"
if /I "%~1"=="/halt" set "PROGRAM_COMMAND=loadfile %ELF% verify exit"
if /I "%~1"=="/unlock" (
    echo Unlocking device...
    "%JLINKEXE%" -Device MSPM0G3507 -if SWD -speed 50 -CommanderScript "%PROJECT_DIR%\jlink_unlock.txt"
    if errorlevel 1 (
        echo Unlock failed.
        exit /b 1
    )
)

echo [1/2] Building project...
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0build_project.ps1"
if errorlevel 1 (
    echo Build failed. Flash was not changed.
    exit /b 1
)

echo [2/2] Programming with J-Link...
"%JLINKEXE%" -Device MSPM0G3507 -if SWD -speed 50 -CommanderScript "%PROJECT_DIR%\jlink_cmd.txt"
if errorlevel 1 (
    echo Programming failed.
    echo Check target power, GND, 3.3V/VTref, SWDIO, SWCLK, and nRST.
    echo Make sure J-Link is connected properly.
    exit /b 1
)

echo Programming completed successfully.
exit /b 0
