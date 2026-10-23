@echo off
setlocal
cd /d "%~dp0"
if not exist build mkdir build
set "COMMON_SOURCES=src\moving_average.c src\pi_controller.c src\wheel_model.c src\simulation.c"
if /I "%~1"=="msvc" goto build_msvc
if /I "%~1"=="gcc" goto build_gcc
if not "%~1"=="" (
    echo Usage: build_windows.bat [gcc^|msvc]
    exit /b 2
)

where gcc >nul 2>nul
if not errorlevel 1 goto build_gcc
where cl >nul 2>nul
if not errorlevel 1 goto build_msvc
echo ERROR: C compiler not found. Install MinGW-w64 GCC or use the VS Developer Command Prompt.
exit /b 1

:build_gcc
gcc -std=c99 -Wall -Wextra -Wpedantic -Werror -O2 -Iinclude %COMMON_SOURCES% src\main.c -o build\wheel_sim.exe -lm
if errorlevel 1 exit /b 1
gcc -std=c99 -Wall -Wextra -Wpedantic -Werror -O2 -Iinclude %COMMON_SOURCES% tests\test_main.c -o build\wheel_tests.exe -lm
if errorlevel 1 exit /b 1
goto verify

:build_msvc
cl /nologo /std:c11 /utf-8 /W4 /WX /O2 /D_CRT_SECURE_NO_WARNINGS /Iinclude %COMMON_SOURCES% src\main.c /Febuild\wheel_sim.exe /Fobuild\
if errorlevel 1 exit /b 1
cl /nologo /std:c11 /utf-8 /W4 /WX /O2 /D_CRT_SECURE_NO_WARNINGS /Iinclude %COMMON_SOURCES% tests\test_main.c /Febuild\wheel_tests.exe /Fobuild\
if errorlevel 1 exit /b 1

:verify
build\wheel_tests.exe
if errorlevel 1 exit /b 1
build\wheel_sim.exe --output build\simulation.csv
if errorlevel 1 exit /b 1
echo SUCCESS: tests passed; output is build\simulation.csv
exit /b 0
