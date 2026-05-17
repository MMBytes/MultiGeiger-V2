@echo off
REM Host-side test runner. Compiles test/test_main.c with a regular host
REM C compiler and runs it. No ESP-IDF involvement.
REM
REM Usage:    _test.cmd
REM
REM Requires gcc on PATH (install MSYS2 / MinGW-w64 or WSL on Windows;
REM Linux + macOS have gcc/clang out of the box). CI runs the same
REM test via the host-test job in .github/workflows/build.yml — local
REM run is optional.
where gcc >nul 2>nul
if errorlevel 1 (
    echo gcc not found on PATH.
    echo.
    echo Local host tests need a regular C compiler. Either:
    echo   - install MSYS2 ^(https://www.msys2.org/^) and run pacman -S mingw-w64-x86_64-gcc
    echo   - install WSL and run apt-get install gcc
    echo   - or just rely on CI ^(github.com/MMBytes/MultiGeiger-V2/actions^)
    echo.
    exit /b 1
)
cd /d "%~dp0"
gcc -I main -Wall -Wextra -Werror -std=c11 -o test\run.exe test\test_main.c
if errorlevel 1 (
    echo Build failed.
    exit /b 1
)
test\run.exe
set "RC=%ERRORLEVEL%"
del test\run.exe >nul 2>nul
exit /b %RC%
