@echo off
REM Host-side test runner. Compiles test/test_main.c and main/history.c
REM (through the test/shim stand-in headers) with a regular host C compiler
REM and runs the result. No ESP-IDF involvement.
REM
REM Usage:    _test.cmd [test-name-substring]
REM
REM Requires gcc on PATH (install MSYS2 / MinGW-w64 or WSL on Windows;
REM Linux + macOS have gcc/clang out of the box). CI runs the same
REM test via the host-test job in .github/workflows/build.yml - local
REM run is optional.
REM
REM Keep this file CRLF and ASCII-only. cmd.exe reads an LF-terminated REM
REM line as a command to EXECUTE rather than a comment, so an LF copy of this
REM file prints one "'M' is not recognized" error per comment line on every
REM run - which is how it shipped from the host-test job's introduction until
REM V2.7.3. Nothing worse happened only because no REM line here contains a
REM redirect; one that did would silently create a junk file in the repo root.
REM .gitattributes now pins the endings so a checkout cannot undo this, but it
REM cannot police the character set - keep the comments 7-bit.
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
REM V2.8.8: same flags and sources as .github/workflows/_host-test.yml
REM (HOST_TEST_CFLAGS / HOST_TEST_SRCS) - keep the two in step. test\shim
REM stands in for the ESP-IDF headers that main\history.c includes.
gcc -I test\shim -I main -DBOARD_HELTEC_V2=1 -Wall -Wextra -Werror -std=c11 -g -O1 -o test\run.exe test\test_main.c main\history.c
if errorlevel 1 (
    echo Build failed.
    exit /b 1
)
REM Optional argument: run only tests whose name contains it (e.g. history_).
test\run.exe %1
set "RC=%ERRORLEVEL%"
del test\run.exe >nul 2>nul
exit /b %RC%
