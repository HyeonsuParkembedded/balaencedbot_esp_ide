@echo off
echo Building and running native tests...

echo.
echo Building native test environment...
pio run -e native

if %errorlevel% neq 0 (
    echo Build failed!
    exit /b 1
)

echo.
echo Running native tests...
pio test -e native -v

echo.
echo Test run complete.
pause