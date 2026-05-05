@echo off
echo Compiling firmware using PlatformIO...
pio run
if %ERRORLEVEL% neq 0 (
    echo Compilation failed!
    exit /b %ERRORLEVEL%
)

echo.
echo Converting ELF to HEX...
set OBJCOPY="%USERPROFILE%\.platformio\packages\toolchain-riscv\bin\riscv-wch-elf-objcopy.exe"
if not exist %OBJCOPY% (
    echo Error: objcopy not found at %OBJCOPY%
    exit /b 1
)

%OBJCOPY% -O ihex .pio\build\genericCH32V203C8T6\firmware.elf .pio\build\genericCH32V203C8T6\firmware.hex
if %ERRORLEVEL% neq 0 (
    echo Conversion failed!
    exit /b %ERRORLEVEL%
)

echo.
echo SUCCESS! Firmware HEX generated at .pio\build\genericCH32V203C8T6\firmware.hex
