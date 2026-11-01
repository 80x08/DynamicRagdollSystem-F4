@echo off
rem Build inside the Visual Studio environment: clang on PATH otherwise hijacks the xmake package (CMake) builds.
rem Pass "config" as the first argument to re-run the configure step (xmake f -m releasedbg --toolchain=msvc).
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" >nul || exit /b 1
cd /d "%~dp0"
set XMAKE=%LOCALAPPDATA%\Programs\xmake\xmake\xmake.exe
if /i "%~1"=="config" (
    "%XMAKE%" f -m releasedbg --toolchain=msvc %2 %3 %4 -y || exit /b 1
)
"%XMAKE%" build -y || exit /b 1