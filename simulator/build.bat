@echo off
rem Same as the plugin's build.bat: build inside the Visual Studio environment so clang on PATH can't hijack packages.
rem "build.bat config" re-runs configure; "build.bat run" builds and starts the simulator.
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" >nul || exit /b 1
cd /d "%~dp0"
set XMAKE=%LOCALAPPDATA%\Programs\xmake\xmake\xmake.exe
if /i "%~1"=="config" (
    "%XMAKE%" f -P . -p windows -a x64 -m releasedbg --toolchain=msvc -y || exit /b 1
)
"%XMAKE%" build -P . -y || exit /b 1
if /i "%~1"=="run" "%XMAKE%" run -P . RagdollLabSim
