@echo off
setlocal enabledelayedexpansion

echo Building Delta Robot Simulator...
echo.

REM Check if CMake is available
where cmake >nul 2>&1
if errorlevel 1 (
    REM Try to find CMake in Visual Studio installation
    set "CMAKE_PATH="
    REM Check VS 2022 (version 17)
    if exist "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" (
        set "CMAKE_PATH=C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin"
    ) else if exist "C:\Program Files\Microsoft Visual Studio\2022\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" (
        set "CMAKE_PATH=C:\Program Files\Microsoft Visual Studio\2022\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin"
    ) else if exist "C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" (
        set "CMAKE_PATH=C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin"
    REM Check VS 2019 (version 16)
    ) else if exist "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" (
        set "CMAKE_PATH=C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin"
    ) else if exist "C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" (
        set "CMAKE_PATH=C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin"
    REM Check for standalone CMake installation
    ) else if exist "C:\Program Files\CMake\bin\cmake.exe" (
        set "CMAKE_PATH=C:\Program Files\CMake\bin"
    ) else if exist "C:\Program Files (x86)\CMake\bin\cmake.exe" (
        set "CMAKE_PATH=C:\Program Files (x86)\CMake\bin"
    )
    
    if defined CMAKE_PATH (
        set "PATH=%CMAKE_PATH%;%PATH%"
        echo Found CMake in: %CMAKE_PATH%
    ) else (
        echo Error: CMake is not installed or not in PATH.
        echo Please install CMake from https://cmake.org/download/
        echo Or install Visual Studio with CMake component.
        exit /b 1
    )
)

REM Check if compiler is available (try MSVC first, then MinGW)
set "USE_MSVC=0"
where cl >nul 2>&1
if not errorlevel 1 (
    set "USE_MSVC=1"
    echo Using MSVC compiler (already in PATH)
) else (
    REM Try to find and set up Visual Studio environment
    set "VS_SETUP="
    if exist "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvarsall.bat" (
        set "VS_SETUP=C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvarsall.bat"
    ) else if exist "C:\Program Files\Microsoft Visual Studio\2025\Community\VC\Auxiliary\Build\vcvarsall.bat" (
        set "VS_SETUP=C:\Program Files\Microsoft Visual Studio\2025\Community\VC\Auxiliary\Build\vcvarsall.bat"
    ) else if exist "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat" (
        set "VS_SETUP=C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat"
    ) else if exist "C:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" (
        set "VS_SETUP=C:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat"
    ) else if exist "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvarsall.bat" (
        set "VS_SETUP=C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvarsall.bat"
    ) else if exist "C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" (
        set "VS_SETUP=C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\VC\Auxiliary\Build\vcvarsall.bat"
    )
    
    if defined VS_SETUP (
        echo Setting up Visual Studio environment from: !VS_SETUP!
        call "!VS_SETUP!" x64
        if errorlevel 1 (
            echo Warning: Failed to set up Visual Studio environment.
            set "VS_SETUP="
        ) else (
            where cl >nul 2>&1
            if not errorlevel 1 (
                set "USE_MSVC=1"
                echo Using MSVC compiler
            )
        )
    )
    
    if "!USE_MSVC!"=="0" (
        where g++ >nul 2>&1
        if errorlevel 1 (
            echo Error: No C++ compiler found. Please install:
            echo - Visual Studio with C++ support
            echo - MinGW-w64
            echo.
            echo Note: If Visual Studio is installed, you may need to run this script from
            echo a Visual Studio Developer Command Prompt, or install the C++ workload.
            exit /b 1
        )
        echo Using MinGW compiler
    )
)

REM Create build directory
if not exist "build" mkdir build
cd build

REM Run CMake
if "!USE_MSVC!"=="1" (
    REM Use MSVC generator - try different versions
    cmake .. -G "Visual Studio 17 2022" -A x64
    if errorlevel 1 (
        cmake .. -G "Visual Studio 16 2019" -A x64
        if errorlevel 1 (
            cmake .. -G "Visual Studio 15 2017" -A x64
            if errorlevel 1 (
                echo Trying default CMake generator...
                cmake ..
            )
        )
    )
) else (
    REM Use MinGW Makefiles
    cmake .. -G "MinGW Makefiles"
    if errorlevel 1 (
        echo Error: Failed to configure CMake with MinGW.
        exit /b 1
    )
)

if errorlevel 1 (
    echo Error: CMake configuration failed.
    exit /b 1
)

REM Build
if "!USE_MSVC!"=="1" (
    REM Build with MSVC
    cmake --build . --config Release
    if errorlevel 1 (
        echo Error: Build failed.
        exit /b 1
    )
    set "EXE_PATH=Release\DeltaRobotSim.exe"
) else (
    REM Build with MinGW
    cmake --build . -- -j%NUMBER_OF_PROCESSORS%
    if errorlevel 1 (
        echo Error: Build failed.
        exit /b 1
    )
    set "EXE_PATH=DeltaRobotSim.exe"
)

cd ..

echo.
echo Build complete! Executable: build\!EXE_PATH!
echo.
echo To run the application:
echo   cd build
if "!USE_MSVC!"=="1" (
    echo   Release\DeltaRobotSim.exe
) else (
    echo   .\DeltaRobotSim.exe
)
echo.

endlocal
