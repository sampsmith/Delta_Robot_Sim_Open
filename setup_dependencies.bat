@echo off
setlocal enabledelayedexpansion

echo Setting up dependencies for Delta Robot Simulator...
echo.

REM Create external directory
if not exist "external" mkdir external
cd external

REM Check if ImGui already exists
if not exist "imgui" (
    echo Downloading ImGui...
    git clone https://github.com/ocornut/imgui.git
    if errorlevel 1 (
        echo Error: Failed to download ImGui. Make sure git is installed.
        exit /b 1
    )
    echo ImGui downloaded successfully
) else (
    echo ImGui already exists, skipping...
)

REM Check if GLM already exists
if not exist "glm" (
    echo Downloading GLM...
    git clone https://github.com/g-truc/glm.git
    if errorlevel 1 (
        echo Error: Failed to download GLM. Make sure git is installed.
        exit /b 1
    )
    echo GLM downloaded successfully
) else (
    echo GLM already exists, skipping...
)

REM Check if GLAD already exists
if not exist "glad" (
    echo.
    echo GLAD needs to be downloaded manually:
    echo 1. Go to https://glad.dav1d.de/
    echo 2. Select:
    echo    - Language: C/C++
    echo    - Specification: OpenGL
    echo    - API gl: Version 3.3
    echo    - Profile: Core
    echo 3. Click 'Generate' and download the ZIP
    echo 4. Extract glad/include/glad/ to external/glad/include/glad/
    echo 5. Extract glad/include/KHR/ to external/glad/include/KHR/
    echo 6. Extract glad/src/glad.c to external/glad/src/glad.c
    echo.
) else (
    echo GLAD already exists, skipping...
)

cd ..

echo.
echo Dependencies setup complete!
echo If GLAD is not yet set up, please follow the instructions above.

endlocal


