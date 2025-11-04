#!/bin/bash

set -e  # Exit on error

IMGUI_VERSION="v1.90.4"
IMGUI_DIR="external/imgui"

echo "==================================="
echo "Delta Robot Simulator - Build Script"
echo "==================================="

# Download ImGui if not present
if [ ! -d "$IMGUI_DIR" ]; then
    echo "ImGui not found. Downloading version $IMGUI_VERSION..."
    mkdir -p external
    cd external
    
    if command -v wget &> /dev/null; then
        wget -q --show-progress "https://github.com/ocornut/imgui/archive/refs/tags/$IMGUI_VERSION.tar.gz" -O imgui.tar.gz
    elif command -v curl &> /dev/null; then
        curl -L --progress-bar "https://github.com/ocornut/imgui/archive/refs/tags/$IMGUI_VERSION.tar.gz" -o imgui.tar.gz
    else
        echo "Error: Neither wget nor curl found. Please install one of them."
        exit 1
    fi
    
    tar -xzf imgui.tar.gz
    mv "imgui-${IMGUI_VERSION#v}" imgui
    rm imgui.tar.gz
    cd ..
    echo "ImGui downloaded successfully!"
else
    echo "ImGui found at $IMGUI_DIR"
fi

# Create build directory
echo "Creating build directory..."
mkdir -p build
cd build

# Configure with CMake
echo "Configuring CMake..."
cmake -DCMAKE_BUILD_TYPE=Release ..

# Build
echo "Building project..."
cmake --build . -- -j$(nproc)

cd ..

echo "==================================="
echo "Build complete!"
echo "Run the simulator with: ./delta_robot_sim"
echo "==================================="
