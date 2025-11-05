#!/bin/bash

set -e

echo "Building Delta Robot Simulator..."

# Create build directory
mkdir -p build
cd build

# Run CMake
cmake ..

# Build
make -j$(nproc)

echo ""
echo "Build complete! Executable: ./build/DeltaRobotSim"
echo ""

