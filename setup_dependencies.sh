#!/bin/bash

set -e

echo "Setting up dependencies for Delta Robot Simulator..."
echo ""

# Create external directory
mkdir -p external
cd external

# Check if ImGui already exists
if [ ! -d "imgui" ]; then
    echo "Downloading ImGui..."
    git clone https://github.com/ocornut/imgui.git
    echo "ImGui downloaded successfully"
else
    echo "ImGui already exists, skipping..."
fi

# Check if GLM already exists
if [ ! -d "glm" ]; then
    echo "Downloading GLM..."
    git clone https://github.com/g-truc/glm.git
    echo "GLM downloaded successfully"
else
    echo "GLM already exists, skipping..."
fi

# Check if GLAD already exists
if [ ! -d "glad" ]; then
    echo ""
    echo "GLAD needs to be downloaded manually:"
    echo "1. Go to https://glad.dav1d.de/"
    echo "2. Select:"
    echo "   - Language: C/C++"
    echo "   - Specification: OpenGL"
    echo "   - API gl: Version 3.3"
    echo "   - Profile: Core"
    echo "3. Click 'Generate' and download the ZIP"
    echo "4. Extract glad/include/glad/ to external/glad/include/glad/"
    echo "5. Extract glad/include/KHR/ to external/glad/include/KHR/"
    echo "6. Extract glad/src/glad.c to external/glad/src/glad.c"
    echo ""
else
    echo "GLAD already exists, skipping..."
fi

echo ""
echo "Dependencies setup complete!"
echo "If GLAD is not yet set up, please follow the instructions above."

