# Open Source Delta Robot - Software Foundation

This is the software foundation for an open source delta robot project. The codebase provides a complete 3D simulation and visualisation platform for developing, testing, and validating delta robot kinematics before hardware integration. Built with C++, OpenGL, and ImGui, the kinematics engine serves as the foundation for real hardware control.

## Features

- Interactive 3D visualisation of a delta robot
- Real-time inverse kinematics as you move the end effector
- ImGui-based control interface
- Adjustable robot configuration parameters
- Camera controls (rotate, zoom)
- Visual feedback for valid/invalid positions

## Requirements

- CMake 3.15 or higher
- C++17 compatible compiler
- OpenGL 3.3 or higher
- GLFW 3.x
- GLM (included as header-only)
- GLAD (for OpenGL loading)
- ImGui (for UI)

## Setup

### 1. Install Dependencies

#### Ubuntu/Debian:
```bash
sudo apt-get update
sudo apt-get install build-essential cmake libglfw3-dev libgl1-mesa-dev
```

#### macOS (with Homebrew):
```bash
brew install cmake glfw
```

### 2. Download GLAD

1. Go to https://glad.dav1d.de/
2. Select:
   - Language: C/C++
   - Specification: OpenGL
   - API gl: Version 3.3
   - Profile: Core
3. Click "Generate"
4. Download the generated ZIP file
5. Extract `glad/include/glad/` to `external/glad/include/glad/`
6. Extract `glad/include/KHR/` to `external/glad/include/KHR/`
7. Extract `glad/src/glad.c` to `external/glad/src/glad.c`

### 3. Download ImGui

```bash
cd external
git clone https://github.com/ocornut/imgui.git
cd imgui
```

Or download the latest release from https://github.com/ocornut/imgui/releases and extract to `external/imgui/`

### 4. Download GLM

```bash
cd external
git clone https://github.com/g-truc/glm.git
```

Or download from https://github.com/g-truc/glm/releases

## Building

### Option 1: Using the build script

```bash
chmod +x build.sh
./build.sh
```

### Option 2: Manual build

```bash
mkdir build
cd build
cmake ..
make
```

The executable will be in the `build/` directory.

## Running

```bash
./build/DeltaRobotSim
```

## Controls

### Mouse Controls:
- **Left Click + Drag**: Rotate camera around the robot
- **Right Click + Drag**: Zoom in/out
- **Scroll Wheel**: Zoom in/out

### ImGui Interface:
- **End Effector Control**: Drag sliders or use buttons to move the end effector
- **Robot Configuration**: Adjust robot dimensions (base radius, arm lengths, etc.)
- **Display Options**: Toggle grid and coordinate axes

## Project Structure

```
delta_robot_sim/
├── CMakeLists.txt          # CMake build configuration
├── build.sh                # Build script
├── README.md               # This file
├── include/
│   ├── DeltaRobot.hpp      # Delta robot kinematics class
│   └── Renderer.hpp        # OpenGL renderer class
├── src/
│   ├── main.cpp            # Main application entry point
│   ├── DeltaRobot.cpp      # Delta robot implementation
│   └── Renderer.cpp        # Renderer implementation
└── external/
    ├── glad/               # GLAD OpenGL loader
    ├── imgui/              # ImGui library
    └── glm/                # GLM math library
```

## Notes

- The robot uses inverse kinematics to automatically calculate joint positions when you move the end effector
- Invalid positions (outside workspace) will be shown in red
- The workspace is approximately spherical, limited by arm lengths
- Z-axis is up (negative Z is down)

## Hardware Controller

The system is designed to communicate with a **NUCLEO-H7S3L8** microcontroller over Ethernet. The NUCLEO-H7S3L8 runs custom firmware that receives binary protocol commands and controls the robot's 3 NEMA 23 stepper motors.

### Communication Protocol
- **Protocol**: Custom binary protocol (no G-code)
- **Connection**: Ethernet (TCP/IP)
- **Port**: 8080 (default)
- **Format**: Binary packets with checksum validation

See `PROTOCOL_DESIGN.md` for detailed protocol specifications.

## Roadmap

This simulation platform serves as the foundation for the open source delta robot project. Planned enhancements include:

- Complete NUCLEO-H7S3L8 firmware implementation
- Hardware interface abstraction layer (stepper motors, endstops)
- Real-time control loops and trajectory planning
- Safety features and limits checking
- Configuration file support for different robot configurations

## Troubleshooting

If you encounter linking errors:
- Ensure GLFW is properly installed
- Check that GLAD and ImGui are in the correct directories
- Verify CMake found all required packages

If the window doesn't appear:
- Check that your graphics drivers support OpenGL 3.3+
- Verify the build completed without errors

