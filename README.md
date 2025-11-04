# Delta Robot Simulator

A high-performance 3D visualization and inverse kinematics simulator for delta parallel robots. Built with C++, OpenGL, and Dear ImGui for real-time testing and validation before deploying to physical hardware.

![Delta Robot](docs/screenshot.png)

## Features

- **Real-time 3D Visualization**: Interactive OpenGL rendering with camera controls
- **Inverse Kinematics**: Fast IK solver for calculating joint angles from end-effector positions
- **Interactive GUI**: ImGui-based control panel with position sliders and joint angle display
- **High Performance**: Optimized with `-O3 -march=native` compiler flags
- **Easy Setup**: Automated build script that downloads dependencies

## Delta Robot Architecture

A delta robot is a parallel manipulator with three arms connected to a triangular base and platform:

- **Base Triangle**: Three fixed joints mounted on a base (120° apart)
- **Upper Arms**: Connected to motors at the base joints
- **Lower Arms**: Parallelogram linkages that maintain platform orientation
- **Platform/End Effector**: The moving platform where tools are attached

### Default Parameters

- Base Radius: 200 mm
- Platform Radius: 50 mm
- Upper Arm Length: 300 mm
- Lower Arm Length: 600 mm
- Workspace: X/Y ±300mm, Z -900mm to -200mm

## Dependencies

### System Requirements

- **C++ Compiler**: GCC 7.5+ or Clang (with C++17 support)
- **CMake**: Version 3.10 or higher
- **OpenGL**: Version 3.0 or higher
- **GLFW3**: For window management and input handling

### Ubuntu/Debian Installation

```bash
sudo apt update
sudo apt install build-essential cmake libglfw3-dev libgl1-mesa-dev
```

### Arch Linux Installation

```bash
sudo pacman -S base-devel cmake glfw-x11 mesa
```

### Automatic Dependencies

The build script automatically downloads:
- **Dear ImGui v1.90.4**: GUI framework

## Building the Project

### Quick Build

```bash
./build.sh
```

The build script will:
1. Download Dear ImGui if not present
2. Create a build directory
3. Configure CMake with optimizations
4. Compile the project using all CPU cores

### Manual Build

```bash
# Download ImGui manually
mkdir -p external
cd external
wget https://github.com/ocornut/imgui/archive/refs/tags/v1.90.4.tar.gz
tar -xzf v1.90.4.tar.gz
mv imgui-1.90.4 imgui
cd ..

# Build
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . -- -j$(nproc)
cd ..
```

## Running the Simulator

```bash
./delta_robot_sim
```

## Controls

### Mouse Controls
- **Left Mouse Drag**: Rotate camera around the robot
- **Scroll Wheel**: Zoom in/out

### GUI Controls
- **Position Sliders**: Adjust X, Y, Z coordinates of end-effector
- **Preset Buttons**: Quick access to common positions
  - Home: Center position (0, 0, -500)
  - Top: Highest reachable point
  - Bottom: Lowest reachable point
  - Corner: Test extreme workspace position
- **Reset Camera**: Return to default view angle

### Visual Indicators
- **Green "IK: VALID"**: Current position is reachable
- **Red "IK: OUT OF REACH"**: Position exceeds robot workspace
- **Joint Angles**: Real-time display in degrees

## Color Legend

- 🔴 **Red (Upper Arms)**: Motor-driven arms from base to elbows
- 🔵 **Blue (Lower Arms)**: Parallelogram linkages to platform
- 🟢 **Green (Platform)**: End-effector platform triangle
- 🟡 **Orange (End Effector)**: Target position sphere
- ⚪ **Gray (Base)**: Fixed base joints and triangle

## Inverse Kinematics

The IK solver uses geometric decomposition:

### Algorithm

For each of the three arms:

1. Calculate platform joint position relative to base joint
2. Compute horizontal distance (dx) and vertical distance (dz)
3. Apply law of cosines to find elbow angle:
   ```
   cos(α) = (L₁² + d² - L₂²) / (2·L₁·d)
   β = atan2(-dz, dx)
   θ = -(β + α)
   ```
   Where:
   - L₁ = upper arm length
   - L₂ = lower arm length
   - d = distance from base to platform joint
   - θ = motor angle (from horizontal)

4. Verify reachability: `|L₁ - L₂| ≤ d ≤ L₁ + L₂`

### Workspace Limits

The reachable workspace forms an approximate inverted cone below the base. Limits depend on:
- Arm length ratio (longer lower arms = larger workspace)
- Base/platform radius difference
- Joint angle constraints

## Configuration

Edit `config/robot_config.json` to customize robot parameters:

```json
{
  "robot_parameters": {
    "base_radius": 200.0,
    "platform_radius": 50.0,
    "upper_arm_length": 300.0,
    "lower_arm_length": 600.0
  }
}
```

> **Note**: Configuration loading is not yet implemented. Modify parameters directly in `src/main.cpp`.

## Project Structure

```
delta_robot_sim/
├── include/
│   ├── DeltaRobot.hpp      # Kinematics interface
│   └── Renderer.hpp         # OpenGL visualization
├── src/
│   ├── main.cpp             # Application entry point & ImGui
│   ├── kinematics/
│   │   └── DeltaRobot.cpp   # IK/FK implementation
│   └── visualization/
│       └── Renderer.cpp     # 3D rendering primitives
├── config/
│   └── robot_config.json    # Robot parameters
├── external/                # Downloaded dependencies
│   └── imgui/               # Dear ImGui (auto-downloaded)
├── build/                   # Build artifacts (generated)
├── CMakeLists.txt           # Build configuration
├── build.sh                 # Automated build script
└── README.md                # This file
```

## Extending the Simulator

### Adding New Features

1. **Forward Kinematics**: Implement sphere intersection algorithm in `DeltaRobot.cpp`
2. **Trajectory Planning**: Add path interpolation between positions
3. **Collision Detection**: Implement self-collision checks
4. **Export Functions**: Save joint angles to CSV for robot deployment
5. **Serial Communication**: Connect to real robot hardware

### Tips for Real Robot Deployment

- Verify IK solutions match your robot's actual configuration
- Test workspace limits with physical constraints
- Add safety margins to prevent mechanical interference
- Calibrate zero positions for each motor
- Implement smooth acceleration/deceleration profiles

## Performance

- Rendering: 60 FPS (vsync enabled)
- IK Calculation: < 0.1 ms per solution
- Optimizations: `-O3 -march=native` (Release build)

## Troubleshooting

### Build Errors

**Error**: `Could not find GLFW3`
```bash
sudo apt install libglfw3-dev  # Ubuntu/Debian
sudo pacman -S glfw-x11        # Arch Linux
```

**Error**: `OpenGL headers not found`
```bash
sudo apt install libgl1-mesa-dev  # Ubuntu/Debian
sudo pacman -S mesa               # Arch Linux
```

### Runtime Issues

**Black window / No rendering**:
- Check OpenGL version: `glxinfo | grep "OpenGL version"`
- Ensure GPU drivers are properly installed

**ImGui not responding**:
- Make sure ImGui was downloaded correctly in `external/imgui/`
- Re-run `./build.sh` to download dependencies

## License

MIT License - Feel free to use this for commercial or personal robotics projects.

## Contributing

Contributions welcome! Areas for improvement:
- Accurate forward kinematics implementation
- Configuration file loading
- Real-time trajectory visualization
- Robot state export/import
- Additional robot variants (different geometries)

## References

- [Delta Robot Kinematics (Trossen Robotics)](https://www.trossenrobotics.com/)
- [Parallel Robots: Mechanics and Control](https://link.springer.com/)
- [Dear ImGui Documentation](https://github.com/ocornut/imgui)

---

**Author**: Sam  
**Created**: 2025  
**Purpose**: Test inverse kinematics before deploying to physical delta robot hardware
