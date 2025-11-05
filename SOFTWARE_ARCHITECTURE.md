# Delta Robot Software Architecture

## Overview

This document describes the complete software architecture for the Delta Robot control system, designed for real-time control with waypoint sequences, looping, and direct hardware communication.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────┐
│                    User Interface (UI)                    │
│  - 3D Visualization                                      │
│  - Waypoint Editor                                       │
│  - Sequence Controller (Play/Pause/Stop/Loop)            │
│  - Motor Control Panel                                   │
│  - Hardware Connection                                   │
└────────────────────┬────────────────────────────────────┘
                     │
┌────────────────────▼────────────────────────────────────┐
│              Sequence Controller                         │
│  - Manages waypoint sequences                            │
│  - Handles looping and repetition                        │
│  - Controls playback state                               │
│  - Real-time trajectory streaming                        │
└────────────────────┬────────────────────────────────────┘
                     │
┌────────────────────▼────────────────────────────────────┐
│              Motor Control System                        │
│  - Kinematics → Motor Angles                             │
│  - Motor Angles → Step Conversion                        │
│  - Trajectory Planning                                   │
│  - Waypoint Management                                   │
└────────────────────┬────────────────────────────────────┘
                     │
┌────────────────────▼────────────────────────────────────┐
│              Hardware Interface                          │
│  - Binary Protocol Encoding                              │
│  - Ethernet Communication                                │
│  - Packet Transmission                                   │
│  - Status Updates                                        │
└────────────────────┬────────────────────────────────────┘
                     │
              Ethernet (TCP/IP)
                     │
┌────────────────────▼────────────────────────────────────┐
│              Teensy Firmware                             │
│  - Packet Reception                                      │
│  - Stepper Motor Control                                 │
│  - Coordinated Movement                                  │
│  - Position Feedback                                     │
└─────────────────────────────────────────────────────────┘
```

## Component Details

### 1. User Interface Layer

**Purpose**: Visual control and monitoring

**Components**:
- **3D Visualization**: Real-time robot rendering
- **Waypoint Editor**: Create, edit, delete waypoints
- **Sequence Panel**: Build and manage sequences
- **Playback Controls**: Play, pause, stop, loop controls
- **Motor Status**: Real-time motor position display
- **Hardware Connection**: Connect/disconnect to Teensy

**Key Features**:
- Visual waypoint path preview
- Real-time robot position tracking
- Sequence timeline visualization
- Progress indicators

---

### 2. Sequence Controller

**Purpose**: Manage waypoint sequence execution with looping

**Responsibilities**:
- Execute waypoint sequences
- Handle looping and repetition
- Control playback state (play/pause/stop)
- Stream trajectory points in real-time
- Monitor sequence progress

**Key Features**:
- **Loop Control**: Repeat sequences N times or infinitely
- **Playback States**: Stop, Play, Pause, Completed
- **Streaming Modes**:
  - **Real-time**: Send trajectory points as they execute
  - **Batch**: Send entire trajectory at once
- **Progress Tracking**: Current waypoint, repeat count, elapsed time

**Usage Flow**:
1. User creates waypoint sequence
2. User configures loop settings
3. User clicks "Play"
4. Controller executes sequence, streaming commands to hardware
5. On completion, loops if configured
6. User can pause/resume/stop at any time

---

### 3. Motor Control System

**Purpose**: Convert kinematics to motor control

**Responsibilities**:
- Convert end-effector positions to motor angles (via DeltaRobot)
- Convert motor angles to stepper steps
- Plan smooth trajectories between waypoints
- Manage waypoint storage
- Track motor states

**Key Features**:
- **Angle to Steps**: Direct conversion with microstepping/gear ratio
- **Trajectory Planning**: Smooth motion profiles (S-curve acceleration)
- **Waypoint Storage**: Save positions with names and durations
- **Motor Configuration**: Per-motor settings (speed, acceleration, etc.)

---

### 4. Hardware Interface

**Purpose**: Communicate with Teensy firmware

**Responsibilities**:
- Encode commands into binary packets
- Send packets over Ethernet
- Receive status updates
- Handle connection management
- Parse responses

**Key Features**:
- **Binary Protocol**: Efficient packet-based communication
- **Direct Step Control**: Send step positions directly (no G-code)
- **Real-time Updates**: Receive position feedback
- **Error Handling**: Connection timeouts, error recovery

---

## Data Flow

### Real-Time Control Loop

```
1. User moves end-effector (or sequence plays)
   ↓
2. DeltaRobot calculates inverse kinematics
   → Motor angles (radians)
   ↓
3. MotorControl converts angles to steps
   → Step positions (int32)
   ↓
4. SequenceController (if playing sequence)
   → Manages trajectory timing
   → Streams points in real-time
   ↓
5. HardwareInterface encodes packet
   → Binary packet (CMD_MOVE_ABS)
   ↓
6. Ethernet transmission
   → TCP/IP to Teensy
   ↓
7. Teensy firmware receives packet
   → Parses step positions
   → Moves motors simultaneously
   ↓
8. Teensy sends status update
   → RESP_STATUS with current positions
   ↓
9. HardwareInterface parses response
   → Updates motor states
   ↓
10. UI updates visualization
    → Shows current robot position
```

---

## Waypoint System

### Creating Waypoints

1. **Manual Creation**:
   - Move robot to desired position
   - Click "Save Current Position"
   - Edit name and duration
   - Waypoint stored with:
     - End-effector position (X, Y, Z)
     - Motor angles (calculated)
     - Duration (time to reach this point)

2. **Programmatic Creation**:
   - Can be added via code
   - Useful for automated sequences

### Building Sequences

1. Select waypoints in desired order
2. Create sequence from waypoint list
3. Configure sequence:
   - Loop: Yes/No
   - Repeat count: N times (0 = infinite)
   - Streaming mode: Real-time or Batch
   - Update rate: Hz for real-time streaming

### Executing Sequences

1. **Play**: Start sequence execution
2. **Pause**: Temporarily stop (can resume)
3. **Stop**: Abort sequence (must restart)
4. **Loop**: Automatically repeat when complete

---

## Real-Time Streaming

### Two Modes

#### Mode 1: Real-Time Streaming (Recommended)

**How it works**:
- Trajectory points sent as they're executed
- Update rate: 100 Hz (configurable)
- Each point sent immediately when reached
- Low latency, smooth motion

**Advantages**:
- Can adjust trajectory in real-time
- Lower memory usage on Teensy
- More responsive to changes

**Use case**: General purpose, dynamic control

#### Mode 2: Batch Mode

**How it works**:
- Entire trajectory sent at once
- Teensy executes from buffer
- Higher memory usage

**Advantages**:
- Teensy can optimize path
- Less network traffic
- Good for complex paths

**Use case**: Pre-planned complex sequences

---

## Protocol Design

### Why Binary Protocol?

1. **Efficiency**: ~10x smaller than text
2. **Speed**: No parsing overhead
3. **Precision**: Direct step values (int32)
4. **Simplicity**: Firmware just reads binary data
5. **Real-time**: Low latency for time-critical commands

### Packet Structure

```
[Type (1 byte)] [Length (2 bytes)] [Payload (N bytes)] [Checksum (1 byte)]
```

### Command Types

- `CMD_MOVE_ABS`: Move to absolute step positions
- `CMD_MOVE_REL`: Move relative from current
- `CMD_ENABLE`: Enable/disable motors
- `CMD_STOP`: Emergency stop
- `CMD_HOME`: Home motors
- `CMD_REQUEST_STATUS`: Get current positions

### Response Types

- `RESP_OK`: Command acknowledged
- `RESP_ERROR`: Command failed
- `RESP_STATUS`: Position update
- `RESP_HOMED`: Homing complete

---

## Best Practices

### 1. Sequence Design

- **Smooth Paths**: Avoid sharp direction changes
- **Reasonable Durations**: Balance speed vs. smoothness
- **Safe Positions**: Test waypoints individually first
- **Home Position**: Always start from known position

### 2. Real-Time Control

- **Update Rate**: 50-100 Hz is sufficient
- **Network Stability**: Use wired Ethernet when possible
- **Timeout Handling**: Implement connection monitoring
- **Error Recovery**: Handle lost packets gracefully

### 3. Motor Configuration

- **Microstepping**: 16x recommended for precision
- **Speed Limits**: Don't exceed motor capabilities
- **Acceleration**: Balance smoothness vs. speed
- **Calibration**: Verify steps per revolution

### 4. Safety

- **Emergency Stop**: Always accessible
- **Software Limits**: Enforce workspace bounds
- **Hardware Limits**: Use end-stops
- **Connection Monitoring**: Detect disconnections

---

## Extension Points

### Future Enhancements

1. **Path Optimization**: 
   - Optimize waypoint order
   - Minimize travel time
   - Collision avoidance

2. **Advanced Trajectories**:
   - Bezier curves
   - Spline interpolation
   - Velocity profiles

3. **Multi-Robot Support**:
   - Control multiple robots
   - Synchronized sequences
   - Coordinated movements

4. **Recording/Playback**:
   - Record manual movements
   - Convert to waypoint sequence
   - Replay with adjustments

5. **G-code Import**:
   - Import from CAD/CAM
   - Convert to waypoint sequences
   - Toolpath visualization

---

## Summary

This architecture provides:

✅ **Complete Software Suite**: Full-featured control application
✅ **Real-Time Control**: Direct step-based protocol
✅ **Waypoint System**: Create, edit, sequence waypoints
✅ **Looping**: Repeat sequences infinitely or N times
✅ **Streaming**: Real-time or batch trajectory execution
✅ **Hardware Integration**: Direct Teensy communication
✅ **Visualization**: 3D robot visualization with path preview

The system is designed to be:
- **Efficient**: Binary protocol, optimized data flow
- **Flexible**: Multiple modes, configurable behavior
- **Robust**: Error handling, connection monitoring
- **User-Friendly**: Intuitive UI, visual feedback

This is the right approach for a custom robot solution with embedded control!

