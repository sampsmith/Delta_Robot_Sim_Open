# Hardware Integration Guide - Delta Robot with NEMA 23 Stepper Motors

## Overview

This document describes the motor control system that has been integrated into the Delta Robot Simulator to enable real hardware control with NEMA 23 stepper motors.

## What Has Been Implemented

### 1. Motor Control System (`MotorControl.hpp/cpp`)
- **Motor Configuration**: Configurable parameters for NEMA 23 steppers:
  - Steps per revolution (default: 200 for standard NEMA 23)
  - Microstepping factor (default: 16 for 1/16 microstepping)
  - Gear ratio (if using gear reduction)
  - Maximum speed and acceleration
  - Home offset calibration
  
- **Angle to Steps Conversion**: Converts motor angles (radians) from kinematics to stepper motor steps
- **Steps to Angle Conversion**: Converts hardware step positions back to angles
- **Trajectory Planning**: Smooth motion planning between waypoints with acceleration/deceleration profiles
- **Motor State Tracking**: Current position, target position, and movement status for each motor

### 2. Hardware Interface (`HardwareInterface.hpp/cpp`)
- **Abstract Interface**: Base class for hardware communication
- **Simulated Hardware**: For testing without physical hardware
- **Serial Hardware Interface**: Framework for real hardware (Arduino/ESP32 compatible)
- **Connection Management**: Connect/disconnect, status monitoring
- **Motor Commands**: Move to absolute/relative positions, enable/disable, home, configure

### 3. Waypoint/Keypoint System
- **Save Positions**: Save current end-effector position and motor angles
- **Load Positions**: Return to saved waypoints
- **Sequence Execution**: Execute multiple waypoints in sequence with smooth trajectories
- **Duration Control**: Adjustable time for each waypoint transition

### 4. UI Integration
- **Motor Control Panel**: View motor status, angles, steps, and configuration
- **Waypoints Panel**: Manage saved positions and execute sequences
- **Hardware Interface Panel**: Connect/disconnect, home motors, view hardware status

## How It Works

### Kinematics → Motor Control Flow

1. **User Sets End-Effector Position**: Via UI or virtual controller
2. **Inverse Kinematics Calculates Motor Angles**: DeltaRobot calculates required motor angles (radians)
3. **Motor Control Converts to Steps**: MotorControl converts angles to stepper steps
4. **Hardware Interface Sends Commands**: Commands sent to motors via serial/USB
5. **Feedback Loop**: Hardware positions read back and converted to angles

### Motor Angle to Steps Calculation

```
Steps = (angle / 2π) × stepsPerRevolution × microstepping × gearRatio
```

For default configuration:
- 200 steps/rev × 16 microstepping = 3200 steps/revolution
- 1.8° per step (full step) → 0.1125° per step (with 1/16 microstepping)

## Next Steps for Real Hardware Integration

### 1. Hardware Requirements

**Motors:**
- 3× NEMA 23 stepper motors (typically 1.8° step angle, 200 steps/rev)
- Recommended: High torque motors (2-3 N⋅m) for smooth operation

**Motor Drivers:**
- 3× Stepper motor drivers (e.g., DRV8825, TMC2208, TMC2209)
- Support for microstepping (1/16 recommended for precision)
- Current rating: Match motor current requirements

**Controller:**
- Arduino (Uno/Nano/Mega) or ESP32
- Minimum: 3× step/direction pins, 3× enable pins
- Optional: End-stop switches for homing

**Power Supply:**
- Sufficient voltage for motors (typically 12-24V)
- Current rating: 3× motor current + headroom

### 2. Firmware Development (Arduino/ESP32)

You'll need to implement firmware that:
- Receives G-code-like commands via serial
- Controls 3 stepper motors simultaneously
- Implements acceleration/deceleration profiles
- Handles homing sequence
- Reports motor positions back

**Example Command Protocol:**
```
G1 X1000 Y2000 Z1500    # Move motors to absolute positions (steps)
G91 G1 X100 Y-50 Z200    # Move motors relative (steps)
G28                     # Home all motors
M17                     # Enable motors
M18                     # Disable motors
M114                    # Report current positions
M203 S1000 T0           # Set speed for motor 0
M204 A500 T1             # Set acceleration for motor 1
```

**Recommended Libraries:**
- `AccelStepper` for smooth acceleration
- `MultiStepper` for coordinated movement
- Serial command parser

### 3. Mechanical Setup

**Motor Mounting:**
- Secure mounting at top of delta robot base
- Precise alignment critical for accuracy
- Motors should be at 120° intervals

**Homing System:**
- End-stop switches or limit switches
- Home position should correspond to motor angle = 0
- Calibrate home offsets in software

**Calibration:**
1. Install end-stops at known positions
2. Run homing sequence
3. Measure actual robot dimensions (arm lengths, base radius, etc.)
4. Update `DeltaRobotConfig` in software
5. Test movements and adjust calibration offsets

### 4. Software Configuration

**Motor Configuration:**
- Adjust `stepsPerRevolution` (usually 200 for NEMA 23)
- Set `microstepping` based on driver settings
- Configure `gearRatio` if using gear reduction
- Set `maxSpeed` and `acceleration` based on motor capabilities

**Robot Dimensions:**
- Measure actual base radius, effector radius
- Measure upper and lower arm lengths
- Update `DeltaRobotConfig` to match physical robot

**Home Calibration:**
- After homing, set home offsets for each motor
- Ensures software position matches physical position

### 5. Testing Procedure

1. **Individual Motor Test**: Test each motor independently
2. **Coordinate Movement**: Test coordinated movement of all 3 motors
3. **Kinematics Validation**: Move to known positions and verify accuracy
4. **Workspace Mapping**: Test full workspace to find limits
5. **Speed/Acceleration Tuning**: Optimize for smooth, accurate motion
6. **Waypoint Testing**: Test saved waypoint sequences

## Example Arduino/ESP32 Firmware Structure

```cpp
#include <AccelStepper.h>
#include <MultiStepper.h>

// Motor pins
#define MOTOR1_STEP 2
#define MOTOR1_DIR 3
#define MOTOR2_STEP 4
#define MOTOR2_DIR 5
#define MOTOR3_STEP 6
#define MOTOR3_DIR 7

AccelStepper motor1(1, MOTOR1_STEP, MOTOR1_DIR);
AccelStepper motor2(1, MOTOR2_STEP, MOTOR2_DIR);
AccelStepper motor3(1, MOTOR3_STEP, MOTOR3_DIR);

MultiStepper steppers;

void setup() {
    // Initialize motors
    motor1.setMaxSpeed(1000);
    motor1.setAcceleration(500);
    // ... similar for motor2, motor3
    
    steppers.addStepper(motor1);
    steppers.addStepper(motor2);
    steppers.addStepper(motor3);
    
    // Initialize serial
    Serial.begin(115200);
}

void loop() {
    // Parse serial commands
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        processCommand(command);
    }
    
    // Run steppers
    steppers.run();
}
```

## Safety Considerations

1. **Emergency Stop**: Implement emergency stop button
2. **Software Limits**: Enforce workspace limits in software
3. **Hardware Limits**: Use end-stops as safety limits
4. **Power Management**: Proper shutdown sequence
5. **Collision Detection**: Monitor for unexpected resistance

## Troubleshooting

**Motors not moving:**
- Check enable pins
- Verify power supply
- Check step/direction wiring
- Test with simple step commands

**Inaccurate positioning:**
- Verify microstepping settings match driver
- Check for mechanical backlash
- Calibrate motor steps per revolution
- Verify gear ratios

**Jittery motion:**
- Reduce acceleration
- Increase microstepping
- Check power supply stability
- Verify wiring connections

**Position drift:**
- Recalibrate home positions
- Check for mechanical slippage
- Verify step calculations

## Additional Resources

- **AccelStepper Library**: https://www.airspayce.com/mikem/arduino/AccelStepper/
- **Delta Robot Kinematics**: Extensive documentation in code comments
- **NEMA 23 Specifications**: Check motor datasheet for exact specifications
- **Stepper Driver Datasheets**: Refer to driver documentation for microstepping settings

## Future Enhancements

- Closed-loop control with encoders
- Real-time trajectory optimization
- G-code file playback
- Advanced calibration routines
- Multi-robot coordination
- Force feedback integration

