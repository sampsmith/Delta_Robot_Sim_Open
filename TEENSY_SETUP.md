# Teensy Ethernet Setup Guide

## Overview

This guide explains how to set up the Teensy 4.1 with Ethernet to control your Delta Robot's NEMA 23 stepper motors.

## Hardware Requirements

### Required Components
- **Teensy 4.1** (with built-in Ethernet or Ethernet shield)
- **3× NEMA 23 Stepper Motors**
- **3× Stepper Motor Drivers** (DRV8825, TMC2208, TMC2209, etc.)
- **Ethernet Connection** (RJ45 cable to router/switch)
- **Power Supply** for motors (12-24V, sufficient current)
- **Power Supply** for Teensy (5V via USB or external)

### Optional Components
- **End-stop switches** for homing
- **LED indicators** for status
- **Emergency stop button**

## Pin Connections

### Motor 1 (X-axis)
- STEP → Pin 2
- DIR → Pin 3
- ENABLE → Pin 4
- End-stop (optional) → Pin 11

### Motor 2 (Y-axis)
- STEP → Pin 5
- DIR → Pin 6
- ENABLE → Pin 7
- End-stop (optional) → Pin 12

### Motor 3 (Z-axis)
- STEP → Pin 8
- DIR → Pin 9
- ENABLE → Pin 10
- End-stop (optional) → Pin 13

### Ethernet
- Built-in Ethernet port on Teensy 4.1
- Or connect Ethernet shield to appropriate pins

## Software Setup

### 1. Install Arduino IDE with Teensyduino

1. Download and install **Arduino IDE** (1.8.x or 2.x)
2. Download **Teensyduino** from: https://www.pjrc.com/teensy/td_download.html
3. Install Teensyduino add-on

### 2. Install Required Libraries

In Arduino IDE, go to **Tools → Manage Libraries** and install:

- **AccelStepper** by Mike McCauley
  - Used for smooth stepper motor control
  - URL: https://www.airspayce.com/mikem/arduino/AccelStepper/

- **NativeEthernet** (for Teensy 4.1)
  - Built-in with Teensyduino, or install from: https://github.com/vjmuzik/NativeEthernet

### 3. Configure Network Settings

Edit `DeltaRobot_Teensy.ino` and update network configuration:

```cpp
IPAddress ip(192, 168, 1, 100);  // Change to your desired IP
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
```

**Important**: Choose an IP address that:
- Is on the same network as your computer
- Doesn't conflict with other devices
- Is outside your router's DHCP range

### 4. Upload Firmware

1. Connect Teensy via USB
2. Select **Tools → Board → Teensy 4.1**
3. Select **Tools → USB Type → Serial + MIDI + Audio**
4. Open `DeltaRobot_Teensy.ino`
5. Click **Upload** button

## Software Configuration (PC Side)

### 1. Update Main Application

The main application (`src/main.cpp`) is already configured to use Ethernet interface by default.

### 2. Connect to Teensy

1. Run the Delta Robot Simulator
2. Open the **Hardware Interface** panel
3. Enter the Teensy's IP address (e.g., `192.168.1.100`)
4. Enter the port (default: `8080`)
5. Click **Connect**

## Command Protocol

The firmware uses a G-code-like protocol over TCP/IP:

### Movement Commands

- `G1 X<steps> Y<steps> Z<steps>` - Move to absolute position
- `G91 G1 X<steps> Y<steps> Z<steps>` - Move relative to current position
- `G28` - Home all motors
- `G92 X<steps>` - Set current position (for X axis, use Y or Z for others)

### Motor Control

- `M17` - Enable all motors
- `M18` - Disable all motors
- `M112` - Emergency stop

### Configuration

- `M203 S<speed> T<motor>` - Set speed (steps/sec) for motor (0-2)
- `M204 A<accel> T<motor>` - Set acceleration (steps/sec²) for motor
- `M350 S<microsteps> T<motor>` - Set microstepping (informational)

### Status

- `M114` - Get current positions (responds with `X:100 Y:200 Z:300`)

### Responses

- `OK` - Command successful
- `ERR:message` - Error occurred
- `X:100 Y:200 Z:300` - Position report (for M114)

## Testing

### 1. Basic Connection Test

1. Upload firmware to Teensy
2. Open Serial Monitor (115200 baud) to see debug messages
3. Check that IP address is displayed
4. From your computer, try: `telnet 192.168.1.100 8080`
5. Send `M114` - should receive position report

### 2. Motor Test

1. Connect one motor at a time
2. Enable motors: `M17`
3. Test movement: `G1 X100` (should move motor 1)
4. Check position: `M114`
5. Disable motors: `M18`

### 3. Software Integration Test

1. Run Delta Robot Simulator
2. Connect to Teensy via Hardware Interface panel
3. Enable motors
4. Move end effector in simulation
5. Verify motors move in sync

## Troubleshooting

### Teensy not connecting to network

- **Check Ethernet cable** - ensure it's properly connected
- **Check network settings** - verify IP, gateway, subnet
- **Check router/switch** - ensure device is on same network
- **Check Serial Monitor** - look for error messages

### Motors not moving

- **Check enable pins** - motors may be disabled
- **Check power supply** - ensure sufficient voltage/current
- **Check step/direction wiring** - verify connections
- **Check motor drivers** - verify driver configuration
- **Send M17** - explicitly enable motors

### Inaccurate positioning

- **Verify microstepping** - check driver microstep settings
- **Check steps per revolution** - should be 200 for NEMA 23
- **Calibrate home position** - use G92 to set known positions
- **Check for mechanical issues** - backlash, loose connections

### Connection drops

- **Check network stability** - use wired connection if possible
- **Increase timeout** - in firmware, adjust receive timeout
- **Check for interference** - keep Ethernet cable away from motor wires
- **Restart connection** - disconnect and reconnect from software

## Advanced Configuration

### Custom Motor Speeds

Edit default speeds in firmware:

```cpp
MotorConfig motorConfigs[3] = {
    {1000.0, 500.0, 16, false},  // Motor 1: speed, accel, microsteps
    {1000.0, 500.0, 16, false},  // Motor 2
    {1000.0, 500.0, 16, false}   // Motor 3
};
```

### End-Stop Configuration

If using end-stops:
- Connect normally-open switches to pins 11, 12, 13
- Use pull-up resistors (built-in on Teensy)
- Homing sequence will move until switch closes

### Multiple Robots

To control multiple robots:
- Use different IP addresses for each Teensy
- Use different ports (e.g., 8080, 8081, 8082)
- Update connection in software for each robot

## Safety Considerations

1. **Emergency Stop**: Implement physical emergency stop button
2. **Software Limits**: Enforce workspace limits in main application
3. **Power Management**: Proper shutdown sequence
4. **Mechanical Limits**: Use end-stops as safety limits
5. **Connection Monitoring**: Detect disconnections and stop motors

## Next Steps

1. **Calibrate**: Measure actual robot dimensions and update configuration
2. **Tune Speeds**: Optimize acceleration and speed for smooth operation
3. **Test Workspace**: Verify full range of motion
4. **Save Waypoints**: Use waypoint system for complex movements
5. **Integrate Sensors**: Add end-stops, encoders, or other feedback

## Support

For issues or questions:
- Check Serial Monitor output for error messages
- Verify network connectivity with ping
- Test individual components separately
- Review firmware code comments for details

