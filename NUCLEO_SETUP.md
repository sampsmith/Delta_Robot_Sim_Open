# NUCLEO-H7S3L8 Ethernet Setup Guide

## Overview

This guide explains how to set up the **NUCLEO-H7S3L8** development board with Ethernet to control your Delta Robot's NEMA 23 stepper motors. The NUCLEO-H7S3L8 features the STM32H7S3L8H6 microcontroller with built-in Ethernet support.

## Hardware Requirements

### Required Components
- **NUCLEO-H7S3L8** development board (STM32H7S3L8H6 microcontroller)
- **3× NEMA 23 Stepper Motors**
- **3× Stepper Motor Drivers** (DRV8825, TMC2208, TMC2209, etc.)
- **Ethernet Connection** (RJ45 cable to router/switch)
- **Power Supply** for motors (12-24V, sufficient current)
- **USB-C Cable** for programming and power (or external power supply)

### Optional Components
- **End-stop switches** for homing
- **LED indicators** for status
- **Emergency stop button**

## NUCLEO-H7S3L8 Specifications

- **Microcontroller**: STM32H7S3L8H6 (ARM Cortex-M7F @ 280 MHz)
- **Ethernet**: Built-in IEEE-802.3-2002 compliant Ethernet PHY
- **GPIO**: 168 pins available (via ST Zio and ST Morpho connectors)
- **Timers**: Multiple advanced timers for precise stepper motor control
- **Memory**: 256KB SRAM, 2MB Flash (plus optional 256MB Octo-SPI flash)
- **Debugging**: STLINK-V3EC on-board programmer/debugger

## Pin Connections

### Motor 1 (X-axis)
- STEP → GPIO pin (e.g., PA0, PB0, or PC0 - configure in STM32CubeMX)
- DIR → GPIO pin (e.g., PA1, PB1, or PC1)
- ENABLE → GPIO pin (e.g., PA2, PB2, or PC2)
- **Limit Switch (Upper Home)** → GPIO pin (configure as input in STM32CubeMX)

### Motor 2 (Y-axis)
- STEP → GPIO pin
- DIR → GPIO pin
- ENABLE → GPIO pin
- **Limit Switch (Upper Home)** → GPIO pin (configure as input in STM32CubeMX)

### Motor 3 (Z-axis)
- STEP → GPIO pin
- DIR → GPIO pin
- ENABLE → GPIO pin
- **Limit Switch (Upper Home)** → GPIO pin (configure as input in STM32CubeMX)

**Note**: Limit switches are connected directly to STM32 GPIO pins. STM32 firmware reads these switches and implements all homing logic. PC software only sends CMD_HOME command.

### Ethernet
- Built-in Ethernet PHY on NUCLEO-H7S3L8
- Connect RJ45 cable directly to the board
- No external shield required

**Note**: Specific pin assignments will be defined in the firmware based on available GPIO and timer resources. The NUCLEO-H7S3L8 has extensive GPIO options that can be configured via STM32CubeMX.

## Software Setup

### 1. Install STM32 Development Environment

**Option A: STM32CubeIDE (Recommended)**
1. Download **STM32CubeIDE** from: https://www.st.com/en/development-tools/stm32cubeide.html
2. Install STM32CubeIDE (includes compiler, debugger, and STM32CubeMX)
3. STM32CubeIDE is based on Eclipse and includes all necessary tools

**Option B: STM32CubeMX + IDE**
1. Download **STM32CubeMX** from: https://www.st.com/en/development-tools/stm32cubemx.html
2. Install **STM32CubeMX** (pin configuration tool)
3. Choose your IDE:
   - **STM32CubeIDE** (Eclipse-based, recommended)
   - **Keil MDK** (commercial)
   - **IAR Embedded Workbench** (commercial)
   - **VSCode** with STM32 extensions

### 2. Install STM32CubeH7 Package

1. Open **STM32CubeMX**
2. Go to **Help → Manage Embedded Software Packages**
3. Install **STM32H7** Series package
4. This includes HAL drivers and middleware for STM32H7 series

### 3. Install Required Libraries

The firmware will use:
- **STM32 HAL Library** (included with STM32CubeH7 package)
- **LwIP** (Lightweight IP stack, included with STM32CubeH7)
- **Custom Stepper Motor Library** (to be implemented, or use HAL timers)

For stepper motor control, you can:
- Use HAL timers in PWM mode for step generation
- Implement custom stepper control using GPIO and timers
- Use existing stepper libraries adapted for STM32

### 4. Configure Network Settings

Edit the firmware network configuration:

```c
// Network configuration (example)
#define NUCLEO_IP_ADDR        "192.168.1.100"
#define NUCLEO_NETMASK        "255.255.255.0"
#define NUCLEO_GATEWAY        "192.168.1.1"
#define NUCLEO_PORT           8080
```

**Important**: Choose an IP address that:
- Is on the same network as your computer
- Doesn't conflict with other devices
- Is outside your router's DHCP range

### 5. Generate Firmware Project

1. Open **STM32CubeMX**
2. Create new project → Select **STM32H7S3L8H6**
3. Configure pins:
   - Enable Ethernet (automatically configures required pins)
   - Assign GPIO pins for STEP, DIR, ENABLE for each motor
   - Configure timers for stepper control (if needed)
4. Configure peripherals:
   - **Ethernet**: Enable LwIP, configure network settings
   - **Timers**: Configure for PWM/step generation
   - **GPIO**: Configure motor control pins
5. Generate code for your IDE

### 6. Build and Upload Firmware

1. Open generated project in STM32CubeIDE (or your IDE)
2. Add custom firmware code (packet parser, motor control, etc.)
3. Build project
4. Connect NUCLEO-H7S3L8 via USB-C
5. Click **Run/Debug** to upload firmware
6. The STLINK-V3EC on-board debugger will handle programming

## Software Configuration (PC Side)

### 1. Update Main Application

The main application (`src/main.cpp`) is already configured to use Ethernet interface by default.

### 2. Connect to NUCLEO-H7S3L8

1. Run the Delta Robot Simulator
2. Open the **Hardware Interface** panel
3. Enter the NUCLEO-H7S3L8's IP address (e.g., `192.168.1.100`)
4. Enter the port (default: `8080`)
5. Click **Connect**

## Communication Protocol

The firmware uses a **custom binary protocol** (no G-code). See `PROTOCOL_DESIGN.md` for complete protocol specification.

### Binary Protocol Overview

- **Packet Format**: `[Type:1][Length:2][Payload:variable][Checksum:1]`
- **Commands**: Move absolute, move relative, set speed, enable motors, etc.
- **Responses**: OK, Error, Status updates, Position reports
- **Sequence Packets**: Send multiple waypoints in a single packet

All communication is binary-encoded for efficiency and low latency.

## Testing

### 1. Basic Connection Test

1. Upload firmware to NUCLEO-H7S3L8
2. Check Serial Monitor (via STLINK Virtual COM port) for debug messages
3. Verify that IP address is displayed
4. From your computer, try: `telnet 192.168.1.100 8080`
5. Ping the device: `ping 192.168.1.100`

### 2. Motor Test

1. Connect one motor at a time
2. Enable motors via software command
3. Test movement via software interface
4. Verify motor positions reported correctly
5. Disable motors when done

### 3. Software Integration Test

1. Run Delta Robot Simulator
2. Connect to NUCLEO-H7S3L8 via Hardware Interface panel
3. Enable motors
4. Move end effector in simulation
5. Verify motors move in sync with simulation

## Troubleshooting

### NUCLEO-H7S3L8 not connecting to network

- **Check Ethernet cable** - ensure it's properly connected to the board
- **Check network settings** - verify IP, gateway, subnet in firmware
- **Check router/switch** - ensure device is on same network
- **Check Serial Monitor** - look for Ethernet initialization messages
- **Verify LwIP configuration** - ensure LwIP is properly initialized
- **Check link status** - Ethernet LED should indicate link status

### Motors not moving

- **Check enable pins** - motors may be disabled
- **Check power supply** - ensure sufficient voltage/current for motors
- **Check step/direction wiring** - verify connections to GPIO pins
- **Check motor drivers** - verify driver configuration
- **Check timer configuration** - ensure timers are configured correctly
- **Enable motors via software** - use enable command in software

### Inaccurate positioning

- **Verify microstepping** - check driver microstep settings
- **Check steps per revolution** - should be 200 for NEMA 23
- **Calibrate home position** - use set position command
- **Check for mechanical issues** - backlash, loose connections
- **Verify timer frequency** - ensure step timing is accurate

### Connection drops

- **Check network stability** - use wired connection (recommended)
- **Increase timeout** - in firmware, adjust LwIP timeout settings
- **Check for interference** - keep Ethernet cable away from motor wires
- **Restart connection** - disconnect and reconnect from software
- **Check LwIP configuration** - verify TCP/IP stack settings

## Advanced Configuration

### Custom Motor Speeds

Configure motor speeds in firmware based on your application:

```c
// Motor configuration (example)
typedef struct {
    float maxSpeed;        // Steps per second
    float acceleration;     // Steps per second squared
    uint8_t microstepping; // Microstepping factor
    bool enabled;
} MotorConfig;

MotorConfig motorConfigs[3] = {
    {1000.0, 500.0, 16, false},  // Motor 1
    {1000.0, 500.0, 16, false},  // Motor 2
    {1000.0, 500.0, 16, false}   // Motor 3
};
```

### End-Stop Configuration

If using end-stops:
- Connect normally-open switches to GPIO pins
- Configure GPIO pins with interrupt capability
- Use HAL GPIO interrupt handlers for homing
- Implement debouncing in software

### Multiple Robots

To control multiple robots:
- Use different IP addresses for each NUCLEO-H7S3L8
- Use different ports (e.g., 8080, 8081, 8082)
- Update connection in software for each robot

## STM32H7S3L8H6 Specific Notes

### Performance
- **280 MHz ARM Cortex-M7F** provides excellent performance for real-time control
- **Multiple timers** available for precise stepper motor step generation
- **DMA support** for efficient data transfer

### Memory
- **2MB Flash** - plenty of space for firmware
- **256KB SRAM** - sufficient for motor control and networking
- **Optional 256MB Octo-SPI flash** for additional storage

### Ethernet
- **Built-in Ethernet PHY** - no external components needed
- **LwIP stack** - efficient TCP/IP implementation
- **Hardware acceleration** - offloads networking tasks from CPU

### Development
- **STM32CubeMX** - visual pin configuration and code generation
- **STM32CubeIDE** - integrated development environment
- **STLINK-V3EC** - on-board debugger and programmer
- **HAL Library** - hardware abstraction layer simplifies development

## Safety Considerations

1. **Emergency Stop**: Implement physical emergency stop button
2. **Software Limits**: Enforce workspace limits in main application
3. **Power Management**: Proper shutdown sequence
4. **Mechanical Limits**: Use end-stops as safety limits
5. **Connection Monitoring**: Detect disconnections and stop motors
6. **Watchdog Timer**: Use STM32 watchdog for fault recovery

## Next Steps

1. **Set up development environment** - Install STM32CubeIDE and STM32CubeH7
2. **Generate base project** - Use STM32CubeMX to configure hardware
3. **Implement firmware** - Use `NUCLEO_FIRMWARE_OUTLINE.md` as guide
4. **Test incrementally** - Test Ethernet, then motors, then full integration
5. **Calibrate**: Measure actual robot dimensions and update configuration
6. **Tune Speeds**: Optimize acceleration and speed for smooth operation
7. **Test Workspace**: Verify full range of motion
8. **Save Waypoints**: Use waypoint system for complex movements

## Resources

- **NUCLEO-H7S3L8 Datasheet**: https://www.st.com/resource/en/data_brief/nucleo-h7s3l8.pdf
- **STM32H7S3L8H6 Reference Manual**: STM32 documentation portal
- **STM32CubeH7**: HAL drivers and examples
- **LwIP Documentation**: Lightweight IP stack documentation
- **STM32CubeIDE**: https://www.st.com/en/development-tools/stm32cubeide.html

## Support

For issues or questions:
- Check Serial Monitor output (via STLINK Virtual COM port)
- Verify network connectivity with ping
- Test individual components separately
- Review firmware code comments for details
- Consult STM32 documentation and community forums
