# Direct Step-Based Protocol Design

## Overview

This document outlines the binary protocol for direct communication between the Delta Robot Simulator and Teensy firmware. The protocol is designed for efficient, real-time control of 3 stepper motors with minimal overhead.

## Design Philosophy

1. **Binary Protocol**: More efficient than text-based commands
2. **Direct Step Control**: Send target step positions directly (no G-code parsing)
3. **Low Latency**: Optimized for real-time control
4. **Synchronized Motion**: All 3 motors move together for coordinated motion
5. **Feedback Loop**: Regular position updates from hardware
6. **Simple & Robust**: Easy to implement and debug

## Packet Structure

### General Packet Format

```
[Packet Type (1 byte)] [Payload Length (2 bytes)] [Payload (variable)] [Checksum (1 byte)]
```

- **Packet Type**: Command or response identifier
- **Payload Length**: Number of bytes in payload (0-65535)
- **Payload**: Command-specific data
- **Checksum**: Simple XOR checksum of all bytes (including header)

### Packet Types

#### Command Packets (PC → Teensy)

| Type | Value | Description |
|------|-------|-------------|
| CMD_MOVE_ABS | 0x01 | Move to absolute step positions |
| CMD_MOVE_REL | 0x02 | Move relative from current positions |
| CMD_SET_SPEED | 0x03 | Set motor speed (steps/sec) |
| CMD_SET_ACCEL | 0x04 | Set motor acceleration (steps/sec²) |
| CMD_ENABLE | 0x05 | Enable/disable motors |
| CMD_STOP | 0x06 | Emergency stop |
| CMD_HOME | 0x07 | Home all motors |
| CMD_SET_POS | 0x08 | Set current position (calibration) |
| CMD_CONFIG | 0x09 | Set motor configuration |
| CMD_PING | 0x0A | Keep-alive/heartbeat |
| CMD_REQUEST_STATUS | 0x0B | Request position/status update |

#### Response Packets (Teensy → PC)

| Type | Value | Description |
|------|-------|-------------|
| RESP_OK | 0x80 | Command acknowledged |
| RESP_ERROR | 0x81 | Command failed |
| RESP_STATUS | 0x82 | Position/status update |
| RESP_PONG | 0x83 | Response to ping |
| RESP_HOMED | 0x84 | Homing complete |

## Command Details

### CMD_MOVE_ABS (0x01) - Move to Absolute Positions

**Payload:**
```
[Step1 (4 bytes, int32_t)] [Step2 (4 bytes, int32_t)] [Step3 (4 bytes, int32_t)]
```

Move all 3 motors to absolute step positions simultaneously. Motors will move in a coordinated manner.

**Response:** RESP_OK when movement started, RESP_ERROR on failure

---

### CMD_MOVE_REL (0x02) - Move Relative

**Payload:**
```
[Delta1 (4 bytes, int32_t)] [Delta2 (4 bytes, int32_t)] [Delta3 (4 bytes, int32_t)]
```

Move all 3 motors relative to current positions.

**Response:** RESP_OK when movement started

---

### CMD_SET_SPEED (0x03) - Set Motor Speed

**Payload:**
```
[Motor Index (1 byte)] [Speed (4 bytes, float)]
```

Set maximum speed for a specific motor (steps per second).

**Response:** RESP_OK

---

### CMD_SET_ACCEL (0x04) - Set Acceleration

**Payload:**
```
[Motor Index (1 byte)] [Acceleration (4 bytes, float)]
```

Set acceleration for a specific motor (steps per second squared).

**Response:** RESP_OK

---

### CMD_ENABLE (0x05) - Enable/Disable Motors

**Payload:**
```
[Enable (1 byte, 0=disable, 1=enable)]
```

Enable or disable all motors.

**Response:** RESP_OK

---

### CMD_STOP (0x06) - Emergency Stop

**Payload:** None (0 bytes)

Immediately stop all motors and abort any current movement.

**Response:** RESP_OK

---

### CMD_HOME (0x07) - Home Motors

**Payload:** None (0 bytes)

Start homing sequence for all motors. Uses end-stops if available.

**Response:** RESP_OK when homing started, RESP_HOMED when complete

---

### CMD_SET_POS (0x08) - Set Current Position

**Payload:**
```
[Motor Index (1 byte)] [Position (4 bytes, int32_t)]
```

Set the current position of a motor (for calibration).

**Response:** RESP_OK

---

### CMD_CONFIG (0x09) - Set Motor Configuration

**Payload:**
```
[Motor Index (1 byte)] [Max Speed (4 bytes, float)] [Acceleration (4 bytes, float)] [Microstepping (1 byte)]
```

Set multiple configuration parameters for a motor at once.

**Response:** RESP_OK

---

### CMD_PING (0x0A) - Keep-Alive

**Payload:** None (0 bytes)

Keep-alive packet to maintain connection. Should be sent periodically.

**Response:** RESP_PONG

---

### CMD_REQUEST_STATUS (0x0B) - Request Status

**Payload:** None (0 bytes)

Request current positions and status from Teensy.

**Response:** RESP_STATUS

---

## Response Details

### RESP_OK (0x80) - Command Acknowledged

**Payload:** None (0 bytes)

Command was received and accepted.

---

### RESP_ERROR (0x81) - Error

**Payload:**
```
[Error Code (1 byte)] [Message Length (1 byte)] [Message (variable)]
```

Command failed. Error codes:
- 0x01: Invalid command
- 0x02: Invalid parameters
- 0x03: Motors not enabled
- 0x04: Movement out of range
- 0x05: Hardware error

---

### RESP_STATUS (0x82) - Status Update

**Payload:**
```
[Position1 (4 bytes, int32_t)] [Position2 (4 bytes, int32_t)] [Position3 (4 bytes, int32_t)]
[Moving Flags (1 byte, bit 0=motor1, bit 1=motor2, bit 2=motor3)]
[Enabled (1 byte, 0/1)]
```

Current positions, movement status, and enable state.

---

### RESP_PONG (0x83) - Ping Response

**Payload:** None (0 bytes)

Response to ping command.

---

### RESP_HOMED (0x84) - Homing Complete

**Payload:**
```
[Position1 (4 bytes, int32_t)] [Position2 (4 bytes, int32_t)] [Position3 (4 bytes, int32_t)]
```

Homing sequence completed. Contains final positions (typically 0,0,0).

---

## Communication Flow

### Typical Control Loop

1. **PC calculates target positions** from kinematics
2. **PC sends CMD_MOVE_ABS** with target step positions
3. **Teensy receives command** and starts coordinated movement
4. **Teensy responds with RESP_OK** (acknowledgment)
5. **Teensy sends periodic RESP_STATUS** updates (e.g., every 100ms)
6. **PC updates motor states** based on feedback
7. **Repeat** as new positions are calculated

### Status Updates

Teensy should send RESP_STATUS packets:
- Periodically (e.g., every 100ms) during movement
- When movement completes
- In response to CMD_REQUEST_STATUS
- When motors are enabled/disabled

### Keep-Alive

PC should send CMD_PING every 5 seconds. If no RESP_PONG received within 1 second, consider connection lost.

---

## Implementation Considerations

### Synchronization

- All 3 motors must move together for coordinated delta robot motion
- Teensy firmware should use MultiStepper or similar for synchronized movement
- Movement should start simultaneously for all motors

### Timing

- Commands should be processed quickly (<10ms latency)
- Status updates can be less frequent (100ms intervals acceptable)
- Use non-blocking movement to allow status updates during motion

### Error Handling

- Invalid commands should return RESP_ERROR
- Out-of-range movements should be rejected
- Hardware errors should be reported immediately
- Connection timeouts should be handled gracefully

### Buffer Management

- Teensy should buffer incoming commands
- Process commands in order
- Queue movements if previous movement not complete
- Limit queue depth to prevent memory issues

---

## Example Packet Sequences

### Example 1: Move to Position

```
PC → Teensy: [0x01][0x0C][0x00][0x00][0x03][0xE8][0x00][0x00][0x07][0xD0][0x00][0x00][0x05][0xDC][0xXX]
             CMD_MOVE_ABS, 12 bytes: X=1000, Y=2000, Z=1500, checksum

Teensy → PC: [0x80][0x00][0x00][0x80]
             RESP_OK, 0 bytes payload, checksum

Teensy → PC: [0x82][0x0D][...positions...][0xXX]
             RESP_STATUS (periodic updates)
```

### Example 2: Enable Motors

```
PC → Teensy: [0x05][0x01][0x01][0xXX]
             CMD_ENABLE, 1 byte: enable=1, checksum

Teensy → PC: [0x80][0x00][0x00][0x80]
             RESP_OK
```

### Example 3: Request Status

```
PC → Teensy: [0x0B][0x00][0x00][0x0B]
             CMD_REQUEST_STATUS, 0 bytes, checksum

Teensy → PC: [0x82][0x0D][...positions...][0xXX]
             RESP_STATUS with current positions
```

---

## Checksum Calculation

Simple XOR checksum of all bytes in packet (including header):

```cpp
uint8_t checksum = 0;
for (int i = 0; i < packet_length - 1; i++) {
    checksum ^= packet[i];
}
packet[packet_length - 1] = checksum;
```

---

## Teensy Firmware Architecture

### Recommended Structure

1. **Ethernet Server**: Accept connections, handle TCP streams
2. **Packet Parser**: Parse incoming binary packets
3. **Command Handler**: Process commands and execute motor control
4. **Motor Controller**: MultiStepper for coordinated motion
5. **Status Reporter**: Send periodic status updates
6. **State Machine**: Track motor states, movement status

### Control Loop

```cpp
void loop() {
    // Handle Ethernet connections
    handleEthernet();
    
    // Parse incoming packets
    if (packetAvailable()) {
        parseAndExecute();
    }
    
    // Run steppers (non-blocking)
    steppers.run();
    
    // Send status updates periodically
    if (millis() - lastStatusUpdate > 100) {
        sendStatus();
        lastStatusUpdate = millis();
    }
    
    // Handle keep-alive
    if (millis() - lastPing > 5000) {
        sendPing();
        lastPing = millis();
    }
}
```

---

## Advantages Over G-Code

1. **Efficiency**: Binary protocol is ~10x smaller than text
2. **Speed**: No text parsing overhead
3. **Precision**: Direct step values, no conversion
4. **Simplicity**: Firmware only needs to handle binary packets
5. **Real-time**: Lower latency for time-critical commands
6. **Robustness**: Checksums for error detection

---

## Next Steps

1. Implement packet encoding/decoding in C++ software
2. Update EthernetHardwareInterface to use binary protocol
3. Create Teensy firmware template with packet parser
4. Add error handling and recovery
5. Test with simulated hardware first
6. Validate with real hardware

