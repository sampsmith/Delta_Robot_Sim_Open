# Teensy Firmware Architecture Outline

## Overview

This document outlines the firmware architecture for the Teensy 4.1 to receive direct step commands and control the Delta Robot's stepper motors.

## Core Components

### 1. Ethernet Communication Layer

**Purpose**: Handle TCP/IP connections and packet communication

**Responsibilities**:
- Accept incoming connections from PC
- Receive binary packets over TCP
- Send response packets
- Handle connection timeouts
- Manage keep-alive (ping/pong)

**Key Functions**:
```cpp
void setupEthernet();
void handleEthernet();
bool packetAvailable();
bool receivePacket(uint8_t* buffer, size_t& length);
bool sendPacket(const uint8_t* buffer, size_t length);
```

---

### 2. Packet Parser

**Purpose**: Parse incoming binary packets according to protocol

**Responsibilities**:
- Validate packet structure (header, length, checksum)
- Extract packet type and payload
- Route to appropriate command handler
- Handle malformed packets

**Key Functions**:
```cpp
bool parsePacket(const uint8_t* data, size_t length, Packet& packet);
bool verifyChecksum(const uint8_t* data, size_t length);
PacketType getPacketType(const Packet& packet);
```

---

### 3. Command Handler

**Purpose**: Execute commands received from PC

**Responsibilities**:
- Process each command type
- Validate parameters
- Queue movements if needed
- Send appropriate responses
- Handle errors gracefully

**Command Handlers**:
```cpp
void handleMoveAbsolute(const Packet& packet);
void handleMoveRelative(const Packet& packet);
void handleSetSpeed(const Packet& packet);
void handleSetAcceleration(const Packet& packet);
void handleEnable(const Packet& packet);
void handleStop(const Packet& packet);
void handleHome(const Packet& packet);
void handleSetPosition(const Packet& packet);
void handleConfig(const Packet& packet);
void handlePing(const Packet& packet);
void handleRequestStatus(const Packet& packet);
```

---

### 4. Motor Controller

**Purpose**: Control 3 stepper motors with coordinated motion

**Responsibilities**:
- Initialize stepper motors
- Execute coordinated movements
- Manage acceleration/deceleration
- Track motor positions
- Handle enable/disable

**Key Functions**:
```cpp
void setupMotors();
void moveToAbsolute(const int32_t positions[3]);
void moveRelative(const int32_t deltas[3]);
void setMotorSpeed(int motor, float speed);
void setMotorAcceleration(int motor, float accel);
void enableMotors(bool enable);
void stopMotors();
bool areMotorsMoving();
void updateMotors();  // Call in loop()
```

**Libraries**:
- Use `AccelStepper` for individual motor control
- Use `MultiStepper` for coordinated movement

---

### 5. Status Reporter

**Purpose**: Send periodic status updates to PC

**Responsibilities**:
- Collect current motor positions
- Check if motors are moving
- Send status packets periodically
- Respond to status requests

**Key Functions**:
```cpp
void sendStatusUpdate();
void collectStatus(StatusInfo& status);
uint32_t getLastStatusTime();
```

**Status Frequency**:
- Every 100ms during movement
- Immediately when movement completes
- On request (CMD_REQUEST_STATUS)

---

### 6. Homing System

**Purpose**: Home motors to known reference positions

**Responsibilities**:
- Move motors until end-stops trigger
- Set position to zero after homing
- Report homing completion
- Handle homing errors

**Key Functions**:
```cpp
void startHoming();
void updateHoming();  // Call in loop() during homing
bool isHoming();
bool isHomed();
```

---

## State Machine

### Motor States

```
DISABLED → ENABLED → MOVING → STOPPED → DISABLED
    ↑                              ↓
    └──────────────────────────────┘
```

### System States

```
INIT → IDLE → MOVING → IDLE
  ↓      ↓       ↓
 ERROR  HOMING  ERROR
```

---

## Main Loop Structure

```cpp
void loop() {
    // 1. Handle Ethernet communication
    handleEthernet();
    
    // 2. Parse incoming packets
    if (packetAvailable()) {
        Packet packet;
        if (parsePacket(...)) {
            handleCommand(packet);
        }
    }
    
    // 3. Update motor controller
    updateMotors();
    
    // 4. Update homing if active
    if (isHoming()) {
        updateHoming();
    }
    
    // 5. Send periodic status updates
    if (millis() - lastStatusTime > STATUS_INTERVAL) {
        sendStatusUpdate();
        lastStatusTime = millis();
    }
    
    // 6. Handle keep-alive
    if (millis() - lastPingTime > PING_INTERVAL) {
        // Check if we received ping recently
        if (millis() - lastPingReceived > TIMEOUT) {
            // Connection lost
            handleTimeout();
        }
    }
}
```

---

## Data Structures

### Motor State
```cpp
struct MotorState {
    int32_t currentPosition;  // Steps from home
    int32_t targetPosition;   // Target steps
    float maxSpeed;           // Steps per second
    float acceleration;       // Steps per second squared
    bool enabled;             // Motor enabled
    bool moving;              // Currently moving
};
```

### System State
```cpp
struct SystemState {
    MotorState motors[3];
    bool motorsEnabled;       // Global enable
    bool homing;              // Homing in progress
    bool homed;               // Successfully homed
    uint32_t lastStatusTime;  // Last status update
    uint32_t lastPingTime;    // Last ping sent
    uint32_t lastPingReceived; // Last ping received
};
```

---

## Pin Configuration

### Motor 1 (X-axis)
- STEP: Pin 2
- DIR: Pin 3
- ENABLE: Pin 4
- ENDSTOP: Pin 11 (optional)

### Motor 2 (Y-axis)
- STEP: Pin 5
- DIR: Pin 6
- ENABLE: Pin 7
- ENDSTOP: Pin 12 (optional)

### Motor 3 (Z-axis)
- STEP: Pin 8
- DIR: Pin 9
- ENABLE: Pin 10
- ENDSTOP: Pin 13 (optional)

---

## Configuration

### Default Settings
```cpp
const float DEFAULT_MAX_SPEED = 1000.0;      // steps/sec
const float DEFAULT_ACCELERATION = 500.0;     // steps/sec²
const int DEFAULT_MICROSTEPPING = 16;
const uint32_t STATUS_INTERVAL = 100;        // ms
const uint32_t PING_INTERVAL = 5000;         // ms
const uint32_t CONNECTION_TIMEOUT = 10000;   // ms
```

### Network Settings
```cpp
IPAddress serverIP(192, 168, 1, 100);
uint16_t serverPort = 8080;
```

---

## Error Handling

### Error Types
- **Invalid Command**: Unknown packet type
- **Invalid Parameters**: Out of range values
- **Motors Not Enabled**: Command requires enabled motors
- **Movement Out of Range**: Target exceeds limits
- **Hardware Error**: Motor driver fault, etc.

### Error Response
```cpp
void sendError(ErrorCode code, const char* message) {
    Packet response;
    response.type = PacketType::RESP_ERROR;
    response.payload.push_back(static_cast<uint8_t>(code));
    response.payload.push_back(strlen(message));
    response.payload.insert(response.payload.end(), 
                           message, message + strlen(message));
    sendPacket(encode(response));
}
```

---

## Synchronization

### Coordinated Movement

All 3 motors must move together. Use `MultiStepper`:

```cpp
MultiStepper steppers;
steppers.addStepper(motor1);
steppers.addStepper(motor2);
steppers.addStepper(motor3);

void moveToAbsolute(const int32_t positions[3]) {
    long targets[3] = {positions[0], positions[1], positions[2]};
    steppers.moveTo(targets);
}
```

### Movement Queue

If a new movement command arrives while motors are moving:
- Option 1: Reject (return error)
- Option 2: Queue (wait for current movement)
- Option 3: Abort and start new (emergency stop then move)

**Recommendation**: Option 2 (queue) for smooth operation, but limit queue depth.

---

## Performance Considerations

### Timing Constraints
- **Packet Processing**: < 10ms
- **Status Updates**: Every 100ms
- **Motor Updates**: Every iteration (non-blocking)
- **Keep-Alive**: Every 5 seconds

### Memory Management
- **Packet Buffer**: 256 bytes (sufficient for all commands)
- **Movement Queue**: 3-5 commands max
- **Status Buffer**: 64 bytes

### Optimization
- Use non-blocking motor updates
- Batch status updates
- Minimize packet parsing overhead
- Use efficient data structures

---

## Testing Strategy

### Unit Tests (Simulated)
1. Packet encoding/decoding
2. Command parsing
3. Motor state management
4. Status reporting

### Integration Tests
1. Ethernet connection
2. Command execution
3. Coordinated movement
4. Error handling

### Hardware Tests
1. Individual motor control
2. Coordinated movement
3. Homing sequence
4. Real-time performance

---

## Implementation Phases

### Phase 1: Basic Communication
- Ethernet setup
- Packet parser
- Basic command handlers (enable, ping, status)

### Phase 2: Motor Control
- Stepper motor initialization
- Individual motor movement
- Coordinated movement

### Phase 3: Advanced Features
- Homing system
- Acceleration control
- Error handling
- Movement queue

### Phase 4: Optimization
- Performance tuning
- Memory optimization
- Robustness improvements

---

## Next Steps

1. **Set up development environment**
   - Install Teensyduino
   - Install required libraries
   - Configure network settings

2. **Implement core components**
   - Start with packet parser
   - Add basic command handlers
   - Integrate motor control

3. **Test incrementally**
   - Test each component separately
   - Test with simulated hardware first
   - Test with real hardware

4. **Iterate and improve**
   - Add features as needed
   - Optimize performance
   - Handle edge cases

