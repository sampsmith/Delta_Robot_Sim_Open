#pragma once

#include <cstdint>
#include <array>
#include <vector>
#include <string>

// Binary protocol definitions for Delta Robot control
// Direct step-based communication (no G-code)

namespace DeltaRobotProtocol {

// Packet types
enum class PacketType : uint8_t {
    // Commands (PC → NUCLEO-H7S3L8)
    CMD_MOVE_ABS = 0x01,
    CMD_MOVE_REL = 0x02,
    CMD_SET_SPEED = 0x03,
    CMD_SET_ACCEL = 0x04,
    CMD_ENABLE = 0x05,
    CMD_STOP = 0x06,
    CMD_HOME = 0x07,
    CMD_SET_POS = 0x08,
    CMD_CONFIG = 0x09,
    CMD_PING = 0x0A,
    CMD_REQUEST_STATUS = 0x0B,
    CMD_SEQUENCE = 0x0C,  // Send waypoint sequence (all at once)
    
    // Responses (NUCLEO-H7S3L8 → PC)
    RESP_OK = 0x80,
    RESP_ERROR = 0x81,
    RESP_STATUS = 0x82,
    RESP_PONG = 0x83,
    RESP_HOMED = 0x84
};

// Error codes
enum class ErrorCode : uint8_t {
    INVALID_COMMAND = 0x01,
    INVALID_PARAMS = 0x02,
    MOTORS_NOT_ENABLED = 0x03,
    MOVEMENT_OUT_OF_RANGE = 0x04,
    HARDWARE_ERROR = 0x05
};

// Packet structure
struct Packet {
    PacketType type;
    std::vector<uint8_t> payload;
    
    Packet() : type(PacketType::CMD_PING) {}
    Packet(PacketType t) : type(t) {}
};

// Packet encoding/decoding
class PacketEncoder {
public:
    // Encode packet to byte array
    static std::vector<uint8_t> encode(const Packet& packet);
    
    // Decode byte array to packet
    static bool decode(const std::vector<uint8_t>& data, Packet& packet);
    
    // Calculate checksum
    static uint8_t calculateChecksum(const std::vector<uint8_t>& data);
    
    // Verify packet checksum
    static bool verifyChecksum(const std::vector<uint8_t>& data);
};

// Command builders (convenience functions)
namespace Commands {
    // Move to absolute step positions
    Packet moveAbsolute(const std::array<int32_t, 3>& positions);
    
    // Move relative to current positions
    Packet moveRelative(const std::array<int32_t, 3>& deltas);
    
    // Set motor speed
    Packet setSpeed(int motorIndex, float speed);
    
    // Set motor acceleration
    Packet setAcceleration(int motorIndex, float acceleration);
    
    // Enable/disable motors
    Packet enableMotors(bool enable);
    
    // Emergency stop
    Packet stop();
    
    // Home motors
    Packet home();
    
    // Set current position
    Packet setPosition(int motorIndex, int32_t position);
    
    // Set motor configuration
    Packet setConfig(int motorIndex, float maxSpeed, float acceleration, uint8_t microstepping);
    
    // Ping/keep-alive
    Packet ping();
    
    // Request status
    Packet requestStatus();
    
    // Send waypoint sequence (all waypoints in one packet)
    // Format: [count:1][waypoint1_steps:12][waypoint1_duration_ms:2][waypoint2_steps:12][waypoint2_duration_ms:2]...
    Packet sequence(const std::vector<std::pair<std::array<int32_t, 3>, uint16_t>>& waypoints);
}

// Response parsers
namespace Responses {
    // Parse OK response
    bool parseOK(const Packet& packet);
    
    // Parse error response
    struct ErrorInfo {
        ErrorCode code;
        std::string message;
    };
    bool parseError(const Packet& packet, ErrorInfo& error);
    
    // Parse status response
    struct StatusInfo {
        std::array<int32_t, 3> positions;
        std::array<bool, 3> moving;
        bool enabled;
    };
    bool parseStatus(const Packet& packet, StatusInfo& status);
    
    // Parse homed response
    bool parseHomed(const Packet& packet, std::array<int32_t, 3>& positions);
    
    // Parse pong response
    bool parsePong(const Packet& packet);
}

// Helper functions for binary data
namespace Binary {
    // Write int32_t to buffer (little-endian)
    void writeInt32(std::vector<uint8_t>& buffer, int32_t value);
    
    // Read int32_t from buffer (little-endian)
    bool readInt32(const std::vector<uint8_t>& buffer, size_t offset, int32_t& value);
    
    // Write float to buffer
    void writeFloat(std::vector<uint8_t>& buffer, float value);
    
    // Read float from buffer
    bool readFloat(const std::vector<uint8_t>& buffer, size_t offset, float& value);
    
    // Write uint16_t to buffer (little-endian)
    void writeUint16(std::vector<uint8_t>& buffer, uint16_t value);
    
    // Read uint16_t from buffer (little-endian)
    bool readUint16(const std::vector<uint8_t>& buffer, size_t offset, uint16_t& value);
}

} // namespace DeltaRobotProtocol

