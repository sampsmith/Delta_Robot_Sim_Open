#include "DeltaRobotProtocol.hpp"
#include <cstring>
#include <algorithm>

namespace DeltaRobotProtocol {

// Binary helper functions
namespace Binary {
    void writeInt32(std::vector<uint8_t>& buffer, int32_t value) {
        buffer.push_back(static_cast<uint8_t>(value & 0xFF));
        buffer.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
        buffer.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
        buffer.push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
    }
    
    bool readInt32(const std::vector<uint8_t>& buffer, size_t offset, int32_t& value) {
        if (offset + 4 > buffer.size()) return false;
        
        value = static_cast<int32_t>(buffer[offset]) |
               (static_cast<int32_t>(buffer[offset + 1]) << 8) |
               (static_cast<int32_t>(buffer[offset + 2]) << 16) |
               (static_cast<int32_t>(buffer[offset + 3]) << 24);
        return true;
    }
    
    void writeFloat(std::vector<uint8_t>& buffer, float value) {
        uint32_t bits;
        std::memcpy(&bits, &value, sizeof(float));
        buffer.push_back(static_cast<uint8_t>(bits & 0xFF));
        buffer.push_back(static_cast<uint8_t>((bits >> 8) & 0xFF));
        buffer.push_back(static_cast<uint8_t>((bits >> 16) & 0xFF));
        buffer.push_back(static_cast<uint8_t>((bits >> 24) & 0xFF));
    }
    
    bool readFloat(const std::vector<uint8_t>& buffer, size_t offset, float& value) {
        if (offset + 4 > buffer.size()) return false;
        
        uint32_t bits = static_cast<uint32_t>(buffer[offset]) |
                       (static_cast<uint32_t>(buffer[offset + 1]) << 8) |
                       (static_cast<uint32_t>(buffer[offset + 2]) << 16) |
                       (static_cast<uint32_t>(buffer[offset + 3]) << 24);
        std::memcpy(&value, &bits, sizeof(float));
        return true;
    }
    
    void writeUint16(std::vector<uint8_t>& buffer, uint16_t value) {
        buffer.push_back(static_cast<uint8_t>(value & 0xFF));
        buffer.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
    }
    
    bool readUint16(const std::vector<uint8_t>& buffer, size_t offset, uint16_t& value) {
        if (offset + 2 > buffer.size()) return false;
        
        value = static_cast<uint16_t>(buffer[offset]) |
               (static_cast<uint16_t>(buffer[offset + 1]) << 8);
        return true;
    }
}

// PacketEncoder implementation
uint8_t PacketEncoder::calculateChecksum(const std::vector<uint8_t>& data) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < data.size() - 1; ++i) {
        checksum ^= data[i];
    }
    return checksum;
}

bool PacketEncoder::verifyChecksum(const std::vector<uint8_t>& data) {
    if (data.size() < 1) return false;
    uint8_t calculated = calculateChecksum(data);
    return calculated == data.back();
}

std::vector<uint8_t> PacketEncoder::encode(const Packet& packet) {
    std::vector<uint8_t> data;
    
    // Packet type
    data.push_back(static_cast<uint8_t>(packet.type));
    
    // Payload length (2 bytes, little-endian)
    uint16_t length = static_cast<uint16_t>(packet.payload.size());
    data.push_back(static_cast<uint8_t>(length & 0xFF));
    data.push_back(static_cast<uint8_t>((length >> 8) & 0xFF));
    
    // Payload
    data.insert(data.end(), packet.payload.begin(), packet.payload.end());
    
    // Checksum (will be calculated)
    data.push_back(0);
    
    // Calculate and set checksum
    data.back() = calculateChecksum(data);
    
    return data;
}

bool PacketEncoder::decode(const std::vector<uint8_t>& data, Packet& packet) {
    if (data.size() < 4) return false;  // Minimum: type + length + checksum
    
    // Verify checksum
    if (!verifyChecksum(data)) return false;
    
    // Read packet type
    packet.type = static_cast<PacketType>(data[0]);
    
    // Read payload length
    uint16_t length = static_cast<uint16_t>(data[1]) |
                     (static_cast<uint16_t>(data[2]) << 8);
    
    // Verify length
    if (data.size() < 3 + length + 1) return false;  // header + payload + checksum
    
    // Extract payload
    packet.payload.assign(data.begin() + 3, data.begin() + 3 + length);
    
    return true;
}

// Command builders
namespace Commands {
    Packet moveAbsolute(const std::array<int32_t, 3>& positions) {
        Packet packet(PacketType::CMD_MOVE_ABS);
        Binary::writeInt32(packet.payload, positions[0]);
        Binary::writeInt32(packet.payload, positions[1]);
        Binary::writeInt32(packet.payload, positions[2]);
        return packet;
    }
    
    Packet moveRelative(const std::array<int32_t, 3>& deltas) {
        Packet packet(PacketType::CMD_MOVE_REL);
        Binary::writeInt32(packet.payload, deltas[0]);
        Binary::writeInt32(packet.payload, deltas[1]);
        Binary::writeInt32(packet.payload, deltas[2]);
        return packet;
    }
    
    Packet setSpeed(int motorIndex, float speed) {
        Packet packet(PacketType::CMD_SET_SPEED);
        packet.payload.push_back(static_cast<uint8_t>(motorIndex));
        Binary::writeFloat(packet.payload, speed);
        return packet;
    }
    
    Packet setAcceleration(int motorIndex, float acceleration) {
        Packet packet(PacketType::CMD_SET_ACCEL);
        packet.payload.push_back(static_cast<uint8_t>(motorIndex));
        Binary::writeFloat(packet.payload, acceleration);
        return packet;
    }
    
    Packet enableMotors(bool enable) {
        Packet packet(PacketType::CMD_ENABLE);
        packet.payload.push_back(enable ? 1 : 0);
        return packet;
    }
    
    Packet stop() {
        return Packet(PacketType::CMD_STOP);
    }
    
    Packet home() {
        return Packet(PacketType::CMD_HOME);
    }
    
    Packet setPosition(int motorIndex, int32_t position) {
        Packet packet(PacketType::CMD_SET_POS);
        packet.payload.push_back(static_cast<uint8_t>(motorIndex));
        Binary::writeInt32(packet.payload, position);
        return packet;
    }
    
    Packet setConfig(int motorIndex, float maxSpeed, float acceleration, uint8_t microstepping) {
        Packet packet(PacketType::CMD_CONFIG);
        packet.payload.push_back(static_cast<uint8_t>(motorIndex));
        Binary::writeFloat(packet.payload, maxSpeed);
        Binary::writeFloat(packet.payload, acceleration);
        packet.payload.push_back(microstepping);
        return packet;
    }
    
    Packet ping() {
        return Packet(PacketType::CMD_PING);
    }
    
    Packet requestStatus() {
        return Packet(PacketType::CMD_REQUEST_STATUS);
    }
    
    Packet sequence(const std::vector<std::pair<std::array<int32_t, 3>, uint16_t>>& waypoints) {
        Packet packet(PacketType::CMD_SEQUENCE);
        
        // Limit to 255 waypoints (uint8_t max)
        uint8_t count = static_cast<uint8_t>(std::min(waypoints.size(), size_t(255)));
        packet.payload.push_back(count);
        
        // Add each waypoint: [steps_m1:4][steps_m2:4][steps_m3:4][duration_ms:2]
        for (size_t i = 0; i < count; ++i) {
            const auto& wp = waypoints[i];
            
            // Write step positions for all 3 motors
            Binary::writeInt32(packet.payload, wp.first[0]);
            Binary::writeInt32(packet.payload, wp.first[1]);
            Binary::writeInt32(packet.payload, wp.first[2]);
            
            // Write duration in milliseconds
            Binary::writeUint16(packet.payload, wp.second);
        }
        
        return packet;
    }
}

// Response parsers
namespace Responses {
    bool parseOK(const Packet& packet) {
        return packet.type == PacketType::RESP_OK && packet.payload.empty();
    }
    
    bool parseError(const Packet& packet, ErrorInfo& error) {
        if (packet.type != PacketType::RESP_ERROR) return false;
        if (packet.payload.size() < 2) return false;
        
        error.code = static_cast<ErrorCode>(packet.payload[0]);
        uint8_t msgLength = packet.payload[1];
        
        if (packet.payload.size() < 2 + msgLength) return false;
        
        error.message.assign(
            packet.payload.begin() + 2,
            packet.payload.begin() + 2 + msgLength
        );
        
        return true;
    }
    
    bool parseStatus(const Packet& packet, StatusInfo& status) {
        if (packet.type != PacketType::RESP_STATUS) return false;
        if (packet.payload.size() < 13) return false;  // 3*4 bytes positions + 1 byte flags + 1 byte enabled
        
        size_t offset = 0;
        if (!Binary::readInt32(packet.payload, offset, status.positions[0])) return false;
        offset += 4;
        if (!Binary::readInt32(packet.payload, offset, status.positions[1])) return false;
        offset += 4;
        if (!Binary::readInt32(packet.payload, offset, status.positions[2])) return false;
        offset += 4;
        
        uint8_t flags = packet.payload[offset++];
        status.moving[0] = (flags & 0x01) != 0;
        status.moving[1] = (flags & 0x02) != 0;
        status.moving[2] = (flags & 0x04) != 0;
        
        status.enabled = packet.payload[offset] != 0;
        
        return true;
    }
    
    bool parseHomed(const Packet& packet, std::array<int32_t, 3>& positions) {
        if (packet.type != PacketType::RESP_HOMED) return false;
        if (packet.payload.size() < 12) return false;  // 3*4 bytes
        
        size_t offset = 0;
        if (!Binary::readInt32(packet.payload, offset, positions[0])) return false;
        offset += 4;
        if (!Binary::readInt32(packet.payload, offset, positions[1])) return false;
        offset += 4;
        if (!Binary::readInt32(packet.payload, offset, positions[2])) return false;
        
        return true;
    }
    
    bool parsePong(const Packet& packet) {
        return packet.type == PacketType::RESP_PONG && packet.payload.empty();
    }
}

} // namespace DeltaRobotProtocol

