#include "HardwareInterface.hpp"
#include "DeltaRobotProtocol.hpp"
#include <iostream>
#include <sstream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

// Simulated Hardware Interface Implementation
SimulatedHardwareInterface::SimulatedHardwareInterface()
    : connectionState_(ConnectionState::Disconnected)
    , motorsEnabled_(false)
{
    simulatedPositions_ = {0, 0, 0};
    simulatedMoving_ = {false, false, false};
}

bool SimulatedHardwareInterface::connect(const std::string& port) {
    std::cout << "[Simulated] Connecting to port: " << port << std::endl;
    connectionState_ = ConnectionState::Connected;
    if (onConnected_) onConnected_();
    return true;
}

void SimulatedHardwareInterface::disconnect() {
    std::cout << "[Simulated] Disconnecting" << std::endl;
    connectionState_ = ConnectionState::Disconnected;
    motorsEnabled_ = false;
    if (onDisconnected_) onDisconnected_();
}

bool SimulatedHardwareInterface::moveMotorsToSteps(const std::array<int32_t, 3>& targetSteps) {
    if (!isConnected() || !motorsEnabled_) return false;
    
    std::cout << "[Simulated] Moving motors to: [" 
              << targetSteps[0] << ", " << targetSteps[1] << ", " << targetSteps[2] << "]" << std::endl;
    
    simulatedPositions_ = targetSteps;
    simulatedMoving_ = {true, true, true};
    
    // Simulate movement completion after a short delay
    // In real implementation, this would be handled by hardware
    return true;
}

bool SimulatedHardwareInterface::moveMotorsRelative(const std::array<int32_t, 3>& stepDeltas) {
    if (!isConnected() || !motorsEnabled_) return false;
    
    std::cout << "[Simulated] Moving motors relative: [" 
              << stepDeltas[0] << ", " << stepDeltas[1] << ", " << stepDeltas[2] << "]" << std::endl;
    
    for (int i = 0; i < 3; ++i) {
        simulatedPositions_[i] += stepDeltas[i];
    }
    simulatedMoving_ = {true, true, true};
    
    return true;
}

bool SimulatedHardwareInterface::sendWaypointSequence(const std::vector<std::pair<std::array<int32_t, 3>, uint16_t>>& waypoints) {
    if (!isConnected() || !motorsEnabled_ || waypoints.empty()) return false;
    
    std::cout << "[Simulated] Sending waypoint sequence with " << waypoints.size() << " waypoints" << std::endl;
    
    // Simulate executing sequence
    for (size_t i = 0; i < waypoints.size(); ++i) {
        const auto& wp = waypoints[i];
        std::cout << "[Simulated] Waypoint " << (i + 1) << "/" << waypoints.size() 
                  << ": Steps=[" << wp.first[0] << ", " << wp.first[1] << ", " << wp.first[2] << "]"
                  << ", Duration=" << wp.second << "ms" << std::endl;
        simulatedPositions_ = wp.first;
        simulatedMoving_ = {true, true, true};
    }
    
    simulatedMoving_ = {false, false, false};
    std::cout << "[Simulated] Sequence complete" << std::endl;
    return true;
}

bool SimulatedHardwareInterface::setMotorSpeed(int motorIndex, float speed) {
    if (motorIndex < 0 || motorIndex >= 3) return false;
    std::cout << "[Simulated] Setting motor " << motorIndex << " speed to " << speed << std::endl;
    return true;
}

bool SimulatedHardwareInterface::setMotorAcceleration(int motorIndex, float acceleration) {
    if (motorIndex < 0 || motorIndex >= 3) return false;
    std::cout << "[Simulated] Setting motor " << motorIndex << " acceleration to " << acceleration << std::endl;
    return true;
}

bool SimulatedHardwareInterface::enableMotors(bool enable) {
    motorsEnabled_ = enable;
    std::cout << "[Simulated] Motors " << (enable ? "enabled" : "disabled") << std::endl;
    return true;
}

bool SimulatedHardwareInterface::stopMotors() {
    std::cout << "[Simulated] Stopping motors" << std::endl;
    simulatedMoving_ = {false, false, false};
    return true;
}

bool SimulatedHardwareInterface::homeMotors() {
    if (!isConnected()) return false;
    std::cout << "[Simulated] Homing motors" << std::endl;
    simulatedPositions_ = {0, 0, 0};
    simulatedMoving_ = {false, false, false};
    return true;
}

bool SimulatedHardwareInterface::setMotorPosition(int motorIndex, int32_t steps) {
    if (motorIndex < 0 || motorIndex >= 3) return false;
    simulatedPositions_[motorIndex] = steps;
    std::cout << "[Simulated] Setting motor " << motorIndex << " position to " << steps << std::endl;
    return true;
}

bool SimulatedHardwareInterface::getMotorPositions(std::array<int32_t, 3>& positions) {
    if (!isConnected()) return false;
    positions = simulatedPositions_;
    return true;
}

bool SimulatedHardwareInterface::getMotorStates(std::array<bool, 3>& isMoving) {
    if (!isConnected()) return false;
    isMoving = simulatedMoving_;
    return true;
}

bool SimulatedHardwareInterface::setMicrostepping(int motorIndex, int microsteps) {
    if (motorIndex < 0 || motorIndex >= 3) return false;
    std::cout << "[Simulated] Setting motor " << motorIndex << " microstepping to " << microsteps << std::endl;
    return true;
}

bool SimulatedHardwareInterface::setMotorConfig(const MotorConfig& config, int motorIndex) {
    if (motorIndex < 0 || motorIndex >= 3) return false;
    std::cout << "[Simulated] Setting motor " << motorIndex << " config" << std::endl;
    return true;
}

// Serial Hardware Interface Implementation
SerialHardwareInterface::SerialHardwareInterface()
    : connectionState_(ConnectionState::Disconnected)
    , serialPort_(nullptr)
{
    // TODO: Initialize serial port library (e.g., libserial, boost::asio, etc.)
}

SerialHardwareInterface::~SerialHardwareInterface() {
    disconnect();
}

bool SerialHardwareInterface::connect(const std::string& port) {
    if (isConnected()) {
        disconnect();
    }
    
    connectionState_ = ConnectionState::Connecting;
    
    // TODO: Implement actual serial port connection
    // Example using libserial or similar:
    // serialPort_ = new SerialPort(port);
    // if (!serialPort_->open()) {
    //     connectionState_ = ConnectionState::Error;
    //     if (onError_) onError_("Failed to open serial port");
    //     return false;
    // }
    
    std::cout << "[Serial] Connecting to " << port << " (not yet implemented)" << std::endl;
    
    // For now, simulate connection
    connectionState_ = ConnectionState::Connected;
    if (onConnected_) onConnected_();
    
    return true;
}

void SerialHardwareInterface::disconnect() {
    if (serialPort_) {
        // TODO: Close serial port
        // delete serialPort_;
        serialPort_ = nullptr;
    }
    connectionState_ = ConnectionState::Disconnected;
    if (onDisconnected_) onDisconnected_();
}

bool SerialHardwareInterface::sendCommand(const std::string& command) {
    if (!isConnected()) return false;
    
    // TODO: Send command over serial
    // std::string cmd = command + "\n";
    // serialPort_->write(cmd);
    
    std::cout << "[Serial] Sending: " << command << std::endl;
    return true;
}

std::string SerialHardwareInterface::readResponse() {
    if (!isConnected()) return "";
    
    // TODO: Read response from serial
    // return serialPort_->readLine();
    
    return "OK";
}

bool SerialHardwareInterface::parseResponse(const std::string& response, bool& success) {
    // Simple response parsing
    success = (response.find("OK") != std::string::npos || 
               response.find("ok") != std::string::npos);
    return true;
}

bool SerialHardwareInterface::moveMotorsToSteps(const std::array<int32_t, 3>& targetSteps) {
    if (!isConnected()) return false;
    
    std::ostringstream cmd;
    cmd << "G1 X" << targetSteps[0] << " Y" << targetSteps[1] << " Z" << targetSteps[2];
    
    bool success = false;
    if (sendCommand(cmd.str())) {
        std::string response = readResponse();
        parseResponse(response, success);
    }
    
    return success;
}

bool SerialHardwareInterface::moveMotorsRelative(const std::array<int32_t, 3>& stepDeltas) {
    if (!isConnected()) return false;
    
    std::ostringstream cmd;
    cmd << "G91 G1 X" << stepDeltas[0] << " Y" << stepDeltas[1] << " Z" << stepDeltas[2];
    
    bool success = false;
    if (sendCommand(cmd.str())) {
        std::string response = readResponse();
        parseResponse(response, success);
    }
    
    return success;
}

bool SerialHardwareInterface::setMotorSpeed(int motorIndex, float speed) {
    if (motorIndex < 0 || motorIndex >= 3) return false;
    
    std::ostringstream cmd;
    cmd << "M203 S" << speed << " T" << motorIndex;
    
    bool success = false;
    if (sendCommand(cmd.str())) {
        std::string response = readResponse();
        parseResponse(response, success);
    }
    
    return success;
}

bool SerialHardwareInterface::setMotorAcceleration(int motorIndex, float acceleration) {
    if (motorIndex < 0 || motorIndex >= 3) return false;
    
    std::ostringstream cmd;
    cmd << "M204 A" << acceleration << " T" << motorIndex;
    
    bool success = false;
    if (sendCommand(cmd.str())) {
        std::string response = readResponse();
        parseResponse(response, success);
    }
    
    return success;
}

bool SerialHardwareInterface::enableMotors(bool enable) {
    std::ostringstream cmd;
    cmd << (enable ? "M17" : "M18");
    
    bool success = false;
    if (sendCommand(cmd.str())) {
        std::string response = readResponse();
        parseResponse(response, success);
    }
    
    return success;
}

bool SerialHardwareInterface::stopMotors() {
    bool success = false;
    if (sendCommand("M112")) {  // Emergency stop
        std::string response = readResponse();
        parseResponse(response, success);
    }
    return success;
}

bool SerialHardwareInterface::homeMotors() {
    bool success = false;
    if (sendCommand("G28")) {
        std::string response = readResponse();
        parseResponse(response, success);
    }
    return success;
}

bool SerialHardwareInterface::setMotorPosition(int motorIndex, int32_t steps) {
    if (motorIndex < 0 || motorIndex >= 3) return false;
    
    std::ostringstream cmd;
    char axis = 'X' + motorIndex;
    cmd << "G92 " << axis << steps;
    
    bool success = false;
    if (sendCommand(cmd.str())) {
        std::string response = readResponse();
        parseResponse(response, success);
    }
    
    return success;
}

bool SerialHardwareInterface::getMotorPositions(std::array<int32_t, 3>& positions) {
    bool success = false;
    if (sendCommand("M114")) {  // Get position
        std::string response = readResponse();
        // TODO: Parse position from response
        // Example: "X:100 Y:200 Z:300"
        success = true;
    }
    return success;
}

bool SerialHardwareInterface::getMotorStates(std::array<bool, 3>& isMoving) {
    // TODO: Implement status query
    return false;
}

bool SerialHardwareInterface::setMicrostepping(int motorIndex, int microsteps) {
    if (motorIndex < 0 || motorIndex >= 3) return false;
    
    std::ostringstream cmd;
    cmd << "M350 S" << microsteps << " T" << motorIndex;
    
    bool success = false;
    if (sendCommand(cmd.str())) {
        std::string response = readResponse();
        parseResponse(response, success);
    }
    
    return success;
}

bool SerialHardwareInterface::setMotorConfig(const MotorConfig& config, int motorIndex) {
    // Set multiple parameters
    bool success = true;
    success &= setMicrostepping(motorIndex, config.microstepping);
    success &= setMotorSpeed(motorIndex, config.maxStepsPerSecond);
    success &= setMotorAcceleration(motorIndex, config.acceleration);
    return success;
}

bool SerialHardwareInterface::sendWaypointSequence(const std::vector<std::pair<std::array<int32_t, 3>, uint16_t>>& waypoints) {
    // Serial interface doesn't support sequence packets yet
    // Fall back to sending waypoints one-by-one
    bool success = true;
    for (const auto& wp : waypoints) {
        success &= moveMotorsToSteps(wp.first);
        if (!success) break;
        // TODO: Wait for duration or use delay
    }
    return success;
}

std::vector<std::string> SerialHardwareInterface::getAvailablePorts() const {
    // TODO: Implement port enumeration
    // This would typically use OS-specific APIs or libraries
    std::vector<std::string> ports;
    // Example: ports.push_back("/dev/ttyUSB0");
    // Example: ports.push_back("/dev/ttyACM0");
    // Example: ports.push_back("COM3");
    return ports;
}

// Ethernet Hardware Interface Implementation
EthernetHardwareInterface::EthernetHardwareInterface()
    : connectionState_(ConnectionState::Disconnected)
    , socketHandle_(nullptr)
    , serverPort_(8080)
    , timeoutMs_(5000)
{
    socketHandle_ = new int(-1);  // Socket file descriptor
}

EthernetHardwareInterface::~EthernetHardwareInterface() {
    disconnect();
    delete static_cast<int*>(socketHandle_);
}

bool EthernetHardwareInterface::parseIPAddress(const std::string& address, std::string& ip, int& port) {
    size_t colonPos = address.find(':');
    if (colonPos != std::string::npos) {
        ip = address.substr(0, colonPos);
        try {
            port = std::stoi(address.substr(colonPos + 1));
        } catch (...) {
            return false;
        }
    } else {
        ip = address;
        port = 8080;  // Default port
    }
    return true;
}

bool EthernetHardwareInterface::connect(const std::string& address) {
    if (isConnected()) {
        disconnect();
    }
    
    connectionState_ = ConnectionState::Connecting;
    
    if (!parseIPAddress(address, serverIP_, serverPort_)) {
        connectionState_ = ConnectionState::Error;
        if (onError_) onError_("Invalid IP address format");
        return false;
    }
    
    int& sockfd = *static_cast<int*>(socketHandle_);
    
    // Create socket
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        connectionState_ = ConnectionState::Error;
        if (onError_) onError_("Failed to create socket");
        return false;
    }
    
    // Set socket to non-blocking for timeout
    int flags = fcntl(sockfd, F_GETFL, 0);
    fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);
    
    // Set up server address
    struct sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(serverPort_);
    
    if (inet_pton(AF_INET, serverIP_.c_str(), &serverAddr.sin_addr) <= 0) {
        connectionState_ = ConnectionState::Error;
        close(sockfd);
        sockfd = -1;
        if (onError_) onError_("Invalid IP address");
        return false;
    }
    
    // Attempt connection with timeout
    int result = ::connect(sockfd, (struct sockaddr*)&serverAddr, sizeof(serverAddr));
    if (result < 0) {
        if (errno == EINPROGRESS) {
            // Connection in progress, wait with timeout
            fd_set writefds;
            struct timeval timeout;
            FD_ZERO(&writefds);
            FD_SET(sockfd, &writefds);
            timeout.tv_sec = timeoutMs_ / 1000;
            timeout.tv_usec = (timeoutMs_ % 1000) * 1000;
            
            result = select(sockfd + 1, NULL, &writefds, NULL, &timeout);
            if (result <= 0) {
                connectionState_ = ConnectionState::Error;
                close(sockfd);
                sockfd = -1;
                if (onError_) onError_("Connection timeout");
                return false;
            }
            
            // Check if connection succeeded
            int so_error;
            socklen_t len = sizeof(so_error);
            getsockopt(sockfd, SOL_SOCKET, SO_ERROR, &so_error, &len);
            if (so_error != 0) {
                connectionState_ = ConnectionState::Error;
                close(sockfd);
                sockfd = -1;
                if (onError_) onError_("Connection failed");
                return false;
            }
        } else {
            connectionState_ = ConnectionState::Error;
            close(sockfd);
            sockfd = -1;
            if (onError_) onError_("Connection failed");
            return false;
        }
    }
    
    // Set socket back to blocking
    flags = fcntl(sockfd, F_GETFL, 0);
    fcntl(sockfd, F_SETFL, flags & ~O_NONBLOCK);
    
    // Set socket options for keep-alive
    int opt = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_KEEPALIVE, &opt, sizeof(opt));
    
    connectionState_ = ConnectionState::Connected;
    std::cout << "[Ethernet] Connected to " << serverIP_ << ":" << serverPort_ << std::endl;
    if (onConnected_) onConnected_();
    
    return true;
}

void EthernetHardwareInterface::disconnect() {
    int& sockfd = *static_cast<int*>(socketHandle_);
    if (sockfd >= 0) {
        close(sockfd);
        sockfd = -1;
        std::cout << "[Ethernet] Disconnected" << std::endl;
    }
    connectionState_ = ConnectionState::Disconnected;
    if (onDisconnected_) onDisconnected_();
}

bool EthernetHardwareInterface::sendCommand(const std::string& command) {
    if (!isConnected()) return false;
    
    int& sockfd = *static_cast<int*>(socketHandle_);
    std::string cmd = command + "\n";
    
    ssize_t sent = send(sockfd, cmd.c_str(), cmd.length(), 0);
    if (sent < 0) {
        connectionState_ = ConnectionState::Error;
        if (onError_) onError_("Send failed");
        return false;
    }
    
    return true;
}

std::string EthernetHardwareInterface::readResponse() {
    // Legacy method for text-based responses (kept for compatibility)
    std::vector<uint8_t> binary = readBinaryResponse();
    return std::string(binary.begin(), binary.end());
}

std::vector<uint8_t> EthernetHardwareInterface::readBinaryResponse() {
    std::vector<uint8_t> response;
    if (!isConnected()) return response;
    
    int& sockfd = *static_cast<int*>(socketHandle_);
    
    // Set receive timeout
    struct timeval timeout;
    timeout.tv_sec = timeoutMs_ / 1000;
    timeout.tv_usec = (timeoutMs_ % 1000) * 1000;
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    
    // Read packet header first (type + length = 3 bytes minimum)
    uint8_t header[3];
    ssize_t received = recv(sockfd, header, 3, 0);
    if (received < 3) {
        if (received == 0) {
            connectionState_ = ConnectionState::Disconnected;
            if (onDisconnected_) onDisconnected_();
        }
        return response;
    }
    
    response.insert(response.end(), header, header + 3);
    
    // Read payload length
    uint16_t payloadLength = static_cast<uint16_t>(header[1]) | 
                            (static_cast<uint16_t>(header[2]) << 8);
    
    // Read payload + checksum
    if (payloadLength > 0) {
        std::vector<uint8_t> payload(payloadLength + 1);  // +1 for checksum
        received = recv(sockfd, payload.data(), payloadLength + 1, 0);
        if (received > 0) {
            response.insert(response.end(), payload.begin(), payload.begin() + received);
        }
    } else {
        // Read checksum byte
        uint8_t checksum;
        received = recv(sockfd, &checksum, 1, 0);
        if (received > 0) {
            response.push_back(checksum);
        }
    }
    
    return response;
}

bool EthernetHardwareInterface::parseResponse(const std::string& response, bool& success) {
    // Parse response format: "OK" or "ERR:message"
    if (response.empty()) {
        success = false;
        return false;
    }
    
    if (response.find("OK") == 0 || response.find("ok") == 0) {
        success = true;
        return true;
    } else if (response.find("ERR:") == 0) {
        success = false;
        if (onError_) onError_(response.substr(4));
        return true;
    }
    
    // Try to parse as position report: "POS X:100 Y:200 Z:300"
    // This is handled in getMotorPositions
    
    success = true;
    return true;
}

bool EthernetHardwareInterface::moveMotorsToSteps(const std::array<int32_t, 3>& targetSteps) {
    if (!isConnected()) return false;
    
    // Use binary protocol instead of G-code
    using namespace DeltaRobotProtocol;
    Packet packet = Commands::moveAbsolute(targetSteps);
    std::vector<uint8_t> data = PacketEncoder::encode(packet);
    
    // Send packet
    int& sockfd = *static_cast<int*>(socketHandle_);
    ssize_t sent = send(sockfd, data.data(), data.size(), 0);
    if (sent < 0) {
        connectionState_ = ConnectionState::Error;
        if (onError_) onError_("Send failed");
        return false;
    }
    
    // Read response
    std::vector<uint8_t> response = readBinaryResponse();
    if (!response.empty()) {
        Packet respPacket;
        if (PacketEncoder::decode(response, respPacket)) {
            return respPacket.type == PacketType::RESP_OK;
        }
    }

    return false;
}

bool EthernetHardwareInterface::sendWaypointSequence(const std::vector<std::pair<std::array<int32_t, 3>, uint16_t>>& waypoints) {
    if (!isConnected() || waypoints.empty()) return false;
    
    // Use binary protocol to send sequence packet
    using namespace DeltaRobotProtocol;
    Packet packet = Commands::sequence(waypoints);
    std::vector<uint8_t> data = PacketEncoder::encode(packet);
    
    // Send packet
    int& sockfd = *static_cast<int*>(socketHandle_);
    ssize_t sent = send(sockfd, data.data(), data.size(), 0);
    if (sent < 0) {
        connectionState_ = ConnectionState::Error;
        if (onError_) onError_("Send sequence failed");
        return false;
    }
    
    // Read response
    std::vector<uint8_t> response = readBinaryResponse();
    if (!response.empty()) {
        Packet respPacket;
        if (PacketEncoder::decode(response, respPacket)) {
            return respPacket.type == PacketType::RESP_OK;
        }
    }
    
    return false;
}

bool EthernetHardwareInterface::moveMotorsRelative(const std::array<int32_t, 3>& stepDeltas) {
    if (!isConnected()) return false;
    
    using namespace DeltaRobotProtocol;
    Packet packet = Commands::moveRelative(stepDeltas);
    std::vector<uint8_t> data = PacketEncoder::encode(packet);
    
    int& sockfd = *static_cast<int*>(socketHandle_);
    ssize_t sent = send(sockfd, data.data(), data.size(), 0);
    if (sent < 0) {
        connectionState_ = ConnectionState::Error;
        if (onError_) onError_("Send failed");
        return false;
    }
    
    std::vector<uint8_t> response = readBinaryResponse();
    if (!response.empty()) {
        Packet respPacket;
        if (PacketEncoder::decode(response, respPacket)) {
            return respPacket.type == PacketType::RESP_OK;
        }
    }
    
    return false;
}

bool EthernetHardwareInterface::setMotorSpeed(int motorIndex, float speed) {
    if (motorIndex < 0 || motorIndex >= 3) return false;
    
    std::ostringstream cmd;
    cmd << "M203 S" << (int)speed << " T" << motorIndex;
    
    bool success = false;
    if (sendCommand(cmd.str())) {
        std::string response = readResponse();
        parseResponse(response, success);
    }
    
    return success;
}

bool EthernetHardwareInterface::setMotorAcceleration(int motorIndex, float acceleration) {
    if (motorIndex < 0 || motorIndex >= 3) return false;
    
    std::ostringstream cmd;
    cmd << "M204 A" << (int)acceleration << " T" << motorIndex;
    
    bool success = false;
    if (sendCommand(cmd.str())) {
        std::string response = readResponse();
        parseResponse(response, success);
    }
    
    return success;
}

bool EthernetHardwareInterface::enableMotors(bool enable) {
    if (!isConnected()) return false;
    
    using namespace DeltaRobotProtocol;
    Packet packet = Commands::enableMotors(enable);
    std::vector<uint8_t> data = PacketEncoder::encode(packet);
    
    int& sockfd = *static_cast<int*>(socketHandle_);
    ssize_t sent = send(sockfd, data.data(), data.size(), 0);
    if (sent < 0) {
        connectionState_ = ConnectionState::Error;
        if (onError_) onError_("Send failed");
        return false;
    }
    
    std::vector<uint8_t> response = readBinaryResponse();
    if (!response.empty()) {
        Packet respPacket;
        if (PacketEncoder::decode(response, respPacket)) {
            return respPacket.type == PacketType::RESP_OK;
        }
    }
    
    return false;
}

bool EthernetHardwareInterface::stopMotors() {
    bool success = false;
    if (sendCommand("M112")) {  // Emergency stop
        std::string response = readResponse();
        parseResponse(response, success);
    }
    return success;
}

bool EthernetHardwareInterface::homeMotors() {
    bool success = false;
    if (sendCommand("G28")) {
        std::string response = readResponse();
        parseResponse(response, success);
    }
    return success;
}

bool EthernetHardwareInterface::setMotorPosition(int motorIndex, int32_t steps) {
    if (motorIndex < 0 || motorIndex >= 3) return false;
    
    std::ostringstream cmd;
    char axis = 'X' + motorIndex;
    cmd << "G92 " << axis << steps;
    
    bool success = false;
    if (sendCommand(cmd.str())) {
        std::string response = readResponse();
        parseResponse(response, success);
    }
    
    return success;
}

bool EthernetHardwareInterface::getMotorPositions(std::array<int32_t, 3>& positions) {
    if (!isConnected()) return false;
    
    using namespace DeltaRobotProtocol;
    Packet packet = Commands::requestStatus();
    std::vector<uint8_t> data = PacketEncoder::encode(packet);
    
    int& sockfd = *static_cast<int*>(socketHandle_);
    ssize_t sent = send(sockfd, data.data(), data.size(), 0);
    if (sent < 0) {
        return false;
    }
    
    std::vector<uint8_t> response = readBinaryResponse();
    if (!response.empty()) {
        Packet respPacket;
        if (PacketEncoder::decode(response, respPacket) && 
            respPacket.type == PacketType::RESP_STATUS) {
            Responses::StatusInfo status;
            if (Responses::parseStatus(respPacket, status)) {
                positions = status.positions;
                return true;
            }
        }
    }
    
    return false;
}

bool EthernetHardwareInterface::getMotorStates(std::array<bool, 3>& isMoving) {
    // TODO: Implement status query command
    // For now, assume all motors are not moving if we can't query
    isMoving = {false, false, false};
    return false;
}

bool EthernetHardwareInterface::setMicrostepping(int motorIndex, int microsteps) {
    if (motorIndex < 0 || motorIndex >= 3) return false;
    
    std::ostringstream cmd;
    cmd << "M350 S" << microsteps << " T" << motorIndex;
    
    bool success = false;
    if (sendCommand(cmd.str())) {
        std::string response = readResponse();
        parseResponse(response, success);
    }
    
    return success;
}

bool EthernetHardwareInterface::setMotorConfig(const MotorConfig& config, int motorIndex) {
    // Set multiple parameters
    bool success = true;
    success &= setMicrostepping(motorIndex, config.microstepping);
    success &= setMotorSpeed(motorIndex, config.maxStepsPerSecond);
    success &= setMotorAcceleration(motorIndex, config.acceleration);
    return success;
}

