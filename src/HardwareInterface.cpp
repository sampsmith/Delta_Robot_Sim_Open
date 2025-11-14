#include "HardwareInterface.hpp"
#include "DeltaRobotProtocol.hpp"
#include <iostream>
#include <sstream>
#include <cstring>

// Platform-specific socket includes
#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #include <io.h>
    #pragma comment(lib, "ws2_32.lib")
    #define close closesocket
    #define GET_SOCKET_ERROR() WSAGetLastError()
    typedef int socklen_t;
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #include <fcntl.h>
    #include <errno.h>
    #define GET_SOCKET_ERROR() errno
#endif

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

// Serial Hardware Interface removed - was using G-code
// If needed in the future, implement using binary protocol instead

// Ethernet Hardware Interface Implementation
#ifdef _WIN32
static int winsockRefCount = 0;
#endif

EthernetHardwareInterface::EthernetHardwareInterface()
    : connectionState_(ConnectionState::Disconnected)
    , socketHandle_(nullptr)
    , serverPort_(8080)
    , timeoutMs_(5000)
{
    socketHandle_ = new int(-1);  // Socket file descriptor
#ifdef _WIN32
    // Initialize Winsock (reference counted)
    if (winsockRefCount == 0) {
        WSADATA wsaData;
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
            std::cerr << "[Ethernet] Failed to initialize Winsock" << std::endl;
        }
    }
    winsockRefCount++;
#endif
}

EthernetHardwareInterface::~EthernetHardwareInterface() {
    disconnect();
    delete static_cast<int*>(socketHandle_);
#ifdef _WIN32
    // Cleanup Winsock (reference counted)
    winsockRefCount--;
    if (winsockRefCount == 0) {
        WSACleanup();
    }
#endif
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
#ifdef _WIN32
    u_long mode = 1;
    ioctlsocket(sockfd, FIONBIO, &mode);
#else
    int flags = fcntl(sockfd, F_GETFL, 0);
    fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);
#endif
    
    // Set up server address
    struct sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(serverPort_);
    
#ifdef _WIN32
    serverAddr.sin_addr.s_addr = inet_addr(serverIP_.c_str());
    if (serverAddr.sin_addr.s_addr == INADDR_NONE) {
        connectionState_ = ConnectionState::Error;
        closesocket(sockfd);
        sockfd = -1;
        if (onError_) onError_("Invalid IP address");
        return false;
    }
#else
    if (inet_pton(AF_INET, serverIP_.c_str(), &serverAddr.sin_addr) <= 0) {
        connectionState_ = ConnectionState::Error;
        close(sockfd);
        sockfd = -1;
        if (onError_) onError_("Invalid IP address");
        return false;
    }
#endif
    
    // Attempt connection with timeout
    int result = ::connect(sockfd, (struct sockaddr*)&serverAddr, sizeof(serverAddr));
    if (result < 0) {
        int error = GET_SOCKET_ERROR();
#ifdef _WIN32
        if (error == WSAEWOULDBLOCK || error == WSAEINPROGRESS) {
#else
        if (error == EINPROGRESS) {
#endif
            // Connection in progress, wait with timeout
            fd_set writefds;
            struct timeval timeout;
            FD_ZERO(&writefds);
            FD_SET(sockfd, &writefds);
            timeout.tv_sec = timeoutMs_ / 1000;
            timeout.tv_usec = (timeoutMs_ % 1000) * 1000;
            
            result = select((int)(sockfd + 1), NULL, &writefds, NULL, &timeout);
            if (result <= 0) {
                connectionState_ = ConnectionState::Error;
#ifdef _WIN32
                closesocket(sockfd);
#else
                close(sockfd);
#endif
                sockfd = -1;
                if (onError_) onError_("Connection timeout");
                return false;
            }
            
            // Check if connection succeeded
            int so_error;
            socklen_t len = sizeof(so_error);
            getsockopt(sockfd, SOL_SOCKET, SO_ERROR, (char*)&so_error, &len);
            if (so_error != 0) {
                connectionState_ = ConnectionState::Error;
#ifdef _WIN32
                closesocket(sockfd);
#else
                close(sockfd);
#endif
                sockfd = -1;
                if (onError_) onError_("Connection failed");
                return false;
            }
        } else {
            connectionState_ = ConnectionState::Error;
#ifdef _WIN32
            closesocket(sockfd);
#else
            close(sockfd);
#endif
            sockfd = -1;
            if (onError_) onError_("Connection failed");
            return false;
        }
    }
    
    // Set socket back to blocking
#ifdef _WIN32
    mode = 0;
    ioctlsocket(sockfd, FIONBIO, &mode);
#else
    flags = fcntl(sockfd, F_GETFL, 0);
    fcntl(sockfd, F_SETFL, flags & ~O_NONBLOCK);
#endif
    
    // Set socket options for keep-alive
    int opt = 1;
#ifdef _WIN32
    setsockopt(sockfd, SOL_SOCKET, SO_KEEPALIVE, (const char*)&opt, sizeof(opt));
#else
    setsockopt(sockfd, SOL_SOCKET, SO_KEEPALIVE, &opt, sizeof(opt));
#endif
    
    connectionState_ = ConnectionState::Connected;
    std::cout << "[Ethernet] Connected to " << serverIP_ << ":" << serverPort_ << std::endl;
    if (onConnected_) onConnected_();
    
    return true;
}

void EthernetHardwareInterface::disconnect() {
    int& sockfd = *static_cast<int*>(socketHandle_);
    if (sockfd >= 0) {
#ifdef _WIN32
        closesocket(sockfd);
#else
        close(sockfd);
#endif
        sockfd = -1;
        std::cout << "[Ethernet] Disconnected" << std::endl;
    }
    connectionState_ = ConnectionState::Disconnected;
    if (onDisconnected_) onDisconnected_();
}

std::vector<uint8_t> EthernetHardwareInterface::readBinaryResponse() {
    std::vector<uint8_t> response;
    if (!isConnected()) return response;
    
    int& sockfd = *static_cast<int*>(socketHandle_);
    
    // Set receive timeout
#ifdef _WIN32
    DWORD timeout = timeoutMs_;
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
#else
    struct timeval timeout;
    timeout.tv_sec = timeoutMs_ / 1000;
    timeout.tv_usec = (timeoutMs_ % 1000) * 1000;
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
#endif
    
    // Read packet header first (type + length = 3 bytes minimum)
    uint8_t header[3];
#ifdef _WIN32
    int received = recv(sockfd, (char*)header, 3, 0);
#else
    ssize_t received = recv(sockfd, header, 3, 0);
#endif
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
#ifdef _WIN32
        received = recv(sockfd, (char*)payload.data(), payloadLength + 1, 0);
#else
        received = recv(sockfd, payload.data(), payloadLength + 1, 0);
#endif
        if (received > 0) {
            response.insert(response.end(), payload.begin(), payload.begin() + received);
        }
    } else {
        // Read checksum byte
        uint8_t checksum;
#ifdef _WIN32
        received = recv(sockfd, (char*)&checksum, 1, 0);
#else
        received = recv(sockfd, &checksum, 1, 0);
#endif
        if (received > 0) {
            response.push_back(checksum);
        }
    }
    
    return response;
}

bool EthernetHardwareInterface::moveMotorsToSteps(const std::array<int32_t, 3>& targetSteps) {
    if (!isConnected()) return false;
    
    // Use binary protocol instead of G-code
    using namespace DeltaRobotProtocol;
    Packet packet = Commands::moveAbsolute(targetSteps);
    std::vector<uint8_t> data = PacketEncoder::encode(packet);
    
    // Send packet
    int& sockfd = *static_cast<int*>(socketHandle_);
#ifdef _WIN32
    int sent = send(sockfd, (const char*)data.data(), (int)data.size(), 0);
#else
    ssize_t sent = send(sockfd, data.data(), data.size(), 0);
#endif
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
#ifdef _WIN32
    int sent = send(sockfd, (const char*)data.data(), (int)data.size(), 0);
#else
    ssize_t sent = send(sockfd, data.data(), data.size(), 0);
#endif
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
#ifdef _WIN32
    int sent = send(sockfd, (const char*)data.data(), (int)data.size(), 0);
#else
    ssize_t sent = send(sockfd, data.data(), data.size(), 0);
#endif
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
    if (motorIndex < 0 || motorIndex >= 3 || !isConnected()) return false;
    
    // Use binary protocol instead of G-code
    using namespace DeltaRobotProtocol;
    Packet packet = Commands::setSpeed(motorIndex, speed);
    std::vector<uint8_t> data = PacketEncoder::encode(packet);
    
    int& sockfd = *static_cast<int*>(socketHandle_);
#ifdef _WIN32
    int sent = send(sockfd, (const char*)data.data(), (int)data.size(), 0);
#else
    ssize_t sent = send(sockfd, data.data(), data.size(), 0);
#endif
    if (sent < 0) {
        connectionState_ = ConnectionState::Error;
        if (onError_) onError_("Send speed command failed");
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

bool EthernetHardwareInterface::setMotorAcceleration(int motorIndex, float acceleration) {
    if (motorIndex < 0 || motorIndex >= 3 || !isConnected()) return false;
    
    // Use binary protocol instead of G-code
    using namespace DeltaRobotProtocol;
    Packet packet = Commands::setAcceleration(motorIndex, acceleration);
    std::vector<uint8_t> data = PacketEncoder::encode(packet);
    
    int& sockfd = *static_cast<int*>(socketHandle_);
#ifdef _WIN32
    int sent = send(sockfd, (const char*)data.data(), (int)data.size(), 0);
#else
    ssize_t sent = send(sockfd, data.data(), data.size(), 0);
#endif
    if (sent < 0) {
        connectionState_ = ConnectionState::Error;
        if (onError_) onError_("Send acceleration command failed");
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

bool EthernetHardwareInterface::enableMotors(bool enable) {
    if (!isConnected()) return false;
    
    using namespace DeltaRobotProtocol;
    Packet packet = Commands::enableMotors(enable);
    std::vector<uint8_t> data = PacketEncoder::encode(packet);
    
    int& sockfd = *static_cast<int*>(socketHandle_);
#ifdef _WIN32
    int sent = send(sockfd, (const char*)data.data(), (int)data.size(), 0);
#else
    ssize_t sent = send(sockfd, data.data(), data.size(), 0);
#endif
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
    if (!isConnected()) return false;
    
    // Use binary protocol instead of G-code
    using namespace DeltaRobotProtocol;
    Packet packet = Commands::stop();
    std::vector<uint8_t> data = PacketEncoder::encode(packet);
    
    int& sockfd = *static_cast<int*>(socketHandle_);
#ifdef _WIN32
    int sent = send(sockfd, (const char*)data.data(), (int)data.size(), 0);
#else
    ssize_t sent = send(sockfd, data.data(), data.size(), 0);
#endif
    if (sent < 0) {
        connectionState_ = ConnectionState::Error;
        if (onError_) onError_("Send stop command failed");
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

bool EthernetHardwareInterface::homeMotors() {
    if (!isConnected()) return false;
    
    // Use binary protocol - CMD_HOME command
    // PC only sends the command - ALL homing logic is handled by STM32 firmware
    // STM32 firmware will:
    // 1. Read 3 limit switches connected to STM32 GPIO pins
    // 2. Move motors upward slowly (negative direction) until all 3 switches trigger
    // 3. Stop motors when all switches hit
    // 4. Set all motor positions to 0 (home position)
    // 5. Send RESP_HOMED with positions [0, 0, 0]
    using namespace DeltaRobotProtocol;
    Packet packet = Commands::home();
    std::vector<uint8_t> data = PacketEncoder::encode(packet);
    
    int& sockfd = *static_cast<int*>(socketHandle_);
#ifdef _WIN32
    int sent = send(sockfd, (const char*)data.data(), (int)data.size(), 0);
#else
    ssize_t sent = send(sockfd, data.data(), data.size(), 0);
#endif
    if (sent < 0) {
        connectionState_ = ConnectionState::Error;
        if (onError_) onError_("Send home command failed");
        return false;
    }
    
    // Read response - STM32 will send RESP_OK (homing started)
    // Then later send RESP_HOMED (homing complete) with positions [0, 0, 0]
    // For now, accept RESP_OK as success (homing started)
    // RESP_HOMED will be sent when homing completes
    std::vector<uint8_t> response = readBinaryResponse();
    if (!response.empty()) {
        Packet respPacket;
        if (PacketEncoder::decode(response, respPacket)) {
            // Accept RESP_OK (homing started) or RESP_HOMED (homing complete)
            if (respPacket.type == PacketType::RESP_HOMED) {
                // Parse homed positions (should be [0, 0, 0])
                std::array<int32_t, 3> positions;
                if (Responses::parseHomed(respPacket, positions)) {
                    std::cout << "[Ethernet] Homing complete. Positions: [" 
                              << positions[0] << ", " << positions[1] << ", " << positions[2] << "]" << std::endl;
                }
            }
            return respPacket.type == PacketType::RESP_OK || 
                   respPacket.type == PacketType::RESP_HOMED;
        }
    }
    
    return false;
}

bool EthernetHardwareInterface::setMotorPosition(int motorIndex, int32_t steps) {
    if (motorIndex < 0 || motorIndex >= 3 || !isConnected()) return false;
    
    // Use binary protocol instead of G-code
    using namespace DeltaRobotProtocol;
    Packet packet = Commands::setPosition(motorIndex, steps);
    std::vector<uint8_t> data = PacketEncoder::encode(packet);
    
    int& sockfd = *static_cast<int*>(socketHandle_);
#ifdef _WIN32
    int sent = send(sockfd, (const char*)data.data(), (int)data.size(), 0);
#else
    ssize_t sent = send(sockfd, data.data(), data.size(), 0);
#endif
    if (sent < 0) {
        connectionState_ = ConnectionState::Error;
        if (onError_) onError_("Send set position command failed");
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

bool EthernetHardwareInterface::getMotorPositions(std::array<int32_t, 3>& positions) {
    if (!isConnected()) return false;
    
    using namespace DeltaRobotProtocol;
    Packet packet = Commands::requestStatus();
    std::vector<uint8_t> data = PacketEncoder::encode(packet);
    
    int& sockfd = *static_cast<int*>(socketHandle_);
#ifdef _WIN32
    int sent = send(sockfd, (const char*)data.data(), (int)data.size(), 0);
#else
    ssize_t sent = send(sockfd, data.data(), data.size(), 0);
#endif
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
    if (!isConnected()) return false;
    
    // Use CMD_REQUEST_STATUS to get current motor states
    using namespace DeltaRobotProtocol;
    Packet packet = Commands::requestStatus();
    std::vector<uint8_t> data = PacketEncoder::encode(packet);
    
    int& sockfd = *static_cast<int*>(socketHandle_);
#ifdef _WIN32
    int sent = send(sockfd, (const char*)data.data(), (int)data.size(), 0);
#else
    ssize_t sent = send(sockfd, data.data(), data.size(), 0);
#endif
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
                isMoving = status.moving;
                return true;
            }
        }
    }
    
    // If we can't get status, assume not moving (safe default)
    isMoving = {false, false, false};
    return false;
}

bool EthernetHardwareInterface::setMicrostepping(int motorIndex, int microsteps) {
    if (motorIndex < 0 || motorIndex >= 3 || !isConnected()) return false;
    
    // Use CMD_CONFIG to set microstepping along with other parameters
    // For microstepping-only change, we'll use the current config values
    // This is a limitation - ideally we'd have a separate CMD_SET_MICROSTEPPING
    // For now, use CMD_CONFIG with current speed/accel and new microstepping
    // Note: This requires knowing current speed/accel, which we don't track
    // Better approach: Use CMD_CONFIG with reasonable defaults
    // Or: Add CMD_SET_MICROSTEPPING to protocol (future enhancement)
    
    // For now, use CMD_CONFIG with default values (will be overwritten by setMotorConfig)
    using namespace DeltaRobotProtocol;
    // Default values if not known - these should be set via setMotorConfig instead
    float defaultSpeed = 32000.0f;  // Conservative default (320,000 is max, use 32,000 for safety)
    float defaultAccel = 500.0f;
    Packet packet = Commands::setConfig(motorIndex, defaultSpeed, defaultAccel, static_cast<uint8_t>(microsteps));
    std::vector<uint8_t> data = PacketEncoder::encode(packet);
    
    int& sockfd = *static_cast<int*>(socketHandle_);
#ifdef _WIN32
    int sent = send(sockfd, (const char*)data.data(), (int)data.size(), 0);
#else
    ssize_t sent = send(sockfd, data.data(), data.size(), 0);
#endif
    if (sent < 0) {
        connectionState_ = ConnectionState::Error;
        if (onError_) onError_("Send microstepping config failed");
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

bool EthernetHardwareInterface::setMotorConfig(const MotorConfig& config, int motorIndex) {
    if (motorIndex < 0 || motorIndex >= 3 || !isConnected()) return false;
    
    // Use CMD_CONFIG to set all parameters at once (more efficient than separate commands)
    // This ensures all settings are applied atomically
    using namespace DeltaRobotProtocol;
    
    // Validate and clamp speed to realistic hardware limits
    // 320,000 steps/sec is theoretical max, but real hardware may be slower
    // Limit to reasonable maximum for safety (e.g., 100,000 steps/sec)
    float maxRealisticSpeed = 100000.0f;  // 100k steps/sec is very fast but achievable
    float clampedSpeed = (config.maxStepsPerSecond < maxRealisticSpeed) ? config.maxStepsPerSecond : maxRealisticSpeed;
    
    // Clamp acceleration to reasonable limits (e.g., 10,000 steps/sec²)
    float maxRealisticAccel = 10000.0f;
    float clampedAccel = (config.acceleration < maxRealisticAccel) ? config.acceleration : maxRealisticAccel;
    
    Packet packet = Commands::setConfig(
        motorIndex, 
        clampedSpeed, 
        clampedAccel, 
        static_cast<uint8_t>(config.microstepping)
    );
    std::vector<uint8_t> data = PacketEncoder::encode(packet);
    
    int& sockfd = *static_cast<int*>(socketHandle_);
#ifdef _WIN32
    int sent = send(sockfd, (const char*)data.data(), (int)data.size(), 0);
#else
    ssize_t sent = send(sockfd, data.data(), data.size(), 0);
#endif
    if (sent < 0) {
        connectionState_ = ConnectionState::Error;
        if (onError_) onError_("Send motor config failed");
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

