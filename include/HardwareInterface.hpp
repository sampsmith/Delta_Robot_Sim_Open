#pragma once

#include <array>
#include <string>
#include <vector>
#include <functional>
#include <memory>
#include "MotorControl.hpp"

// Abstract hardware interface for communicating with motor controllers
// Can be implemented for serial/USB, Arduino, ESP32, etc.

class HardwareInterface {
public:
    enum class ConnectionState {
        Disconnected,
        Connecting,
        Connected,
        Error
    };
    
    virtual ~HardwareInterface() = default;
    
    // Connection management
    virtual bool connect(const std::string& port) = 0;
    virtual void disconnect() = 0;
    virtual ConnectionState getConnectionState() const = 0;
    virtual bool isConnected() const = 0;
    
    // Motor control commands
    virtual bool moveMotorsToSteps(const std::array<int32_t, 3>& targetSteps) = 0;
    virtual bool moveMotorsRelative(const std::array<int32_t, 3>& stepDeltas) = 0;
    
    // Send waypoint sequence (all waypoints at once)
    virtual bool sendWaypointSequence(const std::vector<std::pair<std::array<int32_t, 3>, uint16_t>>& waypoints) = 0;
    virtual bool setMotorSpeed(int motorIndex, float speed) = 0;
    virtual bool setMotorAcceleration(int motorIndex, float acceleration) = 0;
    virtual bool enableMotors(bool enable) = 0;
    virtual bool stopMotors() = 0;
    
    // Homing and calibration
    virtual bool homeMotors() = 0;
    virtual bool setMotorPosition(int motorIndex, int32_t steps) = 0;
    
    // Status queries
    virtual bool getMotorPositions(std::array<int32_t, 3>& positions) = 0;
    virtual bool getMotorStates(std::array<bool, 3>& isMoving) = 0;
    
    // Configuration
    virtual bool setMicrostepping(int motorIndex, int microsteps) = 0;
    virtual bool setMotorConfig(const MotorConfig& config, int motorIndex) = 0;
    
    // Callbacks
    void setOnConnectedCallback(std::function<void()> callback) { onConnected_ = callback; }
    void setOnDisconnectedCallback(std::function<void()> callback) { onDisconnected_ = callback; }
    void setOnErrorCallback(std::function<void(const std::string&)> callback) { onError_ = callback; }
    
protected:
    std::function<void()> onConnected_;
    std::function<void()> onDisconnected_;
    std::function<void(const std::string&)> onError_;
};

// Serial/USB hardware interface removed - was using G-code
// If needed in the future, implement using binary protocol instead

// Ethernet hardware interface for NUCLEO-H7S3L8 over TCP/IP
class EthernetHardwareInterface : public HardwareInterface {
public:
    EthernetHardwareInterface();
    ~EthernetHardwareInterface();
    
    // Connect to NUCLEO-H7S3L8 via IP address and port
    bool connect(const std::string& address) override;  // Format: "IP:PORT" or just "IP" (default port 8080)
    void disconnect() override;
    ConnectionState getConnectionState() const override { return connectionState_; }
    bool isConnected() const override { return connectionState_ == ConnectionState::Connected; }
    
    bool moveMotorsToSteps(const std::array<int32_t, 3>& targetSteps) override;
    bool moveMotorsRelative(const std::array<int32_t, 3>& stepDeltas) override;
    bool sendWaypointSequence(const std::vector<std::pair<std::array<int32_t, 3>, uint16_t>>& waypoints) override;
    bool setMotorSpeed(int motorIndex, float speed) override;
    bool setMotorAcceleration(int motorIndex, float acceleration) override;
    bool enableMotors(bool enable) override;
    bool stopMotors() override;
    
    bool homeMotors() override;
    bool setMotorPosition(int motorIndex, int32_t steps) override;
    
    bool getMotorPositions(std::array<int32_t, 3>& positions) override;
    bool getMotorStates(std::array<bool, 3>& isMoving) override;
    
    bool setMicrostepping(int motorIndex, int microsteps) override;
    bool setMotorConfig(const MotorConfig& config, int motorIndex) override;
    
    // Connection settings
    void setTimeout(int milliseconds) { timeoutMs_ = milliseconds; }
    int getTimeout() const { return timeoutMs_; }
    
private:
    ConnectionState connectionState_;
    void* socketHandle_;  // Opaque pointer to socket implementation
    std::string serverIP_;
    int serverPort_;
    int timeoutMs_;
    
    std::vector<uint8_t> readBinaryResponse();  // Binary packet response
    bool parseIPAddress(const std::string& address, std::string& ip, int& port);
};

// Simulated hardware interface (for testing without hardware)
class SimulatedHardwareInterface : public HardwareInterface {
public:
    SimulatedHardwareInterface();
    
    bool connect(const std::string& port) override;
    void disconnect() override;
    ConnectionState getConnectionState() const override { return connectionState_; }
    bool isConnected() const override { return connectionState_ == ConnectionState::Connected; }
    
    bool moveMotorsToSteps(const std::array<int32_t, 3>& targetSteps) override;
    bool moveMotorsRelative(const std::array<int32_t, 3>& stepDeltas) override;
    bool sendWaypointSequence(const std::vector<std::pair<std::array<int32_t, 3>, uint16_t>>& waypoints) override;
    bool setMotorSpeed(int motorIndex, float speed) override;
    bool setMotorAcceleration(int motorIndex, float acceleration) override;
    bool enableMotors(bool enable) override;
    bool stopMotors() override;
    
    bool homeMotors() override;
    bool setMotorPosition(int motorIndex, int32_t steps) override;
    
    bool getMotorPositions(std::array<int32_t, 3>& positions) override;
    bool getMotorStates(std::array<bool, 3>& isMoving) override;
    
    bool setMicrostepping(int motorIndex, int microsteps) override;
    bool setMotorConfig(const MotorConfig& config, int motorIndex) override;
    
private:
    ConnectionState connectionState_;
    std::array<int32_t, 3> simulatedPositions_;
    std::array<bool, 3> simulatedMoving_;
    bool motorsEnabled_;
};

