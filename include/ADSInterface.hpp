#pragma once

#include "HardwareInterface.hpp"
#include <array>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>

// Forward declarations for ADS types
// On Windows, TwinCAT headers define these (included in .cpp)
// On other platforms, we define minimal types here
#ifndef _WIN32
    typedef unsigned long AmsPort;
    struct AmsAddr {
        unsigned char netId[6];
        unsigned short port;
    };
#else
    // On Windows, use opaque pointer - actual type defined in TwinCAT headers
    typedef void* AmsAddrPtr;
    typedef unsigned long AmsPort;
    // Type alias for pointer to AmsAddr (opaque on Windows)
    typedef AmsAddrPtr AmsAddrPtrType;
#endif

// ADS Hardware Interface for Beckhoff TwinCAT
// Streams joint angles in real-time to PLC via ADS
class ADSInterface : public HardwareInterface {
public:
    ADSInterface();
    ~ADSInterface();
    
    // Connection management
    bool connect(const std::string& address) override;  // Format: "AMS_NET_ID" or "IP:AMS_NET_ID"
    void disconnect() override;
    ConnectionState getConnectionState() const override { return connectionState_; }
    bool isConnected() const override { return connectionState_ == ConnectionState::Connected; }
    
    // Motor control - send joint angles (radians) to PLC
    bool sendJointAngles(const std::array<float, 3>& angles);
    
    // Legacy step-based methods (not used for servo control, but required by interface)
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
    
    // ADS-specific configuration
    void setUpdateRate(float hz) { updateRateHz_ = hz; }
    float getUpdateRate() const { return updateRateHz_; }
    void setVariableName(const std::string& varName) { 
        if (variableName_ != varName) {
            variableName_ = varName;
            // Invalidate symbol handle so it gets refreshed on next write
            symbolHandleValid_ = false;
            symbolHandle_ = 0;
        }
    }
    std::string getVariableName() const { return variableName_; }
    
private:
    ConnectionState connectionState_;
    AmsPort adsPort_;
#ifdef _WIN32
    AmsAddrPtrType targetAddr_;  // Opaque pointer on Windows
#else
    AmsAddr* targetAddr_;
#endif
    std::string amsNetId_;
    std::string variableName_;  // PLC variable name (e.g., "MAIN.fJointAngles")
    float updateRateHz_;
    
    // Real-time streaming thread
    std::thread streamingThread_;
    std::atomic<bool> streamingActive_;
    std::mutex angleMutex_;
    std::array<float, 3> currentAngles_;
    
    // Symbol handle caching
    unsigned long symbolHandle_;
    bool symbolHandleValid_;
    
    // Internal methods
    bool initializeADS();
    void cleanupADS();
    void streamingLoop();
    bool writeAnglesToPLC(const std::array<float, 3>& angles);
    bool parseAMSAddress(const std::string& address, std::string& amsNetId);
};

