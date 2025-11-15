#include "ADSInterface.hpp"
#include <iostream>
#include <sstream>
#include <chrono>
#include <cstring>
#include <iomanip>

// TwinCAT ADS API includes
// These headers are typically installed with TwinCAT at:
// C:\TwinCAT\AdsApi\TcAdsDll\Include\
// Include Windows.h first for BOOL and other Windows types (Windows only)
#include <windows.h>

// Include TwinCAT ADS headers
// CMakeLists.txt sets up include path to TwinCAT installation
// If headers not found, user must install TwinCAT or set TWINCAT_ADS_LIB_PATH
#include <TcAdsDef.h>
#include <TcAdsAPI.h>

// Link against ADS library (handled in CMakeLists.txt, but pragma for MSVC convenience)
#pragma comment(lib, "TcAdsDll.lib")

ADSInterface::ADSInterface()
    : connectionState_(ConnectionState::Disconnected)
    , adsPort_(0)
    , targetAddr_(nullptr)
    , amsNetId_("")
    , variableName_("MAIN.fJointAngles")  // Default PLC variable name
    , updateRateHz_(100.0f)  // 100 Hz default update rate
    , streamingActive_(false)
    , symbolHandle_(0)
    , symbolHandleValid_(false)
{
    currentAngles_ = {0.0f, 0.0f, 0.0f};
}

ADSInterface::~ADSInterface() {
    disconnect();
}

bool ADSInterface::connect(const std::string& address) {
    if (connectionState_ == ConnectionState::Connected) {
        disconnect();
    }
    
    connectionState_ = ConnectionState::Connecting;
    
    // Parse AMS address
    if (!parseAMSAddress(address, amsNetId_)) {
        connectionState_ = ConnectionState::Error;
        if (onError_) onError_("Invalid AMS address format");
        return false;
    }
    
    // Initialize ADS
    if (!initializeADS()) {
        connectionState_ = ConnectionState::Error;
        if (onError_) onError_("Failed to initialize ADS connection");
        return false;
    }
    
    // Test connection by reading device info
    #ifdef _WIN32
        AmsAddr* addr = static_cast<AmsAddr*>(targetAddr_);
        if (addr) {
            char devName[256] = {0};
            AdsVersion version;
            long result = AdsSyncReadDeviceInfoReq(addr, devName, &version);
            if (result == ADSERR_NOERR) {
                std::cout << "[ADS] Successfully connected to PLC!" << std::endl;
                std::cout << "[ADS] Device Name: " << devName << std::endl;
                std::cout << "[ADS] TwinCAT Version: " << version.version << "." 
                          << version.revision << "." << version.build << std::endl;
            } else {
                std::cerr << "[ADS] Warning: Could not read device info (error: 0x" 
                          << std::hex << result << std::dec << ")" << std::endl;
                std::cerr << "[ADS] Connection may still work, but PLC may not be running" << std::endl;
            }
        }
    #endif
    
    connectionState_ = ConnectionState::Connected;
    
    // Start streaming thread
    streamingActive_ = true;
    streamingThread_ = std::thread(&ADSInterface::streamingLoop, this);
    
    if (onConnected_) onConnected_();
    
    std::cout << "[ADS] Connected to " << amsNetId_ << std::endl;
    std::cout << "[ADS] Streaming thread started at " << updateRateHz_ << " Hz" << std::endl;
    return true;
}

void ADSInterface::disconnect() {
    if (connectionState_ == ConnectionState::Disconnected) {
        return;
    }
    
    // Stop streaming
    streamingActive_ = false;
    if (streamingThread_.joinable()) {
        streamingThread_.join();
    }
    
    // Reset symbol handle
    symbolHandleValid_ = false;
    symbolHandle_ = 0;
    
    // Cleanup ADS
    cleanupADS();
    
    connectionState_ = ConnectionState::Disconnected;
    if (onDisconnected_) onDisconnected_();
    
    std::cout << "[ADS] Disconnected" << std::endl;
}

bool ADSInterface::sendJointAngles(const std::array<float, 3>& angles) {
    if (!isConnected()) {
        return false;
    }
    
    // Update current angles (thread-safe)
    {
        std::lock_guard<std::mutex> lock(angleMutex_);
        currentAngles_ = angles;
    }
    
    return true;
}

bool ADSInterface::initializeADS() {
#ifdef _WIN32
    // Open ADS port
    adsPort_ = AdsPortOpen();
    if (adsPort_ == 0) {
        std::cerr << "[ADS] Failed to open ADS port. Is TwinCAT installed?" << std::endl;
        return false;
    }
    
    // Allocate target address
    #ifdef _WIN32
        AmsAddr* addr = new AmsAddr();
        targetAddr_ = addr;
    #else
        targetAddr_ = new AmsAddr();
        AmsAddr* addr = targetAddr_;
    #endif
    if (!targetAddr_) {
        AdsPortClose();
        adsPort_ = 0;
        return false;
    }
    
    // Parse AMS Net ID (format: "1.2.3.4.5.6")
    std::istringstream iss(amsNetId_);
    std::string token;
    int i = 0;
    while (std::getline(iss, token, '.') && i < 6) {
        try {
            addr->netId.b[i] = static_cast<unsigned char>(std::stoi(token));
            i++;
        } catch (...) {
            std::cerr << "[ADS] Invalid AMS Net ID format" << std::endl;
            delete addr;
            targetAddr_ = nullptr;
            AdsPortClose();
            adsPort_ = 0;
            return false;
        }
    }
    
    if (i != 6) {
        std::cerr << "[ADS] AMS Net ID must have 6 components (got " << i << ")" << std::endl;
        delete addr;
        targetAddr_ = nullptr;
        AdsPortClose();
        adsPort_ = 0;
        return false;
    }
    
    // Set port (typically 851 for TwinCAT runtime, 801 for system service)
    addr->port = 851;
    
    // Test connection by reading device info
    AmsAddr localAddr;
    memset(&localAddr, 0, sizeof(localAddr));
    localAddr.port = adsPort_;
    
    long result = AdsGetLocalAddress(&localAddr);
    if (result == ADSERR_NOERR) {
        std::cout << "[ADS] Local AMS Net ID: ";
        for (int j = 0; j < 6; j++) {
            std::cout << static_cast<int>(localAddr.netId.b[j]);
            if (j < 5) std::cout << ".";
        }
        std::cout << std::endl;
    }
    
    std::cout << "[ADS] Initialized port " << adsPort_ << std::endl;
    std::cout << "[ADS] Target AMS Net ID: " << amsNetId_ << ":" << addr->port << std::endl;
    std::cout << "[ADS] PLC Variable: " << variableName_ << std::endl;
    
    return true;
#else
    // Linux/cross-platform: would use ads-lib or similar
    std::cerr << "[ADS] ADS not available on this platform" << std::endl;
    return false;
#endif
}

void ADSInterface::cleanupADS() {
#ifdef _WIN32
    if (adsPort_ != 0) {
        AdsPortClose();
        adsPort_ = 0;
    }
    
    if (targetAddr_) {
        #ifdef _WIN32
            delete static_cast<AmsAddr*>(targetAddr_);
        #else
            delete targetAddr_;
        #endif
        targetAddr_ = nullptr;
    }
#endif
}

void ADSInterface::streamingLoop() {
    const auto period = std::chrono::microseconds(static_cast<long>(1000000.0f / updateRateHz_));
    int writeCount = 0;
    int errorCount = 0;
    auto lastStatsTime = std::chrono::high_resolution_clock::now();
    
    std::cout << "[ADS] Streaming loop started" << std::endl;
    
    while (streamingActive_) {
        auto start = std::chrono::high_resolution_clock::now();
        
        // Get current angles (thread-safe)
        std::array<float, 3> angles;
        {
            std::lock_guard<std::mutex> lock(angleMutex_);
            angles = currentAngles_;
        }
        
        // Write to PLC
        if (writeAnglesToPLC(angles)) {
            writeCount++;
        } else {
            errorCount++;
        }
        
        // Print statistics every 5 seconds
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - lastStatsTime).count();
        if (elapsed >= 5) {
            float successRate = (writeCount > 0) ? (100.0f * (writeCount - errorCount) / writeCount) : 0.0f;
            std::cout << "[ADS] Stats: " << writeCount << " writes, " << errorCount 
                      << " errors, " << std::fixed << std::setprecision(1) << successRate 
                      << "% success rate" << std::endl;
            lastStatsTime = now;
        }
        
        // Sleep to maintain update rate
        auto elapsedTime = std::chrono::high_resolution_clock::now() - start;
        auto sleepTime = period - elapsedTime;
        if (sleepTime.count() > 0) {
            std::this_thread::sleep_for(sleepTime);
        } else {
            // Warn if we're falling behind
            static int lagWarnings = 0;
            if (lagWarnings < 5) {
                std::cerr << "[ADS] Warning: Streaming loop falling behind schedule" << std::endl;
                lagWarnings++;
            }
        }
    }
    
    std::cout << "[ADS] Streaming loop stopped" << std::endl;
}

bool ADSInterface::writeAnglesToPLC(const std::array<float, 3>& angles) {
#ifdef _WIN32
    if (!targetAddr_ || adsPort_ == 0) {
        return false;
    }
    
    #ifdef _WIN32
        AmsAddr* addr = static_cast<AmsAddr*>(targetAddr_);
    #else
        AmsAddr* addr = targetAddr_;
    #endif
    
    // Method 1: Symbol access (recommended for TwinCAT)
    // First, get the symbol handle if we don't have it cached
    if (!symbolHandleValid_ && !variableName_.empty()) {
        // Get symbol handle using AdsSyncReadWriteReq
        // IndexGroup 0xF020 = symbol access by name
        // IndexOffset = length of symbol name (including null terminator)
        // Write: symbol name (null-terminated), Read: symbol handle (unsigned long)
        
        unsigned long indexGroup = 0xF020;
        unsigned long indexOffset = static_cast<unsigned long>(variableName_.length() + 1);
        unsigned long readLength = sizeof(unsigned long);
        unsigned long writeLength = static_cast<unsigned long>(variableName_.length() + 1);
        
        unsigned long handle = 0;
        long result = AdsSyncReadWriteReq(addr, indexGroup, indexOffset, 
                                         readLength, &handle,
                                         writeLength, const_cast<char*>(variableName_.c_str()));
        
        if (result == ADSERR_NOERR) {
            symbolHandle_ = handle;
            symbolHandleValid_ = true;
            std::cout << "[ADS] Got symbol handle: " << symbolHandle_ << " for variable: " << variableName_ << std::endl;
        } else {
            // Try direct index group/offset method as fallback
            // For testing, you can use index group 0x4020 (data area) with manual offset
            std::cerr << "[ADS] Failed to get symbol handle (error: 0x" << std::hex << result 
                      << std::dec << "). Variable '" << variableName_ 
                      << "' may not exist in PLC. Trying direct write..." << std::endl;
            symbolHandleValid_ = false;
        }
    }
    
    // Write using symbol handle (if available)
    if (symbolHandleValid_) {
        unsigned long indexGroup = 0xF020;
        unsigned long indexOffset = symbolHandle_;
        unsigned long length = sizeof(float) * 3;
        
        long result = AdsSyncWriteReq(addr, indexGroup, indexOffset, length, 
                                      const_cast<float*>(angles.data()));
        
        if (result != ADSERR_NOERR) {
            static int errorCount = 0;
            errorCount++;
            if (errorCount % 100 == 0) {  // Log every 100th error to avoid spam
                std::cerr << "[ADS] Write error: 0x" << std::hex << result 
                          << std::dec << " (count: " << errorCount << ")" << std::endl;
            }
            return false;
        }
        
        // Success - reset error counter
        static int successCount = 0;
        successCount++;
        if (successCount == 1 || successCount % 1000 == 0) {
            std::cout << "[ADS] Successfully wrote angles: [" 
                      << angles[0] << ", " << angles[1] << ", " << angles[2] 
                      << "] (count: " << successCount << ")" << std::endl;
        }
        
        return true;
    } else {
        // Fallback: Direct write to index group/offset
        // This requires knowing the exact memory location in the PLC
        // For testing, you can manually set these values
        // IndexGroup 0x4020 = data area, IndexOffset = variable offset in PLC
        unsigned long indexGroup = 0x4020;  // Data area
        unsigned long indexOffset = 0;       // You need to set this based on your PLC layout
        unsigned long length = sizeof(float) * 3;
        
        long result = AdsSyncWriteReq(addr, indexGroup, indexOffset, length, 
                                      const_cast<float*>(angles.data()));
        
        if (result != ADSERR_NOERR) {
            static int errorCount = 0;
            errorCount++;
            if (errorCount % 100 == 0) {
                std::cerr << "[ADS] Direct write error: 0x" << std::hex << result 
                          << std::dec << ". Check PLC variable configuration." << std::endl;
            }
            return false;
        }
        
        return true;
    }
#else
    // Linux/cross-platform implementation
    return false;
#endif
}

bool ADSInterface::parseAMSAddress(const std::string& address, std::string& amsNetId) {
    // Parse address format: "AMS_NET_ID" or "IP:AMS_NET_ID"
    // For now, assume direct AMS Net ID format: "1.2.3.4.5.6"
    
    size_t colonPos = address.find(':');
    if (colonPos != std::string::npos) {
        // Format: "IP:AMS_NET_ID" - extract AMS Net ID part
        amsNetId = address.substr(colonPos + 1);
    } else {
        // Assume it's just the AMS Net ID
        amsNetId = address;
    }
    
    // Validate format (should be "X.X.X.X.X.X" where X is 0-255)
    std::istringstream iss(amsNetId);
    std::string token;
    int count = 0;
    while (std::getline(iss, token, '.')) {
        try {
            int val = std::stoi(token);
            if (val < 0 || val > 255) {
                return false;
            }
            count++;
        } catch (...) {
            return false;
        }
    }
    
    return count == 6;
}

// Legacy step-based methods (not used for servo control)
bool ADSInterface::moveMotorsToSteps(const std::array<int32_t, 3>& targetSteps) {
    // Not applicable for servo control - angles are sent directly
    return false;
}

bool ADSInterface::moveMotorsRelative(const std::array<int32_t, 3>& stepDeltas) {
    return false;
}

bool ADSInterface::sendWaypointSequence(const std::vector<std::pair<std::array<int32_t, 3>, uint16_t>>& waypoints) {
    return false;
}

bool ADSInterface::setMotorSpeed(int motorIndex, float speed) {
    // Speed control handled by PLC
    return true;
}

bool ADSInterface::setMotorAcceleration(int motorIndex, float acceleration) {
    // Acceleration control handled by PLC
    return true;
}

bool ADSInterface::enableMotors(bool enable) {
    // Motor enable handled by PLC
    return true;
}

bool ADSInterface::stopMotors() {
    // Stop handled by PLC
    return true;
}

bool ADSInterface::homeMotors() {
    // Homing handled by PLC
    return true;
}

bool ADSInterface::setMotorPosition(int motorIndex, int32_t steps) {
    return false;
}

bool ADSInterface::getMotorPositions(std::array<int32_t, 3>& positions) {
    // Position feedback not implemented - PLC handles this
    return false;
}

bool ADSInterface::getMotorStates(std::array<bool, 3>& isMoving) {
    return false;
}

bool ADSInterface::setMicrostepping(int motorIndex, int microsteps) {
    return false;
}

bool ADSInterface::setMotorConfig(const MotorConfig& config, int motorIndex) {
    return false;
}

