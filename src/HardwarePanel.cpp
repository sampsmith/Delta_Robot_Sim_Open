#include "UIPanels.hpp"
#include "HardwareInterface.hpp"
#include "ADSInterface.hpp"
#include "MotorControl.hpp"
#include <sstream>
#include <cstring>

HardwarePanel::HardwarePanel(HardwareInterface* hardwareInterface, MotorControl& motorControl)
    : hardwareInterface_(hardwareInterface), motorControl_(motorControl) {
    std::strncpy(ipAddress_, "1.1.1.1.1.1", sizeof(ipAddress_) - 1);
    ipAddress_[sizeof(ipAddress_) - 1] = '\0';
    port_ = 8080;
}

void HardwarePanel::render() {
    ImGui::BeginChild("HardwareContent", ImVec2(0, 0), false, ImGuiWindowFlags_AlwaysVerticalScrollbar);
    
    if (!hardwareInterface_) {
        ImGui::Text("Hardware Interface not available");
        ImGui::EndChild();
        return;
    }
    
    auto state = hardwareInterface_->getConnectionState();
    const char* stateStr = "Unknown";
    switch (state) {
        case HardwareInterface::ConnectionState::Disconnected:
            stateStr = "Disconnected";
            hardwareConnected_ = false;
            break;
        case HardwareInterface::ConnectionState::Connecting:
            stateStr = "Connecting...";
            hardwareConnected_ = false;
            break;
        case HardwareInterface::ConnectionState::Connected:
            stateStr = "Connected";
            hardwareConnected_ = true;
            break;
        case HardwareInterface::ConnectionState::Error:
            stateStr = "Error";
            hardwareConnected_ = false;
            break;
    }
    
    ImGui::Text("Connection Status: %s", stateStr);
    ImGui::Separator();
    
    if (!hardwareConnected_) {
        ImGui::Text("Connect to Beckhoff TwinCAT PLC:");
        ImGui::InputText("AMS Net ID", ipAddress_, sizeof(ipAddress_));
        ImGui::SetTooltip("Format: X.X.X.X.X.X (e.g., 1.1.1.1.1.1)");
        
        // Check if this is an ADS interface
        ADSInterface* adsInterface = dynamic_cast<ADSInterface*>(hardwareInterface_);
        if (adsInterface) {
            if (ImGui::Button("Connect to PLC")) {
                hardwareInterface_->connect(ipAddress_);
            }
            
            ImGui::Separator();
            ImGui::Text("ADS Configuration:");
            float updateRate = adsInterface->getUpdateRate();
            if (ImGui::DragFloat("Update Rate (Hz)", &updateRate, 1.0f, 10.0f, 500.0f, "%.0f")) {
                adsInterface->setUpdateRate(updateRate);
            }
            ImGui::SetTooltip("How often to send joint angles to PLC (higher = smoother but more network traffic)");
            
            char varName[256];
            std::strncpy(varName, adsInterface->getVariableName().c_str(), sizeof(varName) - 1);
            varName[sizeof(varName) - 1] = '\0';
            if (ImGui::InputText("PLC Variable Name", varName, sizeof(varName))) {
                adsInterface->setVariableName(varName);
            }
            ImGui::SetTooltip("PLC variable name for joint angles (e.g., MAIN.fJointAngles)");
            
            ImGui::Separator();
            ImGui::Text("Default AMS Net ID: 1.1.1.1.1.1");
            ImGui::Text("Format: X.X.X.X.X.X (6 numbers, 0-255 each)");
        } else {
            // Fallback for other interfaces
            ImGui::InputInt("Port", &port_);
            std::ostringstream address;
            address << ipAddress_ << ":" << port_;
            
            if (ImGui::Button("Connect")) {
                hardwareInterface_->connect(address.str());
            }
        }
    } else {
        if (ImGui::Button("Disconnect")) {
            hardwareInterface_->disconnect();
            motorsEnabled_ = false;
        }
        
        ImGui::Separator();
        ImGui::Text("Homing System");
        ImGui::Separator();
        
        // Home status
        bool isHomed = motorControl_.isHomed();
        if (isHomed) {
            ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "✓ Robot is Homed");
        } else {
            ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "✗ Robot Not Homed");
        }
        
        if (ImGui::Button("Home Motors (Physical Sensor)")) {
            // Send CMD_HOME command - STM32 firmware handles all homing logic
            // STM32 reads 3 limit switches and executes homing sequence
            // After homing completes, STM32 sends RESP_HOMED with [0, 0, 0]
            if (hardwareInterface_->homeMotors()) {
                motorControl_.resetToSoftwareHome();
            }
        }
        
        ImGui::Separator();
        ImGui::Text("Motor Control:");
        if (ImGui::Button("Enable Motors")) {
            if (hardwareConnected_) {
                motorsEnabled_ = hardwareInterface_->enableMotors(true);
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Disable Motors")) {
            if (hardwareConnected_) {
                hardwareInterface_->enableMotors(false);
                motorsEnabled_ = false;
            }
        }
        
        ImGui::Text("Motors: %s", motorsEnabled_ ? "Enabled" : "Disabled");
        
        ImGui::Separator();
        ImGui::Text("Hardware telemetry will be provided via ADS servo integration.");
    }
    
    ImGui::EndChild();
}

