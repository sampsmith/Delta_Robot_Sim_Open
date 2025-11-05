#include "UIPanels.hpp"
#include "HardwareInterface.hpp"
#include "MotorControl.hpp"
#include <sstream>
#include <cstring>

HardwarePanel::HardwarePanel(HardwareInterface* hardwareInterface, MotorControl& motorControl)
    : hardwareInterface_(hardwareInterface), motorControl_(motorControl) {
    std::strncpy(ipAddress_, "192.168.1.100", sizeof(ipAddress_) - 1);
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
        ImGui::Text("Connect to NUCLEO-H7S3L8:");
        ImGui::InputText("IP Address", ipAddress_, sizeof(ipAddress_));
        ImGui::InputInt("Port", &port_);
        
        std::ostringstream address;
        address << ipAddress_ << ":" << port_;
        
        if (ImGui::Button("Connect")) {
            hardwareInterface_->connect(address.str());
        }
        
        ImGui::Separator();
        ImGui::Text("Default: 192.168.1.100:8080");
        ImGui::Text("Format: IP:PORT or just IP");
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
            hardwareInterface_->homeMotors();
            
            std::array<int32_t, 3> physicalHome;
            if (hardwareInterface_->getMotorPositions(physicalHome)) {
                for (int i = 0; i < 3; ++i) {
                    motorControl_.setPhysicalHomePosition(i, physicalHome[i]);
                }
            }
        }
        
        ImGui::Separator();
        ImGui::Text("Home Configuration:");
        
        const auto& homePos = motorControl_.getHomePosition();
        for (int i = 0; i < 3; ++i) {
            ImGui::PushID(i);
            ImGui::Text("Motor %d:", i + 1);
            
            int32_t physicalHome = homePos.physicalHomeSteps[i];
            if (ImGui::InputInt("Physical Home (steps)", &physicalHome)) {
                motorControl_.setPhysicalHomePosition(i, physicalHome);
            }
            ImGui::SameLine();
            ImGui::TextDisabled("(?)");
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Step position when upper limit sensor is triggered");
            }
            
            int32_t homeOffset = homePos.homeOffsetSteps[i];
            if (ImGui::InputInt("Home Offset (steps)", &homeOffset)) {
                motorControl_.setHomeOffsetSteps(i, homeOffset);
            }
            ImGui::SameLine();
            ImGui::TextDisabled("(?)");
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Offset from physical sensor to official home position");
            }
            
            ImGui::Text("Official Home: %d steps", homePos.officialHomeSteps[i]);
            
            ImGui::PopID();
        }
        
        if (ImGui::Button("Set Current as Official Home")) {
            const auto& motorStates = motorControl_.getMotorStates();
            for (int i = 0; i < 3; ++i) {
                int32_t currentSteps = motorStates[i].currentSteps;
                motorControl_.setOfficialHomePosition(i, currentSteps);
            }
            motorControl_.setHomed(true);
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
        std::array<int32_t, 3> hwPositions;
        if (hardwareInterface_->getMotorPositions(hwPositions)) {
            ImGui::Text("Hardware Positions:");
            ImGui::Text("X: %d  Y: %d  Z: %d", hwPositions[0], hwPositions[1], hwPositions[2]);
        }
    }
    
    ImGui::EndChild();
}

