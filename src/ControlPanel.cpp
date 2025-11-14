#include "UIPanels.hpp"
#include "DeltaRobot.hpp"
#include "MotorControl.hpp"
#include "HardwareInterface.hpp"
#include <cmath>
#include <algorithm>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

ControlPanel::ControlPanel(DeltaRobot& robot, MotorControl& motorControl, HardwareInterface* hardwareInterface)
    : robot_(robot)
    , motorControl_(motorControl)
    , hardwareInterface_(hardwareInterface) {
}

void ControlPanel::render() {
    ImGui::BeginChild("ControlContent", ImVec2(0, 0), false, ImGuiWindowFlags_AlwaysVerticalScrollbar);
    
    updateAngleHistory();
    renderHomingControls();
    ImGui::Separator();
    renderLiveJointAngles();
    ImGui::Separator();
    
    // Virtual Controller
    ImGui::Text("Virtual Controller");
    ImGui::Separator();
    
    // X Axis controls
    ImGui::Text("X Axis:");
    float buttonWidth = 100.0f * uiScale;
    float buttonHeight = 50.0f * uiScale;
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.8f, 0.2f, 0.2f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(1.0f, 0.3f, 0.3f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.6f, 0.1f, 0.1f, 1.0f));
    ImGui::Button("X+", ImVec2(buttonWidth, buttonHeight));
    controllerButtonXPos = ImGui::IsItemActive();
    ImGui::PopStyleColor(3);
    
    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.8f, 0.2f, 0.2f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(1.0f, 0.3f, 0.3f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.6f, 0.1f, 0.1f, 1.0f));
    ImGui::Button("X-", ImVec2(buttonWidth, buttonHeight));
    controllerButtonXNeg = ImGui::IsItemActive();
    ImGui::PopStyleColor(3);
    
    ImGui::Spacing();
    
    // Y Axis controls
    ImGui::Text("Y Axis:");
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.8f, 0.2f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 1.0f, 0.3f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.1f, 0.6f, 0.1f, 1.0f));
    ImGui::Button("Y+", ImVec2(buttonWidth, buttonHeight));
    controllerButtonYPos = ImGui::IsItemActive();
    ImGui::PopStyleColor(3);
    
    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.8f, 0.2f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 1.0f, 0.3f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.1f, 0.6f, 0.1f, 1.0f));
    ImGui::Button("Y-", ImVec2(buttonWidth, buttonHeight));
    controllerButtonYNeg = ImGui::IsItemActive();
    ImGui::PopStyleColor(3);
    
    ImGui::Spacing();
    
    // Z Axis controls
    ImGui::Text("Z Axis:");
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.2f, 0.8f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 0.3f, 1.0f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.1f, 0.1f, 0.6f, 1.0f));
    ImGui::Button("Z+", ImVec2(buttonWidth, buttonHeight));
    controllerButtonZPos = ImGui::IsItemActive();
    ImGui::PopStyleColor(3);
    
    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.2f, 0.8f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 0.3f, 1.0f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.1f, 0.1f, 0.6f, 1.0f));
    ImGui::Button("Z-", ImVec2(buttonWidth, buttonHeight));
    controllerButtonZNeg = ImGui::IsItemActive();
    ImGui::PopStyleColor(3);
    
    ImGui::Separator();
    ImGui::DragFloat("Speed", &controllerSpeed, 0.001f, 0.001f, 0.1f, "%.3f m/s");
    
    ImGui::Separator();
    ImGui::Spacing();
    
    // End Effector Position Control
    ImGui::Text("End Effector Position");
    ImGui::Separator();
    
    glm::vec3 pos = robot_.getState().endEffectorPos;
    float posArray[3] = { pos.x, pos.y, pos.z };
    
    ImGui::Text("Position (m):");
    if (ImGui::DragFloat3("##Position", posArray, 0.01f, -1.0f, 1.0f)) {
        glm::vec3 newPos(posArray[0], posArray[1], posArray[2]);
        if (robot_.setEndEffectorPosition(newPos)) {
            motorControl_.setTargetAngles(robot_.getMotorAngles());
        }
    }
    
    ImGui::Separator();
    ImGui::Text("Quick Movements:");
    if (ImGui::Button("Centre")) {
        glm::vec3 centerPos(0.0f, 0.0f, -0.35f);
        robot_.setEndEffectorPosition(centerPos);
        motorControl_.setTargetAngles(robot_.getMotorAngles());
    }
    ImGui::SameLine();
    if (ImGui::Button("Up")) {
        pos.z += 0.05f;
        robot_.setEndEffectorPosition(pos);
        motorControl_.setTargetAngles(robot_.getMotorAngles());
    }
    ImGui::SameLine();
    if (ImGui::Button("Down")) {
        pos.z -= 0.05f;
        robot_.setEndEffectorPosition(pos);
        motorControl_.setTargetAngles(robot_.getMotorAngles());
    }
    
    ImGui::Separator();
    
    bool isValid = robot_.getState().isValid;
    if (isValid) {
        ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "Status: Valid Position");
        ImGui::Separator();
        ImGui::Text("Motor Angles (degrees):");
        const auto& motorAngles = robot_.getMotorAngles();
        for (int i = 0; i < 3; ++i) {
            float angleDeg = motorAngles[i] * 180.0f / M_PI;
            ImGui::Text("Motor %d: %.2f°", i + 1, angleDeg);
        }
    } else {
        ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "Status: Invalid Position");
    }
    
    ImGui::Separator();
    ImGui::Spacing();
    
    // Motor Status
    ImGui::Text("Motor Status");
    ImGui::Separator();
    const auto& motorStates = motorControl_.getMotorStates();
    for (int i = 0; i < 3; ++i) {
        float angleDeg = motorStates[i].currentAngle * 180.0f / M_PI;
        ImGui::Text("Motor %d: %.2f° %s", 
            i + 1, angleDeg, motorStates[i].isMoving ? "[Moving]" : "");
    }
    
    ImGui::EndChild();
}

void ControlPanel::renderHomingControls() {
    ImGui::Text("Robot Homing");
    bool isHomed = motorControl_.isHomed();
    ImVec4 statusColor = isHomed ? ImVec4(0.2f, 0.9f, 0.2f, 1.0f) : ImVec4(0.9f, 0.2f, 0.2f, 1.0f);
    ImGui::SameLine();
    ImGui::TextColored(statusColor, isHomed ? "✓ Homed" : "✗ Not Homed");
    
    if (ImGui::Button("Home Robot")) {
        bool hardwareAttempted = false;
        bool usedHardware = false;
        bool homingSuccess = false;
        
        bool hardwareReady = hardwareInterface_ && hardwareInterface_->isConnected();
        if (hardwareReady) {
            hardwareAttempted = true;
            homingSuccess = hardwareInterface_->homeMotors();
            usedHardware = homingSuccess;
            if (homingSuccess) {
                motorControl_.setHomed(true);
            }
        }
        
        if (!homingSuccess) {
            motorControl_.resetToSoftwareHome();
            homingSuccess = true;
        }
        
        if (homingSuccess) {
            lastHomingResult_ = HomingResult::Success;
            homingStatusColor_ = ImVec4(0.2f, 0.8f, 0.2f, 1.0f);
            if (usedHardware) {
                homingStatusMessage_ = "Homing command sent to hardware controller.";
            } else if (hardwareAttempted) {
                homingStatusMessage_ = "Hardware homing unavailable; software homing reset all angles to 0°.";
            } else {
                homingStatusMessage_ = "Software homing complete. Joint angles reset to 0°.";
            }
        } else {
            lastHomingResult_ = HomingResult::Failure;
            homingStatusMessage_ = "Unable to home robot. Check hardware connection.";
            homingStatusColor_ = ImVec4(0.9f, 0.3f, 0.2f, 1.0f);
        }
    }
    
    if (lastHomingResult_ != HomingResult::None && !homingStatusMessage_.empty()) {
        ImGui::TextColored(homingStatusColor_, "%s", homingStatusMessage_.c_str());
    }
}

void ControlPanel::renderLiveJointAngles() {
    ImGui::Text("Live Joint Angles");
    const auto& motorStates = motorControl_.getMotorStates();
    const auto relativeAngles = motorControl_.getAnglesRelativeToHome();
    
    for (int i = 0; i < 3; ++i) {
        float angleDeg = relativeAngles[i] * 180.0f / M_PI;
        auto& history = angleHistoryDeg_[i];
        
        std::string label = "Motor " + std::to_string(i + 1);
        ImGui::Text("%s", label.c_str());
        
        if (!history.empty()) {
            float minVal = *std::min_element(history.begin(), history.end());
            float maxVal = *std::max_element(history.begin(), history.end());
            if (minVal == maxVal) {
                minVal -= 1.0f;
                maxVal += 1.0f;
            }
            
            std::string plotId = "##MotorPlot" + std::to_string(i);
            ImGui::PlotLines(
                plotId.c_str(),
                history.data(),
                static_cast<int>(history.size()),
                0,
                nullptr,
                minVal,
                maxVal,
                ImVec2(0, 60.0f * uiScale)
            );
        } else {
            ImGui::Text("Collecting data...");
        }
        
        ImGui::SameLine();
        ImGui::Text("Current: %.2f°", angleDeg);
    }
}

void ControlPanel::updateAngleHistory() {
    const auto& motorStates = motorControl_.getMotorStates();
    const auto relativeAngles = motorControl_.getAnglesRelativeToHome();
    for (int i = 0; i < 3; ++i) {
        float angleDeg = relativeAngles[i] * 180.0f / M_PI;
        auto& history = angleHistoryDeg_[i];
        history.push_back(angleDeg);
        if (history.size() > kAngleHistoryCapacity_) {
            history.erase(history.begin());
        }
    }
}

