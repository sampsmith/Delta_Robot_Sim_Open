#include "UIPanels.hpp"
#include "DeltaRobot.hpp"
#include "MotorControl.hpp"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

ControlPanel::ControlPanel(DeltaRobot& robot, MotorControl& motorControl)
    : robot_(robot), motorControl_(motorControl) {
}

void ControlPanel::render() {
    ImGui::BeginChild("ControlContent", ImVec2(0, 0), false, ImGuiWindowFlags_AlwaysVerticalScrollbar);
    
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
        int32_t steps = motorStates[i].currentSteps;
        ImGui::Text("Motor %d: %.2f° (%d steps) %s", 
            i + 1, angleDeg, steps, motorStates[i].isMoving ? "[Moving]" : "");
    }
    
    ImGui::Separator();
    auto requiredSteps = motorControl_.getRequiredSteps();
    ImGui::Text("Required Steps:");
    ImGui::Text("X: %d  Y: %d  Z: %d", requiredSteps[0], requiredSteps[1], requiredSteps[2]);
    
    ImGui::EndChild();
}

