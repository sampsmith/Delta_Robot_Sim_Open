#include "UIPanels.hpp"
#include "DeltaRobot.hpp"
#include "MotorControl.hpp"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

RobotSetupPanel::RobotSetupPanel(DeltaRobot& robot, MotorControl& motorControl)
    : robot_(robot), motorControl_(motorControl) {
}

int RobotSetupPanel::setupSubTab_ = 0;

void RobotSetupPanel::render() {
    ImGui::BeginChild("SetupContent", ImVec2(0, 0), false, ImGuiWindowFlags_AlwaysVerticalScrollbar);
    
    const char* setupTabNames[] = { "Dimensions", "Motors", "Workspace" };
    if (ImGui::BeginTabBar("SetupSubTabs")) {
        if (ImGui::BeginTabItem(setupTabNames[0])) {
            renderDimensionsTab();
            ImGui::EndTabItem();
        }
        
        if (ImGui::BeginTabItem(setupTabNames[1])) {
            renderMotorsTab();
            ImGui::EndTabItem();
        }
        
        if (ImGui::BeginTabItem(setupTabNames[2])) {
            renderWorkspaceTab();
            ImGui::EndTabItem();
        }
        
        ImGui::EndTabBar();
    }
    
    ImGui::EndChild();
}

void RobotSetupPanel::renderDimensionsTab() {
    DeltaRobotConfig config = robot_.getConfig();
    bool configChanged = false;
    
    ImGui::Text("Robot Geometry Configuration");
    ImGui::Separator();
    
    // Base Plate (Top Plate) Configuration
    ImGui::Text("Base Plate (Top Plate):");
    float oldBasePlateRadius = config.basePlateRadius;
    if (ImGui::DragFloat("Base Plate Radius (m)", &config.basePlateRadius, 0.001f, 0.05f, 0.5f, "%.3f")) {
        config.baseRadius = config.basePlateRadius;
        // Automatically update baseJointRadius to match basePlateRadius
        // This ensures arms move with the plate size
        if (std::abs(config.baseJointRadius - oldBasePlateRadius) < 0.001f || 
            config.baseJointRadius > config.basePlateRadius) {
            // Only auto-sync if baseJointRadius was at the old plate radius or outside plate
            config.baseJointRadius = config.basePlateRadius * 0.8f; // Position joints at 80% of plate radius
        }
        configChanged = true;
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Outer radius of the base plate (top plate) where motors are mounted");
    }
    
    if (ImGui::DragFloat("Base Plate Height (m)", &config.basePlateHeight, 0.001f, -0.5f, 0.5f, "%.3f")) {
        config.baseHeight = config.basePlateHeight;
        configChanged = true;
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Height of base plate above origin (usually 0)");
    }
    
    if (ImGui::DragFloat("Base Plate Thickness (m)", &config.basePlateThickness, 0.001f, 0.005f, 0.05f, "%.3f")) {
        configChanged = true;
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Thickness of the base plate");
    }
    
    ImGui::Separator();
    
    // Effector Plate (Bottom Plate) Configuration
    ImGui::Text("Effector Plate (Bottom Plate):");
    float oldEffectorPlateRadius = config.effectorPlateRadius;
    if (ImGui::DragFloat("Effector Plate Radius (m)", &config.effectorPlateRadius, 0.001f, 0.01f, 0.2f, "%.3f")) {
        config.effectorRadius = config.effectorPlateRadius;
        // Automatically update effectorJointRadius to match effectorPlateRadius
        // This ensures arms move with the effector plate size
        if (std::abs(config.effectorJointRadius - oldEffectorPlateRadius) < 0.001f || 
            config.effectorJointRadius > config.effectorPlateRadius) {
            // Only auto-sync if effectorJointRadius was at the old plate radius or outside plate
            config.effectorJointRadius = config.effectorPlateRadius * 0.8f; // Position joints at 80% of plate radius
        }
        configChanged = true;
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Outer radius of the effector plate (bottom plate) where arms connect");
    }
    
    if (ImGui::DragFloat("Effector Plate Thickness (m)", &config.effectorPlateThickness, 0.001f, 0.002f, 0.02f, "%.3f")) {
        configChanged = true;
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Thickness of the effector plate");
    }
    
    ImGui::Separator();
    
    // Joint Positions
    ImGui::Text("Joint Positions:");
    if (ImGui::DragFloat("Base Joint Radius (m)", &config.baseJointRadius, 0.001f, 0.05f, config.basePlateRadius, "%.3f")) {
        configChanged = true;
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Distance from base center to motor joint center (auto-synced with base plate radius)");
    }
    
    // Show ratio to base plate
    if (config.basePlateRadius > 0.001f) {
        float ratio = config.baseJointRadius / config.basePlateRadius;
        ImGui::Text("  Joint/Plate ratio: %.1f%%", ratio * 100.0f);
    }
    
    if (ImGui::DragFloat("Effector Joint Radius (m)", &config.effectorJointRadius, 0.001f, 0.01f, config.effectorPlateRadius, "%.3f")) {
        configChanged = true;
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Distance from effector center to arm joint center (auto-synced with effector plate radius)");
    }
    
    // Show ratio to effector plate
    if (config.effectorPlateRadius > 0.001f) {
        float ratio = config.effectorJointRadius / config.effectorPlateRadius;
        ImGui::Text("  Joint/Plate ratio: %.1f%%", ratio * 100.0f);
    }
    
    ImGui::Separator();
    
    // Arm Dimensions
    ImGui::Text("Arm Dimensions:");
    if (ImGui::DragFloat("Upper Arm Length (m)", &config.upperArmLength, 0.001f, 0.05f, 1.0f, "%.3f")) {
        configChanged = true;
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Length from motor joint to elbow joint (bicep)");
    }
    
    if (ImGui::DragFloat("Lower Arm Length (m)", &config.lowerArmLength, 0.001f, 0.05f, 1.0f, "%.3f")) {
        configChanged = true;
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Length from elbow joint to end effector joint (forearm)");
    }
    
    if (ImGui::DragFloat("Arm Thickness (m)", &config.armThickness, 0.001f, 0.005f, 0.05f, "%.3f")) {
        configChanged = true;
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Thickness/diameter of the arm rods");
    }
    
    ImGui::Separator();
    
    // Motor Limits
    ImGui::Text("Motor Limits:");
    float minAngleDeg = config.minMotorAngle * 180.0f / M_PI;
    float maxAngleDeg = config.maxMotorAngle * 180.0f / M_PI;
    if (ImGui::DragFloat("Min Motor Angle (deg)", &minAngleDeg, 0.1f, -90.0f, 0.0f, "%.1f")) {
        config.minMotorAngle = minAngleDeg * M_PI / 180.0f;
        configChanged = true;
    }
    if (ImGui::DragFloat("Max Motor Angle (deg)", &maxAngleDeg, 0.1f, 0.0f, 90.0f, "%.1f")) {
        config.maxMotorAngle = maxAngleDeg * M_PI / 180.0f;
        configChanged = true;
    }
    
    if (configChanged) {
        robot_.setConfig(config);
    }
}

void RobotSetupPanel::renderMotorsTab() {
    MotorConfig motorConfig = motorControl_.getMotorConfig(0);
    bool motorChanged = false;
    
    ImGui::Text("Servo Configuration");
    ImGui::Separator();
    
    ImGui::Text("Global Limits:");
    float maxVelDeg = motorConfig.maxAngularVelocity * 180.0f / M_PI;
    if (ImGui::DragFloat("Max Angular Velocity (deg/s)", &maxVelDeg, 1.0f, 10.0f, 720.0f, "%.1f")) {
        motorConfig.maxAngularVelocity = maxVelDeg * M_PI / 180.0f;
        motorChanged = true;
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Limit joint slew rate when commanding servo angles.");
    }
    
    float maxAccelDeg = motorConfig.maxAngularAcceleration * 180.0f / M_PI;
    if (ImGui::DragFloat("Max Angular Accel (deg/s²)", &maxAccelDeg, 5.0f, 10.0f, 5000.0f, "%.0f")) {
        motorConfig.maxAngularAcceleration = maxAccelDeg * M_PI / 180.0f;
        motorChanged = true;
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Caps how aggressively the servo targets can change.");
    }
    
    ImGui::Separator();
    ImGui::TextWrapped("These values apply to all three joints. Use the section below for individual tweaks such as inversion or home offsets.");
    
    if (motorChanged) {
        for (int i = 0; i < 3; ++i) {
            motorControl_.setMotorConfig(i, motorConfig);
        }
    }
    
    ImGui::Separator();
    if (ImGui::TreeNode("Individual Motor Adjustments")) {
        for (int i = 0; i < 3; ++i) {
            ImGui::PushID(i);
            MotorConfig individualConfig = motorControl_.getMotorConfig(i);
            bool individualChanged = false;
            
            ImGui::Text("Motor %d:", i + 1);
            if (ImGui::Checkbox("Inverted", &individualConfig.inverted)) {
                individualChanged = true;
            }
            
            float homeOffset = individualConfig.homeOffset * 180.0f / M_PI;
            if (ImGui::DragFloat("Home Offset (deg)", &homeOffset, 0.1f, -180.0f, 180.0f, "%.2f")) {
                individualConfig.homeOffset = homeOffset * M_PI / 180.0f;
                individualChanged = true;
            }
            
            if (individualChanged) {
                motorControl_.setMotorConfig(i, individualConfig);
            }
            ImGui::PopID();
        }
        ImGui::TreePop();
    }
}

void RobotSetupPanel::renderWorkspaceTab() {
    ImGui::Text("Workspace Information");
    ImGui::Separator();
    
    float maxReach = robot_.getMaxReach();
    float minHeight = robot_.getMinHeight();
    float maxHeight = robot_.getMaxHeight();
    
    ImGui::Text("Workspace Bounds:");
    ImGui::BulletText("Maximum Reach: %.3f m (%.1f cm)", maxReach, maxReach * 100.0f);
    ImGui::BulletText("Height Range: %.3f m to %.3f m", minHeight, maxHeight);
    ImGui::BulletText("Height Span: %.3f m (%.1f cm)", maxHeight - minHeight, (maxHeight - minHeight) * 100.0f);
    
    ImGui::Separator();
    float workspaceVolume = (4.0f / 3.0f) * M_PI * maxReach * maxReach * (maxHeight - minHeight);
    ImGui::Text("Workspace Volume:");
    ImGui::BulletText("Volume: %.3f m³", workspaceVolume);
    ImGui::BulletText("Volume: %.1f liters", workspaceVolume * 1000.0f);
    
    ImGui::Separator();
    DeltaRobotConfig config = robot_.getConfig();
    ImGui::Text("Current Configuration:");
    ImGui::BulletText("Base Radius: %.3f m", config.baseRadius);
    ImGui::BulletText("Effector Radius: %.3f m", config.effectorRadius);
    ImGui::BulletText("Upper Arm: %.3f m", config.upperArmLength);
    ImGui::BulletText("Lower Arm: %.3f m", config.lowerArmLength);
    ImGui::BulletText("Motor Angle Range: %.1f° to %.1f°", 
        config.minMotorAngle * 180.0f / M_PI,
        config.maxMotorAngle * 180.0f / M_PI);
    
}

