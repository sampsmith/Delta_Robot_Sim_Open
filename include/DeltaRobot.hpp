#pragma once

#include <glm/glm.hpp>
#include <vector>
#include <array>

struct DeltaRobotConfig {
    // Base Plate (Top Plate - where motors are mounted)
    float basePlateRadius = 0.15f;        // Radius of base plate (m)
    float basePlateHeight = 0.0f;          // Height of base plate above origin (m)
    float basePlateThickness = 0.01f;      // Thickness of base plate (m)
    
    // Effector Plate (Bottom Plate - end effector)
    float effectorPlateRadius = 0.05f;     // Radius of effector plate (m)
    float effectorPlateThickness = 0.005f;  // Thickness of effector plate (m)
    
    // Arm Dimensions
    float upperArmLength = 0.20f;          // Upper arm (bicep) length (m)
    float lowerArmLength = 0.30f;          // Lower arm (forearm) length (m)
    float armThickness = 0.01f;            // Arm thickness/diameter (m)
    
    // Joint Positions
    float baseJointRadius = 0.12f;         // Distance from base center to motor joint (m)
    float effectorJointRadius = 0.04f;     // Distance from effector center to arm joint (m)
    
    // Motor Limits
    float minMotorAngle = -60.0f * 3.14159265358979323846f / 180.0f;  // Minimum motor angle (radians)
    float maxMotorAngle = 60.0f * 3.14159265358979323846f / 180.0f;    // Maximum motor angle (radians)
    
    // Legacy compatibility (mapped to new values)
    float baseRadius = 0.15f;               // Maps to basePlateRadius
    float effectorRadius = 0.05f;           // Maps to effectorPlateRadius
    float baseHeight = 0.0f;                // Maps to basePlateHeight
};

struct ArmJoint {
    glm::vec3 position;
    float angle;  // Joint angle in radians (motor angle for upper joints, elbow angle for lower joints)
    glm::vec3 direction;  // Direction vector of the joint axis/arm
};

struct DeltaRobotState {
    glm::vec3 endEffectorPos;  // Target end effector position
    std::array<ArmJoint, 3> upperJoints;  // Base joints (motor joints)
    std::array<ArmJoint, 3> lowerJoints;  // Elbow joints
    std::array<glm::vec3, 3> effectorJoints;  // End effector joints
    std::array<float, 3> motorAngles;  // Motor angles in radians (0 = horizontal, positive = downward)
    bool isValid = false;  // Whether the position is reachable
};

class DeltaRobot {
public:
    DeltaRobot();
    DeltaRobot(const DeltaRobotConfig& config);
    
    // Set end effector position and calculate inverse kinematics
    bool setEndEffectorPosition(const glm::vec3& position);
    
    // Get current state
    const DeltaRobotState& getState() const { return state_; }
    
    // Get configuration
    const DeltaRobotConfig& getConfig() const { return config_; }
    void setConfig(const DeltaRobotConfig& config);
    
    // Get workspace bounds (approximate)
    float getMaxReach() const;
    float getMinHeight() const;
    float getMaxHeight() const;
    
    // Check if position is reachable
    bool isPositionReachable(const glm::vec3& position) const;
    
    // Clamp position to workspace (returns closest reachable position)
    glm::vec3 clampToWorkspace(const glm::vec3& position) const;
    
    // Get motor angles (in radians)
    const std::array<float, 3>& getMotorAngles() const { return state_.motorAngles; }

private:
    DeltaRobotConfig config_;
    DeltaRobotState state_;
    
    // Calculate inverse kinematics for one arm
    bool calculateArmIK(int armIndex, const glm::vec3& targetPos, ArmJoint& upperJoint, ArmJoint& lowerJoint, glm::vec3& effectorJoint);
    
    // Base joint positions (three arms at 120 degree intervals)
    glm::vec3 getBaseJointPosition(int armIndex) const;
    
    // End effector joint positions relative to centre
    glm::vec3 getEffectorJointOffset(int armIndex) const;
};

