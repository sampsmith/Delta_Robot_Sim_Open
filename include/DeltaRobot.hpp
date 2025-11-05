#pragma once

#include <glm/glm.hpp>
#include <vector>
#include <array>

struct DeltaRobotConfig {
    float baseRadius = 0.15f;      // Base platform radius
    float effectorRadius = 0.05f;  // End effector platform radius
    float upperArmLength = 0.20f;  // Upper arm (bicep) length
    float lowerArmLength = 0.30f;  // Lower arm (forearm) length
    float baseHeight = 0.0f;       // Height of base platform
    float jointAngle = 0.0f;       // Angle of joints on base (for future use)
    float minMotorAngle = -60.0f * 3.14159265358979323846f / 180.0f;  // Minimum motor angle (degrees to radians)
    float maxMotorAngle = 60.0f * 3.14159265358979323846f / 180.0f;    // Maximum motor angle (degrees to radians)
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

