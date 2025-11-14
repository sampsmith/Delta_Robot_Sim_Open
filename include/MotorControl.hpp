#pragma once

#include <array>
#include <vector>
#include <string>
#include <glm/glm.hpp>
#include "DeltaRobot.hpp"

// Motor configuration for servo-driven joints (angle-based control)
struct MotorConfig {
    float maxAngularVelocity = 5.0f;      // rad/s
    float maxAngularAcceleration = 20.0f; // rad/s²
    float homeOffset = 0.0f;              // Offset from home position (radians)
    bool inverted = false;                // Motor direction inversion
};

// Motor state
struct MotorState {
    float currentAngle = 0.0f;           // Current motor angle (radians)
    bool isMoving = false;               // Whether motor is currently moving
    bool isHomed = false;                // Whether motor has been homed
};

// Trajectory point for smooth motion
struct TrajectoryPoint {
    std::array<float, 3> motorAngles;    // Target motor angles (radians)
    float time;                          // Time at this point (seconds)
};

// Forward declaration
class MotorControl;

// Waypoint/keypoint for saved positions
struct Waypoint {
    std::string name;
    glm::vec3 endEffectorPos;           // End-effector position (for display/kinematics)
    std::array<float, 3> motorAngles;   // Motor angles in radians (for kinematics)
    float duration;                      // Time to reach this waypoint (seconds)
};

class MotorControl {
public:
    MotorControl();
    
    // Configuration
    void setMotorConfig(int motorIndex, const MotorConfig& config);
    const MotorConfig& getMotorConfig(int motorIndex) const { return motorConfigs_[motorIndex]; }
    
    // Convert motor angle (radians) to stepper steps
    int32_t angleToSteps(int motorIndex, float angleRadians) const;
    
    // Convert stepper steps to motor angle (radians)
    float stepsToAngle(int motorIndex, int32_t steps) const;
    
    // Set target motor angles and calculate required steps
    void setTargetAngles(const std::array<float, 3>& angles);
    
    // Get current motor states
    const std::array<MotorState, 3>& getMotorStates() const { return motorStates_; }
    
    // Trajectory planning
    std::vector<TrajectoryPoint> planTrajectory(
        const std::array<float, 3>& startAngles,
        const std::array<float, 3>& endAngles,
        float duration
    ) const;
    
    // Generate smooth motion profile (S-curve acceleration)
    void generateMotionProfile(
        float startPos, float endPos, float maxSpeed, float acceleration,
        float dt, std::vector<float>& positions, std::vector<float>& velocities
    ) const;
    
    // Update motor states (simulation step)
    void update(float deltaTime);
    
    // Homing
    void setHomed(int motorIndex, bool homed = true);
    void setHomeOffset(int motorIndex, float offsetRadians);
    
    bool isHomed() const { return isHomed_; }
    void setHomed(bool homed = true) { isHomed_ = homed; }
    void resetToSoftwareHome();
    std::array<float, 3> getAnglesRelativeToHome() const;
    const std::array<float, 3>& getSoftwareHomeAngles() const { return softwareHomeAngles_;; }
    
    // Waypoint management
    void addWaypoint(const Waypoint& waypoint);
    void removeWaypoint(size_t index);
    const std::vector<Waypoint>& getWaypoints() const { return waypoints_; }
    void clearWaypoints();
    
    // Execute waypoint sequence
    bool executeWaypointSequence(const std::vector<size_t>& waypointIndices);
    
    // Check if motors are at target position
    bool isAtTarget(float tolerance = 0.001f) const;
    
private:
    std::array<MotorConfig, 3> motorConfigs_;
    std::array<MotorState, 3> motorStates_;
    std::array<float, 3> targetAngles_;
    std::array<float, 3> softwareHomeAngles_;
    bool isHomed_ = false;
    bool softwareHomeActive_ = false;
    
    // Trajectory execution
    std::vector<TrajectoryPoint> currentTrajectory_;
    size_t currentTrajectoryIndex_;
    float trajectoryTime_;
    bool executingTrajectory_;
    
    // Waypoints
    std::vector<Waypoint> waypoints_;
    
};

