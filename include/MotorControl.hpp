#pragma once

#include <array>
#include <vector>
#include <string>
#include <glm/glm.hpp>
#include "DeltaRobot.hpp"

// Motor configuration for NEMA 23 stepper motors with 10:1 planetary gearbox
struct MotorConfig {
    // NEMA 23 Specifications
    int stepsPerRevolution = 200;        // Standard NEMA 23: 200 steps/rev (1.8 degrees/step)
    int microstepping = 16;              // Microstepping factor (1, 2, 4, 8, 16, 32, etc.)
    float gearRatio = 10.0f;             // Planetary gearbox ratio (10:1 = motor turns 10x for 1x arm rotation)
    
    // Motor Speed Limits (in motor shaft terms)
    float maxMotorRPM = 600.0f;          // Maximum motor RPM (typical NEMA 23: 600-1200 RPM)
    float maxStepsPerSecond = 0.0f;       // Calculated from RPM (auto-calculated)
    
    // Motion Parameters (in arm output terms after gearbox)
    float maxArmAngularVelocity = 0.0f;  // Maximum arm angular velocity (rad/s) - calculated from motor RPM
    float maxEndEffectorVelocity = 0.0f; // Maximum end-effector linear velocity (m/s) - calculated
    
    // Acceleration
    float acceleration = 500.0f;          // Acceleration (steps per second squared at motor)
    float armAngularAcceleration = 0.0f; // Acceleration at arm (rad/s²) - calculated
    
    // Calibration
    float homeOffset = 0.0f;             // Offset from home position (radians)
    bool inverted = false;               // Motor direction inversion
    
    // Calculate dependent values (call after updating motor parameters)
    void calculateDependentValues(float upperArmLength = 0.20f) {
        #ifndef M_PI
        #define M_PI 3.14159265358979323846f
        #endif
        
        // Calculate total steps per revolution at motor shaft
        float totalStepsPerRev = stepsPerRevolution * microstepping;
        
        // Calculate max steps per second from RPM
        maxStepsPerSecond = (maxMotorRPM / 60.0f) * totalStepsPerRev;
        
        // Calculate max arm angular velocity (after gearbox reduction)
        // Motor RPM -> Arm RPM = Motor RPM / gearRatio
        float maxArmRPM = maxMotorRPM / gearRatio;
        maxArmAngularVelocity = (maxArmRPM / 60.0f) * 2.0f * M_PI;  // rad/s
        
        // Calculate max end-effector velocity based on arm length
        // At maximum arm extension, linear velocity = angular velocity * arm length
        maxEndEffectorVelocity = maxArmAngularVelocity * upperArmLength;
        
        // Calculate arm angular acceleration from motor acceleration
        // Acceleration is in steps/sec² at motor, convert to rad/s² at arm
        float motorAngularAccel = (acceleration / totalStepsPerRev) * 2.0f * M_PI;  // rad/s² at motor
        armAngularAcceleration = motorAngularAccel / gearRatio;  // rad/s² at arm (after gearbox)
    }
    
    // Convert end-effector velocity (m/s) to required motor RPM
    float endEffectorVelocityToMotorRPM(float velocityMps, float upperArmLength) const {
        if (upperArmLength <= 0.0f) return 0.0f;
        // v = ω * r, so ω = v / r
        float armAngularVel = velocityMps / upperArmLength;  // rad/s
        float armRPM = (armAngularVel / (2.0f * M_PI)) * 60.0f;  // RPM at arm
        float motorRPM = armRPM * gearRatio;  // RPM at motor (account for gearbox)
        return motorRPM;
    }
    
    // Convert end-effector velocity (m/s) to required steps per second
    float endEffectorVelocityToStepsPerSecond(float velocityMps, float upperArmLength) const {
        float motorRPM = endEffectorVelocityToMotorRPM(velocityMps, upperArmLength);
        float totalStepsPerRev = stepsPerRevolution * microstepping;
        return (motorRPM / 60.0f) * totalStepsPerRev;
    }
};

// Motor state
struct MotorState {
    float currentAngle = 0.0f;           // Current motor angle (radians)
    int32_t currentSteps = 0;            // Current step position (relative to home)
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
    std::array<int32_t, 3> stepPositions; // Step positions (for hardware - primary control)
    float duration;                      // Time to reach this waypoint (seconds)
    
    // Initialize step positions from motor angles
    void calculateSteps(const MotorControl& motorControl);
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
    
    // Home position system
    struct HomePosition {
        std::array<int32_t, 3> physicalHomeSteps;  // Physical sensor position (upper limit switch)
        std::array<int32_t, 3> officialHomeSteps;  // Official home position (after offset)
        std::array<int32_t, 3> homeOffsetSteps;     // Offset from physical sensor to official home
        bool isHomed = false;                       // Whether robot has been homed
    };
    
    void setPhysicalHomePosition(int motorIndex, int32_t physicalHomeSteps);
    void setHomeOffsetSteps(int motorIndex, int32_t offsetSteps);
    void setOfficialHomePosition(int motorIndex, int32_t officialHomeSteps);
    const HomePosition& getHomePosition() const { return homePosition_; }
    bool isHomed() const { return homePosition_.isHomed; }
    void setHomed(bool homed = true) { homePosition_.isHomed = homed; }
    
    // Waypoint management
    void addWaypoint(const Waypoint& waypoint);
    void removeWaypoint(size_t index);
    const std::vector<Waypoint>& getWaypoints() const { return waypoints_; }
    void clearWaypoints();
    
    // Execute waypoint sequence
    bool executeWaypointSequence(const std::vector<size_t>& waypointIndices);
    
    // Get steps needed for each motor to reach target
    std::array<int32_t, 3> getRequiredSteps() const;
    
    // Check if motors are at target position
    bool isAtTarget(float tolerance = 0.001f) const;
    
    // Velocity calculations based on robot geometry
    void updateVelocityLimits(const DeltaRobotConfig& robotConfig);
    
    // Calculate required motor speeds for end-effector velocity
    std::array<float, 3> calculateMotorSpeedsForVelocity(
        const glm::vec3& endEffectorVelocity,
        const DeltaRobotConfig& robotConfig,
        const std::array<float, 3>& currentMotorAngles
    ) const;
    
private:
    HomePosition homePosition_;
    
private:
    std::array<MotorConfig, 3> motorConfigs_;
    std::array<MotorState, 3> motorStates_;
    std::array<float, 3> targetAngles_;
    
    // Trajectory execution
    std::vector<TrajectoryPoint> currentTrajectory_;
    size_t currentTrajectoryIndex_;
    float trajectoryTime_;
    bool executingTrajectory_;
    
    // Waypoints
    std::vector<Waypoint> waypoints_;
    
    // Helper functions
    float calculateTotalSteps(int motorIndex) const;
    float clampAngle(int motorIndex, float angle) const;
};

