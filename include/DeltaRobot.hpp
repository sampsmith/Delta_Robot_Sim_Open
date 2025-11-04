#ifndef DELTA_ROBOT_HPP
#define DELTA_ROBOT_HPP

#include <array>
#include <cmath>
#include <optional>

struct Vec3 {
    float x, y, z;
    
    Vec3(float x = 0.0f, float y = 0.0f, float z = 0.0f) : x(x), y(y), z(z) {}
    
    float length() const {
        return std::sqrt(x*x + y*y + z*z);
    }
    
    Vec3 operator-(const Vec3& other) const {
        return Vec3(x - other.x, y - other.y, z - other.z);
    }
    
    Vec3 operator+(const Vec3& other) const {
        return Vec3(x + other.x, y + other.y, z + other.z);
    }
    
    Vec3 operator*(float scalar) const {
        return Vec3(x * scalar, y * scalar, z * scalar);
    }
};

struct DeltaRobotParams {
    float base_radius;      // Radius of base triangle (distance from center to joint)
    float platform_radius;  // Radius of end-effector platform
    float upper_arm_length; // Length of upper arm (connected to motors)
    float lower_arm_length; // Length of parallelogram arms (forearm)
    
    DeltaRobotParams(float br = 200.0f, float pr = 50.0f, 
                     float upper = 300.0f, float lower = 600.0f)
        : base_radius(br), platform_radius(pr), 
          upper_arm_length(upper), lower_arm_length(lower) {}
};

class DeltaRobot {
public:
    DeltaRobot(const DeltaRobotParams& params = DeltaRobotParams());
    
    // Inverse kinematics: calculate joint angles for desired end-effector position
    // Returns std::nullopt if position is unreachable
    std::optional<std::array<float, 3>> inverseKinematics(const Vec3& position);
    
    // Forward kinematics: calculate end-effector position from joint angles
    std::optional<Vec3> forwardKinematics(const std::array<float, 3>& angles);
    
    // Get current end-effector position
    Vec3 getEndEffectorPosition() const { return end_effector_pos_; }
    
    // Get current joint angles (in radians)
    std::array<float, 3> getJointAngles() const { return joint_angles_; }
    
    // Set target position (updates joint angles via IK)
    bool setTargetPosition(const Vec3& target);
    
    // Get robot parameters
    const DeltaRobotParams& getParams() const { return params_; }
    
    // Get base joint positions (3 points on base triangle)
    std::array<Vec3, 3> getBaseJointPositions() const;
    
    // Get elbow positions for current joint angles
    std::array<Vec3, 3> getElbowPositions() const;
    
    // Get platform joint positions (3 points on platform triangle)
    std::array<Vec3, 3> getPlatformJointPositions() const;

private:
    DeltaRobotParams params_;
    std::array<float, 3> joint_angles_;  // Current joint angles (radians)
    Vec3 end_effector_pos_;
    
    // Helper: solve IK for single arm (returns angle in radians)
    std::optional<float> solveArmIK(const Vec3& base_joint, const Vec3& platform_joint);
    
    // Helper: calculate elbow position for given base joint and angle
    Vec3 calculateElbowPosition(const Vec3& base_joint, float angle) const;
};

#endif // DELTA_ROBOT_HPP
