#include "DeltaRobot.hpp"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

DeltaRobot::DeltaRobot(const DeltaRobotParams& params)
    : params_(params)
    , joint_angles_({0.0f, 0.0f, 0.0f})
    , end_effector_pos_(0.0f, 0.0f, -500.0f)
{
    // Initialize with home position
    setTargetPosition(end_effector_pos_);
}

std::array<Vec3, 3> DeltaRobot::getBaseJointPositions() const {
    std::array<Vec3, 3> positions;
    
    for (int i = 0; i < 3; ++i) {
        float angle = i * 2.0f * M_PI / 3.0f;  // 120 degrees apart
        positions[i] = Vec3(
            params_.base_radius * std::cos(angle),
            params_.base_radius * std::sin(angle),
            0.0f
        );
    }
    
    return positions;
}

std::array<Vec3, 3> DeltaRobot::getPlatformJointPositions() const {
    std::array<Vec3, 3> positions;
    
    for (int i = 0; i < 3; ++i) {
        float angle = i * 2.0f * M_PI / 3.0f + M_PI;  // Offset by 180 degrees
        positions[i] = end_effector_pos_ + Vec3(
            params_.platform_radius * std::cos(angle),
            params_.platform_radius * std::sin(angle),
            0.0f
        );
    }
    
    return positions;
}

Vec3 DeltaRobot::calculateElbowPosition(const Vec3& base_joint, float angle) const {
    // Calculate elbow position based on base joint and arm angle
    // Angle is measured from horizontal plane, negative is downward
    float arm_x = params_.upper_arm_length * std::cos(angle);
    float arm_z = -params_.upper_arm_length * std::sin(angle);
    
    // Direction from origin to base joint (normalized)
    float base_angle = std::atan2(base_joint.y, base_joint.x);
    
    Vec3 elbow = base_joint;
    elbow.x += arm_x * std::cos(base_angle);
    elbow.y += arm_x * std::sin(base_angle);
    elbow.z += arm_z;
    
    return elbow;
}

std::array<Vec3, 3> DeltaRobot::getElbowPositions() const {
    std::array<Vec3, 3> positions;
    auto base_joints = getBaseJointPositions();
    
    for (int i = 0; i < 3; ++i) {
        positions[i] = calculateElbowPosition(base_joints[i], joint_angles_[i]);
    }
    
    return positions;
}

std::optional<float> DeltaRobot::solveArmIK(const Vec3& base_joint, const Vec3& platform_joint) {
    // Calculate the required angle for one arm to reach from base_joint to platform_joint
    
    // Vector from base joint to platform joint
    Vec3 diff = platform_joint - base_joint;
    float dx = std::sqrt(diff.x * diff.x + diff.y * diff.y);  // Horizontal distance
    float dz = diff.z;  // Vertical distance (negative when platform below base)
    
    // Using law of cosines to find elbow position
    // We need to find angle θ such that:
    // - Upper arm rotates by θ from horizontal
    // - Lower arm connects elbow to platform joint
    
    float L1 = params_.upper_arm_length;
    float L2 = params_.lower_arm_length;
    
    // Distance squared from base to platform
    float d_squared = dx * dx + dz * dz;
    float d = std::sqrt(d_squared);
    
    // Check if target is reachable
    if (d > L1 + L2 || d < std::abs(L1 - L2)) {
        return std::nullopt;  // Out of reach
    }
    
    // Use law of cosines to find the angle
    // This is a simplified 2D solution in the plane of motion
    float cos_alpha = (L1 * L1 + d_squared - L2 * L2) / (2.0f * L1 * d);
    
    if (cos_alpha < -1.0f || cos_alpha > 1.0f) {
        return std::nullopt;
    }
    
    float alpha = std::acos(cos_alpha);
    float beta = std::atan2(-dz, dx);
    
    // The joint angle (from horizontal, negative is down)
    float theta = -(beta + alpha);
    
    return theta;
}

std::optional<std::array<float, 3>> DeltaRobot::inverseKinematics(const Vec3& position) {
    auto base_joints = getBaseJointPositions();
    std::array<float, 3> angles;
    
    // Calculate platform joint positions for the target end-effector position
    for (int i = 0; i < 3; ++i) {
        float angle = i * 2.0f * M_PI / 3.0f + M_PI;
        Vec3 platform_joint = position + Vec3(
            params_.platform_radius * std::cos(angle),
            params_.platform_radius * std::sin(angle),
            0.0f
        );
        
        auto result = solveArmIK(base_joints[i], platform_joint);
        if (!result) {
            return std::nullopt;  // Position unreachable
        }
        
        angles[i] = *result;
    }
    
    return angles;
}

std::optional<Vec3> DeltaRobot::forwardKinematics(const std::array<float, 3>& angles) {
    // Forward kinematics is more complex for delta robots
    // This is a simplified version - for accurate FK, you'd solve the intersection
    // of three spheres (one for each lower arm)
    
    auto base_joints = getBaseJointPositions();
    std::array<Vec3, 3> elbows;
    
    for (int i = 0; i < 3; ++i) {
        elbows[i] = calculateElbowPosition(base_joints[i], angles[i]);
    }
    
    // Approximate: average of the three sphere centers adjusted by forearm length
    // This is a rough estimate; proper FK requires solving sphere intersections
    Vec3 center(0, 0, 0);
    for (const auto& elbow : elbows) {
        center = center + elbow;
    }
    center = center * (1.0f / 3.0f);
    
    // Adjust by approximate forearm distance
    center.z -= params_.lower_arm_length * 0.7f;  // Rough approximation
    
    return center;
}

bool DeltaRobot::setTargetPosition(const Vec3& target) {
    auto angles = inverseKinematics(target);
    
    if (angles) {
        joint_angles_ = *angles;
        end_effector_pos_ = target;
        return true;
    }
    
    return false;
}
