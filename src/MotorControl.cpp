#include "MotorControl.hpp"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

MotorControl::MotorControl() 
    : currentTrajectoryIndex_(0)
    , trajectoryTime_(0.0f)
    , executingTrajectory_(false)
    , softwareHomeActive_(false)
    , isHomed_(false)
{
    // Default configuration for all motors
    for (int i = 0; i < 3; ++i) {
        motorConfigs_[i] = MotorConfig();
        motorStates_[i] = MotorState();
        targetAngles_[i] = 0.0f;
        softwareHomeAngles_[i] = 0.0f;
    }
}

void MotorControl::setMotorConfig(int motorIndex, const MotorConfig& config) {
    if (motorIndex >= 0 && motorIndex < 3) {
        motorConfigs_[motorIndex] = config;
    }
}

void MotorControl::setTargetAngles(const std::array<float, 3>& angles) {
    targetAngles_ = angles;
    executingTrajectory_ = false;
}

std::vector<TrajectoryPoint> MotorControl::planTrajectory(
    const std::array<float, 3>& startAngles,
    const std::array<float, 3>& endAngles,
    float duration
) const {
    std::vector<TrajectoryPoint> trajectory;
    
    if (duration <= 0.0f) {
        // Instant movement
        TrajectoryPoint point;
        point.motorAngles = endAngles;
        point.time = 0.0f;
        trajectory.push_back(point);
        return trajectory;
    }
    
    // Calculate number of segments based on duration and update rate
    const float dt = 0.01f; // 100 Hz update rate
    int numSegments = static_cast<int>(duration / dt);
    numSegments = std::max(1, numSegments);
    
    // Generate trajectory points
    for (int i = 0; i <= numSegments; ++i) {
        float t = static_cast<float>(i) / numSegments;
        
        // Simple linear interpolation for now
        // Can be enhanced with S-curve or cubic spline
        TrajectoryPoint point;
        for (int j = 0; j < 3; ++j) {
            point.motorAngles[j] = startAngles[j] + (endAngles[j] - startAngles[j]) * t;
        }
        point.time = t * duration;
        trajectory.push_back(point);
    }
    
    return trajectory;
}

void MotorControl::generateMotionProfile(
    float startPos, float endPos, float maxSpeed, float acceleration,
    float dt, std::vector<float>& positions, std::vector<float>& velocities
) const {
    positions.clear();
    velocities.clear();
    
    float distance = endPos - startPos;
    float direction = (distance > 0) ? 1.0f : -1.0f;
    distance = std::abs(distance);
    
    if (distance < 0.001f) {
        positions.push_back(startPos);
        velocities.push_back(0.0f);
        return;
    }
    
    // Calculate time to reach max speed
    float timeToMaxSpeed = maxSpeed / acceleration;
    float distanceToMaxSpeed = 0.5f * acceleration * timeToMaxSpeed * timeToMaxSpeed;
    
    // Determine if we reach max speed
    bool reachesMaxSpeed = (distance > 2.0f * distanceToMaxSpeed);
    
    float currentPos = startPos;
    float currentVel = 0.0f;
    float currentTime = 0.0f;
    
    if (reachesMaxSpeed) {
        // Acceleration phase
        while (currentVel < maxSpeed) {
            positions.push_back(currentPos);
            velocities.push_back(currentVel * direction);
            
            currentVel += acceleration * dt;
            currentPos += currentVel * dt * direction;
            currentTime += dt;
        }
        
        // Constant velocity phase
        float remainingDistance = distance - 2.0f * distanceToMaxSpeed;
        float constantTime = remainingDistance / maxSpeed;
        int constantSteps = static_cast<int>(constantTime / dt);
        
        for (int i = 0; i < constantSteps; ++i) {
            positions.push_back(currentPos);
            velocities.push_back(currentVel * direction);
            currentPos += maxSpeed * dt * direction;
            currentTime += dt;
        }
        
        // Deceleration phase
        while (currentVel > 0.0f) {
            positions.push_back(currentPos);
            velocities.push_back(currentVel * direction);
            
            currentVel -= acceleration * dt;
            currentVel = std::max(0.0f, currentVel);
            currentPos += currentVel * dt * direction;
            currentTime += dt;
        }
    } else {
        // Triangular profile (no constant velocity phase)
        float peakTime = std::sqrt(distance / acceleration);
        float peakVel = acceleration * peakTime;
        
        while (currentTime < peakTime * 2.0f) {
            positions.push_back(currentPos);
            velocities.push_back(currentVel * direction);
            
            if (currentTime < peakTime) {
                // Acceleration
                currentVel += acceleration * dt;
            } else {
                // Deceleration
                currentVel -= acceleration * dt;
                currentVel = std::max(0.0f, currentVel);
            }
            
            currentPos += currentVel * dt * direction;
            currentTime += dt;
        }
    }
    
    // Ensure we end at target
    positions.push_back(endPos);
    velocities.push_back(0.0f);
}

void MotorControl::update(float deltaTime) {
    if (executingTrajectory_ && !currentTrajectory_.empty()) {
        trajectoryTime_ += deltaTime;
        
        // Find current trajectory segment
        while (currentTrajectoryIndex_ < currentTrajectory_.size() - 1 &&
               trajectoryTime_ >= currentTrajectory_[currentTrajectoryIndex_ + 1].time) {
            currentTrajectoryIndex_++;
        }
        
        if (currentTrajectoryIndex_ >= currentTrajectory_.size() - 1) {
            // Trajectory complete
            targetAngles_ = currentTrajectory_.back().motorAngles;
            executingTrajectory_ = false;
        } else {
            // Interpolate between current and next point
            const TrajectoryPoint& p0 = currentTrajectory_[currentTrajectoryIndex_];
            const TrajectoryPoint& p1 = currentTrajectory_[currentTrajectoryIndex_ + 1];
            
            float t = 0.0f;
            if (p1.time > p0.time) {
                t = (trajectoryTime_ - p0.time) / (p1.time - p0.time);
            }
            t = std::clamp(t, 0.0f, 1.0f);
            
            for (int i = 0; i < 3; ++i) {
                targetAngles_[i] = p0.motorAngles[i] + (p1.motorAngles[i] - p0.motorAngles[i]) * t;
            }
        }
    }
    
    // Update motor states (simulate movement toward target)
    for (int i = 0; i < 3; ++i) {
        float error = targetAngles_[i] - motorStates_[i].currentAngle;
        float maxAngularVelocity = motorConfigs_[i].maxAngularVelocity;
        if (maxAngularVelocity <= 0.0f) {
            maxAngularVelocity = 1.0f;
        }
        float maxDelta = maxAngularVelocity * deltaTime;
        
        if (std::abs(error) > 0.0001f) {
            motorStates_[i].isMoving = true;
            float step = std::clamp(error, -maxDelta, maxDelta);
            motorStates_[i].currentAngle += step;
        } else {
            motorStates_[i].isMoving = false;
            motorStates_[i].currentAngle = targetAngles_[i];
        }
    }
}

void MotorControl::setHomed(int motorIndex, bool homed) {
    if (motorIndex >= 0 && motorIndex < 3) {
        motorStates_[motorIndex].isHomed = homed;
    }
}

void MotorControl::setHomeOffset(int motorIndex, float offsetRadians) {
    if (motorIndex >= 0 && motorIndex < 3) {
        motorConfigs_[motorIndex].homeOffset = offsetRadians;
    }
}

void MotorControl::addWaypoint(const Waypoint& waypoint) {
    waypoints_.push_back(waypoint);
}

void MotorControl::resetToSoftwareHome() {
    for (int i = 0; i < 3; ++i) {
        softwareHomeAngles_[i] = motorStates_[i].currentAngle;
        motorStates_[i].isHomed = true;
        targetAngles_[i] = motorStates_[i].currentAngle;
    }
    isHomed_ = true;
    softwareHomeActive_ = true;
}

std::array<float, 3> MotorControl::getAnglesRelativeToHome() const {
    std::array<float, 3> relativeAngles;
    for (int i = 0; i < 3; ++i) {
        float offset = softwareHomeActive_ ? softwareHomeAngles_[i] : 0.0f;
        relativeAngles[i] = motorStates_[i].currentAngle - offset;
    }
    return relativeAngles;
}

void MotorControl::removeWaypoint(size_t index) {
    if (index < waypoints_.size()) {
        waypoints_.erase(waypoints_.begin() + index);
    }
}

void MotorControl::clearWaypoints() {
    waypoints_.clear();
}

bool MotorControl::executeWaypointSequence(const std::vector<size_t>& waypointIndices) {
    if (waypointIndices.empty()) return false;
    
    // Build trajectory from waypoints
    currentTrajectory_.clear();
    float totalTime = 0.0f;
    
    std::array<float, 3> startAngles = motorStates_[0].isHomed ? 
        std::array<float, 3>{motorStates_[0].currentAngle, motorStates_[1].currentAngle, motorStates_[2].currentAngle} :
        std::array<float, 3>{0.0f, 0.0f, 0.0f};
    
    for (size_t idx : waypointIndices) {
        if (idx >= waypoints_.size()) continue;
        
        const Waypoint& wp = waypoints_[idx];
        std::vector<TrajectoryPoint> segment = planTrajectory(startAngles, wp.motorAngles, wp.duration);
        
        // Adjust time offsets
        for (auto& point : segment) {
            point.time += totalTime;
        }
        
        currentTrajectory_.insert(currentTrajectory_.end(), segment.begin(), segment.end());
        startAngles = wp.motorAngles;
        totalTime += wp.duration;
    }
    
    if (!currentTrajectory_.empty()) {
        currentTrajectoryIndex_ = 0;
        trajectoryTime_ = 0.0f;
        executingTrajectory_ = true;
        return true;
    }
    
    return false;
}

bool MotorControl::isAtTarget(float tolerance) const {
    for (int i = 0; i < 3; ++i) {
        float error = std::abs(targetAngles_[i] - motorStates_[i].currentAngle);
        if (error > tolerance) {
            return false;
        }
    }
    return true;
}

