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
{
    // Default configuration for all motors
    for (int i = 0; i < 3; ++i) {
        motorConfigs_[i] = MotorConfig();
        motorStates_[i] = MotorState();
        targetAngles_[i] = 0.0f;
    }
    
    // Initialize home position
    homePosition_.isHomed = false;
    for (int i = 0; i < 3; ++i) {
        homePosition_.physicalHomeSteps[i] = 0;
        homePosition_.officialHomeSteps[i] = 0;
        homePosition_.homeOffsetSteps[i] = 0;
    }
}

void MotorControl::setMotorConfig(int motorIndex, const MotorConfig& config) {
    if (motorIndex >= 0 && motorIndex < 3) {
        motorConfigs_[motorIndex] = config;
        // Recalculate dependent values
        motorConfigs_[motorIndex].calculateDependentValues(0.20f);  // Default arm length, will be updated
    }
}

int32_t MotorControl::angleToSteps(int motorIndex, float angleRadians) const {
    if (motorIndex < 0 || motorIndex >= 3) return 0;
    
    const MotorConfig& config = motorConfigs_[motorIndex];
    
    // Account for home offset
    float adjustedAngle = angleRadians - config.homeOffset;
    
    // Convert radians to steps
    // Steps = (angle / 2π) * stepsPerRevolution * microstepping * gearRatio
    float revolutions = adjustedAngle / (2.0f * M_PI);
    float totalStepsPerRev = config.stepsPerRevolution * config.microstepping * config.gearRatio;
    int32_t steps = static_cast<int32_t>(revolutions * totalStepsPerRev);
    
    if (config.inverted) {
        steps = -steps;
    }
    
    return steps;
}

float MotorControl::stepsToAngle(int motorIndex, int32_t steps) const {
    if (motorIndex < 0 || motorIndex >= 3) return 0.0f;
    
    const MotorConfig& config = motorConfigs_[motorIndex];
    
    int32_t adjustedSteps = config.inverted ? -steps : steps;
    
    // Convert steps to radians
    float totalStepsPerRev = config.stepsPerRevolution * config.microstepping * config.gearRatio;
    float revolutions = static_cast<float>(adjustedSteps) / totalStepsPerRev;
    float angle = revolutions * 2.0f * M_PI;
    
    // Account for home offset
    return angle + config.homeOffset;
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
        // Use calculated max steps per second, convert to angular velocity
        float maxAngularVelocity = motorConfigs_[i].maxStepsPerSecond / calculateTotalSteps(i);
        float angularVelocity = std::min(std::abs(error) / deltaTime, maxAngularVelocity);
        
        if (std::abs(error) > 0.0001f) {
            motorStates_[i].isMoving = true;
            float step = (error > 0) ? angularVelocity * deltaTime : -angularVelocity * deltaTime;
            motorStates_[i].currentAngle += step;
            motorStates_[i].currentSteps = angleToSteps(i, motorStates_[i].currentAngle);
        } else {
            motorStates_[i].isMoving = false;
            motorStates_[i].currentAngle = targetAngles_[i];
            motorStates_[i].currentSteps = angleToSteps(i, motorStates_[i].currentAngle);
        }
    }
}

void MotorControl::setHomed(int motorIndex, bool homed) {
    if (motorIndex >= 0 && motorIndex < 3) {
        motorStates_[motorIndex].isHomed = homed;
        if (homed) {
            motorStates_[motorIndex].currentSteps = 0;
        }
    }
}

void MotorControl::setHomeOffset(int motorIndex, float offsetRadians) {
    if (motorIndex >= 0 && motorIndex < 3) {
        motorConfigs_[motorIndex].homeOffset = offsetRadians;
    }
}

void Waypoint::calculateSteps(const MotorControl& motorControl) {
    for (int i = 0; i < 3; ++i) {
        stepPositions[i] = motorControl.angleToSteps(i, motorAngles[i]);
    }
}

void MotorControl::addWaypoint(const Waypoint& waypoint) {
    Waypoint wp = waypoint;
    // Ensure step positions are calculated
    if (wp.stepPositions[0] == 0 && wp.stepPositions[1] == 0 && wp.stepPositions[2] == 0) {
        wp.calculateSteps(*this);
    }
    waypoints_.push_back(wp);
}

void MotorControl::setPhysicalHomePosition(int motorIndex, int32_t physicalHomeSteps) {
    if (motorIndex >= 0 && motorIndex < 3) {
        homePosition_.physicalHomeSteps[motorIndex] = physicalHomeSteps;
        // Recalculate official home
        homePosition_.officialHomeSteps[motorIndex] = physicalHomeSteps + homePosition_.homeOffsetSteps[motorIndex];
    }
}

void MotorControl::setHomeOffsetSteps(int motorIndex, int32_t offsetSteps) {
    if (motorIndex >= 0 && motorIndex < 3) {
        homePosition_.homeOffsetSteps[motorIndex] = offsetSteps;
        // Recalculate official home
        homePosition_.officialHomeSteps[motorIndex] = homePosition_.physicalHomeSteps[motorIndex] + offsetSteps;
    }
}

void MotorControl::setOfficialHomePosition(int motorIndex, int32_t officialHomeSteps) {
    if (motorIndex >= 0 && motorIndex < 3) {
        homePosition_.officialHomeSteps[motorIndex] = officialHomeSteps;
        // Calculate offset from physical home
        homePosition_.homeOffsetSteps[motorIndex] = officialHomeSteps - homePosition_.physicalHomeSteps[motorIndex];
    }
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

std::array<int32_t, 3> MotorControl::getRequiredSteps() const {
    std::array<int32_t, 3> steps;
    for (int i = 0; i < 3; ++i) {
        int32_t targetSteps = angleToSteps(i, targetAngles_[i]);
        steps[i] = targetSteps - motorStates_[i].currentSteps;
    }
    return steps;
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

float MotorControl::calculateTotalSteps(int motorIndex) const {
    if (motorIndex < 0 || motorIndex >= 3) return 1.0f;
    const MotorConfig& config = motorConfigs_[motorIndex];
    return static_cast<float>(config.stepsPerRevolution * config.microstepping * config.gearRatio);
}

float MotorControl::clampAngle(int motorIndex, float angle) const {
    // Angles are already constrained by DeltaRobot kinematics
    return angle;
}

void MotorControl::updateVelocityLimits(const DeltaRobotConfig& robotConfig) {
    // Update velocity limits for all motors based on robot geometry
    for (int i = 0; i < 3; ++i) {
        motorConfigs_[i].calculateDependentValues(robotConfig.upperArmLength);
    }
}

std::array<float, 3> MotorControl::calculateMotorSpeedsForVelocity(
    const glm::vec3& endEffectorVelocity,
    const DeltaRobotConfig& robotConfig,
    const std::array<float, 3>& currentMotorAngles
) const {
    std::array<float, 3> motorSpeeds = {0.0f, 0.0f, 0.0f};
    
    // For delta robot, we need to calculate the Jacobian to convert end-effector velocity
    // to joint velocities. For now, use a simplified approach based on arm length.
    
    // The relationship between end-effector velocity and motor angular velocity
    // depends on the current configuration. For a simplified model:
    // v_end = ω_arm * L_upper (approximate at maximum extension)
    
    float speedMagnitude = glm::length(endEffectorVelocity);
    if (speedMagnitude > 0.0f) {
        // Calculate required arm angular velocity
        float armAngularVel = speedMagnitude / robotConfig.upperArmLength;  // rad/s
        
        // Convert to motor RPM (accounting for gearbox)
        float armRPM = (armAngularVel / (2.0f * M_PI)) * 60.0f;  // RPM at arm
        float motorRPM = armRPM * motorConfigs_[0].gearRatio;  // RPM at motor
        
        // Convert to steps per second
        float totalStepsPerRev = motorConfigs_[0].stepsPerRevolution * motorConfigs_[0].microstepping;
        float stepsPerSecond = (motorRPM / 60.0f) * totalStepsPerRev;
        
        // Apply to all motors (simplified - in reality each motor speed would differ)
        for (int i = 0; i < 3; ++i) {
            motorSpeeds[i] = stepsPerSecond;
        }
    }
    
    return motorSpeeds;
}

