#include "SequenceController.hpp"
#include "DeltaRobotProtocol.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

SequenceController::SequenceController(MotorControl& motorControl, HardwareInterface* hardwareInterface)
    : motorControl_(motorControl)
    , hardwareInterface_(hardwareInterface)
    , currentSequenceIndex_(0)
    , currentRepeat_(0)
    , state_(PlaybackState::Stopped)
    , currentTrajectoryIndex_(0)
    , trajectoryTime_(0.0f)
    , lastStreamTime_(0.0f)
    , sequenceStartTime_(0.0f)
{
    stats_ = Statistics();
}

void SequenceController::setSequence(const std::vector<size_t>& waypointIndices) {
    currentSequence_ = waypointIndices;
    currentSequenceIndex_ = 0;
    currentRepeat_ = 0;
    stats_.completedRepeats = 0;
}

void SequenceController::clearSequence() {
    currentSequence_.clear();
    stop();
}

bool SequenceController::play() {
    if (currentSequence_.empty()) return false;
    
    state_ = PlaybackState::Playing;
    currentSequenceIndex_ = 0;
    currentRepeat_ = 0;
    trajectoryTime_ = 0.0f;
    lastStreamTime_ = 0.0f;
    sequenceStartTime_ = 0.0f;  // Will be set on first update
    
    stats_.completedRepeats = 0;
    stats_.elapsedTime = 0.0f;
    
    // If batch mode AND hardware connected, send entire sequence packet at once
    // (For simulation, we'll execute trajectory normally)
    if (config_.streamingMode == StreamingMode::TrajectoryBatch && 
        hardwareInterface_ && hardwareInterface_->isConnected()) {
        
        const auto& waypoints = motorControl_.getWaypoints();
        std::vector<std::pair<std::array<int32_t, 3>, uint16_t>> sequenceData;
        
        // Build sequence data with step positions and durations
        // Calculate durations based on step distances and motor speed limits
        std::array<int32_t, 3> previousSteps = {0, 0, 0};
        if (currentSequence_.size() > 0 && currentSequence_[0] < waypoints.size()) {
            // Get current motor positions as starting point
            const auto& motorStates = motorControl_.getMotorStates();
            for (int i = 0; i < 3; ++i) {
                previousSteps[i] = motorStates[i].currentSteps;
            }
        }
        
        for (size_t seqIdx = 0; seqIdx < currentSequence_.size(); ++seqIdx) {
            size_t idx = currentSequence_[seqIdx];
            if (idx >= waypoints.size()) continue;
            const Waypoint& wp = waypoints[idx];
            
            // Calculate step distance for each motor
            std::array<int32_t, 3> stepDistances;
            float maxRequiredSpeed = 0.0f;
            for (int i = 0; i < 3; ++i) {
                stepDistances[i] = std::abs(wp.stepPositions[i] - previousSteps[i]);
                
                // Calculate minimum duration based on motor max speed
                const auto& motorConfig = motorControl_.getMotorConfig(i);
                if (stepDistances[i] > 0 && motorConfig.maxStepsPerSecond > 0.0f) {
                    // Minimum time = distance / max_speed
                    float minDuration = static_cast<float>(stepDistances[i]) / motorConfig.maxStepsPerSecond;
                    maxRequiredSpeed = std::max(maxRequiredSpeed, minDuration);
                }
            }
            
            // Apply speed multiplier to user-specified duration
            float adjustedDuration = wp.duration / config_.speedMultiplier;
            
            // Ensure duration is at least the minimum required for motor speed limits
            // Add 10% safety margin
            float minDurationWithMargin = maxRequiredSpeed * 1.1f;
            if (adjustedDuration < minDurationWithMargin) {
                adjustedDuration = minDurationWithMargin;
                std::cout << "[Sequence] Warning: Waypoint " << idx 
                          << " duration too short, adjusted to " << adjustedDuration 
                          << "s (minimum: " << minDurationWithMargin << "s)" << std::endl;
            }
            
            // Clamp duration to reasonable limits (10ms to 65.5s for uint16_t in ms)
            adjustedDuration = std::max(0.01f, std::min(adjustedDuration, 65.5f));
            
            uint16_t durationMs = static_cast<uint16_t>(adjustedDuration * 1000.0f);
            if (durationMs < 10) durationMs = 10;  // Minimum 10ms
            
            sequenceData.push_back({wp.stepPositions, durationMs});
            
            // Update previous steps for next iteration
            previousSteps = wp.stepPositions;
        }
        
        // Send entire sequence in one packet
        if (!sequenceData.empty()) {
            bool success = hardwareInterface_->sendWaypointSequence(sequenceData);
            if (success) {
                std::cout << "[Sequence] Sent " << sequenceData.size() 
                          << " waypoints in sequence packet to NUCLEO-H7S3L8" << std::endl;
                // NUCLEO-H7S3L8 will handle execution, we monitor progress via status updates
                // But we still execute trajectory for visualization
            }
        }
    }
    
    // Start first waypoint
    startNextWaypoint();
    
    return true;
}

bool SequenceController::pause() {
    if (state_ == PlaybackState::Playing) {
        state_ = PlaybackState::Paused;
        return true;
    }
    return false;
}

bool SequenceController::stop() {
    state_ = PlaybackState::Stopped;
    currentSequenceIndex_ = 0;
    currentRepeat_ = 0;
    trajectoryTime_ = 0.0f;
    currentTrajectory_.clear();
    stats_.elapsedTime = 0.0f;
    
    // Stop motors if hardware connected
    if (hardwareInterface_ && hardwareInterface_->isConnected()) {
        hardwareInterface_->stopMotors();
    }
    
    return true;
}

bool SequenceController::resume() {
    if (state_ == PlaybackState::Paused) {
        state_ = PlaybackState::Playing;
        return true;
    }
    return false;
}

void SequenceController::update(float deltaTime) {
    if (state_ != PlaybackState::Playing) return;
    
    if (sequenceStartTime_ == 0.0f) {
        sequenceStartTime_ = 0.0f;  // Track sequence start
    }
    
    stats_.elapsedTime += deltaTime;
    trajectoryTime_ += deltaTime;
    
    if (currentTrajectory_.empty()) {
        // No trajectory, move to next waypoint
        if (currentSequenceIndex_ < currentSequence_.size()) {
            startNextWaypoint();
        } else {
            // Sequence complete
            onSequenceComplete();
        }
        return;
    }
    
    // Execute trajectory
    if (config_.streamingMode == StreamingMode::RealTimeStream) {
        // Real-time streaming: send points as we execute them
        float streamInterval = 1.0f / config_.updateRate;
        
        while (currentTrajectoryIndex_ < currentTrajectory_.size() - 1 &&
               trajectoryTime_ >= currentTrajectory_[currentTrajectoryIndex_ + 1].time) {
            
            // Update motor control for visualization (always)
            const TrajectoryPoint& point = currentTrajectory_[currentTrajectoryIndex_ + 1];
            motorControl_.setTargetAngles(point.motorAngles);
            
            // Send this trajectory point to hardware if connected
            if (hardwareInterface_ && hardwareInterface_->isConnected()) {
                streamTrajectoryPoint(point);
            }
            
            currentTrajectoryIndex_++;
        }
        
        // Check if trajectory complete
        if (currentTrajectoryIndex_ >= currentTrajectory_.size() - 1) {
            // Ensure we send the final point
            if (hardwareInterface_ && hardwareInterface_->isConnected() && 
                !currentTrajectory_.empty()) {
                streamTrajectoryPoint(currentTrajectory_.back());
            }
            
            // Wait for completion if required
            if (config_.waitForCompletion) {
                if (checkPositionReached()) {
                    currentTrajectory_.clear();
                    currentSequenceIndex_++;
                    trajectoryTime_ = 0.0f;
                }
            } else {
                currentTrajectory_.clear();
                currentSequenceIndex_++;
                trajectoryTime_ = 0.0f;
            }
        }
    } else {
        // Batch mode: Execute trajectory in simulation (hardware will execute from packet)
        // We still need to execute the trajectory for visualization
        if (!currentTrajectory_.empty()) {
            // Interpolate through trajectory points for visualization
            while (currentTrajectoryIndex_ < currentTrajectory_.size() - 1 &&
                   trajectoryTime_ >= currentTrajectory_[currentTrajectoryIndex_ + 1].time) {
                
                // Update motor control to show trajectory progress
                const TrajectoryPoint& point = currentTrajectory_[currentTrajectoryIndex_ + 1];
                motorControl_.setTargetAngles(point.motorAngles);
                
                currentTrajectoryIndex_++;
            }
            
            // Check if trajectory complete
            if (currentTrajectoryIndex_ >= currentTrajectory_.size() - 1) {
                // Ensure final point is set
                if (!currentTrajectory_.empty()) {
                    motorControl_.setTargetAngles(currentTrajectory_.back().motorAngles);
                }
                
                // Wait for completion or duration
                if (config_.waitForCompletion) {
                    if (checkPositionReached()) {
                        currentTrajectory_.clear();
                        currentSequenceIndex_++;
                        trajectoryTime_ = 0.0f;
                    }
                } else {
                    // Wait for estimated duration
                    if (trajectoryTime_ >= currentTrajectory_.back().time) {
                        currentTrajectory_.clear();
                        currentSequenceIndex_++;
                        trajectoryTime_ = 0.0f;
                    }
                }
            }
        }
    }
    
    // Check if we need to move to next waypoint
    if (currentTrajectory_.empty() && currentSequenceIndex_ < currentSequence_.size()) {
        startNextWaypoint();
    } else if (currentTrajectory_.empty() && currentSequenceIndex_ >= currentSequence_.size()) {
        // Sequence complete
        onSequenceComplete();
    }
}

void SequenceController::startNextWaypoint() {
    if (currentSequenceIndex_ >= currentSequence_.size()) {
        onSequenceComplete();
        return;
    }
    
    size_t waypointIdx = currentSequence_[currentSequenceIndex_];
    const auto& waypoints = motorControl_.getWaypoints();
    
    if (waypointIdx >= waypoints.size()) {
        currentSequenceIndex_++;
        return;
    }
    
    const Waypoint& wp = waypoints[waypointIdx];
    
    // Get current motor angles
    const auto& motorStates = motorControl_.getMotorStates();
    std::array<float, 3> startAngles = {
        motorStates[0].currentAngle,
        motorStates[1].currentAngle,
        motorStates[2].currentAngle
    };
    
    // Plan trajectory to waypoint
    // Apply speed multiplier to duration (multiplier > 1.0 = faster, < 1.0 = slower)
    float adjustedDuration = wp.duration / config_.speedMultiplier;
    currentTrajectory_ = motorControl_.planTrajectory(
        startAngles, wp.motorAngles, adjustedDuration
    );
    
    if (currentTrajectory_.empty()) {
        currentSequenceIndex_++;
        return;
    }
    
    currentTrajectoryIndex_ = 0;
    trajectoryTime_ = 0.0f;
    
    // Send trajectory based on mode
    if (config_.streamingMode == StreamingMode::TrajectoryBatch) {
        // Batch mode: send waypoint step positions directly to NUCLEO-H7S3L8
        // NUCLEO-H7S3L8 will interpolate between waypoints
        sendMoveCommand(wp.stepPositions);
    }
    // Real-time mode: points will be sent in update() loop
}

void SequenceController::sendWaypointSequence(const std::vector<size_t>& waypointIndices) {
    // Send entire sequence of waypoint step positions to NUCLEO-H7S3L8
    // This allows NUCLEO-H7S3L8 to handle interpolation between waypoints
    if (!hardwareInterface_ || !hardwareInterface_->isConnected()) {
        return;
    }
    
    const auto& waypoints = motorControl_.getWaypoints();
    
    // Build sequence packet with all waypoint step positions
    // Format: [count][waypoint1_steps][waypoint2_steps]...
    // For now, send each waypoint individually
    // TODO: Could optimize to send all at once in a single packet
    
    for (size_t idx : waypointIndices) {
        if (idx >= waypoints.size()) continue;
        const Waypoint& wp = waypoints[idx];
        sendMoveCommand(wp.stepPositions);
    }
}

void SequenceController::streamTrajectoryPoint(const TrajectoryPoint& point) {
    // Convert motor angles to steps
    std::array<int32_t, 3> targetSteps;
    for (int i = 0; i < 3; ++i) {
        targetSteps[i] = motorControl_.angleToSteps(i, point.motorAngles[i]);
    }
    
    sendMoveCommand(targetSteps);
}

void SequenceController::sendMoveCommand(const std::array<int32_t, 3>& targetSteps) {
    // Always update motor control target (for simulation)
    // This ensures sequence works even without hardware connected
    std::array<float, 3> targetAngles;
    for (int i = 0; i < 3; ++i) {
        targetAngles[i] = motorControl_.stepsToAngle(i, targetSteps[i]);
    }
    motorControl_.setTargetAngles(targetAngles);
    
    // Send move command to hardware if connected
    if (hardwareInterface_ && hardwareInterface_->isConnected()) {
        hardwareInterface_->moveMotorsToSteps(targetSteps);
    }
}

bool SequenceController::checkPositionReached() {
    // Check if motors have reached target position
    if (config_.waitForCompletion) {
        return motorControl_.isAtTarget(config_.positionTolerance);
    }
    return true;  // If not waiting, assume reached
}

void SequenceController::onSequenceComplete() {
    currentSequenceIndex_ = 0;
    currentRepeat_++;
    stats_.completedRepeats = currentRepeat_;
    
    // Check if we should loop
    if (config_.loop && (config_.repeatCount == 0 || currentRepeat_ < config_.repeatCount)) {
        // Start next repeat
        trajectoryTime_ = 0.0f;
        startNextWaypoint();
    } else {
        // Sequence fully complete
        state_ = PlaybackState::Completed;
        currentTrajectory_.clear();
    }
}

void SequenceController::onRepeatComplete() {
    // Called when a single repeat completes
    // This is handled in onSequenceComplete()
}

float SequenceController::getProgress() const {
    if (currentSequence_.empty() || currentTrajectory_.empty()) {
        return 0.0f;
    }
    
    float waypointProgress = static_cast<float>(currentSequenceIndex_) / currentSequence_.size();
    
    if (!currentTrajectory_.empty() && currentTrajectory_.back().time > 0.0f) {
        float trajectoryProgress = trajectoryTime_ / currentTrajectory_.back().time;
        waypointProgress += (trajectoryProgress / currentSequence_.size());
    }
    
    return std::clamp(waypointProgress, 0.0f, 1.0f);
}

size_t SequenceController::getCurrentWaypointIndex() const {
    if (currentSequenceIndex_ < currentSequence_.size()) {
        return currentSequence_[currentSequenceIndex_];
    }
    return 0;
}

size_t SequenceController::getCurrentSequenceIndex() const {
    return currentSequenceIndex_;
}

int SequenceController::getCurrentRepeat() const {
    return currentRepeat_;
}

