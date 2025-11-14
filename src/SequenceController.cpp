#include "SequenceController.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

SequenceController::SequenceController(MotorControl& motorControl)
    : motorControl_(motorControl)
    , currentSequenceIndex_(0)
    , currentRepeat_(0)
    , state_(PlaybackState::Stopped)
    , currentTrajectoryIndex_(0)
    , trajectoryTime_(0.0f)
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
    sequenceStartTime_ = 0.0f;  // Will be set on first update
    
    stats_.completedRepeats = 0;
    stats_.elapsedTime = 0.0f;
    
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
    
    if (!currentTrajectory_.empty()) {
        while (currentTrajectoryIndex_ < currentTrajectory_.size() - 1 &&
               trajectoryTime_ >= currentTrajectory_[currentTrajectoryIndex_ + 1].time) {
            
            const TrajectoryPoint& point = currentTrajectory_[currentTrajectoryIndex_ + 1];
            motorControl_.setTargetAngles(point.motorAngles);
            currentTrajectoryIndex_++;
        }
        
        if (currentTrajectoryIndex_ >= currentTrajectory_.size() - 1) {
            if (!currentTrajectory_.empty()) {
                motorControl_.setTargetAngles(currentTrajectory_.back().motorAngles);
            }
            
            if (config_.waitForCompletion) {
                if (checkPositionReached()) {
                    currentTrajectory_.clear();
                    currentSequenceIndex_++;
                    trajectoryTime_ = 0.0f;
                }
            } else if (trajectoryTime_ >= currentTrajectory_.back().time) {
                currentTrajectory_.clear();
                currentSequenceIndex_++;
                trajectoryTime_ = 0.0f;
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

