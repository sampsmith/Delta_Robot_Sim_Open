#pragma once

#include "MotorControl.hpp"
#include "HardwareInterface.hpp"
#include <vector>
#include <string>
#include <memory>

// Sequence playback controller for waypoint sequences
// Handles looping, real-time streaming, and playback control

enum class PlaybackState {
    Stopped,
    Playing,
    Paused,
    Completed
};

enum class StreamingMode {
    TrajectoryBatch,    // Send entire trajectory at once (batch mode)
    RealTimeStream      // Send trajectory points in real-time as they're executed
};

struct SequenceConfig {
    bool loop = false;                    // Loop the sequence
    int repeatCount = 1;                  // Number of times to repeat (0 = infinite loop)
    StreamingMode streamingMode = StreamingMode::RealTimeStream;
    float updateRate = 100.0f;            // Hz - how often to send packets in real-time mode
    bool waitForCompletion = true;        // Wait for motors to reach target before next command
    float positionTolerance = 0.001f;     // Tolerance for position reached
    float speedMultiplier = 1.0f;         // Speed multiplier (0.1x to 5.0x) - affects all waypoint durations
};

class SequenceController {
public:
    SequenceController(MotorControl& motorControl, HardwareInterface* hardwareInterface);
    
    // Sequence management
    void setSequence(const std::vector<size_t>& waypointIndices);
    void clearSequence();
    const std::vector<size_t>& getSequence() const { return currentSequence_; }
    
    // Playback control
    bool play();
    bool pause();
    bool stop();
    bool resume();
    PlaybackState getState() const { return state_; }
    
    // Configuration
    void setConfig(const SequenceConfig& config) { config_ = config; }
    const SequenceConfig& getConfig() const { return config_; }
    
    // Loop control
    void setLoop(bool loop) { config_.loop = loop; }
    void setRepeatCount(int count) { config_.repeatCount = count; }
    
    // Update (call every frame)
    void update(float deltaTime);
    
    // Progress information
    float getProgress() const;  // 0.0 to 1.0
    size_t getCurrentWaypointIndex() const;
    size_t getCurrentSequenceIndex() const;
    int getCurrentRepeat() const;
    
    // Real-time streaming
    void setStreamingMode(StreamingMode mode) { config_.streamingMode = mode; }
    bool isStreaming() const { return state_ == PlaybackState::Playing && config_.streamingMode == StreamingMode::RealTimeStream; }
    
    // Statistics
    struct Statistics {
        int totalRepeats = 0;
        int completedRepeats = 0;
        float totalTime = 0.0f;
        float elapsedTime = 0.0f;
    };
    const Statistics& getStatistics() const { return stats_; }
    
private:
    MotorControl& motorControl_;
    HardwareInterface* hardwareInterface_;
    
    // Sequence
    std::vector<size_t> currentSequence_;
    size_t currentSequenceIndex_;
    int currentRepeat_;
    
    // Playback state
    PlaybackState state_;
    SequenceConfig config_;
    
    // Trajectory execution
    std::vector<TrajectoryPoint> currentTrajectory_;
    size_t currentTrajectoryIndex_;
    float trajectoryTime_;
    float lastStreamTime_;
    
    // Statistics
    Statistics stats_;
    float sequenceStartTime_;
    
    // Internal methods
    void startNextWaypoint();
    void streamTrajectoryPoint(const TrajectoryPoint& point);
    void sendMoveCommand(const std::array<int32_t, 3>& targetSteps);
    void sendWaypointSequence(const std::vector<size_t>& waypointIndices);
    bool checkPositionReached();
    void onSequenceComplete();
    void onRepeatComplete();
};

