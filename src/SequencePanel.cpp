#include "UIPanels.hpp"
#include "DeltaRobot.hpp"
#include "MotorControl.hpp"
#include "SequenceController.hpp"
#include <cmath>
#include <sstream>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

SequencePanel::SequencePanel(DeltaRobot& robot, MotorControl& motorControl, SequenceController* sequenceController)
    : robot_(robot), motorControl_(motorControl), sequenceController_(sequenceController) {
}

void SequencePanel::render() {
    if (!sequenceController_) {
        ImGui::Text("Sequence Controller not available");
        return;
    }
    
    ImGui::BeginChild("SequenceContent", ImVec2(0, 0), false, ImGuiWindowFlags_AlwaysVerticalScrollbar);
    
    auto& waypoints = const_cast<std::vector<Waypoint>&>(motorControl_.getWaypoints());
    
    ImGui::Text("Sequence Builder");
    ImGui::Separator();
    
    if (waypointSelected_.size() != waypoints.size()) {
        waypointSelected_.resize(waypoints.size(), false);
    }
    
    ImGui::Text("Select Waypoints (in order):");
    for (size_t i = 0; i < waypoints.size(); ++i) {
        bool selected = waypointSelected_[i];
        if (ImGui::Checkbox(waypoints[i].name.c_str(), &selected)) {
            waypointSelected_[i] = selected;
        }
    }
    
    if (ImGui::Button("Build Sequence")) {
        std::vector<size_t> sequence;
        for (size_t i = 0; i < waypointSelected_.size(); ++i) {
            if (waypointSelected_[i]) {
                sequence.push_back(i);
            }
        }
        sequenceController_->setSequence(sequence);
    }
    
    ImGui::Separator();
    ImGui::Text("Playback Controls");
    PlaybackState state = sequenceController_->getState();
    
    const char* stateStr = "Unknown";
    switch (state) {
        case PlaybackState::Stopped: stateStr = "Stopped"; break;
        case PlaybackState::Playing: stateStr = "Playing"; break;
        case PlaybackState::Paused: stateStr = "Paused"; break;
        case PlaybackState::Completed: stateStr = "Completed"; break;
    }
    ImGui::Text("State: %s", stateStr);
    
    if (state == PlaybackState::Stopped || state == PlaybackState::Completed) {
        if (ImGui::Button("Play")) {
            sequenceController_->play();
        }
    } else if (state == PlaybackState::Playing) {
        if (ImGui::Button("Pause")) {
            sequenceController_->pause();
        }
        ImGui::SameLine();
        if (ImGui::Button("Stop")) {
            sequenceController_->stop();
        }
    } else if (state == PlaybackState::Paused) {
        if (ImGui::Button("Resume")) {
            sequenceController_->resume();
        }
        ImGui::SameLine();
        if (ImGui::Button("Stop")) {
            sequenceController_->stop();
        }
    }
    
    ImGui::Separator();
    SequenceConfig config = sequenceController_->getConfig();
    bool configChanged = false;
    
    if (ImGui::Checkbox("Loop Sequence", &config.loop)) {
        configChanged = true;
    }
    
    if (config.loop) {
        if (ImGui::RadioButton("Infinite Loop", config.repeatCount == 0)) {
            config.repeatCount = 0;
            configChanged = true;
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("Repeat N Times", config.repeatCount > 0)) {
            config.repeatCount = 1;
            configChanged = true;
        }
        
        if (config.repeatCount > 0) {
            int repeatCount = config.repeatCount;
            if (ImGui::InputInt("Repeat Count", &repeatCount, 1, 10)) {
                config.repeatCount = std::max(1, repeatCount);
                configChanged = true;
            }
        }
    }
    
    if (configChanged) {
        sequenceController_->setConfig(config);
    }
    
    ImGui::Separator();
    ImGui::Text("Speed Control");
    ImGui::Separator();
    
    float speedMultiplier = config.speedMultiplier;
    if (ImGui::DragFloat("Speed Multiplier", &speedMultiplier, 0.1f, 0.1f, 5.0f, "%.2fx")) {
        speedMultiplier = std::clamp(speedMultiplier, 0.1f, 5.0f);
        config.speedMultiplier = speedMultiplier;
        sequenceController_->setConfig(config);
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Speed multiplier for sequence playback:\n1.0x = Normal speed\n2.0x = Double speed (faster)\n0.5x = Half speed (slower)\nAffects all waypoint durations");
    }
    
    // Speed presets
    ImGui::Text("Presets:");
    if (ImGui::Button("0.25x (Slow)")) {
        config.speedMultiplier = 0.25f;
        sequenceController_->setConfig(config);
    }
    ImGui::SameLine();
    if (ImGui::Button("0.5x (Half)")) {
        config.speedMultiplier = 0.5f;
        sequenceController_->setConfig(config);
    }
    ImGui::SameLine();
    if (ImGui::Button("1.0x (Normal)")) {
        config.speedMultiplier = 1.0f;
        sequenceController_->setConfig(config);
    }
    ImGui::SameLine();
    if (ImGui::Button("2.0x (Fast)")) {
        config.speedMultiplier = 2.0f;
        sequenceController_->setConfig(config);
    }
    
    // Show effective speed info
    ImGui::Text("Effective Speed: %.1f%%", speedMultiplier * 100.0f);
    if (speedMultiplier > 1.0f) {
        ImGui::TextColored(ImVec4(0.2f, 0.8f, 0.2f, 1.0f), "Faster than normal");
    } else if (speedMultiplier < 1.0f) {
        ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.2f, 1.0f), "Slower than normal");
    } else {
        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "Normal speed");
    }
    
    // Show real motor speeds
    MotorConfig motorCfg = motorControl_.getMotorConfig(0);
    DeltaRobotConfig robotCfg = robot_.getConfig();
    motorCfg.calculateDependentValues(robotCfg.upperArmLength);
    
    float effectiveMotorRPM = motorCfg.maxMotorRPM * speedMultiplier;
    float effectiveArmRPM = effectiveMotorRPM / motorCfg.gearRatio;
    float effectiveEndEffectorVel = motorCfg.maxEndEffectorVelocity * speedMultiplier;
    
    ImGui::Separator();
    ImGui::Text("Real Motor Speeds at %.2fx:", speedMultiplier);
    ImGui::BulletText("Motor RPM: %.0f", effectiveMotorRPM);
    ImGui::BulletText("Arm RPM: %.1f", effectiveArmRPM);
    ImGui::BulletText("End Effector Velocity: %.3f m/s (%.1f mm/s)", 
        effectiveEndEffectorVel, effectiveEndEffectorVel * 1000.0f);
    
    ImGui::Separator();
    ImGui::Text("Streaming Configuration");
    ImGui::Separator();
    int streamingMode = static_cast<int>(config.streamingMode);
    const char* modes[] = { "Batch Mode", "Real-Time Stream" };
    if (ImGui::Combo("Streaming Mode", &streamingMode, modes, 2)) {
        config.streamingMode = static_cast<StreamingMode>(streamingMode);
        sequenceController_->setConfig(config);
    }
    
    if (config.streamingMode == StreamingMode::RealTimeStream) {
        float updateRate = config.updateRate;
        if (ImGui::DragFloat("Update Rate (Hz)", &updateRate, 1.0f, 10.0f, 200.0f)) {
            config.updateRate = updateRate;
            sequenceController_->setConfig(config);
        }
    }
    
    ImGui::Separator();
    if (state == PlaybackState::Playing || state == PlaybackState::Paused) {
        float progress = sequenceController_->getProgress();
        ImGui::ProgressBar(progress);
        ImGui::Text("Progress: %.1f%%", progress * 100.0f);
        
        auto stats = sequenceController_->getStatistics();
        ImGui::Text("Repeat: %d / %s", 
            stats.completedRepeats,
            config.repeatCount == 0 ? "∞" : std::to_string(config.repeatCount).c_str());
        ImGui::Text("Elapsed Time: %.2f s", stats.elapsedTime);
        
        size_t currentWP = sequenceController_->getCurrentWaypointIndex();
        if (currentWP < waypoints.size()) {
            ImGui::Text("Current Waypoint: %s", waypoints[currentWP].name.c_str());
        }
    }
    
    ImGui::EndChild();
}

