#pragma once

#include <imgui.h>
#include <glm/glm.hpp>
#include <vector>
#include <string>
#include <cstring>

// Forward declarations
class DeltaRobot;
class MotorControl;
class HardwareInterface;
class SequenceController;
struct Waypoint;
struct MotorConfig;
struct DeltaRobotConfig;

// Include necessary headers for enums/structs
#include "SequenceController.hpp"
#include "MotorControl.hpp"
#include "DeltaRobot.hpp"

// Base class for all UI panels
class UIPanel {
public:
    virtual ~UIPanel() = default;
    virtual void render() = 0;
    virtual const char* getName() const = 0;
};

// Control Panel - Virtual controller and motor status
class ControlPanel : public UIPanel {
public:
    ControlPanel(DeltaRobot& robot, MotorControl& motorControl);
    void render() override;
    const char* getName() const override { return "Control"; }
    
    // Controller state
    bool controllerButtonXPos = false;
    bool controllerButtonXNeg = false;
    bool controllerButtonYPos = false;
    bool controllerButtonYNeg = false;
    bool controllerButtonZPos = false;
    bool controllerButtonZNeg = false;
    float controllerSpeed = 0.01f;
    
    // UI scale
    float uiScale = 1.0f;
    
private:
    DeltaRobot& robot_;
    MotorControl& motorControl_;
};

// Robot Setup Panel - Dimensions, motors, workspace
class RobotSetupPanel : public UIPanel {
public:
    RobotSetupPanel(DeltaRobot& robot, MotorControl& motorControl);
    void render() override;
    const char* getName() const override { return "Robot Setup"; }
    
private:
    DeltaRobot& robot_;
    MotorControl& motorControl_;
    
    void renderDimensionsTab();
    void renderMotorsTab();
    void renderWorkspaceTab();
    
    static int setupSubTab_;
};

// Waypoint Panel - Save and manage waypoints
class WaypointPanel : public UIPanel {
public:
    WaypointPanel(DeltaRobot& robot, MotorControl& motorControl);
    void render() override;
    const char* getName() const override { return "Waypoints"; }
    
private:
    DeltaRobot& robot_;
    MotorControl& motorControl_;
};

// Sequence Panel - Build and play sequences
class SequencePanel : public UIPanel {
public:
    SequencePanel(DeltaRobot& robot, MotorControl& motorControl, SequenceController* sequenceController);
    void render() override;
    const char* getName() const override { return "Sequence"; }
    
private:
    DeltaRobot& robot_;
    MotorControl& motorControl_;
    SequenceController* sequenceController_;
    
    std::vector<bool> waypointSelected_;
};

// Hardware Panel - Connection and homing
class HardwarePanel : public UIPanel {
public:
    HardwarePanel(HardwareInterface* hardwareInterface, MotorControl& motorControl);
    void render() override;
    const char* getName() const override { return "Hardware"; }
    
    // Connection state
    bool& hardwareConnected() { return hardwareConnected_; }
    bool& motorsEnabled() { return motorsEnabled_; }
    
private:
    HardwareInterface* hardwareInterface_;
    MotorControl& motorControl_;
    bool hardwareConnected_ = false;
    bool motorsEnabled_ = false;
    
    char ipAddress_[64];
    int port_;
};

// Display Panel - Display options
class DisplayPanel : public UIPanel {
public:
    DisplayPanel(bool& showGrid, bool& showAxes);
    void render() override;
    const char* getName() const override { return "Display"; }
    
private:
    bool& showGrid_;
    bool& showAxes_;
};

// UI Manager - Coordinates all panels
class UIManager {
public:
    UIManager(DeltaRobot& robot, MotorControl& motorControl, 
              HardwareInterface* hardwareInterface, SequenceController* sequenceController,
              bool& showGrid, bool& showAxes);
    
    void render(float deltaTime);
    
    // Controller state access
    bool getControllerButtonXPos() const { return controlPanel_.controllerButtonXPos; }
    bool getControllerButtonXNeg() const { return controlPanel_.controllerButtonXNeg; }
    bool getControllerButtonYPos() const { return controlPanel_.controllerButtonYPos; }
    bool getControllerButtonYNeg() const { return controlPanel_.controllerButtonYNeg; }
    bool getControllerButtonZPos() const { return controlPanel_.controllerButtonZPos; }
    bool getControllerButtonZNeg() const { return controlPanel_.controllerButtonZNeg; }
    float getControllerSpeed() const { return controlPanel_.controllerSpeed; }
    
    // Hardware state access
    bool& hardwareConnected() { return hardwarePanel_.hardwareConnected(); }
    bool& motorsEnabled() { return hardwarePanel_.motorsEnabled(); }
    
    // UI scale
    void setUIScale(float scale) { controlPanel_.uiScale = scale; }
    
private:
    ControlPanel controlPanel_;
    RobotSetupPanel robotSetupPanel_;
    WaypointPanel waypointPanel_;
    SequencePanel sequencePanel_;
    HardwarePanel hardwarePanel_;
    DisplayPanel displayPanel_;
    
    bool showMainControlPanel_ = true;
    
    void renderMainWindow();
};

