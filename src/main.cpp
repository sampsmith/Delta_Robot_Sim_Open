#include <GLFW/glfw3.h>
#ifdef __linux__
#include <GL/gl.h>
#endif
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <glm/glm.hpp>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <cfloat>
#include <memory>
#include <string>
#include <vector>
#include <sstream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "DeltaRobot.hpp"
#include "Renderer.hpp"
#include "MotorControl.hpp"
#include "HardwareInterface.hpp"
#include "ADSInterface.hpp"
#include "SequenceController.hpp"
#include "UIPanels.hpp"

const int WINDOW_WIDTH = 1280;
const int WINDOW_HEIGHT = 720;

// Forward declaration
GLFWwindow* g_window = nullptr;

// Global state
DeltaRobot robot;
Renderer renderer;
MotorControl motorControl;
std::unique_ptr<HardwareInterface> hardwareInterface;
SequenceController* sequenceController = nullptr;  // Will be initialized after hardwareInterface
UIManager* uiManager = nullptr;  // Will be initialized after sequenceController

// Display state
bool showGrid = true;
bool showAxes = true;
bool showWorkspace = false;
bool showLabels = false;
bool wireframeMode = false;

// Mouse state
double lastMouseX = 0.0;
double lastMouseY = 0.0;
bool mouseLeftPressed = false;
bool mouseRightPressed = false;

void framebufferSizeCallback(GLFWwindow* window, int width, int height) {
    renderer.setViewport(width, height);
}

void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        mouseLeftPressed = (action == GLFW_PRESS);
    }
    if (button == GLFW_MOUSE_BUTTON_RIGHT) {
        mouseRightPressed = (action == GLFW_PRESS);
    }
}

void cursorPosCallback(GLFWwindow* window, double xpos, double ypos) {
    if (mouseLeftPressed || mouseRightPressed) {
        // Camera controls
        double deltaX = xpos - lastMouseX;
        double deltaY = ypos - lastMouseY;
        renderer.processMouseInput(deltaX, deltaY, mouseLeftPressed, mouseRightPressed);
    }
    lastMouseX = xpos;
    lastMouseY = ypos;
}

void scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
    // Zoom with scroll wheel
    glm::vec3 camPos = renderer.getCameraPosition();
    glm::vec3 camTarget = renderer.getCameraTarget();
    glm::vec3 direction = glm::normalize(camPos - camTarget);
    float distance = glm::length(camPos - camTarget);
    
    distance += yoffset * 0.1f;
    distance = std::max(0.1f, std::min(5.0f, distance));
    
    renderer.setCameraPosition(camTarget + direction * distance);
}

int main() {
    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }
    
    // Configure GLFW - use compatibility profile for immediate mode OpenGL
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
    
    // Create window
    GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, 
                                         "Delta Robot Visualisation", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    
    g_window = window;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync
    
    // Set callbacks
    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetCursorPosCallback(window, cursorPosCallback);
    glfwSetScrollCallback(window, scrollCallback);
    
    // Initialize renderer
    if (!renderer.initialize(window)) {
        std::cerr << "Failed to initialize renderer" << std::endl;
        glfwTerminate();
        return -1;
    }
    
    renderer.setViewport(WINDOW_WIDTH, WINDOW_HEIGHT);
    
    // Initialize ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
    
    // Scale ImGui for high-DPI displays
    // Get monitor DPI scale
    float xscale, yscale;
    glfwGetWindowContentScale(window, &xscale, &yscale);
    float uiScale = (xscale + yscale) / 2.0f;
    if (uiScale > 1.0f) {
        ImGui::GetStyle().ScaleAllSizes(uiScale);
        ImGui::GetIO().FontGlobalScale = uiScale;
    }
    
    ImGui::StyleColorsDark();
    
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");
    
    // Initial end effector position
    glm::vec3 endEffectorPos(0.0f, 0.0f, -0.35f);
    robot.setEndEffectorPosition(endEffectorPos);
    
    // Initialize motor control with default servo configuration (angle-based)
    MotorConfig defaultConfig;
    defaultConfig.maxAngularVelocity = 5.0f;
    defaultConfig.maxAngularAcceleration = 20.0f;
    defaultConfig.homeOffset = 0.0f;
    defaultConfig.inverted = false;
    
    for (int i = 0; i < 3; ++i) {
        motorControl.setMotorConfig(i, defaultConfig);
    }

    // Initialize ADS hardware interface for Beckhoff TwinCAT
    hardwareInterface = std::make_unique<ADSInterface>();
    
    // Initialize sequence controller
    sequenceController = new SequenceController(motorControl);
    
    // Initialize UI Manager
    uiManager = new UIManager(robot, motorControl, hardwareInterface.get(), sequenceController, 
                              showGrid, showAxes, showWorkspace, showLabels, wireframeMode);
    if (uiScale > 1.0f) {
        uiManager->setUIScale(uiScale);
    }
    
    // Set renderer reference for display panel camera presets
    uiManager->setRendererForDisplayPanel(&renderer);
    
    // Sync initial motor angles
    motorControl.setTargetAngles(robot.getMotorAngles());
    
    // Main loop
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        
        // Start ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        
        // Render 3D scene
        renderer.beginFrame();
        
        if (showGrid) {
            renderer.renderGrid(0.5f, 0.05f);
        }
        
        if (showAxes) {
            renderer.renderCoordinateAxes(0.15f);
        }
        
        // Render workspace boundary if enabled
        if (showWorkspace) {
            renderer.renderWorkspaceBoundary(robot);
        }
        
        // Render trajectory path if sequence is playing
        if (sequenceController && sequenceController->getState() == PlaybackState::Playing) {
            const auto& waypoints = motorControl.getWaypoints();
            const auto& sequence = sequenceController->getSequence();
            if (!sequence.empty()) {
                std::vector<glm::vec3> waypointPositions;
                for (size_t idx : sequence) {
                    if (idx < waypoints.size()) {
                        waypointPositions.push_back(waypoints[idx].endEffectorPos);
                    }
                }
                if (!waypointPositions.empty()) {
                    renderer.renderTrajectoryPath(waypointPositions, 
                                                 robot.getState().endEffectorPos, 
                                                 true);
                }
            }
        }
        
        renderer.renderDeltaRobot(robot);
        
        // Update end effector position based on virtual controller
        static double lastTime = 0.0;
        double currentTime = glfwGetTime();
        double deltaTime = currentTime - lastTime;
        lastTime = currentTime;
        
        if (deltaTime > 0.0 && deltaTime < 0.1 && uiManager) {  // Limit delta time to prevent jumps
            glm::vec3 endEffectorPos = robot.getState().endEffectorPos;
            glm::vec3 delta(0.0f);
            
            float controllerSpeed = uiManager->getControllerSpeed();
            if (uiManager->getControllerButtonXPos()) delta.x += controllerSpeed * (float)deltaTime;
            if (uiManager->getControllerButtonXNeg()) delta.x -= controllerSpeed * (float)deltaTime;
            if (uiManager->getControllerButtonYPos()) delta.y += controllerSpeed * (float)deltaTime;
            if (uiManager->getControllerButtonYNeg()) delta.y -= controllerSpeed * (float)deltaTime;
            if (uiManager->getControllerButtonZPos()) delta.z += controllerSpeed * (float)deltaTime;
            if (uiManager->getControllerButtonZNeg()) delta.z -= controllerSpeed * (float)deltaTime;
            
            if (glm::length(delta) > 0.0001f) {
                glm::vec3 newPos = endEffectorPos + delta;
                // Clamp to workspace to prevent stretching
                newPos = robot.clampToWorkspace(newPos);
                if (robot.setEndEffectorPosition(newPos)) {
                    // Update motor control target only if position was successfully set
                    motorControl.setTargetAngles(robot.getMotorAngles());
                }
            }
        }
        
        // Update motor control
        motorControl.update(deltaTime);
        
        // Stream joint angles to ADS/Beckhoff PLC in real-time
        if (hardwareInterface && hardwareInterface->isConnected()) {
            ADSInterface* adsInterface = dynamic_cast<ADSInterface*>(hardwareInterface.get());
            if (adsInterface) {
                // Get current motor angles from motor control
                const auto& motorStates = motorControl.getMotorStates();
                std::array<float, 3> jointAngles = {
                    motorStates[0].currentAngle,
                    motorStates[1].currentAngle,
                    motorStates[2].currentAngle
                };
                // Send to PLC (ADS handles synchronization internally)
                adsInterface->sendJointAngles(jointAngles);
            }
        }
        
        // Update sequence controller (handles playback and streaming)
        if (sequenceController) {
            sequenceController->update(deltaTime);
        }
        
        // Sync robot visual position from motor control during sequence playback
        // Interpolate between waypoint positions based on trajectory progress
        if (sequenceController && sequenceController->getState() == PlaybackState::Playing) {
            const auto& waypoints = motorControl.getWaypoints();
            const auto& sequence = sequenceController->getSequence();
            size_t currentSeqIdx = sequenceController->getCurrentSequenceIndex();
            
            if (currentSeqIdx < sequence.size()) {
                size_t waypointIdx = sequence[currentSeqIdx];
                if (waypointIdx < waypoints.size()) {
                    const Waypoint& targetWP = waypoints[waypointIdx];
                    
                    // Get start position (from previous waypoint or current robot position)
                    glm::vec3 startPos = robot.getState().endEffectorPos;
                    if (currentSeqIdx > 0) {
                        size_t prevIdx = sequence[currentSeqIdx - 1];
                        if (prevIdx < waypoints.size()) {
                            startPos = waypoints[prevIdx].endEffectorPos;
                        }
                    }
                    
                    // Get current motor angles to calculate interpolation factor
                    const auto& motorStates = motorControl.getMotorStates();
                    std::array<float, 3> currentAngles = {
                        motorStates[0].currentAngle,
                        motorStates[1].currentAngle,
                        motorStates[2].currentAngle
                    };
                    
                    // Calculate interpolation factor based on motor angle progress
                    float totalAngleDiff = 0.0f;
                    float remainingAngleDiff = 0.0f;
                    for (int i = 0; i < 3; ++i) {
                        float targetAngle = targetWP.motorAngles[i];
                        float startAngle = (currentSeqIdx > 0 && sequence[currentSeqIdx - 1] < waypoints.size()) 
                            ? waypoints[sequence[currentSeqIdx - 1]].motorAngles[i]
                            : currentAngles[i];
                        
                        float totalDiff = std::abs(targetAngle - startAngle);
                        float remainingDiff = std::abs(targetAngle - currentAngles[i]);
                        totalAngleDiff += totalDiff;
                        remainingAngleDiff += remainingDiff;
                    }
                    
                    // Calculate interpolation factor (0 = at start, 1 = at target)
                    float t = 0.0f;
                    if (totalAngleDiff > 0.001f) {
                        t = 1.0f - (remainingAngleDiff / totalAngleDiff);
                        t = std::clamp(t, 0.0f, 1.0f);
                    }
                    
                    // Interpolate position
                    glm::vec3 interpolatedPos = startPos + (targetWP.endEffectorPos - startPos) * t;
                    // Clamp to workspace to prevent stretching
                    interpolatedPos = robot.clampToWorkspace(interpolatedPos);
                    robot.setEndEffectorPosition(interpolatedPos);
                }
            }
        }
        
        // Render UI using UIManager
        if (uiManager) {
            uiManager->render((float)deltaTime);
        }
        
        // Render ImGui
        ImGui::Render();
        
        // Render ImGui draw data
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        
        glfwSwapBuffers(window);
    }
    
    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    
    // Cleanup
    delete uiManager;
    delete sequenceController;
    
    renderer.shutdown();
    glfwTerminate();
    
    return 0;
}

