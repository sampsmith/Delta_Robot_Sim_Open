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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "DeltaRobot.hpp"
#include "Renderer.hpp"

const int WINDOW_WIDTH = 1280;
const int WINDOW_HEIGHT = 720;

// Forward declaration
GLFWwindow* g_window = nullptr;

// Global state
DeltaRobot robot;
Renderer renderer;
bool showControls = true;
bool showRobotInfo = true;
bool showGrid = true;
bool showAxes = true;

// Mouse state
double lastMouseX = 0.0;
double lastMouseY = 0.0;
bool mouseLeftPressed = false;
bool mouseRightPressed = false;

// Virtual controller state
bool controllerButtonXPos = false;
bool controllerButtonXNeg = false;
bool controllerButtonYPos = false;
bool controllerButtonYNeg = false;
bool controllerButtonZPos = false;
bool controllerButtonZNeg = false;
float controllerSpeed = 0.01f;  // Movement speed in m/s

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
    
    ImGui::StyleColorsDark();
    
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");
    
    // Initial end effector position
    glm::vec3 endEffectorPos(0.0f, 0.0f, -0.35f);
    robot.setEndEffectorPosition(endEffectorPos);
    
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
        
        renderer.renderDeltaRobot(robot);
        
        // Update end effector position based on virtual controller
        static double lastTime = 0.0;
        double currentTime = glfwGetTime();
        double deltaTime = currentTime - lastTime;
        lastTime = currentTime;
        
        if (deltaTime > 0.0 && deltaTime < 0.1) {  // Limit delta time to prevent jumps
            glm::vec3 endEffectorPos = robot.getState().endEffectorPos;
            glm::vec3 delta(0.0f);
            
            if (controllerButtonXPos) delta.x += controllerSpeed * (float)deltaTime;
            if (controllerButtonXNeg) delta.x -= controllerSpeed * (float)deltaTime;
            if (controllerButtonYPos) delta.y += controllerSpeed * (float)deltaTime;
            if (controllerButtonYNeg) delta.y -= controllerSpeed * (float)deltaTime;
            if (controllerButtonZPos) delta.z += controllerSpeed * (float)deltaTime;
            if (controllerButtonZNeg) delta.z -= controllerSpeed * (float)deltaTime;
            
            if (glm::length(delta) > 0.0001f) {
                glm::vec3 newPos = endEffectorPos + delta;
                robot.setEndEffectorPosition(newPos);
            }
        }
        
        // Virtual Controller Panel
        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(200, 300), ImGuiCond_FirstUseEver);
        ImGui::Begin("Virtual Controller", nullptr, ImGuiWindowFlags_NoResize);
        
        ImGui::Text("End Effector Control");
        ImGui::Separator();
        
        // X Axis controls
        ImGui::Text("X Axis:");
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.8f, 0.2f, 0.2f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(1.0f, 0.3f, 0.3f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.6f, 0.1f, 0.1f, 1.0f));
        ImGui::Button("X+", ImVec2(80, 40));
        controllerButtonXPos = ImGui::IsItemActive();
        ImGui::PopStyleColor(3);
        
        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.8f, 0.2f, 0.2f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(1.0f, 0.3f, 0.3f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.6f, 0.1f, 0.1f, 1.0f));
        ImGui::Button("X-", ImVec2(80, 40));
        controllerButtonXNeg = ImGui::IsItemActive();
        ImGui::PopStyleColor(3);
        
        ImGui::Spacing();
        
        // Y Axis controls
        ImGui::Text("Y Axis:");
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.8f, 0.2f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 1.0f, 0.3f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.1f, 0.6f, 0.1f, 1.0f));
        ImGui::Button("Y+", ImVec2(80, 40));
        controllerButtonYPos = ImGui::IsItemActive();
        ImGui::PopStyleColor(3);
        
        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.8f, 0.2f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 1.0f, 0.3f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.1f, 0.6f, 0.1f, 1.0f));
        ImGui::Button("Y-", ImVec2(80, 40));
        controllerButtonYNeg = ImGui::IsItemActive();
        ImGui::PopStyleColor(3);
        
        ImGui::Spacing();
        
        // Z Axis controls
        ImGui::Text("Z Axis:");
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.2f, 0.8f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 0.3f, 1.0f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.1f, 0.1f, 0.6f, 1.0f));
        ImGui::Button("Z+", ImVec2(80, 40));
        controllerButtonZPos = ImGui::IsItemActive();
        ImGui::PopStyleColor(3);
        
        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.2f, 0.8f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 0.3f, 1.0f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.1f, 0.1f, 0.6f, 1.0f));
        ImGui::Button("Z-", ImVec2(80, 40));
        controllerButtonZNeg = ImGui::IsItemActive();
        ImGui::PopStyleColor(3);
        
        ImGui::Separator();
        ImGui::DragFloat("Speed", &controllerSpeed, 0.001f, 0.001f, 0.1f, "%.3f m/s");
        
        ImGui::End();
        
        // ImGui windows
        if (showControls) {
            ImGui::Begin("End Effector Control", &showControls);
            
            glm::vec3 pos = robot.getState().endEffectorPos;
            float posArray[3] = { pos.x, pos.y, pos.z };
            
            ImGui::Text("Position (m):");
            if (ImGui::DragFloat3("##Position", posArray, 0.01f, -1.0f, 1.0f)) {
                glm::vec3 newPos(posArray[0], posArray[1], posArray[2]);
                if (robot.setEndEffectorPosition(newPos)) {
                    endEffectorPos = newPos;
                }
            }
            
            ImGui::Separator();
            
            ImGui::Text("Quick Movements:");
            if (ImGui::Button("Centre")) {
                endEffectorPos = glm::vec3(0.0f, 0.0f, -0.35f);
                robot.setEndEffectorPosition(endEffectorPos);
            }
            ImGui::SameLine();
            if (ImGui::Button("Up")) {
                endEffectorPos.z += 0.05f;
                robot.setEndEffectorPosition(endEffectorPos);
            }
            ImGui::SameLine();
            if (ImGui::Button("Down")) {
                endEffectorPos.z -= 0.05f;
                robot.setEndEffectorPosition(endEffectorPos);
            }
            
            ImGui::Separator();
            
            bool isValid = robot.getState().isValid;
            if (isValid) {
                ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "Status: Valid Position");
                
                // Display motor angles
                ImGui::Separator();
                ImGui::Text("Motor Angles (degrees):");
                const auto& motorAngles = robot.getMotorAngles();
                for (int i = 0; i < 3; ++i) {
                    float angleDeg = motorAngles[i] * 180.0f / M_PI;
                    ImGui::Text("Motor %d: %.2f°", i + 1, angleDeg);
                }
            } else {
                ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "Status: Invalid Position");
            }
            
            ImGui::End();
        }
        
        if (showRobotInfo) {
            ImGui::Begin("Robot Configuration", &showRobotInfo);
            
            DeltaRobotConfig config = robot.getConfig();
            float configValues[4] = {
                config.baseRadius,
                config.effectorRadius,
                config.upperArmLength,
                config.lowerArmLength
            };
            const char* labels[] = { "Base Radius", "Effector Radius", "Upper Arm", "Lower Arm" };
            
            bool configChanged = false;
            for (int i = 0; i < 4; ++i) {
                if (ImGui::DragFloat(labels[i], &configValues[i], 0.01f, 0.01f, 1.0f)) {
                    configChanged = true;
                }
            }
            
            if (configChanged) {
                config.baseRadius = configValues[0];
                config.effectorRadius = configValues[1];
                config.upperArmLength = configValues[2];
                config.lowerArmLength = configValues[3];
                robot.setConfig(config);
            }
            
            ImGui::Separator();
            ImGui::Text("Workspace:");
            ImGui::Text("Max Reach: %.3f m", robot.getMaxReach());
            ImGui::Text("Height Range: %.3f to %.3f m", robot.getMinHeight(), robot.getMaxHeight());
            
            ImGui::End();
        }
        
        // Display options
        ImGui::Begin("Display Options");
        ImGui::Checkbox("Show Grid", &showGrid);
        ImGui::Checkbox("Show Axes", &showAxes);
        ImGui::End();
        
        // Camera controls info
        ImGui::Begin("Controls");
        ImGui::Text("Camera:");
        ImGui::BulletText("Left Click + Drag: Rotate");
        ImGui::BulletText("Right Click + Drag: Zoom");
        ImGui::BulletText("Scroll Wheel: Zoom");
        ImGui::Separator();
        ImGui::Text("End Effector:");
        ImGui::BulletText("Use Virtual Controller panel");
        ImGui::BulletText("Press and hold X/Y/Z buttons");
        ImGui::End();
        
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
    
    renderer.shutdown();
    glfwTerminate();
    
    return 0;
}

