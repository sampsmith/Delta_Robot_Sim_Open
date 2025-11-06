#include "UIPanels.hpp"
#include "DeltaRobot.hpp"
#include "MotorControl.hpp"
#include "HardwareInterface.hpp"
#include "SequenceController.hpp"
#include <GLFW/glfw3.h>
#include <cfloat>
#include <algorithm>

UIManager::UIManager(DeltaRobot& robot, MotorControl& motorControl, 
                     HardwareInterface* hardwareInterface, SequenceController* sequenceController,
                     bool& showGrid, bool& showAxes, bool& showWorkspace, bool& showLabels, bool& wireframeMode)
    : controlPanel_(robot, motorControl)
    , robotSetupPanel_(robot, motorControl)
    , waypointPanel_(robot, motorControl)
    , sequencePanel_(robot, motorControl, sequenceController)
    , hardwarePanel_(hardwareInterface, motorControl)
    , displayPanel_(showGrid, showAxes, showWorkspace, showLabels, wireframeMode) {
}

void UIManager::render(float deltaTime) {
    if (!showMainControlPanel_) return;
    
    // Get actual window size for proper scaling on high-res displays
    GLFWwindow* window = glfwGetCurrentContext();
    if (!window) return;
    
    int windowWidth, windowHeight;
    glfwGetWindowSize(window, &windowWidth, &windowHeight);
    
    // Calculate panel size: 30% of window width, minimum 500px, maximum 1200px
    float panelWidth = windowWidth * 0.30f;
    if (panelWidth < 500.0f) panelWidth = 500.0f;
    if (panelWidth > 1200.0f) panelWidth = 1200.0f;
    
    // Position on the right side, full height, resizable
    ImGui::SetNextWindowPos(ImVec2(windowWidth - panelWidth, 0), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(panelWidth, windowHeight), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSizeConstraints(ImVec2(400, 400), ImVec2(FLT_MAX, FLT_MAX));
    
    ImGui::Begin("Control Panel", &showMainControlPanel_, ImGuiWindowFlags_None);
    
    // Main tabs for different sections
    if (ImGui::BeginTabBar("MainTabs")) {
        if (ImGui::BeginTabItem(controlPanel_.getName())) {
            controlPanel_.render();
            ImGui::EndTabItem();
        }
        
        if (ImGui::BeginTabItem(robotSetupPanel_.getName())) {
            robotSetupPanel_.render();
            ImGui::EndTabItem();
        }
        
        if (ImGui::BeginTabItem(waypointPanel_.getName())) {
            waypointPanel_.render();
            ImGui::EndTabItem();
        }
        
        if (ImGui::BeginTabItem(sequencePanel_.getName())) {
            sequencePanel_.render();
            ImGui::EndTabItem();
        }
        
        if (ImGui::BeginTabItem(hardwarePanel_.getName())) {
            hardwarePanel_.render();
            ImGui::EndTabItem();
        }
        
        if (ImGui::BeginTabItem(displayPanel_.getName())) {
            displayPanel_.render();
            ImGui::EndTabItem();
        }
        
        ImGui::EndTabBar();
    }
    
    ImGui::End();
}

