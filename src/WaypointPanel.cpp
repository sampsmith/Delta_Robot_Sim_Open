#include "UIPanels.hpp"
#include "DeltaRobot.hpp"
#include "MotorControl.hpp"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

WaypointPanel::WaypointPanel(DeltaRobot& robot, MotorControl& motorControl)
    : robot_(robot), motorControl_(motorControl) {
}

void WaypointPanel::render() {
    ImGui::BeginChild("WaypointsContent", ImVec2(0, 0), false, ImGuiWindowFlags_AlwaysVerticalScrollbar);
    
    auto& waypoints = const_cast<std::vector<Waypoint>&>(motorControl_.getWaypoints());
    
    if (ImGui::Button("Save Current Position")) {
        Waypoint wp;
        wp.name = "Waypoint " + std::to_string(waypoints.size() + 1);
        wp.endEffectorPos = robot_.getState().endEffectorPos;
        wp.motorAngles = robot_.getMotorAngles();
        wp.duration = 2.0f;
        motorControl_.addWaypoint(wp);
    }
    
    ImGui::Separator();
    ImGui::Text("Saved Waypoints (%zu):", waypoints.size());
    
    for (size_t i = 0; i < waypoints.size(); ++i) {
        ImGui::PushID((int)i);
        ImGui::Text("%zu. %s", i + 1, waypoints[i].name.c_str());
        ImGui::SameLine();
        if (ImGui::SmallButton("Go")) {
            robot_.setEndEffectorPosition(waypoints[i].endEffectorPos);
            motorControl_.setTargetAngles(waypoints[i].motorAngles);
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Delete")) {
            motorControl_.removeWaypoint(i);
        }
        
        if (ImGui::TreeNode((void*)(intptr_t)i, "Details")) {
            float pos[3] = {waypoints[i].endEffectorPos.x, 
                           waypoints[i].endEffectorPos.y, 
                           waypoints[i].endEffectorPos.z};
            ImGui::Text("Position: (%.3f, %.3f, %.3f)", pos[0], pos[1], pos[2]);
            ImGui::Text("Motor Angles: (%.2f°, %.2f°, %.2f°)", 
                waypoints[i].motorAngles[0] * 180.0f / M_PI,
                waypoints[i].motorAngles[1] * 180.0f / M_PI,
                waypoints[i].motorAngles[2] * 180.0f / M_PI);
            ImGui::DragFloat("Duration", &waypoints[i].duration, 0.1f, 0.1f, 10.0f);
            ImGui::TreePop();
        }
        ImGui::PopID();
    }
    
    if (!waypoints.empty()) {
        ImGui::Separator();
        if (ImGui::Button("Clear All")) {
            motorControl_.clearWaypoints();
        }
    }
    
    ImGui::EndChild();
}

