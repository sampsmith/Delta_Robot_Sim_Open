#include "UIPanels.hpp"

DisplayPanel::DisplayPanel(bool& showGrid, bool& showAxes)
    : showGrid_(showGrid), showAxes_(showAxes) {
}

void DisplayPanel::render() {
    ImGui::BeginChild("DisplayContent", ImVec2(0, 0), false);
    
    ImGui::Text("Display Options");
    ImGui::Separator();
    ImGui::Checkbox("Show Grid", &showGrid_);
    ImGui::Checkbox("Show Axes", &showAxes_);
    
    ImGui::Separator();
    ImGui::Spacing();
    ImGui::Text("Controls");
    ImGui::Separator();
    ImGui::Text("Camera:");
    ImGui::BulletText("Left Click + Drag: Rotate");
    ImGui::BulletText("Right Click + Drag: Zoom");
    ImGui::BulletText("Scroll Wheel: Zoom");
    ImGui::Separator();
    ImGui::Text("End Effector:");
    ImGui::BulletText("Use Virtual Controller");
    ImGui::BulletText("Press and hold X/Y/Z buttons");
    
    ImGui::EndChild();
}

