#include "UIPanels.hpp"
#include "Renderer.hpp"

DisplayPanel::DisplayPanel(bool& showGrid, bool& showAxes, bool& showWorkspace, bool& showLabels, bool& wireframeMode)
    : showGrid_(showGrid), showAxes_(showAxes), showWorkspace_(showWorkspace), 
      showLabels_(showLabels), wireframeMode_(wireframeMode) {
}

void DisplayPanel::render() {
    ImGui::BeginChild("DisplayContent", ImVec2(0, 0), false);
    
    ImGui::Text("Display Options");
    ImGui::Separator();
    ImGui::Checkbox("Show Grid", &showGrid_);
    ImGui::Checkbox("Show Axes", &showAxes_);
    ImGui::Checkbox("Show Workspace Boundary", &showWorkspace_);
    ImGui::Checkbox("Show Position Labels", &showLabels_);
    ImGui::Checkbox("Wireframe Mode", &wireframeMode_);
    
    ImGui::Separator();
    ImGui::Spacing();
    
    // Camera Presets
    if (renderer_) {
        ImGui::Text("Camera Presets");
        ImGui::Separator();
        if (ImGui::Button("Top View")) {
            renderer_->setCameraPosition(glm::vec3(0.0f, 0.0f, 1.0f));
            renderer_->setCameraTarget(glm::vec3(0.0f, 0.0f, -0.2f));
        }
        ImGui::SameLine();
        if (ImGui::Button("Front View")) {
            renderer_->setCameraPosition(glm::vec3(0.0f, -1.0f, 0.0f));
            renderer_->setCameraTarget(glm::vec3(0.0f, 0.0f, -0.2f));
        }
        if (ImGui::Button("Side View")) {
            renderer_->setCameraPosition(glm::vec3(1.0f, 0.0f, 0.0f));
            renderer_->setCameraTarget(glm::vec3(0.0f, 0.0f, -0.2f));
        }
        ImGui::SameLine();
        if (ImGui::Button("Isometric")) {
            renderer_->setCameraPosition(glm::vec3(0.7f, 0.7f, 0.5f));
            renderer_->setCameraTarget(glm::vec3(0.0f, 0.0f, -0.2f));
        }
    }
    
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

