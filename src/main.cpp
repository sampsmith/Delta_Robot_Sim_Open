#include "DeltaRobot.hpp"
#include "Renderer.hpp"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <GLFW/glfw3.h>
#include <iostream>
#include <cmath>

static bool mouse_dragging = false;
static double last_mouse_x = 0.0;
static double last_mouse_y = 0.0;
static Renderer* g_renderer = nullptr;

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS) {
            // Check if mouse is over ImGui window
            ImGuiIO& io = ImGui::GetIO();
            if (!io.WantCaptureMouse) {
                mouse_dragging = true;
                glfwGetCursorPos(window, &last_mouse_x, &last_mouse_y);
            }
        } else if (action == GLFW_RELEASE) {
            mouse_dragging = false;
        }
    }
}

void cursor_position_callback(GLFWwindow* window, double xpos, double ypos) {
    if (mouse_dragging && g_renderer) {
        double dx = xpos - last_mouse_x;
        double dy = ypos - last_mouse_y;
        g_renderer->getCamera().rotate(dx, dy);
        last_mouse_x = xpos;
        last_mouse_y = ypos;
    }
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    ImGuiIO& io = ImGui::GetIO();
    if (!io.WantCaptureMouse && g_renderer) {
        g_renderer->getCamera().zoom(yoffset);
    }
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    if (g_renderer) {
        g_renderer->resize(width, height);
    }
}

int main(int argc, char** argv) {
    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_SAMPLES, 4);  // 4x MSAA

    GLFWwindow* window = glfwCreateWindow(1600, 900, "Delta Robot Simulator", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);  // Enable vsync

    // Set callbacks
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetCursorPosCallback(window, cursor_position_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    // Initialize ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    // Initialize renderer
    Renderer renderer;
    g_renderer = &renderer;
    renderer.initialize();

    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    renderer.resize(width, height);

    // Initialize delta robot with default parameters
    DeltaRobotParams params;
    params.base_radius = 200.0f;
    params.platform_radius = 50.0f;
    params.upper_arm_length = 300.0f;
    params.lower_arm_length = 600.0f;

    DeltaRobot robot(params);

    // UI state
    Vec3 target_position = robot.getEndEffectorPosition();
    bool ik_valid = true;

    // Main loop
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // Start ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Control Panel
        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(400, 600), ImGuiCond_FirstUseEver);
        ImGui::Begin("Delta Robot Control");

        ImGui::Text("Application: %.1f FPS", io.Framerate);
        ImGui::Separator();

        ImGui::Text("Robot Parameters");
        ImGui::Text("Base Radius: %.1f mm", params.base_radius);
        ImGui::Text("Platform Radius: %.1f mm", params.platform_radius);
        ImGui::Text("Upper Arm: %.1f mm", params.upper_arm_length);
        ImGui::Text("Lower Arm: %.1f mm", params.lower_arm_length);
        ImGui::Separator();

        ImGui::Text("End Effector Position");
        float pos[3] = {target_position.x, target_position.y, target_position.z};
        
        if (ImGui::SliderFloat("X (mm)", &pos[0], -300.0f, 300.0f)) {
            target_position.x = pos[0];
            ik_valid = robot.setTargetPosition(target_position);
        }
        
        if (ImGui::SliderFloat("Y (mm)", &pos[1], -300.0f, 300.0f)) {
            target_position.y = pos[1];
            ik_valid = robot.setTargetPosition(target_position);
        }
        
        if (ImGui::SliderFloat("Z (mm)", &pos[2], -900.0f, -200.0f)) {
            target_position.z = pos[2];
            ik_valid = robot.setTargetPosition(target_position);
        }

        ImGui::Separator();

        if (ik_valid) {
            ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "IK: VALID");
        } else {
            ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "IK: OUT OF REACH");
        }

        ImGui::Separator();
        ImGui::Text("Joint Angles (degrees)");
        auto angles = robot.getJointAngles();
        ImGui::Text("Joint 1: %.2f°", angles[0] * 180.0f / M_PI);
        ImGui::Text("Joint 2: %.2f°", angles[1] * 180.0f / M_PI);
        ImGui::Text("Joint 3: %.2f°", angles[2] * 180.0f / M_PI);

        ImGui::Separator();
        ImGui::Text("Camera Controls");
        ImGui::Text("Left Mouse: Rotate");
        ImGui::Text("Scroll: Zoom");
        
        if (ImGui::Button("Reset Camera")) {
            renderer.getCamera().reset();
        }

        ImGui::Separator();
        ImGui::Text("Presets");
        
        if (ImGui::Button("Home Position")) {
            target_position = Vec3(0.0f, 0.0f, -500.0f);
            ik_valid = robot.setTargetPosition(target_position);
        }
        
        ImGui::SameLine();
        if (ImGui::Button("Bottom")) {
            target_position = Vec3(0.0f, 0.0f, -850.0f);
            ik_valid = robot.setTargetPosition(target_position);
        }
        
        if (ImGui::Button("Top")) {
            target_position = Vec3(0.0f, 0.0f, -250.0f);
            ik_valid = robot.setTargetPosition(target_position);
        }
        
        ImGui::SameLine();
        if (ImGui::Button("Corner")) {
            target_position = Vec3(150.0f, 150.0f, -500.0f);
            ik_valid = robot.setTargetPosition(target_position);
        }

        ImGui::Separator();
        ImGui::Text("Color Legend:");
        ImGui::TextColored(ImVec4(0.8f, 0.3f, 0.3f, 1.0f), "■ Upper Arms");
        ImGui::TextColored(ImVec4(0.3f, 0.6f, 0.9f, 1.0f), "■ Lower Arms");
        ImGui::TextColored(ImVec4(0.2f, 0.8f, 0.2f, 1.0f), "■ Platform");
        ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.0f, 1.0f), "■ End Effector");

        ImGui::End();

        // Render 3D scene
        renderer.render(robot);

        // Render ImGui
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
