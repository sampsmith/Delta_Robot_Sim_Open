#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "DeltaRobot.hpp"

struct GLFWwindow;

class Renderer {
public:
    Renderer();
    ~Renderer();
    
    bool initialize(GLFWwindow* window);
    void shutdown();
    
    void beginFrame();
    void endFrame();
    
    void renderDeltaRobot(const DeltaRobot& robot);
    void renderGrid(float size = 2.0f, float spacing = 0.1f);
    void renderCoordinateAxes(float length = 0.2f);
    
    // Camera controls
    void setCameraPosition(const glm::vec3& position);
    void setCameraTarget(const glm::vec3& target);
    glm::vec3 getCameraPosition() const { return cameraPos_; }
    glm::vec3 getCameraTarget() const { return cameraTarget_; }
    
    // Viewport
    void setViewport(int width, int height);
    
    // Mouse controls
    void processMouseInput(float deltaX, float deltaY, bool leftButton, bool rightButton);
    
    // Get view and projection matrices for custom rendering
    glm::mat4 getViewMatrix() const;
    glm::mat4 getProjectionMatrix() const;
    
    // End effector gizmo controls
    void renderEndEffectorGizmo(const glm::vec3& position, float size = 0.03f);
    int pickGizmoAxis(const glm::vec3& gizmoPos, float gizmoSize, 
                     int mouseX, int mouseY, const glm::mat4& view, const glm::mat4& proj);
    glm::vec3 getGizmoDragDelta(int axis, float mouseDeltaX, float mouseDeltaY, 
                                const glm::vec3& gizmoPos, const glm::mat4& view, const glm::mat4& proj);

private:
    GLFWwindow* window_;
    
    // Camera
    glm::vec3 cameraPos_;
    glm::vec3 cameraTarget_;
    glm::vec3 cameraUp_;
    float cameraDistance_;
    float cameraAngleX_;
    float cameraAngleY_;
    
    // Projection
    int viewportWidth_;
    int viewportHeight_;
    float fov_;
    float nearPlane_;
    float farPlane_;
    
    // Rendering helpers
    void setup3DProjection();
    void setup2DProjection();
    void updateCameraPosition();
    void renderCylinder(const glm::vec3& start, const glm::vec3& end, float radius, const glm::vec3& color);
    void renderSphere(const glm::vec3& position, float radius, const glm::vec3& color);
    void renderLine(const glm::vec3& start, const glm::vec3& end, const glm::vec3& color, float width = 1.0f);
};

