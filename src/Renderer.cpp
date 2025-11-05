#include <GLFW/glfw3.h>
#ifdef __linux__
#include <GL/gl.h>
#endif
#include "Renderer.hpp"
#include <cmath>
#include <vector>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

Renderer::Renderer() 
    : window_(nullptr)
    , cameraPos_(0.0f, 0.0f, 0.5f)
    , cameraTarget_(0.0f, 0.0f, -0.2f)
    , cameraUp_(0.0f, 0.0f, 1.0f)
    , cameraDistance_(0.8f)
    , cameraAngleX_(0.0f)
    , cameraAngleY_(M_PI / 4.0f)
    , viewportWidth_(800)
    , viewportHeight_(600)
    , fov_(45.0f)
    , nearPlane_(0.01f)
    , farPlane_(10.0f)
{
}

Renderer::~Renderer() {
    shutdown();
}

bool Renderer::initialize(GLFWwindow* window) {
    window_ = window;
    
    // For compatibility profile, OpenGL functions are available directly
    // No need to load via GLAD
    
    // Enable depth testing
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    
    // Enable smooth lines
    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    
    // Update camera position based on angles
    updateCameraPosition();
    
    return true;
}

void Renderer::updateCameraPosition() {
    cameraPos_.x = cameraTarget_.x + cameraDistance_ * std::cos(cameraAngleY_) * std::cos(cameraAngleX_);
    cameraPos_.y = cameraTarget_.y + cameraDistance_ * std::cos(cameraAngleY_) * std::sin(cameraAngleX_);
    cameraPos_.z = cameraTarget_.z + cameraDistance_ * std::sin(cameraAngleY_);
}

void Renderer::shutdown() {
    // Cleanup if needed
}

void Renderer::beginFrame() {
    glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    setup3DProjection();
}

void Renderer::endFrame() {
    // Frame end handled by ImGui
}

void Renderer::setViewport(int width, int height) {
    viewportWidth_ = width;
    viewportHeight_ = height;
    glViewport(0, 0, width, height);
}

void Renderer::setup3DProjection() {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    
    float aspect = (float)viewportWidth_ / (float)viewportHeight_;
    float f = 1.0f / std::tan(fov_ * M_PI / 360.0f);
    float proj[16] = {
        f / aspect, 0, 0, 0,
        0, f, 0, 0,
        0, 0, (farPlane_ + nearPlane_) / (nearPlane_ - farPlane_), -1,
        0, 0, (2.0f * farPlane_ * nearPlane_) / (nearPlane_ - farPlane_), 0
    };
    glLoadMatrixf(proj);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    glm::vec3 forward = glm::normalize(cameraTarget_ - cameraPos_);
    glm::vec3 right = glm::normalize(glm::cross(forward, cameraUp_));
    glm::vec3 up = glm::cross(right, forward);
    
    float view[16] = {
        right.x, up.x, -forward.x, 0,
        right.y, up.y, -forward.y, 0,
        right.z, up.z, -forward.z, 0,
        -glm::dot(right, cameraPos_), -glm::dot(up, cameraPos_), glm::dot(forward, cameraPos_), 1
    };
    
    glLoadMatrixf(view);
}

void Renderer::setup2DProjection() {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, viewportWidth_, viewportHeight_, 0, -1, 1);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void Renderer::processMouseInput(float deltaX, float deltaY, bool leftButton, bool rightButton) {
    if (leftButton) {
        cameraAngleX_ += deltaX * 0.01f;
        cameraAngleY_ += deltaY * 0.01f;
        cameraAngleY_ = std::clamp(cameraAngleY_, static_cast<float>(-M_PI / 2.0 + 0.1), static_cast<float>(M_PI / 2.0 - 0.1));
        updateCameraPosition();
    }
    
    if (rightButton) {
        cameraDistance_ += deltaY * 0.01f;
        cameraDistance_ = std::max(0.1f, std::min(5.0f, cameraDistance_));
        updateCameraPosition();
    }
}

glm::mat4 Renderer::getViewMatrix() const {
    glm::vec3 forward = glm::normalize(cameraTarget_ - cameraPos_);
    glm::vec3 right = glm::normalize(glm::cross(forward, cameraUp_));
    glm::vec3 up = glm::cross(right, forward);
    
    return glm::lookAt(cameraPos_, cameraTarget_, up);
}

glm::mat4 Renderer::getProjectionMatrix() const {
    float aspect = (float)viewportWidth_ / (float)viewportHeight_;
    return glm::perspective(glm::radians(fov_), aspect, nearPlane_, farPlane_);
}

void Renderer::setCameraPosition(const glm::vec3& position) {
    cameraPos_ = position;
    cameraDistance_ = glm::length(cameraPos_ - cameraTarget_);
    glm::vec3 dir = glm::normalize(cameraPos_ - cameraTarget_);
    cameraAngleX_ = std::atan2(dir.y, dir.x);
    cameraAngleY_ = std::asin(dir.z);
}

void Renderer::setCameraTarget(const glm::vec3& target) {
    cameraTarget_ = target;
    updateCameraPosition();
}

void Renderer::renderCylinder(const glm::vec3& start, const glm::vec3& end, float radius, const glm::vec3& color) {
    glColor3f(color.r, color.g, color.b);
    
    glm::vec3 direction = end - start;
    float length = glm::length(direction);
    if (length < 0.001f) return;
    
    direction = direction / length;
    
    // Calculate perpendicular vectors for cylinder cross-section
    glm::vec3 up(0, 0, 1);
    glm::vec3 right = glm::cross(direction, up);
    if (glm::length(right) < 0.001f) {
        right = glm::vec3(1, 0, 0);
    }
    right = glm::normalize(right);
    glm::vec3 perp = glm::cross(right, direction);
    perp = glm::normalize(perp);
    
    // Draw cylinder as a tube
    const int segments = 8;
    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i <= segments; ++i) {
        float angle = 2.0f * M_PI * i / segments;
        glm::vec3 offset = (right * std::cos(angle) + perp * std::sin(angle)) * radius;
        glm::vec3 p1 = start + offset;
        glm::vec3 p2 = end + offset;
        glNormal3f(offset.x / radius, offset.y / radius, offset.z / radius);
        glVertex3f(p1.x, p1.y, p1.z);
        glVertex3f(p2.x, p2.y, p2.z);
    }
    glEnd();
}

void Renderer::renderSphere(const glm::vec3& position, float radius, const glm::vec3& color) {
    glColor3f(color.r, color.g, color.b);
    
    const int segments = 12;
    const int rings = 12;
    
    for (int i = 0; i < rings; ++i) {
        float theta1 = M_PI * i / rings;
        float theta2 = M_PI * (i + 1) / rings;
        
        glBegin(GL_QUAD_STRIP);
        for (int j = 0; j <= segments; ++j) {
            float phi = 2.0f * M_PI * j / segments;
            
            float x1 = std::sin(theta1) * std::cos(phi);
            float y1 = std::sin(theta1) * std::sin(phi);
            float z1 = std::cos(theta1);
            
            float x2 = std::sin(theta2) * std::cos(phi);
            float y2 = std::sin(theta2) * std::sin(phi);
            float z2 = std::cos(theta2);
            
            glNormal3f(x1, y1, z1);
            glVertex3f(position.x + x1 * radius, position.y + y1 * radius, position.z + z1 * radius);
            glNormal3f(x2, y2, z2);
            glVertex3f(position.x + x2 * radius, position.y + y2 * radius, position.z + z2 * radius);
        }
        glEnd();
    }
}

void Renderer::renderLine(const glm::vec3& start, const glm::vec3& end, const glm::vec3& color, float width) {
    glLineWidth(width);
    glColor3f(color.r, color.g, color.b);
    glBegin(GL_LINES);
    glVertex3f(start.x, start.y, start.z);
    glVertex3f(end.x, end.y, end.z);
    glEnd();
    glLineWidth(1.0f);
}

void Renderer::renderGrid(float size, float spacing) {
    glLineWidth(1.0f);
    glColor3f(0.3f, 0.3f, 0.3f);
    
    glBegin(GL_LINES);
    int lines = (int)(size / spacing);
    for (int i = -lines; i <= lines; ++i) {
        float pos = i * spacing;
        // X-axis lines
        glVertex3f(pos, -size, 0.0f);
        glVertex3f(pos, size, 0.0f);
        // Y-axis lines
        glVertex3f(-size, pos, 0.0f);
        glVertex3f(size, pos, 0.0f);
    }
    glEnd();
}

void Renderer::renderCoordinateAxes(float length) {
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    
    // X axis - red
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(length, 0.0f, 0.0f);
    
    // Y axis - green
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, length, 0.0f);
    
    // Z axis - blue
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, length);
    
    glEnd();
    glLineWidth(1.0f);
}

void Renderer::renderDeltaRobot(const DeltaRobot& robot) {
    const DeltaRobotState& state = robot.getState();
    const DeltaRobotConfig& config = robot.getConfig();
    
    if (!state.isValid) {
        // Render invalid state in red
        glColor3f(1.0f, 0.0f, 0.0f);
        renderSphere(state.endEffectorPos, 0.01f, glm::vec3(1.0f, 0.0f, 0.0f));
        return;
    }
    
    // Render base platform
    glColor3f(0.5f, 0.5f, 0.5f);
    for (int i = 0; i < 3; ++i) {
        glm::vec3 baseJoint = state.upperJoints[i].position;
        renderSphere(baseJoint, 0.015f, glm::vec3(0.6f, 0.6f, 0.6f));
    }
    
    // Render upper arms
    for (int i = 0; i < 3; ++i) {
        renderCylinder(state.upperJoints[i].position, 
                      state.lowerJoints[i].position,
                      0.008f, glm::vec3(0.2f, 0.6f, 0.9f));
    }
    
    // Render elbow joints
    for (int i = 0; i < 3; ++i) {
        renderSphere(state.lowerJoints[i].position, 0.012f, glm::vec3(0.8f, 0.8f, 0.2f));
    }
    
    // Render lower arms
    for (int i = 0; i < 3; ++i) {
        renderCylinder(state.lowerJoints[i].position,
                      state.effectorJoints[i],
                      0.008f, glm::vec3(0.9f, 0.3f, 0.3f));
    }
    
    // Render end effector joints
    for (int i = 0; i < 3; ++i) {
        renderSphere(state.effectorJoints[i], 0.010f, glm::vec3(0.9f, 0.9f, 0.9f));
    }
    
    // Render end effector platform (centre)
    renderSphere(state.endEffectorPos, 0.015f, glm::vec3(0.2f, 0.9f, 0.2f));
    
    // Draw lines from effector joints to centre
    for (int i = 0; i < 3; ++i) {
        renderLine(state.effectorJoints[i], state.endEffectorPos, 
                  glm::vec3(0.5f, 0.5f, 0.5f), 1.0f);
    }
    
    // Gizmo removed - using virtual controller instead
}

void Renderer::renderEndEffectorGizmo(const glm::vec3& position, float size) {
    // Render X axis (red)
    glm::vec3 xAxis = glm::vec3(1.0f, 0.0f, 0.0f) * size;
    renderLine(position, position + xAxis, glm::vec3(1.0f, 0.0f, 0.0f), 3.0f);
    renderSphere(position + xAxis, size * 0.3f, glm::vec3(1.0f, 0.0f, 0.0f));
    
    // Render Y axis (green)
    glm::vec3 yAxis = glm::vec3(0.0f, 1.0f, 0.0f) * size;
    renderLine(position, position + yAxis, glm::vec3(0.0f, 1.0f, 0.0f), 3.0f);
    renderSphere(position + yAxis, size * 0.3f, glm::vec3(0.0f, 1.0f, 0.0f));
    
    // Render Z axis (blue)
    glm::vec3 zAxis = glm::vec3(0.0f, 0.0f, 1.0f) * size;
    renderLine(position, position + zAxis, glm::vec3(0.0f, 0.0f, 1.0f), 3.0f);
    renderSphere(position + zAxis, size * 0.3f, glm::vec3(0.0f, 0.0f, 1.0f));
}

int Renderer::pickGizmoAxis(const glm::vec3& gizmoPos, float gizmoSize, 
                            int mouseX, int mouseY, const glm::mat4& view, const glm::mat4& proj) {
    // Convert mouse position to normalized device coordinates
    float x = (2.0f * mouseX) / viewportWidth_ - 1.0f;
    float y = 1.0f - (2.0f * mouseY) / viewportHeight_;
    
    // Create ray from camera through mouse position
    glm::vec4 rayClip = glm::vec4(x, y, -1.0f, 1.0f);
    glm::vec4 rayEye = glm::inverse(proj) * rayClip;
    rayEye = glm::vec4(rayEye.x, rayEye.y, -1.0f, 0.0f);
    glm::vec4 rayWorld = glm::inverse(view) * rayEye;
    glm::vec3 rayDir = glm::normalize(glm::vec3(rayWorld));
    
    // Get camera position
    glm::vec3 camPos = cameraPos_;
    
    // Test each axis
    float minDist = 1e10f;
    int pickedAxis = -1;
    float pickRadius = gizmoSize * 0.5f;
    
    glm::vec3 axes[3] = {
        glm::vec3(1.0f, 0.0f, 0.0f),  // X
        glm::vec3(0.0f, 1.0f, 0.0f),  // Y
        glm::vec3(0.0f, 0.0f, 1.0f)   // Z
    };
    
    for (int i = 0; i < 3; ++i) {
        glm::vec3 axisEnd = gizmoPos + axes[i] * gizmoSize;
        
        // Find closest point on ray to axis line
        glm::vec3 axisDir = glm::normalize(axes[i]);
        glm::vec3 toAxis = gizmoPos - camPos;
        float t = glm::dot(toAxis, axisDir);
        glm::vec3 closestPoint = gizmoPos + axisDir * t;
        
        // Distance from ray to axis
        glm::vec3 toClosest = closestPoint - camPos;
        float dist = glm::length(glm::cross(toClosest, rayDir));
        
        if (dist < pickRadius && dist < minDist) {
            minDist = dist;
            pickedAxis = i;
        }
        
        // Also check the arrow head (sphere at end)
        glm::vec3 toArrow = axisEnd - camPos;
        float arrowDist = glm::length(glm::cross(toArrow, rayDir));
        if (arrowDist < pickRadius && arrowDist < minDist) {
            minDist = arrowDist;
            pickedAxis = i;
        }
    }
    
    return pickedAxis;
}

glm::vec3 Renderer::getGizmoDragDelta(int axis, float mouseDeltaX, float mouseDeltaY, 
                                      const glm::vec3& gizmoPos, const glm::mat4& view, const glm::mat4& proj) {
    if (axis < 0 || axis > 2) return glm::vec3(0.0f);
    
    glm::vec3 axes[3] = {
        glm::vec3(1.0f, 0.0f, 0.0f),  // X
        glm::vec3(0.0f, 1.0f, 0.0f),  // Y
        glm::vec3(0.0f, 0.0f, 1.0f)   // Z
    };
    
    glm::vec3 axisDir = axes[axis];
    
    // Project axis direction to screen space
    glm::vec4 axisStart4 = proj * view * glm::vec4(gizmoPos, 1.0f);
    glm::vec4 axisEnd4 = proj * view * glm::vec4(gizmoPos + axisDir * 0.1f, 1.0f);
    
    if (axisStart4.w > 0.0f && axisEnd4.w > 0.0f) {
        axisStart4 /= axisStart4.w;
        axisEnd4 /= axisEnd4.w;
        
        glm::vec2 axisStartScreen = glm::vec2(axisStart4.x, axisStart4.y);
        glm::vec2 axisEndScreen = glm::vec2(axisEnd4.x, axisEnd4.y);
        glm::vec2 axisScreenDir = glm::normalize(axisEndScreen - axisStartScreen);
        
        // Calculate movement along axis based on mouse delta
        float mouseDelta = axisScreenDir.x * mouseDeltaX + axisScreenDir.y * mouseDeltaY;
        float sensitivity = 0.01f;  // Adjust this for movement speed
        
        return axisDir * mouseDelta * sensitivity;
    }
    
    return glm::vec3(0.0f);
}

