#include <GLFW/glfw3.h>
#ifdef __linux__
#include <GL/gl.h>
#endif
#include "Renderer.hpp"
#include <cmath>
#include <vector>
#include <array>
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
    
    // Enable smooth points
    glEnable(GL_POINT_SMOOTH);
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
    
    // Enable blending for transparency
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    // Enable lighting
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    
    // Enable normal vector normalization
    glEnable(GL_NORMALIZE);
    
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
    // Clear buffers first
    glClearColor(0.05f, 0.05f, 0.08f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // Render gradient background
    renderGradientBackground();
    
    // Now setup 3D projection and lighting for 3D rendering
    setup3DProjection();
    setupLighting();
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

void Renderer::renderCylinder(const glm::vec3& start, const glm::vec3& end, float radius, const glm::vec3& color, bool useLighting) {
    if (!useLighting) {
        glDisable(GL_LIGHTING);
    }
    
    glColor3f(color.r, color.g, color.b);
    
    glm::vec3 direction = end - start;
    float length = glm::length(direction);
    if (length < 0.001f) {
        if (!useLighting) {
            glEnable(GL_LIGHTING);
        }
        return;
    }
    
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
    
    // Draw cylinder as a tube with more segments for smoother appearance
    const int segments = 16;
    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i <= segments; ++i) {
        float angle = 2.0f * M_PI * i / segments;
        glm::vec3 offset = (right * std::cos(angle) + perp * std::sin(angle)) * radius;
        glm::vec3 normal = glm::normalize(offset);
        glm::vec3 p1 = start + offset;
        glm::vec3 p2 = end + offset;
        glNormal3f(normal.x, normal.y, normal.z);
        glVertex3f(p1.x, p1.y, p1.z);
        glVertex3f(p2.x, p2.y, p2.z);
    }
    glEnd();
    
    // Draw end caps
    glBegin(GL_TRIANGLE_FAN);
    glNormal3f(-direction.x, -direction.y, -direction.z);
    glVertex3f(start.x, start.y, start.z);
    for (int i = 0; i <= segments; ++i) {
        float angle = 2.0f * M_PI * i / segments;
        glm::vec3 offset = (right * std::cos(angle) + perp * std::sin(angle)) * radius;
        glm::vec3 p = start + offset;
        glVertex3f(p.x, p.y, p.z);
    }
    glEnd();
    
    glBegin(GL_TRIANGLE_FAN);
    glNormal3f(direction.x, direction.y, direction.z);
    glVertex3f(end.x, end.y, end.z);
    for (int i = 0; i <= segments; ++i) {
        float angle = 2.0f * M_PI * i / segments;
        glm::vec3 offset = (right * std::cos(angle) + perp * std::sin(angle)) * radius;
        glm::vec3 p = end + offset;
        glVertex3f(p.x, p.y, p.z);
    }
    glEnd();
    
    if (!useLighting) {
        glEnable(GL_LIGHTING);
    }
}

void Renderer::renderSphere(const glm::vec3& position, float radius, const glm::vec3& color, bool useLighting) {
    if (!useLighting) {
        glDisable(GL_LIGHTING);
    }
    
    glColor3f(color.r, color.g, color.b);
    
    // Increased segments for smoother spheres
    const int segments = 20;
    const int rings = 20;
    
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
    
    if (!useLighting) {
        glEnable(GL_LIGHTING);
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
    glDisable(GL_LIGHTING);
    
    int lines = (int)(size / spacing);
    float majorSpacing = spacing * 5.0f;  // Every 5th line is major
    
    // Draw minor grid lines (darker, thinner)
    glLineWidth(0.5f);
    glColor4f(0.15f, 0.15f, 0.2f, 0.6f);
    glBegin(GL_LINES);
    for (int i = -lines; i <= lines; ++i) {
        float pos = i * spacing;
        // Skip major lines
        if (std::abs(std::fmod(pos, majorSpacing)) > 0.001f) {
            // X-axis lines
            glVertex3f(pos, -size, 0.0f);
            glVertex3f(pos, size, 0.0f);
            // Y-axis lines
            glVertex3f(-size, pos, 0.0f);
            glVertex3f(size, pos, 0.0f);
        }
    }
    glEnd();
    
    // Draw major grid lines (brighter, thicker)
    glLineWidth(1.5f);
    glColor4f(0.25f, 0.25f, 0.35f, 0.8f);
    glBegin(GL_LINES);
    int majorLines = (int)(size / majorSpacing);
    for (int i = -majorLines; i <= majorLines; ++i) {
        float pos = i * majorSpacing;
        // X-axis lines
        glVertex3f(pos, -size, 0.0f);
        glVertex3f(pos, size, 0.0f);
        // Y-axis lines
        glVertex3f(-size, pos, 0.0f);
        glVertex3f(size, pos, 0.0f);
    }
    glEnd();
    
    glLineWidth(1.0f);
    glEnable(GL_LIGHTING);
}

void Renderer::renderCoordinateAxes(float length) {
    glDisable(GL_LIGHTING);
    glLineWidth(3.0f);
    
    // X axis - red with arrow head
    glColor3f(0.9f, 0.2f, 0.2f);
    glBegin(GL_LINES);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(length, 0.0f, 0.0f);
    glEnd();
    renderSphere(glm::vec3(length, 0.0f, 0.0f), length * 0.08f, glm::vec3(0.9f, 0.2f, 0.2f), false);
    
    // Y axis - green with arrow head
    glColor3f(0.2f, 0.9f, 0.2f);
    glBegin(GL_LINES);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, length, 0.0f);
    glEnd();
    renderSphere(glm::vec3(0.0f, length, 0.0f), length * 0.08f, glm::vec3(0.2f, 0.9f, 0.2f), false);
    
    // Z axis - blue with arrow head
    glColor3f(0.2f, 0.4f, 0.9f);
    glBegin(GL_LINES);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, length);
    glEnd();
    renderSphere(glm::vec3(0.0f, 0.0f, length), length * 0.08f, glm::vec3(0.2f, 0.4f, 0.9f), false);
    
    glLineWidth(1.0f);
    glEnable(GL_LIGHTING);
}

void Renderer::renderDeltaRobot(const DeltaRobot& robot) {
    const DeltaRobotState& state = robot.getState();
    const DeltaRobotConfig& config = robot.getConfig();
    
    if (!state.isValid) {
        // Render invalid state in red
        glDisable(GL_LIGHTING);
        renderSphere(state.endEffectorPos, 0.01f, glm::vec3(1.0f, 0.0f, 0.0f), false);
        glEnable(GL_LIGHTING);
        return;
    }
    
    // Set material properties for metallic look
    float specular[] = {0.8f, 0.8f, 0.8f, 1.0f};
    float shininess[] = {64.0f};
    glMaterialfv(GL_FRONT, GL_SPECULAR, specular);
    glMaterialfv(GL_FRONT, GL_SHININESS, shininess);
    
    // Calculate base plate center (average of upper joints)
    glm::vec3 baseCenter(0.0f);
    for (int i = 0; i < 3; ++i) {
        baseCenter += state.upperJoints[i].position;
    }
    baseCenter /= 3.0f;
    baseCenter.z = config.basePlateHeight;
    
    // Render base plate (top platform - solid circular plate)
    // Make it thicker and more solid-looking
    float basePlateThickness = config.basePlateThickness > 0.001f ? config.basePlateThickness * 2.0f : 0.02f;
    renderCircularPlate(baseCenter, config.basePlateRadius, basePlateThickness, 
                       glm::vec3(0.6f, 0.6f, 0.65f));
    
    // Render motor mounts at base joints
    for (int i = 0; i < 3; ++i) {
        glm::vec3 baseJoint = state.upperJoints[i].position;
        glm::vec3 mountDirection = glm::normalize(baseJoint - baseCenter);
        mountDirection.z = 0.0f; // Horizontal
        if (glm::length(mountDirection) > 0.001f) {
            mountDirection = glm::normalize(mountDirection);
        } else {
            mountDirection = glm::vec3(1.0f, 0.0f, 0.0f);
        }
        renderMotorMount(baseJoint, mountDirection, 0.025f, glm::vec3(0.5f, 0.5f, 0.5f));
    }
    
    // Render pivot joints at base (where motors connect to arms)
    for (int i = 0; i < 3; ++i) {
        renderPivotJoint(state.upperJoints[i].position, 0.012f, glm::vec3(0.8f, 0.8f, 0.85f));
    }
    
    // Render upper arms (parallel link design - typical delta robot)
    // Smaller rod diameter for more realistic appearance
    float upperArmRadius = config.armThickness > 0.0f ? config.armThickness * 0.3f : 0.004f;
    float upperArmSeparation = config.armThickness > 0.0f ? config.armThickness * 1.5f : 0.018f;
    for (int i = 0; i < 3; ++i) {
        renderParallelLinkArm(state.upperJoints[i].position, 
                              state.lowerJoints[i].position,
                              upperArmRadius, upperArmSeparation, 
                              glm::vec3(0.2f, 0.5f, 0.9f));
    }
    
    // Render elbow joints (pivot joints)
    for (int i = 0; i < 3; ++i) {
        renderPivotJoint(state.lowerJoints[i].position, 0.016f, glm::vec3(0.9f, 0.7f, 0.3f));
    }
    
    // Render lower arms (parallel link design - typical delta robot)
    // Smaller rod diameter for more realistic appearance
    float lowerArmRadius = config.armThickness > 0.0f ? config.armThickness * 0.3f : 0.004f;
    float lowerArmSeparation = config.armThickness > 0.0f ? config.armThickness * 1.5f : 0.018f;
    for (int i = 0; i < 3; ++i) {
        renderParallelLinkArm(state.lowerJoints[i].position,
                             state.effectorJoints[i],
                             lowerArmRadius, lowerArmSeparation,
                             glm::vec3(0.85f, 0.35f, 0.25f));
    }
    
    // Calculate effector plate corners (triangular plate)
    std::array<glm::vec3, 3> effectorCorners;
    for (int i = 0; i < 3; ++i) {
        effectorCorners[i] = state.effectorJoints[i];
    }
    
    // Render effector plate (bottom platform - solid triangular plate)
    // Make it thicker and more solid-looking
    float effectorPlateThickness = config.effectorPlateThickness > 0.001f ? config.effectorPlateThickness * 2.0f : 0.01f;
    renderTriangularPlate(state.endEffectorPos, effectorCorners, 
                         effectorPlateThickness, 
                         glm::vec3(0.3f, 0.9f, 0.4f));
    
    // Render pivot joints at effector (where arms connect to effector)
    for (int i = 0; i < 3; ++i) {
        renderPivotJoint(state.effectorJoints[i], 0.014f, glm::vec3(0.95f, 0.95f, 0.95f));
    }
    
    // Render center indicator on effector
    glDisable(GL_LIGHTING);
    float emissive[] = {0.1f, 0.4f, 0.1f, 1.0f};
    glMaterialfv(GL_FRONT, GL_EMISSION, emissive);
    renderSphere(state.endEffectorPos, 0.008f, glm::vec3(0.3f, 0.9f, 0.4f), false);
    float noEmissive[] = {0.0f, 0.0f, 0.0f, 1.0f};
    glMaterialfv(GL_FRONT, GL_EMISSION, noEmissive);
    glEnable(GL_LIGHTING);
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

void Renderer::setupLighting() {
    // Position light above and to the front-right
    glm::vec3 lightPos = cameraPos_ + glm::vec3(0.3f, 0.3f, 0.5f);
    float lightPosArray[] = {lightPos.x, lightPos.y, lightPos.z, 1.0f};
    
    // Ambient light (subtle overall illumination)
    float ambient[] = {0.25f, 0.25f, 0.3f, 1.0f};
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    
    // Diffuse light (main illumination)
    float diffuse[] = {0.9f, 0.9f, 0.95f, 1.0f};
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
    
    // Specular light (highlights)
    float specular[] = {1.0f, 1.0f, 1.0f, 1.0f};
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
    
    // Position the light
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosArray);
    
    // Add a second fill light from opposite side
    glEnable(GL_LIGHT1);
    glm::vec3 fillLightPos = cameraPos_ + glm::vec3(-0.2f, -0.2f, 0.3f);
    float fillLightPosArray[] = {fillLightPos.x, fillLightPos.y, fillLightPos.z, 1.0f};
    float fillAmbient[] = {0.15f, 0.15f, 0.2f, 1.0f};
    float fillDiffuse[] = {0.4f, 0.4f, 0.45f, 1.0f};
    glLightfv(GL_LIGHT1, GL_AMBIENT, fillAmbient);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, fillDiffuse);
    glLightfv(GL_LIGHT1, GL_POSITION, fillLightPosArray);
}

void Renderer::renderGradientBackground() {
    // Save current matrix state
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, viewportWidth_, viewportHeight_, 0, -1, 1);
    
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    
    // Disable depth test and lighting for background
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    
    // Draw gradient from dark blue/purple at top to darker at bottom
    glBegin(GL_QUADS);
    // Top-left
    glColor4f(0.08f, 0.08f, 0.12f, 1.0f);
    glVertex2f(0.0f, 0.0f);
    // Top-right
    glColor4f(0.08f, 0.08f, 0.12f, 1.0f);
    glVertex2f((float)viewportWidth_, 0.0f);
    // Bottom-right
    glColor4f(0.05f, 0.05f, 0.08f, 1.0f);
    glVertex2f((float)viewportWidth_, (float)viewportHeight_);
    // Bottom-left
    glColor4f(0.05f, 0.05f, 0.08f, 1.0f);
    glVertex2f(0.0f, (float)viewportHeight_);
    glEnd();
    
    // Restore matrix state
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    
    // Re-enable depth test (lighting will be set up after this)
    glEnable(GL_DEPTH_TEST);
}

void Renderer::renderTrajectoryPath(const std::vector<glm::vec3>& waypoints, const glm::vec3& currentPos, bool isPlaying) {
    if (waypoints.empty()) return;
    
    glDisable(GL_LIGHTING);
    
    // Draw waypoint path
    glLineWidth(2.0f);
    glColor4f(0.4f, 0.7f, 1.0f, 0.6f);
    glBegin(GL_LINE_STRIP);
    for (size_t i = 0; i < waypoints.size(); ++i) {
        glVertex3f(waypoints[i].x, waypoints[i].y, waypoints[i].z);
    }
    // Close loop if needed
    if (waypoints.size() > 1) {
        glVertex3f(waypoints[0].x, waypoints[0].y, waypoints[0].z);
    }
    glEnd();
    
    // Draw waypoint markers
    for (size_t i = 0; i < waypoints.size(); ++i) {
        glm::vec3 wp = waypoints[i];
        float dist = glm::length(currentPos - wp);
        bool isActive = dist < 0.01f && isPlaying;
        
        // Waypoint sphere
        glm::vec3 color = isActive ? glm::vec3(1.0f, 1.0f, 0.2f) : glm::vec3(0.3f, 0.6f, 1.0f);
        renderSphere(wp, 0.012f, color, false);
        
        // Number indicator (simple line)
        glLineWidth(1.0f);
        glColor4f(color.r, color.g, color.b, 0.8f);
        renderLine(wp, wp + glm::vec3(0.0f, 0.0f, 0.02f), color, 1.0f);
    }
    
    // Draw current position indicator
    if (isPlaying) {
        glColor4f(1.0f, 0.8f, 0.2f, 0.9f);
        renderSphere(currentPos, 0.015f, glm::vec3(1.0f, 0.8f, 0.2f), false);
    }
    
    glLineWidth(1.0f);
    glEnable(GL_LIGHTING);
}

void Renderer::renderCircularPlate(const glm::vec3& center, float radius, float thickness, const glm::vec3& color, bool useLighting) {
    if (!useLighting) {
        glDisable(GL_LIGHTING);
    }
    
    glColor3f(color.r, color.g, color.b);
    
    const int segments = 32;
    float halfThickness = thickness * 0.5f;
    
    // Top face
    glBegin(GL_TRIANGLE_FAN);
    glNormal3f(0.0f, 0.0f, 1.0f);
    glVertex3f(center.x, center.y, center.z + halfThickness);
    for (int i = 0; i <= segments; ++i) {
        float angle = 2.0f * M_PI * i / segments;
        float x = center.x + radius * std::cos(angle);
        float y = center.y + radius * std::sin(angle);
        glVertex3f(x, y, center.z + halfThickness);
    }
    glEnd();
    
    // Bottom face
    glBegin(GL_TRIANGLE_FAN);
    glNormal3f(0.0f, 0.0f, -1.0f);
    glVertex3f(center.x, center.y, center.z - halfThickness);
    for (int i = 0; i <= segments; ++i) {
        float angle = 2.0f * M_PI * i / segments;
        float x = center.x + radius * std::cos(angle);
        float y = center.y + radius * std::sin(angle);
        glVertex3f(x, y, center.z - halfThickness);
    }
    glEnd();
    
    // Edge (cylindrical side)
    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i <= segments; ++i) {
        float angle = 2.0f * M_PI * i / segments;
        float x = radius * std::cos(angle);
        float y = radius * std::sin(angle);
        glm::vec3 normal = glm::normalize(glm::vec3(x, y, 0.0f));
        glNormal3f(normal.x, normal.y, normal.z);
        glVertex3f(center.x + x, center.y + y, center.z + halfThickness);
        glVertex3f(center.x + x, center.y + y, center.z - halfThickness);
    }
    glEnd();
    
    if (!useLighting) {
        glEnable(GL_LIGHTING);
    }
}

void Renderer::renderTriangularPlate(const glm::vec3& center, const std::array<glm::vec3, 3>& corners, float thickness, const glm::vec3& color, bool useLighting) {
    if (!useLighting) {
        glDisable(GL_LIGHTING);
    }
    
    glColor3f(color.r, color.g, color.b);
    
    float halfThickness = thickness * 0.5f;
    
    // Calculate normal (pointing downward)
    glm::vec3 v1 = corners[1] - corners[0];
    glm::vec3 v2 = corners[2] - corners[0];
    glm::vec3 normal = glm::normalize(glm::cross(v1, v2));
    
    // Top face (facing up)
    glBegin(GL_TRIANGLES);
    glNormal3f(-normal.x, -normal.y, -normal.z);
    for (int i = 0; i < 3; ++i) {
        glm::vec3 top = corners[i] + normal * halfThickness;
        glVertex3f(top.x, top.y, top.z);
    }
    glEnd();
    
    // Bottom face (facing down)
    glBegin(GL_TRIANGLES);
    glNormal3f(normal.x, normal.y, normal.z);
    for (int i = 2; i >= 0; --i) {
        glm::vec3 bottom = corners[i] - normal * halfThickness;
        glVertex3f(bottom.x, bottom.y, bottom.z);
    }
    glEnd();
    
    // Edges (three rectangular sides)
    for (int i = 0; i < 3; ++i) {
        int next = (i + 1) % 3;
        glm::vec3 top1 = corners[i] + normal * halfThickness;
        glm::vec3 top2 = corners[next] + normal * halfThickness;
        glm::vec3 bottom1 = corners[i] - normal * halfThickness;
        glm::vec3 bottom2 = corners[next] - normal * halfThickness;
        
        glm::vec3 edgeDir = glm::normalize(corners[next] - corners[i]);
        glm::vec3 edgeNormal = glm::cross(edgeDir, normal);
        
        glBegin(GL_QUADS);
        glNormal3f(edgeNormal.x, edgeNormal.y, edgeNormal.z);
        glVertex3f(top1.x, top1.y, top1.z);
        glVertex3f(top2.x, top2.y, top2.z);
        glVertex3f(bottom2.x, bottom2.y, bottom2.z);
        glVertex3f(bottom1.x, bottom1.y, bottom1.z);
        glEnd();
    }
    
    if (!useLighting) {
        glEnable(GL_LIGHTING);
    }
}

void Renderer::renderMotorMount(const glm::vec3& position, const glm::vec3& direction, float size, const glm::vec3& color) {
    glColor3f(color.r, color.g, color.b);
    
    // Create a motor mount as a cylinder/box structure
    glm::vec3 up(0.0f, 0.0f, 1.0f);
    glm::vec3 right = glm::normalize(glm::cross(direction, up));
    if (glm::length(right) < 0.001f) {
        right = glm::vec3(1.0f, 0.0f, 0.0f);
    }
    glm::vec3 forward = glm::normalize(glm::cross(up, right));
    
    float mountHeight = size * 0.6f;
    float mountRadius = size * 0.4f;
    
    // Base cylinder
    glm::vec3 baseBottom = position - up * (mountHeight * 0.5f);
    glm::vec3 baseTop = position + up * (mountHeight * 0.5f);
    renderCylinder(baseBottom, baseTop, mountRadius, color);
    
    // Top cap (motor housing representation)
    renderSphere(position + up * (mountHeight * 0.3f), mountRadius * 0.8f, glm::vec3(color.r * 0.8f, color.g * 0.8f, color.b * 0.8f));
}

void Renderer::renderPivotJoint(const glm::vec3& position, float radius, const glm::vec3& color) {
    // Render as a sphere with a cylindrical pivot indicator
    renderSphere(position, radius, color);
    
    // Add a small ring to indicate it's a pivot
    glDisable(GL_LIGHTING);
    glColor3f(color.r * 0.7f, color.g * 0.7f, color.b * 0.7f);
    glLineWidth(2.0f);
    
    const int segments = 16;
    glBegin(GL_LINE_LOOP);
    for (int i = 0; i < segments; ++i) {
        float angle = 2.0f * M_PI * i / segments;
        float x = position.x + radius * 1.2f * std::cos(angle);
        float y = position.y + radius * 1.2f * std::sin(angle);
        glVertex3f(x, y, position.z);
    }
    glEnd();
    
    glLineWidth(1.0f);
    glEnable(GL_LIGHTING);
}

void Renderer::renderParallelLinkArm(const glm::vec3& start, const glm::vec3& end, float radius, float separation, const glm::vec3& color, bool useLighting) {
    // Render a parallel link arm (two parallel rods) - typical delta robot design
    glm::vec3 direction = end - start;
    float length = glm::length(direction);
    if (length < 0.001f) return;
    
    direction = direction / length;
    
    // Calculate perpendicular vector for separation
    glm::vec3 up(0.0f, 0.0f, 1.0f);
    glm::vec3 perp = glm::normalize(glm::cross(direction, up));
    if (glm::length(perp) < 0.001f) {
        // If direction is parallel to up, use a different perpendicular
        perp = glm::normalize(glm::cross(direction, glm::vec3(1.0f, 0.0f, 0.0f)));
    }
    
    glm::vec3 offset = perp * (separation * 0.5f);
    
    // Render two parallel rods
    glm::vec3 rod1Start = start + offset;
    glm::vec3 rod1End = end + offset;
    renderCylinder(rod1Start, rod1End, radius, color, useLighting);
    
    glm::vec3 rod2Start = start - offset;
    glm::vec3 rod2End = end - offset;
    renderCylinder(rod2Start, rod2End, radius, color, useLighting);
    
    // Add connecting cross members at ends (optional, makes it look more rigid)
    if (useLighting) {
        glDisable(GL_LIGHTING);
    }
    glColor3f(color.r * 0.8f, color.g * 0.8f, color.b * 0.8f);
    renderLine(rod1Start, rod2Start, color, 1.0f);
    renderLine(rod1End, rod2End, color, 1.0f);
    if (useLighting) {
        glEnable(GL_LIGHTING);
    }
}

void Renderer::renderWorkspaceBoundary(const DeltaRobot& robot) {
    const DeltaRobotConfig& config = robot.getConfig();
    
    // Estimate workspace boundary (simplified - circular approximation)
    float maxReach = config.upperArmLength + config.lowerArmLength;
    float minZ = -maxReach * 0.8f;
    float maxZ = -0.1f;
    float radius = maxReach * 0.7f;
    
    glDisable(GL_LIGHTING);
    glLineWidth(1.0f);
    glColor4f(0.3f, 0.3f, 0.5f, 0.4f);
    
    // Draw workspace boundary as semi-transparent wireframe sphere
    const int segments = 24;
    const int rings = 12;
    
    // Draw horizontal rings
    for (int i = 0; i <= rings; ++i) {
        float z = minZ + (maxZ - minZ) * i / rings;
        float currentRadius = radius * std::sqrt(1.0f - std::pow((z - (minZ + maxZ) / 2.0f) / ((maxZ - minZ) / 2.0f), 2.0f));
        
        glBegin(GL_LINE_LOOP);
        for (int j = 0; j <= segments; ++j) {
            float angle = 2.0f * M_PI * j / segments;
            float x = currentRadius * std::cos(angle);
            float y = currentRadius * std::sin(angle);
            glVertex3f(x, y, z);
        }
        glEnd();
    }
    
    // Draw vertical lines
    for (int i = 0; i < segments; ++i) {
        float angle = 2.0f * M_PI * i / segments;
        float x = radius * std::cos(angle);
        float y = radius * std::sin(angle);
        
        glBegin(GL_LINES);
        glVertex3f(x, y, minZ);
        glVertex3f(x, y, maxZ);
        glEnd();
    }
    
    glEnable(GL_LIGHTING);
}

