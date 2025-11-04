#include "Renderer.hpp"
#include <GL/gl.h>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ========== Camera ==========

Camera::Camera()
    : distance(1500.0f)
    , rotation_x(30.0f)
    , rotation_y(45.0f)
    , target(0.0f, 0.0f, -400.0f)
{}

void Camera::rotate(float dx, float dy) {
    rotation_y += dx * 0.5f;
    rotation_x += dy * 0.5f;
    
    // Clamp vertical rotation
    if (rotation_x > 89.0f) rotation_x = 89.0f;
    if (rotation_x < -89.0f) rotation_x = -89.0f;
}

void Camera::zoom(float delta) {
    distance -= delta * 50.0f;
    if (distance < 500.0f) distance = 500.0f;
    if (distance > 3000.0f) distance = 3000.0f;
}

void Camera::reset() {
    distance = 1500.0f;
    rotation_x = 30.0f;
    rotation_y = 45.0f;
}

void Camera::apply() const {
    glTranslatef(0.0f, 0.0f, -distance);
    glRotatef(rotation_x, 1.0f, 0.0f, 0.0f);
    glRotatef(rotation_y, 0.0f, 0.0f, 1.0f);
    glTranslatef(-target.x, -target.y, -target.z);
}

// ========== Renderer ==========

Renderer::Renderer()
    : width_(800)
    , height_(600)
{}

Renderer::~Renderer() {}

void Renderer::initialize() {
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    
    // Set up light
    GLfloat light_pos[] = {1000.0f, 1000.0f, 1000.0f, 0.0f};
    GLfloat light_ambient[] = {0.3f, 0.3f, 0.3f, 1.0f};
    GLfloat light_diffuse[] = {0.8f, 0.8f, 0.8f, 1.0f};
    
    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    
    glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
}

void Renderer::resize(int width, int height) {
    width_ = width;
    height_ = height;
    
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    
    float aspect = (float)width / (float)height;
    float fov = 45.0f;
    float near = 10.0f;
    float far = 10000.0f;
    
    float f = 1.0f / std::tan(fov * M_PI / 360.0f);
    
    GLfloat matrix[16] = {
        f / aspect, 0, 0, 0,
        0, f, 0, 0,
        0, 0, (far + near) / (near - far), -1,
        0, 0, (2 * far * near) / (near - far), 0
    };
    
    glMultMatrixf(matrix);
    glMatrixMode(GL_MODELVIEW);
}

void Renderer::render(const DeltaRobot& robot) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    camera_.apply();
    
    // Draw scene elements
    drawGrid(1000.0f, 20);
    drawAxes(300.0f);
    
    // Draw robot
    drawBase(robot);
    drawArms(robot);
    drawPlatform(robot);
}

void Renderer::drawGrid(float size, int divisions) {
    glDisable(GL_LIGHTING);
    glColor3f(0.2f, 0.2f, 0.25f);
    glLineWidth(1.0f);
    
    glBegin(GL_LINES);
    float step = size / divisions;
    for (int i = -divisions / 2; i <= divisions / 2; ++i) {
        float pos = i * step;
        glVertex3f(pos, -size / 2, 0);
        glVertex3f(pos, size / 2, 0);
        glVertex3f(-size / 2, pos, 0);
        glVertex3f(size / 2, pos, 0);
    }
    glEnd();
    
    glEnable(GL_LIGHTING);
}

void Renderer::drawAxes(float length) {
    glDisable(GL_LIGHTING);
    glLineWidth(3.0f);
    
    glBegin(GL_LINES);
    // X axis (red)
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0, 0, 0);
    glVertex3f(length, 0, 0);
    
    // Y axis (green)
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0, 0, 0);
    glVertex3f(0, length, 0);
    
    // Z axis (blue)
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, length);
    glEnd();
    
    glEnable(GL_LIGHTING);
}

void Renderer::drawSphere(const Vec3& center, float radius, float r, float g, float b) {
    glColor3f(r, g, b);
    
    glPushMatrix();
    glTranslatef(center.x, center.y, center.z);
    
    const int slices = 16;
    const int stacks = 16;
    
    for (int i = 0; i < stacks; ++i) {
        float lat0 = M_PI * (-0.5f + (float)i / stacks);
        float lat1 = M_PI * (-0.5f + (float)(i + 1) / stacks);
        float z0 = radius * std::sin(lat0);
        float z1 = radius * std::sin(lat1);
        float r0 = radius * std::cos(lat0);
        float r1 = radius * std::cos(lat1);
        
        glBegin(GL_QUAD_STRIP);
        for (int j = 0; j <= slices; ++j) {
            float lng = 2 * M_PI * (float)j / slices;
            float x = std::cos(lng);
            float y = std::sin(lng);
            
            glNormal3f(x * r0, y * r0, z0);
            glVertex3f(x * r0, y * r0, z0);
            glNormal3f(x * r1, y * r1, z1);
            glVertex3f(x * r1, y * r1, z1);
        }
        glEnd();
    }
    
    glPopMatrix();
}

void Renderer::drawCylinder(const Vec3& start, const Vec3& end, float radius,
                           float r, float g, float b) {
    glColor3f(r, g, b);
    
    Vec3 diff = end - start;
    float length = diff.length();
    
    if (length < 0.001f) return;
    
    glPushMatrix();
    glTranslatef(start.x, start.y, start.z);
    
    // Align cylinder with direction vector
    Vec3 up(0, 0, 1);
    Vec3 axis(diff.y * up.z - diff.z * up.y,
              diff.z * up.x - diff.x * up.z,
              diff.x * up.y - diff.y * up.x);
    float axis_length = axis.length();
    
    if (axis_length > 0.001f) {
        float angle = std::acos((diff.z) / length) * 180.0f / M_PI;
        glRotatef(angle, axis.x / axis_length, axis.y / axis_length, axis.z / axis_length);
    } else if (diff.z < 0) {
        glRotatef(180.0f, 1.0f, 0.0f, 0.0f);
    }
    
    const int slices = 12;
    
    // Draw cylinder sides
    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i <= slices; ++i) {
        float angle = 2.0f * M_PI * i / slices;
        float x = radius * std::cos(angle);
        float y = radius * std::sin(angle);
        
        glNormal3f(x / radius, y / radius, 0);
        glVertex3f(x, y, 0);
        glVertex3f(x, y, length);
    }
    glEnd();
    
    glPopMatrix();
}

void Renderer::drawLine(const Vec3& start, const Vec3& end, float width,
                       float r, float g, float b) {
    glDisable(GL_LIGHTING);
    glColor3f(r, g, b);
    glLineWidth(width);
    
    glBegin(GL_LINES);
    glVertex3f(start.x, start.y, start.z);
    glVertex3f(end.x, end.y, end.z);
    glEnd();
    
    glEnable(GL_LIGHTING);
}

void Renderer::drawBase(const DeltaRobot& robot) {
    auto base_joints = robot.getBaseJointPositions();
    
    // Draw base triangle
    for (size_t i = 0; i < 3; ++i) {
        size_t next = (i + 1) % 3;
        drawLine(base_joints[i], base_joints[next], 3.0f, 0.3f, 0.3f, 0.3f);
    }
    
    // Draw base joints
    for (const auto& joint : base_joints) {
        drawSphere(joint, 15.0f, 0.6f, 0.6f, 0.6f);
    }
}

void Renderer::drawArms(const DeltaRobot& robot) {
    auto base_joints = robot.getBaseJointPositions();
    auto elbows = robot.getElbowPositions();
    auto platform_joints = robot.getPlatformJointPositions();
    
    for (size_t i = 0; i < 3; ++i) {
        // Upper arm (base to elbow)
        drawCylinder(base_joints[i], elbows[i], 8.0f, 0.8f, 0.3f, 0.3f);
        
        // Elbow joint
        drawSphere(elbows[i], 12.0f, 0.9f, 0.5f, 0.2f);
        
        // Lower arm (elbow to platform)
        drawCylinder(elbows[i], platform_joints[i], 6.0f, 0.3f, 0.6f, 0.9f);
    }
}

void Renderer::drawPlatform(const DeltaRobot& robot) {
    auto platform_joints = robot.getPlatformJointPositions();
    Vec3 effector = robot.getEndEffectorPosition();
    
    // Draw platform triangle
    for (size_t i = 0; i < 3; ++i) {
        size_t next = (i + 1) % 3;
        drawLine(platform_joints[i], platform_joints[next], 3.0f, 0.2f, 0.8f, 0.2f);
    }
    
    // Draw platform joints
    for (const auto& joint : platform_joints) {
        drawSphere(joint, 10.0f, 0.3f, 0.9f, 0.3f);
    }
    
    // Draw end effector
    drawSphere(effector, 20.0f, 1.0f, 0.8f, 0.0f);
}
