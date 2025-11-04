#ifndef RENDERER_HPP
#define RENDERER_HPP

#include "DeltaRobot.hpp"
#include <GL/gl.h>

class Camera {
public:
    Camera();
    
    void rotate(float dx, float dy);
    void zoom(float delta);
    void reset();
    
    void apply() const;
    
    float distance;
    float rotation_x;
    float rotation_y;
    Vec3 target;
};

class Renderer {
public:
    Renderer();
    ~Renderer();
    
    void initialize();
    void render(const DeltaRobot& robot);
    void resize(int width, int height);
    
    Camera& getCamera() { return camera_; }
    
private:
    Camera camera_;
    int width_;
    int height_;
    
    // Drawing primitives
    void drawCylinder(const Vec3& start, const Vec3& end, float radius, 
                     float r, float g, float b);
    void drawSphere(const Vec3& center, float radius, float r, float g, float b);
    void drawLine(const Vec3& start, const Vec3& end, float width,
                 float r, float g, float b);
    void drawGrid(float size, int divisions);
    void drawAxes(float length);
    
    // Robot components
    void drawBase(const DeltaRobot& robot);
    void drawArms(const DeltaRobot& robot);
    void drawPlatform(const DeltaRobot& robot);
};

#endif // RENDERER_HPP
