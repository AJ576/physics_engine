#include "rigidBody.hpp"
#include <algorithm>
#pragma once // read once per programam.

// Spring class
class Spring {
    // This is a square "box type" of structure
private:
    double x; // left edge
    double y; // bottom edge (ground level)
    double width; // platform width
    double height; // platform height
    double springConstant; // spring constant. Higher value means more springy.
    // Could add a damping factor to the spring later.

public:
    Spring(double x, double y, double width, double height, double springConstant = 500.0);
    // Getters
    // Note: getters are const because they do not modify the object.
    double getX() const { return x; }
    double getY() const { return y; }
    double getWidth() const { return width; }
    double getHeight() const { return height; }
    double getSpringConstant() const { return springConstant; }

    // Setters
    // Note: setters are not const because they modify the object.
    void setX(double v) { x = v; }
    void setY(double v) { y = v; }
    void setWidth(double v) { width = v; }
    void setHeight(double v) { height = v; }
    void setSpringConstant(double v) { springConstant = v; }
};

// Defined once in spring.cpp (not in header) to avoid duplicate symbols at link time.
bool isBallOnSpring(const RigidBody& ball, const Spring& spring);
void applySpringImpulse(RigidBody& ball, const Spring& spring);