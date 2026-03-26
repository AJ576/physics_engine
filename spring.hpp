#ifndef SPRING_HPP
#define SPRING_HPP

#include "rigidBody.hpp"

class Spring {
    // This is a square "box type" of structure
public:
    double x; // left edge
    double y; // bottom edge (ground level)
    double width; // platform width
    double height; // platform height
    double springConstant; // spring constant. Higher value means more springy.
    // Could add a damping factor to the spring later.

    Spring(double x, double y, double width, double height, double springConstant = 500.0);
};

// Realized that I could make the values private and use getters and setters. Will do that later.
// TODO: Make the values private and use getters and setters.

bool isBallOnSpring(const RigidBody& ball, const Spring& spring);
void applySpringImpulse(RigidBody& ball, const Spring& spring);

#endif // SPRING_HPP