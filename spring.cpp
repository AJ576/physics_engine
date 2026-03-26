#include "spring.hpp"

// Spring constructor
Spring::Spring(double x, double y, double width, double height, double springConstant)
    : x(x), y(y), width(width), height(height), springConstant(springConstant) {}

bool isBallOnSpring(const RigidBody& ball, const Spring& spring) {
    auto pos = ball.getPosition();
    double r = ball.getRadius();
    double ballBottom = pos[1] - r;
    double springTop = spring.getY() + spring.getHeight();

    return pos[0] >= spring.getX() && pos[0] <= spring.getX() + spring.getWidth() &&
           ballBottom <= springTop && ballBottom >= spring.getY();
}

void applySpringImpulse(RigidBody& ball, const Spring& spring) {
    auto pos = ball.getPosition();
    auto vel = ball.getVelocity();
    double r = ball.getRadius();
    double ballBottom = pos[1] - r;
    double springTop = spring.getY() + spring.getHeight();

    double compression = springTop - ballBottom;

    if (compression > 0 && vel[1] < 0) {
        double bounceStrength = 1.0 + spring.getSpringConstant() * compression * 0.1;
        ball.setVelocityY(-vel[1] * std::min(bounceStrength, 2.0));
        ball.setPositionY(springTop + r);
    }
}