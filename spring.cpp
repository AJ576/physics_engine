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
        double impactSpeed = -vel[1];

        // Energy-return (restitution) coefficient in [0, 1].
        // The spring compresses on impact, stores the ball's kinetic energy,
        // and puts it back on release. A stiffer spring (higher K) returns
        // more of the impact energy. It is capped at 1.0 so the spring can
        // never add energy to the system: the rebound speed is at most the
        // impact speed, so only energy the ball already had is regained.
        double restitution = std::min(1.0, spring.getSpringConstant() * compression * 0.1);

        ball.setVelocityY(impactSpeed * restitution);
        ball.setPositionY(springTop + r);
    }
}