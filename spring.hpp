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

bool isBallOnSpring(const RigidBody& ball, const Spring& spring) {
    auto pos = ball.getPosition();
    double r = ball.getRadius();
    double ballBottom = pos[1] - r;
    double springTop = spring.y + spring.height;

    return pos[0] >= spring.x && pos[0] <= spring.x + spring.width &&
                    ballBottom <= springTop && ballBottom >= spring.y;
}

void applySpringImpulse(RigidBody& ball, const Spring& spring) {
    auto pos = ball.getPosition();
    auto vel = ball.getVelocity();
    double r = ball.getRadius();
    double ballBottom = pos[1] - r;
    double springTop = spring.y + spring.height;

    double compression = springTop - ballBottom;

    if (compression > 0 && vel[1] < 0) { // Penetrating and moving down.
        double bounceStrength = 1.0 + spring.springConstant * compression * 0.1; // Bounce strength is a function of the compression and the spring constant.
        ball.setVelocityY(-vel[1] * std::min(bounceStrength, 2.0)); // Limit the bounce strength to 2.0 to prevent overshooting.
        ball.setPositionY(springTop + r); // Move the ball back up to the spring.
        // I think this is only moving the ball verttically. Might need a horizontal component to the bounce. Not sure though.
        // TODO: Add a horizontal component to the bounce.
    }
}