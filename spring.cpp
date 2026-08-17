#include "spring.hpp"
#include <cmath>

// Spring constructor
Spring::Spring(double x, double y, double width, double height, double springConstant,
               double restitution, double compressionMass)
    : x(x), y(y), width(width), height(height), springConstant(springConstant),
      restitution(restitution), compressionMass(compressionMass) {}

void Spring::compress(double amount) {
    double maxCompression = 0.9 * height;
    double target = std::min(amount, maxCompression);

    if (target <= compression) {
        return; // Already compressed more than this hit pushes.
    }

    compression = target;

    // Give the spring an upward recoil velocity: the natural frequency of the
    // spring gives the speed at which it snaps back.
    double omega = std::sqrt(springConstant / compressionMass);
    compressionVelocity = -omega * compression;
}

void Spring::update(double dt) {
    if (compression == 0.0 && compressionVelocity == 0.0) {
        return;
    }

    // Damped harmonic oscillator relaxing the top back to its resting height.
    double omega = std::sqrt(springConstant / compressionMass);
    double damping = 0.2; // damping ratio: low = a visible recoil wobble
    double accel = -omega * omega * compression - 2.0 * damping * omega * compressionVelocity;

    compressionVelocity += accel * dt;
    compression += compressionVelocity * dt;

    if (compression < 0.0) {
        compression = 0.0;
        compressionVelocity = 0.0;
    }
}

bool detectSpringHit(const RigidBody& ball, const Spring& spring, bool& hitFromAbove) {
    double r = ball.getRadius();
    auto pos = ball.getPosition();
    auto prev = ball.getPrevPosition();

    double springLeft = spring.getX();
    double springRight = spring.getX() + spring.getWidth();
    double springTop = spring.getCurrentTop();
    double springBottom = spring.getY();

    // Horizontal overlap, taking the swept horizontal extent into account so a
    // fast-moving ball cannot skip past the spring's sides either.
    double minX = std::min(prev[0], pos[0]) - r;
    double maxX = std::max(prev[0], pos[0]) + r;
    if (maxX < springLeft || minX > springRight) {
        return false;
    }

    double prevBottom = prev[1] - r;
    double prevTop = prev[1] + r;
    double curBottom = pos[1] - r;
    double curTop = pos[1] + r;

    // Swept tests: did the ball's path cross one of the spring's faces this step?
    // This is what catches tunneling — checking only the current position would
    // miss a ball that jumped all the way through in a single step.
    bool crossedTopFromAbove = prevBottom >= springTop && curBottom <= springTop;
    bool crossedBottomFromBelow = prevTop <= springBottom && curTop >= springBottom;

    if (crossedTopFromAbove) {
        hitFromAbove = true;
        return true;
    }
    if (crossedBottomFromBelow) {
        hitFromAbove = false;
        return true;
    }

    // Current position overlaps the spring box (ball is partially inside it).
    bool inside = curBottom <= springTop && curTop >= springBottom;
    if (inside) {
        // Decide which face the ball came through using the previous position.
        if (prevBottom >= springTop) {
            hitFromAbove = true;
        } else if (prevTop <= springBottom) {
            hitFromAbove = false;
        } else {
            // Was already inside: exit through the face it is least embedded in.
            hitFromAbove = (springTop - curBottom) <= (curTop - springBottom);
        }
        return true;
    }

    return false;
}

void applySpringImpulse(RigidBody& ball, Spring& spring, bool hitFromAbove) {
    auto vel = ball.getVelocity();
    double r = ball.getRadius();
    double springTop = spring.getCurrentTop();
    double springBottom = spring.getY();

    if (hitFromAbove) {
        // Ball hit the top face of the spring.
        double impactSpeed = -vel[1];

        // Slow downward contact: just rest on the spring instead of bouncing,
        // so it does not jitter forever on tiny impacts.
        if (impactSpeed < 1.0) {
            ball.setPositionY(springTop + r);
            if (vel[1] < 0.0) {
                ball.setVelocityY(0.0);
            }
            return;
        }

        // Physically-motivated max compression of a mass m hitting a spring
        // with constant k at speed v: x = v * sqrt(m / k).
        double compressionDepth = impactSpeed * std::sqrt(ball.getMass() / spring.getSpringConstant());
        spring.compress(compressionDepth);

        // Resolve the position so the ball is not partially inside the spring.
        ball.setPositionY(spring.getCurrentTop() + r);

        // Return the stored impact energy back to the ball.
        ball.setVelocityY(impactSpeed * spring.getRestitution());
    } else {
        // Ball hit the bottom face of the spring (e.g. upward gravity).
        ball.setPositionY(springBottom - r);
        if (vel[1] > 0.0) {
            ball.setVelocityY(-vel[1] * spring.getRestitution());
        }
    }
}
