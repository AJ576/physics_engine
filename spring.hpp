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
    double height; // platform height (resting/relaxed height)
    double springConstant; // spring constant. Higher value means more springy.
    double restitution = 0.85; // [0, 1] fraction of impact energy returned to the ball.
    double compression = 0.0; // current downward displacement of the spring top (m).
    double compressionVelocity = 0.0; // how fast the compression is changing (m/s).
    double compressionMass = 1.0; // effective mass that drives the recoil animation.

public:
    Spring(double x, double y, double width, double height, double springConstant = 500.0,
           double restitution = 0.85, double compressionMass = 1.0);
    // Getters
    // Note: getters are const because they do not modify the object.
    double getX() const { return x; }
    double getY() const { return y; }
    double getWidth() const { return width; }
    double getHeight() const { return height; }
    double getSpringConstant() const { return springConstant; }
    double getRestitution() const { return restitution; }
    double getCompression() const { return compression; }

    // Current (possibly compressed) height and the y-coordinate of the top surface.
    // The bottom stays fixed at y while the top moves down during compression.
    double getCurrentHeight() const { return std::max(0.0, height - compression); }
    double getCurrentTop() const { return y + getCurrentHeight(); }

    // Setters
    // Note: setters are not const because they modify the object.
    void setX(double v) { x = v; }
    void setY(double v) { y = v; }
    void setWidth(double v) { width = v; }
    void setHeight(double v) { height = v; }
    void setSpringConstant(double v) { springConstant = v; }
    void setRestitution(double v) { restitution = v; }

    // Compress the spring by `amount` (takes the deepest compression if it is
    // already partly compressed) and give it an upward recoil velocity so it
    // springs back.
    void compress(double amount);

    // Relax the compression back to zero with a damped spring oscillator.
    void update(double dt);
};

// Defined once in spring.cpp (not in header) to avoid duplicate symbols at link time.
// Continuous (swept) collision test: true if the ball contacted the spring this
// step, even if it moved fast enough to tunnel through the spring's top surface.
// hitFromAbove is set to true when the ball hit the top face (from above) and
// false when it hit the bottom face (from below).
bool detectSpringHit(const RigidBody& ball, const Spring& spring, bool& hitFromAbove);

// Resolve the collision: pushes the ball out so it is not partially inside the
// spring, compresses the spring based on the impact, and returns the impact
// energy back to the ball as an upward (or downward) rebound velocity.
void applySpringImpulse(RigidBody& ball, Spring& spring, bool hitFromAbove);
