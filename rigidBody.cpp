#include "rigidBody.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

double springConstant = 100.0; // N/m - initialized with default value
double e = 0.7; //coefficent of restitution should be 1 for perfectly elastic collision

void RigidBody::updateColorFromSpeed() {
    const double vx = velocity_[0];
    const double vy = velocity_[1];
    const double speed = std::sqrt(vx * vx + vy * vy);

    const double maxSpeedForColor = 500.0;
    double t = speed / maxSpeedForColor;
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;

    auto lerp = [](int a, int b, double u) {
        return static_cast<int>(std::round(a + (b - a) * u));
    };

    const std::array<int, 3> purple = {170, 60, 255};
    const std::array<int, 3> blue = {40, 190, 255};
    const std::array<int, 3> red = {255, 40, 40};

    int r = 0;
    int g = 0;
    int b = 0;

    if (t <= 0.5) {
        const double u = t / 0.5;
        r = lerp(purple[0], blue[0], u);
        g = lerp(purple[1], blue[1], u);
        b = lerp(purple[2], blue[2], u);
    } else {
        const double u = (t - 0.5) / 0.5;
        r = lerp(blue[0], red[0], u);
        g = lerp(blue[1], red[1], u);
        b = lerp(blue[2], red[2], u);
    }

    color_ = {r, g, b, 255};
}

// RigidBody constructor
RigidBody::RigidBody(double radius, double mass, 
                     std::array<double, 2> position,
                     std::array<double, 2> velocity)
    : radius_(radius), position_(position), velocity_(velocity)
{
    // Initialize mass and calculate inverse mass
    mass_ = mass;
    invMass_ = (mass > 0.0) ? (1.0 / mass) : 0.0;
    
    // Initialize other members
    acceleration_ = {0.0, 0.0};
    force_ = {0.0, 0.0};
    updateColorFromSpeed();
}

// RigidBody method implementation
void RigidBody::numericalIntegration(double dt) {
    if (mass_ == 0.0) return;

    acceleration_[0] = force_[0] * invMass_;
    acceleration_[1] = force_[1] * invMass_;

    // Use old velocity for position update (constant-accel kinematics)
    position_[0] += velocity_[0] * dt + 0.5 * acceleration_[0] * dt * dt;
    position_[1] += velocity_[1] * dt + 0.5 * acceleration_[1] * dt * dt;

    velocity_[0] += acceleration_[0] * dt;
    velocity_[1] += acceleration_[1] * dt;

    updateColorFromSpeed();

    force_[0] = 0.0;
    force_[1] = 0.0;
}


