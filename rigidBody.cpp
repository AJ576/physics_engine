#include "rigidBody.hpp"
#include <iostream>

double springConstant = 100.0; // N/m - initialized with default value
double e = 1; //coefficent of restitution should be 1 for perfectly elastic collision

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
}

// RigidBody method implementation
void RigidBody::numericalIntegration(double dt) {
    if (mass_ == 0.0) return;

    // Numerical Integration. 
    // We are using Semi-Implicit Euler Method for integration. So, we need to update the velocity first and then the position.
    // This is because the velocity is dependent on the acceleration, and the position is dependent on the velocity.

    // Acceleration. Now uses inverse mass. 
    //multiplication is faster than division and usally more accurate. OPTIMIZATIONS!
    acceleration_[0] = force_[0] * invMass_;
    acceleration_[1] = force_[1] * invMass_;

    // Velocity
    velocity_[0] += acceleration_[0] * dt;
    velocity_[1] += acceleration_[1] * dt;
    
    // Position
    position_[0] += velocity_[0] * dt;
    position_[1] += velocity_[1] * dt;

    // Reset force
    force_[0] = 0.0;
    force_[1] = 0.0;
}


