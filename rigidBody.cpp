#include "rigidBody.hpp"
#include <iostream>

float springConstant = 100.0f; // N/m - initialized with default value

// RigidBody method implementation
void RigidBody::numericalIntegration(float dt) {
    if (mass == 0.0f) return;

    // Numerical Integration. 
    // We are using Semi-Implicit Euler Method for integration. So, we need to update the velocity first and then the position.
    // This is because the velocity is dependent on the acceleration, and the position is dependent on the velocity.

    // Acceleration
    acceleration[0] = force[0] / mass;
    acceleration[1] = force[1] / mass;

    // Velocity
    velocity[0] += acceleration[0] * dt;
    velocity[1] += acceleration[1] * dt;
    
    // Position
    position[0] += velocity[0] * dt;
    position[1] += velocity[1] * dt;

    // Reset force
    force[0] = 0.0f;
    force[1] = 0.0f;
}

// TimeManager method implementations
TimeManager::TimeManager(){
    last_time = Clock::now();
}

void TimeManager::tick()
{
    auto current_time = Clock::now();
    Duration passed_time = current_time-last_time;
    last_time = current_time;

    accumulator += passed_time.count();
}

bool TimeManager::physicsTime()
{
    if(accumulator > fixedDeltaTime)
    {
        accumulator-=fixedDeltaTime;
        return true;
    }
    return false;
}

// Free function implementations
bool areColliding(const RigidBody& body1, const RigidBody& body2) {
    float dx = body1.position[0] - body2.position[0];
    float dy = body1.position[1] - body2.position[1];
    float distSq = dx * dx + dy * dy;
    float rSum = body1.radius + body2.radius;
    return distSq < rSum * rSum;
}

void calculateForce(RigidBody& body1, RigidBody& body2) {
    if (areColliding(body1, body2)) {
        // Hooke's Law for Elastic Collision
        // F = -k * (x - x0)
        // Where k is the spring constant and x0 is the equilibrium position.
        // We are not using a spring constant, so we need to calculate the new force based on the new position and velocity.
        body1.force[0] = -springConstant * (body1.position[0] - body2.position[0]);
        body1.force[1] = -springConstant * (body1.position[1] - body2.position[1]);
        body2.force[0] = -springConstant * (body2.position[0] - body1.position[0]);
        body2.force[1] = -springConstant * (body2.position[1] - body1.position[1]);
    }
}

void runPhysics(RigidBody& body1, RigidBody& body2, const TimeManager& TIME)
{
    // Checking if colliding
    // If colliding, calculate the force between the two bodies.
    if (areColliding(body1, body2)) {
        calculateForce(body1, body2);
    }
    body1.numericalIntegration(TIME.fixedDeltaTime); // Update the position and velocity of the first body.
    body2.numericalIntegration(TIME.fixedDeltaTime); // Update the position and velocity of the second body.
    // print the position and velocity of the two bodies.
    std::cout<<"Body 1 Position: "<<body1.position[0]<<", "<<body1.position[1]<<std::endl;
    std::cout<<"Body 1 Velocity: "<<body1.velocity[0]<<", "<<body1.velocity[1]<<std::endl;
    std::cout<<"Body 2 Position: "<<body2.position[0]<<", "<<body2.position[1]<<std::endl;
    std::cout<<"Body 2 Velocity: "<<body2.velocity[0]<<", "<<body2.velocity[1]<<std::endl;
}
