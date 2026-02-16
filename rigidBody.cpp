#include "rigidBody.hpp"
#include <iostream>

float springConstant = 100.0f; // N/m - initialized with default value

// RigidBody method implementation


void RigidBody::numericalIntegration(float dt) {
    if (mass_ == 0.0f) return;

    // Numerical Integration. 
    // We are using Semi-Implicit Euler Method for integration. So, we need to update the velocity first and then the position.
    // This is because the velocity is dependent on the acceleration, and the position is dependent on the velocity.

    // Acceleration
    acceleration_[0] = force_[0] / mass_;
    acceleration_[1] = force_[1] / mass_;

    // Velocity
    velocity_[0] += acceleration_[0] * dt;
    velocity_[1] += acceleration_[1] * dt;
    
    // Position
    position_[0] += velocity_[0] * dt;
    position_[1] += velocity_[1] * dt;

    // Reset force
    force_[0] = 0.0f;
    force_[1] = 0.0f;
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
    std::array<float, 2> pos1 = body1.getPosition();
    std::array<float, 2> pos2 = body2.getPosition();
    float dx = pos1[0] - pos2[0];
    float dy = pos1[1] - pos2[1];
    float distSq = dx * dx + dy * dy;
    float rSum = body1.getRadius() + body2.getRadius();
    return distSq < rSum * rSum;
}

void calculateForce(RigidBody& body1, RigidBody& body2) {
    // Hooke's Law for Elastic Collision
    // F = -k * (x - x0)
    // Where k is the spring constant and x0 is the equilibrium position.
    std::array<float, 2> pos1 = body1.getPosition();
    std::array<float, 2> pos2 = body2.getPosition();
    
    body1.setForceX(springConstant * (pos1[0] - pos2[0]));
    body1.setForceY(springConstant * (pos1[1] - pos2[1]));
    body2.setForceX(springConstant * (pos2[0] - pos1[0]));
    body2.setForceY(springConstant * (pos2[1] - pos1[1]));
}


void resolveCollision( RigidBody& b1, RigidBody& b2)
{
    //get vector between the 2 centers.
    //NOTE this is an arrow pointing from b1 to b2
    std::array<float, 2> pos1 = b1.getPosition();
    std::array<float, 2> pos2 = b2.getPosition();
    float dx = pos2[0] - pos1[0];
    float dy = pos2[1] - pos1[1];

    //now get distance that currently is AND  min Distance that is legal
    float distance = sqrt(dx*dx + dy*dy);
    float minDistance = b1.getRadius() + b2.getRadius();

    //if phasing
    if (distance < minDistance) {
        float nx, ny;
        // calculate the Normal (Unit Vector)
        // use distance to make sure the vector length is 1.0
        if (distance == 0)
        {
            nx = 0.001;
            ny = 0;
        }
        else
        {
            nx = dx / distance; 
            ny = dy / distance;
        }
       
        // calculate the overlap
        float overlap = minDistance - distance;

        // move them along the normal to separate them
        //  move each by half the overlap so they meet in the middle
        float percent = 0.5f; // Push them 50% each
        float separationX = nx * overlap * percent;
        float separationY = ny * overlap * percent;

        b1.setPositionX(pos1[0] - separationX);
        b1.setPositionY(pos1[1] - separationY);
        b2.setPositionX(pos2[0] + separationX);
        b2.setPositionY(pos2[1] + separationY);
    }
}

void borderCheck(RigidBody& body1, std::array<float, 2> border) {
    float radius = body1.getRadius();
    std::array<float, 2> pos = body1.getPosition();
    std::array<float, 2> vel = body1.getVelocity();

    for (int i = 0; i < 2; ++i) {
        // Lower bound (0 + radius)
        if (pos[i] < radius) {
            pos[i] = radius; 
            vel[i] *= -1;
        }
        // Upper bound (border - radius)
        else if (pos[i] > border[i] - radius) {
            pos[i] = border[i] - radius;
            vel[i] *= -1;
        }
    }
    
    body1.setPosition(pos);
    body1.setVelocity(vel);
}

void runPhysics(RigidBody& body1, RigidBody& body2, const TimeManager& TIME)
{
    // Checking if colliding
    // If colliding, calculate the force between the two bodies.
    std::array<float, 2> border = {800, 600};
    borderCheck(body1,border);
    borderCheck(body2,border);
    if (areColliding(body1, body2)) {
        resolveCollision(body1, body2);
        calculateForce(body1, body2);
        
        // std::array<float, 2> pos1 = body1.getPosition();
        // std::array<float, 2> vel1 = body1.getVelocity();
        // std::array<float, 2> pos2 = body2.getPosition();
        // std::array<float, 2> vel2 = body2.getVelocity();
        // std::cout<<"Body 1 Position: "<<pos1[0]<<", "<<pos1[1]<<std::endl;
        // std::cout<<"Body 1 Velocity: "<<vel1[0]<<", "<<vel1[1]<<std::endl;
        // std::cout<<"Body 2 Position: "<<pos2[0]<<", "<<pos2[1]<<std::endl;
        // std::cout<<"Body 2 Velocity: "<<vel2[0]<<", "<<vel2[1]<<std::endl;
    }
    body1.numericalIntegration(TIME.fixedDeltaTime); // Update the position and velocity of the first body.
    body2.numericalIntegration(TIME.fixedDeltaTime); // Update the position and velocity of the second body.

    
}

