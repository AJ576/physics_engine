#include "physics.hpp"
#include <iostream>
#include <cmath>

extern double springConstant; // N/m
extern double e; // coefficient of restitution

// TimeManager method implementations
TimeManager::TimeManager(){
    last_time = Clock::now();
}

void TimeManager::tick()
{
    auto current_time = Clock::now();
    Duration passed_time = current_time-last_time;
    last_time = current_time;

    double dt = passed_time.count();

    // CAP: If the frame took longer than 0.25 seconds (resize/lag), 
    // just pretend it took 0.016 seconds (1 frame at 60fps).
    if (dt > 0.25) {
        dt = 0.016; 
    }

    accumulator += dt;
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

void TimeManager::reset() {
    last_time = Clock::now();
    accumulator = 0.0; // Optional: clear the lag buffer too
}

// WorldPhysics method implementations
WorldPhysics::WorldPhysics(std::array<double, 2> border) : border_(border) {}

// Free function implementations -> WorldPhysics method implementations
bool WorldPhysics::areColliding(const RigidBody& body1, const RigidBody& body2,
                                double& nx, double& ny, double& overlap) {
    std::array<double, 2> pos1 = body1.getPosition();
    std::array<double, 2> pos2 = body2.getPosition();

    double dx = pos2[0] - pos1[0];
    double dy = pos2[1] - pos1[1];

    double distanceSq = dx * dx + dy * dy;
    double minDistance = body1.getRadius() + body2.getRadius();
    double minDistanceSq = minDistance * minDistance;

    if (distanceSq >= minDistanceSq) {
        overlap = 0.0;
        nx = 0.0;
        ny = 0.0;
        return false;
    }

    double distance = std::sqrt(distanceSq);

    // calculate the Normal (Unit Vector)
    // use distance to make sure the vector length is 1.0
    if (distance == 0.0) {
        nx = 1.0;
        ny = 0.0;
        overlap = minDistance;
    } else {
        nx = dx / distance;
        ny = dy / distance;
        overlap = minDistance - distance;
    }

    return true;
}

void WorldPhysics::calculateForce(RigidBody& body1, RigidBody& body2) {
    // Hooke's Law for Elastic Collision
    // F = -k * (x - x0)
    // Where k is the spring constant and x0 is the equilibrium position.
    std::array<double, 2> pos1 = body1.getPosition();
    std::array<double, 2> pos2 = body2.getPosition();
    
    body1.setForceX(springConstant * (pos1[0] - pos2[0]));
    body1.setForceY(springConstant * (pos1[1] - pos2[1]));
    body2.setForceX(springConstant * (pos2[0] - pos1[0]));
    body2.setForceY(springConstant * (pos2[1] - pos1[1]));
}

void WorldPhysics::calculateImpulse(RigidBody& body1, RigidBody& body2, double nx, double ny)
{
    auto vel1 = body1.getVelocity();
    auto vel2 = body2.getVelocity();

    //now find velocity relative to each other
    auto v_rel_x = vel2[0] - vel1[0];
    auto v_rel_y = vel2[1] - vel1[1];

    //find the nomral by doing a dot product
    double v_rel_normal = (v_rel_x * nx) + (v_rel_y * ny);

    //if they are already seperating or sliding no need to apply an oppsite impulse over and over.
    if (v_rel_normal >= 0)
    {
        return;
    }

    //calculate impuse by formula
    //      −(1 + e) * v_rel_normal
    //j = --------------------------------
    //      (1/m1 + 1/m2)

    double numerator = -1.0 * (1.0 + e) * v_rel_normal;
    double invMassSum = body1.getInvMass() + body2.getInvMass();

    //if both inv masses are 0 its imovable vs imovable. this means sim broke, 
    //no need to break it more with calculations
    if (invMassSum == 0)
    {
        return;
    }

    double j = numerator / invMassSum; //impulse

    //now calculate new velcoties and set them
    body1.setVelocityX(vel1[0] - ((j * body1.getInvMass()) * nx));
    body1.setVelocityY(vel1[1] - ((j * body1.getInvMass()) * ny));

    body2.setVelocityX(vel2[0] + ((j * body2.getInvMass()) * nx));
    body2.setVelocityY(vel2[1] + ((j * body2.getInvMass()) * ny));
}

void WorldPhysics::resolveCollision(RigidBody& b1, RigidBody& b2)
{
    double nx = 0.0, ny = 0.0, overlap = 0.0;

    // if not colliding, return immediately
    if (!areColliding(b1, b2, nx, ny, overlap)) {
        return;
    }

    std::array<double, 2> pos1 = b1.getPosition();
    std::array<double, 2> pos2 = b2.getPosition();

    //find total inv mass sum
    double invMassSum = b1.getInvMass() + b2.getInvMass();

    //we cooked if this happens
    if(invMassSum == 0)
    {
        return;
    }

    //now move them along the normal based on their respective masses.
    //find out how much they move by mulaiplying the overlap to the individual InvMass/sum Invmass
    double move1 = overlap * (b1.getInvMass() / invMassSum);
    double move2 = overlap * (b2.getInvMass() / invMassSum);

    //then find components along x and y by multiplying this to the normal
    double dx1 = nx * move1;
    double dx2 = nx * move2;

    double dy1 = ny * move1;
    double dy2 = ny * move2;

    b1.setPositionX(pos1[0] - dx1);
    b1.setPositionY(pos1[1] - dy1);

    b2.setPositionX(pos2[0] + dx2);
    b2.setPositionY(pos2[1] + dy2);

    // after positional correction, resolve velocity with impulse
    calculateImpulse(b1, b2, nx, ny);
}

void WorldPhysics::borderCheck(RigidBody& body1) {
    double radius = body1.getRadius();
    std::array<double, 2> pos = body1.getPosition();
    std::array<double, 2> vel = body1.getVelocity();

    for (int i = 0; i < 2; ++i) {
        // Lower bound (0 + radius)
        if (pos[i] < radius) {
            pos[i] = radius; 
            vel[i] *= -1.0;
        }
        // Upper bound (border - radius)
        else if (pos[i] > border_[i] - radius) {
            pos[i] = border_[i] - radius;
            vel[i] *= -1.0;
        }
    }
    
    body1.setPosition(pos);
    body1.setVelocity(vel);
}

void WorldPhysics::addBody(const RigidBody& body) {
    bodies.push_back(body);
}

std::vector<RigidBody>& WorldPhysics::getBodies() {
    return bodies;
}

const std::vector<RigidBody>& WorldPhysics::getBodies() const {
    return bodies;
}

void WorldPhysics::runPhysics(const TimeManager& TIME)
{
    //First step numerical integration
    for (size_t i = 0; i < bodies.size(); i++) {
        bodies[i].numericalIntegration(TIME.fixedDeltaTime);
    }

    //do this AFTER numerical integration
    for (size_t i = 0; i < bodies.size(); i++) {
        for (size_t j = i+1; j < bodies.size(); j++) {
            resolveCollision(bodies[i], bodies[j]);
            //calculateForce(bodies[i], bodies[j]); //DO NOT USE CALC FORCE
        }
    }

    //Final step, border check
    for (size_t i = 0; i < bodies.size(); i++) {
        borderCheck(bodies[i]);
    }
}

