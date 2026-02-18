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
    std::array<double, 2> pos1 = body1.getPosition();
    std::array<double, 2> pos2 = body2.getPosition();
    double dx = pos1[0] - pos2[0];
    double dy = pos1[1] - pos2[1];
    double distSq = dx * dx + dy * dy;
    double rSum = body1.getRadius() + body2.getRadius();
    return distSq < rSum * rSum;
}

void calculateForce(RigidBody& body1, RigidBody& body2) {
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

void calculateImpulse(RigidBody&body1, RigidBody& body2)
{
    auto pos1 = body1.getPosition();
    auto pos2 = body2.getPosition();

    auto vel1 = body1.getVelocity();
    auto vel2 = body2.getVelocity();

    //find the collision vector between the 2
    double dx = pos2[0] - pos1[0];
    double dy = pos2[1] - pos1[1];

    //normalize it
    double distance = sqrt(dx*dx + dy*dy); // find distance between the 2 so we can nromalize
    
    if (distance == 0)
    {
        return;
    }

    double nx = dx/distance;
    double ny = dy/distance;

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

    double j = numerator/invMassSum; //impulse

    //now calculate new velcoties and set them
    body1.setVelocityX(vel1[0] - ((j * body1.getInvMass()) * nx));
    body1.setVelocityY(vel1[1] - ((j * body1.getInvMass()) * ny));

    body2.setVelocityX(vel2[0] + ((j * body2.getInvMass()) * nx));
    body2.setVelocityY(vel2[1] + ((j * body2.getInvMass()) * ny));
    
}

void resolveCollision( RigidBody& b1, RigidBody& b2)
{
    //get vector between the 2 centers.
    //NOTE this is an arrow pointing from b1 to b2
    //so in the final calc we subtract from b1 and add to b2 along the normal.
    std::array<double, 2> pos1 = b1.getPosition();
    std::array<double, 2> pos2 = b2.getPosition();
    double dx = pos2[0] - pos1[0];
    double dy = pos2[1] - pos1[1];

    //now get distance that currently is AND  min Distance that is legal
    double distance = sqrt(dx*dx + dy*dy);
    double minDistance = b1.getRadius() + b2.getRadius();

    //if phasing
    if (distance < minDistance) {
        double nx, ny;
        // calculate the Normal (Unit Vector)
        // use distance to make sure the vector length is 1.0
        if (distance == 0)
        {
            nx = 1;
            ny = 0.0;
        }
        else
        {
            nx = dx / distance; 
            ny = dy / distance;
        }
       
        // calculate the overlap
        double overlap = minDistance - distance;

        //find total inv mass sum
        double invMassSum = b1.getInvMass() + b2.getInvMass();

        //we cooked if this happens
        if(invMassSum == 0)
        {
            return;
        }

        //now move them along the normal based on their respective masses.
        //find out how much they move by mulaiplying the overlap to the individual InvMass/sum Invmass
        double move1 = overlap*(b1.getInvMass()/invMassSum);
        double move2 = overlap*(b2.getInvMass()/invMassSum);

        //then find components along x and y by multiplying this to the normal
        double dx1 = nx*move1;
        double dx2 = nx*move2;

        double dy1 = ny*move1;
        double dy2 = ny*move2;

        b1.setPositionX(pos1[0]-dx1);
        b1.setPositionY(pos1[1]-dy1);

        b2.setPositionX(pos2[0]+dx2);
        b2.setPositionY(pos2[1]+dy2);

    }
}

void borderCheck(RigidBody& body1, std::array<double, 2> border) {
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
        else if (pos[i] > border[i] - radius) {
            pos[i] = border[i] - radius;
            vel[i] *= -1.0;
        }
    }
    
    body1.setPosition(pos);
    body1.setVelocity(vel);
}

void runPhysics(std::vector<RigidBody>& bodies, const TimeManager& TIME)
{
    std::array<double, 2> border = {800.0, 600.0};
    
    //First step numerical integration
    for (size_t i = 0; i < bodies.size(); i++) {
        bodies[i].numericalIntegration(TIME.fixedDeltaTime);
    }

    //do this AFTER numerical integration
    for (size_t i = 0; i < bodies.size(); i++) {
        for (size_t j = i+1; j < bodies.size(); j++) {
            if (areColliding(bodies[i], bodies[j])) {
                resolveCollision(bodies[i], bodies[j]);
                //calculateForce(bodies[i], bodies[j]); //DO NOT USE CALC FORCE
                calculateImpulse(bodies[i], bodies[j]);
            }
        }
    }


    //Final step, border check
    for (size_t i = 0; i < bodies.size(); i++) {
        borderCheck(bodies[i],border);
    }
}

