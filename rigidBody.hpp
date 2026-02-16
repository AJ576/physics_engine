#ifndef RIGIDBODY_HPP
#define RIGIDBODY_HPP
#pragma once // read once per programam.
#include <array>
#include <chrono>

using Clock = std::chrono::high_resolution_clock;
using TimePoint = std::chrono::time_point<Clock>;
using Duration = std::chrono::duration<double>;

extern double springConstant; // N/m

class RigidBody {
    private:
        double radius_; // m
        double mass_; // kg
        double invMass_; // 1/kg (inverse mass for physics calculations)
        std::array<double, 2> position_; // m
        std::array<double, 2> velocity_; // m/s
        std::array<double, 2> acceleration_; // m/s^2
        std::array<double, 2> force_; // N

    public:
        // Constructor
        RigidBody(double radius = 1.0, double mass = 1.0, 
                  std::array<double, 2> position = {0.0, 0.0},
                  std::array<double, 2> velocity = {0.0, 0.0});
        // Getters
        double getRadius() const { return radius_; }
        double getMass() const { return mass_; }
        double getInvMass() const { return invMass_; }
        std::array<double, 2> getPosition() const { return position_; }
        std::array<double, 2> getVelocity() const { return velocity_; }
        std::array<double, 2> getAcceleration() const { return acceleration_; }
        std::array<double, 2> getForce() const { return force_; }

        // Setters
        void setRadius(double radius) { radius_ = radius; }
        void setMass(double mass) { 
            mass_ = mass; 
            invMass_ = (mass > 0.0) ? (1.0 / mass) : 0.0;
        }
        void setPosition(const std::array<double, 2>& position) { position_ = position; }
        void setVelocity(const std::array<double, 2>& velocity) { velocity_ = velocity; }
        void setAcceleration(const std::array<double, 2>& acceleration) { acceleration_ = acceleration; }
        void setForce(const std::array<double, 2>& force) { force_ = force; }

        // Individual component setters cuz we lazy
        void setPositionX(double x) { position_[0] = x; }
        void setPositionY(double y) { position_[1] = y; }
        void setVelocityX(double vx) { velocity_[0] = vx; }
        void setVelocityY(double vy) { velocity_[1] = vy; }
        void setForceX(double fx) { force_[0] = fx; }
        void setForceY(double fy) { force_[1] = fy; }

        void numericalIntegration(double dt);
};

class TimeManager {
    private:
        double accumulator = 0.0;
        TimePoint last_time;
    
    public:
        const double fixedDeltaTime = 1.0/120.0; //60HZ

        TimeManager();
        void tick();
        bool physicsTime();
};

bool areColliding(const RigidBody& body1, const RigidBody& body2);
void resolveCollision(const RigidBody b1, const RigidBody& b2);
void calculateForce(RigidBody& body1, RigidBody& body2);
void calculateImpulse(RigidBody& body1, RigidBody& body2);
void runPhysics(RigidBody& body1, RigidBody& body2, const TimeManager& TIME);
void borderCheck(RigidBody& body1,std::array <float,2> border);

#endif // RIGIDBODY_HPP
