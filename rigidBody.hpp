#ifndef RIGIDBODY_HPP
#define RIGIDBODY_HPP
#pragma once // read once per programam.
#include <array>
#include <chrono>

using Clock = std::chrono::high_resolution_clock;
using TimePoint = std::chrono::time_point<Clock>;
using Duration = std::chrono::duration<double>;

extern float springConstant; // N/m

class RigidBody {
    private:
        float radius_; // m
        float mass_; // kg
        std::array<float, 2> position_; // m
        std::array<float, 2> velocity_; // m/s
        std::array<float, 2> acceleration_; // m/s^2
        std::array<float, 2> force_; // N

    public:
        // Getters
        float getRadius() const { return radius_; }
        float getMass() const { return mass_; }
        std::array<float, 2> getPosition() const { return position_; }
        std::array<float, 2> getVelocity() const { return velocity_; }
        std::array<float, 2> getAcceleration() const { return acceleration_; }
        std::array<float, 2> getForce() const { return force_; }

        // Setters
        void setRadius(float radius) { radius_ = radius; }
        void setMass(float mass) { mass_ = mass; }
        void setPosition(const std::array<float, 2>& position) { position_ = position; }
        void setVelocity(const std::array<float, 2>& velocity) { velocity_ = velocity; }
        void setAcceleration(const std::array<float, 2>& acceleration) { acceleration_ = acceleration; }
        void setForce(const std::array<float, 2>& force) { force_ = force; }

        // Individual component setters cuz we lazy
        void setPositionX(float x) { position_[0] = x; }
        void setPositionY(float y) { position_[1] = y; }
        void setVelocityX(float vx) { velocity_[0] = vx; }
        void setVelocityY(float vy) { velocity_[1] = vy; }
        void setForceX(float fx) { force_[0] = fx; }
        void setForceY(float fy) { force_[1] = fy; }

        void numericalIntegration(float dt);
};

class TimeManager {
    private:
        double accumulator = 0.0;
        TimePoint last_time;
    
    public:
        const double fixedDeltaTime = 1.0/60.0; //60HZ

        TimeManager();
        void tick();
        bool physicsTime();
};

bool areColliding(const RigidBody& body1, const RigidBody& body2);
void resolveCollision(const RigidBody b1, const RigidBody& b2);
void calculateForce(RigidBody& body1, RigidBody& body2);
void runPhysics(RigidBody& body1, RigidBody& body2, const TimeManager& TIME);
void borderCheck(RigidBody& body1,std::array <float,2> border);

#endif // RIGIDBODY_HPP
