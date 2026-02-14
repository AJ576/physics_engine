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
    public:
        float radius; // m
        float mass; // kg
        std::array<float, 2> position; // m
        std::array<float, 2> velocity; // m/s
        std::array<float, 2> acceleration; // m/s^2
        std::array<float, 2> force; // N

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
void calculateForce(RigidBody& body1, RigidBody& body2);
void runPhysics(RigidBody& body1, RigidBody& body2, const TimeManager& TIME);

#endif // RIGIDBODY_HPP
