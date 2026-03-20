#ifndef PHYSICS_HPP
#define PHYSICS_HPP

#include "rigidBody.hpp"
#include <vector>
#include <array>
#include <chrono>
#include <unordered_map>

using Clock = std::chrono::high_resolution_clock;
using TimePoint = std::chrono::time_point<Clock>;
using Duration = std::chrono::duration<double>;

class TimeManager {
    private:
        double accumulator = 0.0;
        TimePoint last_time;

    public:
        const double fixedDeltaTime = 1.0/60.0; //60HZ

        TimeManager();
        void tick();
        bool physicsTime();
        void reset();
};

class WorldPhysics {
private:
    std::vector<RigidBody> bodies;
    std::array<double, 2> border_;
    std::array<double, 2> gravity_ = {0.0, -9.8}; // world gravity (x,y)

    // add back grid members used by physics.cpp
    std::unordered_map<long long, std::vector<RigidBody*>> grid;
    double grid_size = 10;

    void applyGlobalForces(double dt);
    bool areColliding(const RigidBody& body1, const RigidBody& body2,
                    double& nx, double& ny, double& overlap);
    void calculateForce(RigidBody& body1, RigidBody& body2);
    void calculateImpulse(RigidBody& body1, RigidBody& body2, double nx, double ny);
    void resolveCollision(RigidBody& b1, RigidBody& b2);
    void borderCheck(RigidBody& body1);

public:
    WorldPhysics(std::array<double, 2> border);

    void addBody(const RigidBody& body);
    std::vector<RigidBody>& getBodies();
    const std::vector<RigidBody>& getBodies() const;
    void runPhysics(const TimeManager& TIME);
    
    std::array<double, 2> getBorder() const { return border_; }
    void setBorder(std::array<double, 2> border) { border_ = border; }
    void setGravity(const std::array<double, 2>& g) { gravity_ = g; }
    std::array<double, 2> getGravity() const { return gravity_; }
};

#endif // PHYSICS_HPP
