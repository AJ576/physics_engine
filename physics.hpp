#ifndef PHYSICS_HPP
#define PHYSICS_HPP

#include "rigidBody.hpp"
#include "spring.hpp"
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

struct StepProfile {
    double forcesTimeUs = 0.0;
    double integrationTimeUs = 0.0;
    double gridBuildTimeUs = 0.0;
    double collisionTimeUs = 0.0;
    double springTimeUs = 0.0;
    double borderTimeUs = 0.0;
    double totalTimeUs = 0.0;
    size_t collisionPairsChecked = 0;
    size_t collisionsResolved = 0;
};

class WorldPhysics {
    
    private:
        std::vector<RigidBody> bodies;
        std::array<double, 2> border_;
        std::array<double, 2> gravity_ = {0.0, -9.8}; // world gravity (x,y)
        std::vector<Spring> springs_; // Global vector of springs.
        std::unordered_map<long long, std::vector<RigidBody*>> grid;
        double grid_size = 10;

    public:
        WorldPhysics(std::array<double, 2> border);

        void addBody(const RigidBody& body);
        void clearBodies();
        std::vector<RigidBody>& getBodies();
        const std::vector<RigidBody>& getBodies() const;

        void addSpring(const Spring& spring);
        void clearSprings();
        const std::vector<Spring>& getSprings() const;

        void runPhysics(const TimeManager& TIME);
        void runPhysicsNaive(const TimeManager& TIME);
        StepProfile runPhysicsProfiled(const TimeManager& TIME);
        StepProfile runPhysicsNaiveProfiled(const TimeManager& TIME);
        
        std::array<double, 2> getBorder() const { return border_; }
        void setBorder(std::array<double, 2> border) { border_ = border; }
        void setGravity(const std::array<double, 2>& g) { gravity_ = g; }
        std::array<double, 2> getGravity() const { return gravity_; }

        double getGridSize() const { return grid_size; }
        void setGridSize(double size) { grid_size = size; }

        // Energy and momentum metrics for physical validation & benchmarking
        double calculateTotalKineticEnergy() const;
        double calculateTotalPotentialEnergy() const;
        double calculateTotalEnergy() const;
        std::array<double, 2> calculateTotalMomentum() const;

        // Core physics methods exposed for granular unit testing & benchmarking
        void applyGlobalForces();
        bool areColliding(const RigidBody& body1, const RigidBody& body2,
                          double& nx, double& ny, double& overlap);
        void calculateForce(RigidBody& body1, RigidBody& body2);
        void calculateImpulse(RigidBody& body1, RigidBody& body2, double nx, double ny);
        bool resolveCollision(RigidBody& b1, RigidBody& b2);
        void borderCheck(RigidBody& body1, double dt);
};
#endif // PHYSICS_HPP
