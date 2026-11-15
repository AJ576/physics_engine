#pragma once
// The only file the benchmark harness uses to talk to the engine. When the
// engine API changes, only this adapter changes; the timing and statistics
// code in bench_main.cpp stays the same, so before/after numbers are
// produced by identical measurement code.
#include "physics/physics.hpp"
#include "physics/scene.hpp"

#include <cstddef>
#include <memory>

namespace bench {

struct SimConfig {
    double gravityY = 0.0;     // px/s^2 (negative = down)
    double restitution = 1.0;
};

struct StepCounts {
    std::size_t candidatePairs = 0;  // pairs that reached the narrowphase test
    std::size_t contacts = 0;        // pairs found overlapping
};

class Sim {
public:
    Sim(const phys::SceneSpec& scene, const SimConfig& cfg) : world_({scene.width, scene.height}) {
        setRestitution(cfg.restitution);
        world_.setGravity({0.0, cfg.gravityY});
        for (const auto& b : scene.bodies)
            world_.addBody(RigidBody(b.radius, b.mass, {b.x, b.y}, {b.vx, b.vy}));
    }
    void step() { world_.runPhysics(time_); }               // grid broadphase (default)
    void stepBruteForce() { world_.runPhysicsNaive(time_); }  // O(N^2) all pairs
    StepCounts stepCounted() {
        StepProfile p = world_.runPhysicsProfiled(time_);
        return {p.collisionPairsChecked, p.collisionsResolved};
    }
    static const char* engineName() { return "legacy (single-pass projection + impulse, hash grid)"; }

private:
    WorldPhysics world_;
    TimeManager time_;
};

}  // namespace bench
