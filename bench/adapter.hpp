#pragma once
// The only file the benchmark harness uses to talk to the engine. When the
// engine API changes, only this adapter changes; the timing and statistics
// code in bench_main.cpp stays the same, so before/after numbers are
// produced by identical measurement code.
#include "physics/scene.hpp"
#include "physics/world.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace bench {

struct SimConfig {
    double gravityY = 0.0;  // px/s^2 (negative = down)
    double restitution = 1.0;
    double cellSize = 0.0;  // 0 = engine default
    int threads = 1;
};

struct StepCounts {
    std::size_t candidatePairs = 0;  // pairs that reached the narrowphase test
    std::size_t contacts = 0;        // body-body contacts created
};

class Sim {
public:
    Sim(const phys::SceneSpec& scene, const SimConfig& cfg) {
        phys::WorldConfig c;
        c.width = scene.width;
        c.height = scene.height;
        c.gravity = {0.0, cfg.gravityY};
        c.restitution = cfg.restitution;
        if (cfg.cellSize > 0.0) c.cellSize = cfg.cellSize;
        c.threads = cfg.threads;
        world_ = phys::World(c);
        for (const auto& b : scene.bodies) world_.addBody(phys::makeBody(b.radius, b.mass, {b.x, b.y}, {b.vx, b.vy}));
    }
    void step() { world_.step(); }
    void stepBruteForce() {
        world_.config().broadphase = phys::Broadphase::BruteForce;
        world_.step();
        world_.config().broadphase = phys::Broadphase::Grid;
    }
    StepCounts stepCounted() {
        world_.step();
        return {world_.lastStepStats().candidatePairs, world_.lastStepStats().contacts};
    }
    // FNV-1a over every body's position, velocity, angle and spin: equal
    // hashes mean bit-identical simulation results.
    std::uint64_t stateHash() const {
        std::uint64_t h = 1469598103934665603ull;
        auto mix = [&h](double d) {
            unsigned char bytes[sizeof d];
            std::memcpy(bytes, &d, sizeof d);
            for (unsigned char c : bytes) { h ^= c; h *= 1099511628211ull; }
        };
        for (const auto& b : world_.bodies()) {
            mix(b.pos.x); mix(b.pos.y); mix(b.vel.x); mix(b.vel.y); mix(b.angle); mix(b.angVel);
        }
        return h;
    }
    static const char* engineName() { return "phys::World (sequential impulses, 8 iterations, swept-AABB hash grid)"; }

private:
    phys::World world_;
};

}  // namespace bench
