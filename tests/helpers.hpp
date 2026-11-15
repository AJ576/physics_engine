#pragma once
#include "physics/physics.hpp"
#include "physics/scene.hpp"

#include <cstdint>
#include <cstring>

namespace testing {

// Restores the global restitution after a test changes it.
struct RestitutionGuard {
    double saved = getRestitution();
    explicit RestitutionGuard(double e) { setRestitution(e); }
    ~RestitutionGuard() { setRestitution(saved); }
};

inline void load(WorldPhysics& w, const phys::SceneSpec& s) {
    w.clearBodies();
    w.setBorder({s.width, s.height});
    for (const auto& b : s.bodies) w.addBody(RigidBody(b.radius, b.mass, {b.x, b.y}, {b.vx, b.vy}));
}

// FNV-1a over the raw bytes of every body's position and velocity.
inline std::uint64_t stateHash(const WorldPhysics& w) {
    std::uint64_t h = 1469598103934665603ull;
    auto mix = [&h](double d) {
        unsigned char bytes[sizeof d];
        std::memcpy(bytes, &d, sizeof d);
        for (unsigned char c : bytes) { h ^= c; h *= 1099511628211ull; }
    };
    for (const auto& b : w.getBodies()) {
        mix(b.getPosition()[0]); mix(b.getPosition()[1]);
        mix(b.getVelocity()[0]); mix(b.getVelocity()[1]);
    }
    return h;
}

}  // namespace testing
