#pragma once
#include "physics/scene.hpp"
#include "physics/world.hpp"

#include <cstdint>
#include <cstring>

namespace testing {

inline void load(phys::World& w, const phys::SceneSpec& s) {
    w.config().width = s.width;
    w.config().height = s.height;
    for (const auto& b : s.bodies) w.addBody(phys::makeBody(b.radius, b.mass, {b.x, b.y}, {b.vx, b.vy}));
}

// FNV-1a over the raw bytes of every body's position, velocity, angle and spin.
inline std::uint64_t stateHash(const phys::World& w) {
    std::uint64_t h = 1469598103934665603ull;
    auto mix = [&h](double d) {
        unsigned char bytes[sizeof d];
        std::memcpy(bytes, &d, sizeof d);
        for (unsigned char c : bytes) { h ^= c; h *= 1099511628211ull; }
    };
    for (const auto& b : w.bodies()) {
        mix(b.pos.x); mix(b.pos.y); mix(b.vel.x); mix(b.vel.y); mix(b.angle); mix(b.angVel);
    }
    return h;
}

}  // namespace testing
