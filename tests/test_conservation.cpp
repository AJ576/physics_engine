// Conservation with e = 1. Every test prints the measured drift.
#include <doctest/doctest.h>

#include "physics/scene.hpp"
#include "physics/world.hpp"
#include "world_helpers.hpp"

#include <cmath>

using phys::Vec2;
using phys::World;
using phys::WorldConfig;

namespace {
struct Drift {
    double ke, keMin, momentum, angular;
    std::size_t contacts;
};
// 2,000 bodies from makeConstantDensityScene(seed 3), gravity off, e = 1, 600 steps (10 s).
Drift elasticRun(double areaPerBody, bool walls, double mu) {
    const auto scene = phys::makeConstantDensityScene(2000, 3, areaPerBody);
    WorldConfig c;
    c.gravity = {0, 0};
    c.restitution = 1.0;
    c.friction = mu;
    World w(c);
    testing::load(w, scene);
    w.config().walls = walls;
    const double ke0 = w.kineticEnergy(), L0 = w.angularMomentum();
    const Vec2 p0 = w.linearMomentum();
    double sumAbsP = 0.0;
    for (const auto& b : w.bodies()) sumAbsP += b.mass * b.vel.length();
    Drift d{0, 0, 0, 0, 0};
    double keMin = ke0;
    for (int s = 0; s < 600; ++s) {
        w.step();
        d.contacts += w.lastStepStats().contacts;
        keMin = std::min(keMin, w.kineticEnergy());
    }
    d.ke = w.kineticEnergy() / ke0 - 1.0;
    d.keMin = keMin / ke0 - 1.0;
    d.momentum = (w.linearMomentum() - p0).length() / sumAbsP;
    d.angular = (w.angularMomentum() - L0) / std::abs(L0);
    return d;
}
}  // namespace

TEST_CASE("conservation: linear momentum to rounding (e = 1, no gravity, no walls, dense)") {
    for (double mu : {0.0, 0.3}) {
        const Drift d = elasticRun(30.0, false, mu);
        MESSAGE("mu=" << mu << " contacts=" << d.contacts << " |dp|/sum|p| = " << d.momentum);
        CHECK(d.contacts > 100000);
        CHECK(d.momentum < 1e-12);
    }
}

TEST_CASE("conservation: kinetic energy within 1% over 10 s at e = 1 (demo density, no walls, no friction)") {
    const Drift d = elasticRun(96.0, false, 0.0);
    MESSAGE("contacts=" << d.contacts << " KE_N/KE_0 - 1 = " << d.ke << " (min " << d.keMin << ")");
    CHECK(std::abs(d.ke) < 0.01);
}

TEST_CASE("conservation (measured, limitation): energy at e = 1 with walls, dense packing, friction; angular momentum") {
    // Regression guards around measured values; the numbers are reported in
    // docs/BENCHMARKS.md. Friction converts translation into spin and then
    // dissipates; dense packing means many simultaneous contacts; angular
    // momentum changes because position-only corrections (split impulse,
    // CCD motion clamp) move bodies without a matching velocity change.
    const Drift walls = elasticRun(96.0, true, 0.0);
    const Drift dense = elasticRun(30.0, false, 0.0);
    const Drift friction = elasticRun(96.0, true, 0.3);
    MESSAGE("walls, mu=0:   KE " << walls.ke << "  contacts " << walls.contacts);
    MESSAGE("dense, mu=0:   KE " << dense.ke << "  dL/|L0| " << dense.angular << "  contacts " << dense.contacts);
    MESSAGE("walls, mu=0.3: KE " << friction.ke << "  contacts " << friction.contacts);
    CHECK(std::abs(walls.ke) < 0.05);
    CHECK(std::abs(dense.ke) < 0.15);
    CHECK(std::abs(dense.angular) < 0.05);
    CHECK(friction.ke < 0.0);  // friction only removes energy
}
