// Regression tests for the bugs found in the Phase 1 audit of the old
// engine, rewritten for phys::World. A test marked doctest::should_fail()
// documents a bug that is still present (doctest reports it as passing only
// while it fails); the fixing commit removes the marker.
//
// Fixed earlier on the old engine and covered in test_world.cpp:
// timestep cap / monotonic hitch handling, cell-index overflow (UBSan),
// grid missing pairs for small cells, constant-acceleration integrator.
#include <doctest/doctest.h>

#include "physics/scene.hpp"
#include "physics/world.hpp"

#include <cmath>

using phys::makeBody;
using phys::World;
using phys::WorldConfig;

TEST_CASE("bug: fast circles tunnel through each other (discrete detection only)" * doctest::should_fail()) {
    // Two r = 2 circles, 7 px apart, closing at 600 px/s (10 px per step,
    // more than the 4 px sum of radii). They must bounce, not pass through.
    WorldConfig c;
    c.gravity = {0, 0};
    c.restitution = 1.0;
    World w(c);
    w.addBody(makeBody(2.0, 1.0, {396.5, 300.0}, {300.0, 0.0}));
    w.addBody(makeBody(2.0, 1.0, {403.5, 300.0}, {-300.0, 0.0}));
    for (int s = 0; s < 10; ++s) w.step();
    CHECK(w.bodies()[0].pos.x < w.bodies()[1].pos.x);
}

TEST_CASE("fixed: an inelastic pile comes to rest (position correction no longer injects energy)") {
    // 400 bodies dropped from rest into a 100 x 100 box with e = 0. After
    // 10 s an inelastic pile should be at rest: RMS speed below a quarter of
    // the speed gravity adds in one step, $\tfrac14 g\,\Delta t \approx 4.1$ px/s.
    // Old engine: 48.9 px/s.
    WorldConfig c;
    c.width = c.height = 100.0;
    c.restitution = 0.0;
    World w(c);
    phys::SplitMix64 rng(11);
    for (int i = 0; i < 400; ++i) w.addBody(makeBody(2.0, 1.0, {rng.uniform(2.0, 98.0), rng.uniform(2.0, 98.0)}));
    for (int s = 0; s < 600; ++s) w.step();
    double sumV2 = 0.0;
    for (const auto& b : w.bodies()) sumV2 += b.vel.lengthSq();
    const double rms = std::sqrt(sumV2 / 400.0), limit = 0.25 * 980.0 / 60.0;
    MESSAGE("RMS speed after 10 s: " << rms << " px/s (limit " << limit << ")");
    CHECK(rms < limit);
}

TEST_CASE("fixed: semi-implicit Euler keeps spring energy bounded over 1e5 steps (World)") {
    // $\ddot{x} = -\omega^2 x$, $\omega = 10$ rad/s, $\Delta t = 1/60$: force applied by hand each step.
    WorldConfig c;
    c.gravity = {0, 0};
    c.walls = false;
    World w(c);
    const double k = 100.0, m = 1.0;
    w.addBody(makeBody(1.0, m, {1.0, 0.0}));
    for (int s = 0; s < 100000; ++s) {
        w.bodies()[0].force = {-k * w.bodies()[0].pos.x, 0.0};
        w.step();
    }
    const auto& b = w.bodies()[0];
    const double e = 0.5 * m * b.vel.x * b.vel.x + 0.5 * k * b.pos.x * b.pos.x;
    MESSAGE("E/E0 after 1e5 steps = " << e / 50.0);
    CHECK(std::abs(e / 50.0 - 1.0) < 0.1);
}
