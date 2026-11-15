// One test per bug found in the Phase 1 audit. Each is written against the
// behavior we want. Tests marked doctest::should_fail() document a bug that
// is still present: doctest reports them as passing only while they fail, so
// the fix commit must remove the marker.
#include <doctest/doctest.h>

#include "helpers.hpp"

#include <cmath>

using testing::RestitutionGuard;

TEST_CASE("bug: fast circles tunnel through each other (discrete detection only)" * doctest::should_fail()) {
    // Two r = 2 circles, 7 px apart, closing at 600 px/s (10 px per step,
    // more than the 4 px sum of radii). They must bounce, not pass through.
    RestitutionGuard e(1.0);
    WorldPhysics w({800, 600});
    w.setGravity({0.0, 0.0});
    w.addBody(RigidBody(2.0, 1.0, {396.5, 300.0}, {300.0, 0.0}));
    w.addBody(RigidBody(2.0, 1.0, {403.5, 300.0}, {-300.0, 0.0}));
    TimeManager t;
    for (int s = 0; s < 10; ++s) w.runPhysics(t);
    CHECK(w.getBodies()[0].getPosition()[0] < w.getBodies()[1].getPosition()[0]);
}

TEST_CASE("bug: an inelastic pile never comes to rest (positional correction injects energy)" *
          doctest::should_fail()) {
    // 400 bodies dropped from rest into a 100 x 100 box with e = 0. After
    // 10 s an inelastic pile should be at rest: RMS speed below a quarter of
    // the speed gravity adds in one step, $\tfrac14 g\,\Delta t \approx 4.1$ px/s.
    RestitutionGuard e(0.0);
    WorldPhysics w({100.0, 100.0});
    const double g = 980.0, dt = 1.0 / 60.0;
    w.setGravity({0.0, -g});
    phys::SplitMix64 rng(11);
    for (int i = 0; i < 400; ++i)
        w.addBody(RigidBody(2.0, 1.0, {rng.uniform(2.0, 98.0), rng.uniform(2.0, 98.0)}, {0.0, 0.0}));
    TimeManager t;
    for (int s = 0; s < 600; ++s) w.runPhysics(t);
    double sumV2 = 0.0;
    for (const auto& b : w.getBodies()) sumV2 += b.getVelocity()[0] * b.getVelocity()[0] + b.getVelocity()[1] * b.getVelocity()[1];
    const double rms = std::sqrt(sumV2 / 400.0);
    MESSAGE("RMS speed after 10 s: " << rms << " px/s (limit " << 0.25 * g * dt << ")");
    CHECK(rms < 0.25 * g * dt);
}

TEST_CASE("bug: the integrator gains energy on a spring (not semi-implicit Euler)" * doctest::should_fail()) {
    // $\ddot{x} = -\omega^2 x$ with $\omega = 10$ rad/s, $\Delta t = 1/60$, $10^5$ steps.
    // A symplectic integrator keeps $E$ within $O(\omega\Delta t)$ of $E_0$.
    const double k = 100.0, m = 1.0, dt = 1.0 / 60.0;
    RigidBody b(1.0, m, {1.0, 0.0}, {0.0, 0.0});
    const double e0 = 0.5 * k * 1.0;
    for (int s = 0; s < 100000; ++s) {
        b.setForce({-k * b.getPosition()[0], 0.0});
        b.numericalIntegration(dt);
    }
    const double x = b.getPosition()[0], v = b.getVelocity()[0];
    const double e = 0.5 * m * v * v + 0.5 * k * x * x;
    MESSAGE("E/E0 after 1e5 steps = " << e / e0);
    CHECK(std::abs(e / e0 - 1.0) < 0.1);
}

namespace {
int stepsFor(double frameSeconds) {
    TimeManager t;
    t.advance(frameSeconds);
    int n = 0;
    while (t.physicsTime()) ++n;
    return n;
}
}  // namespace

TEST_CASE("fixed: steps per frame are capped (spiral of death)") {
    CHECK(stepsFor(0.24) <= 8);
}

TEST_CASE("fixed: a longer hitch never runs fewer physics steps than a shorter one") {
    CHECK(stepsFor(0.26) >= stepsFor(0.24));
}

TEST_CASE("bug: grid misses collisions when cell size < body diameter" * doctest::should_fail()) {
    // K isolated overlapping pairs of r = 4 bodies with cell size 3.
    WorldPhysics w({100000.0, 100000.0});
    w.setGravity({0.0, 0.0});
    w.setGridSize(3.0);
    const int K = 100;
    for (int k = 0; k < K; ++k) {
        const double cx = 1000.0 + 200.0 * k;
        w.addBody(RigidBody(4.0, 1.0, {cx, 1000.0}, {0.0, 0.0}));
        w.addBody(RigidBody(4.0, 1.0, {cx + 7.5, 1000.0}, {0.0, 0.0}));
    }
    TimeManager t;
    CHECK(w.runPhysicsProfiled(t).collisionsResolved == static_cast<size_t>(K));
}

TEST_CASE("bug (UBSan only): huge coordinates overflow the float->int cell conversion") {
    // floor(1e12 / 10) does not fit in int: undefined behavior at
    // physics.cpp cell computation. Passes in release, aborts under the asan
    // preset (float-cast-overflow) until the cell index is guarded.
    WorldPhysics w({800, 600});
    w.setGravity({0.0, 0.0});
    w.addBody(RigidBody(2.0, 1.0, {1.0e12, 300.0}, {0.0, 0.0}));
    w.addBody(RigidBody(2.0, 1.0, {400.0, 300.0}, {0.0, 0.0}));
    TimeManager t;
    w.runPhysics(t);
    for (const auto& b : w.getBodies()) {
        CHECK(std::isfinite(b.getPosition()[0]));
        CHECK(b.getPosition()[0] <= 800.0);
    }
}
