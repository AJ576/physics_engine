#include <doctest/doctest.h>

#include "physics/world.hpp"

#include <cmath>

using doctest::Approx;
using phys::makeBody;
using phys::Vec2;
using phys::World;
using phys::WorldConfig;

namespace {
WorldConfig noGravity() {
    WorldConfig c;
    c.gravity = {0, 0};
    c.walls = false;
    return c;
}
// Anchor (static) at the origin, mass m at x0 on a spring (k, c, rest L0).
World massOnSpring(double m, double x0, double v0, double k, double c, double L0) {
    World w(noGravity());
    w.addBody(makeBody(1.0, 0.0, {0, 0}));
    w.addBody(makeBody(1.0, m, {x0, 0}, {v0, 0}));
    w.addSpring({0, 1, L0, k, c});
    return w;
}
}  // namespace

TEST_CASE("spring: one step applies $F = -k(\\ell - \\ell_0) - c\\,\\dot{\\ell}$ to the free end") {
    const double k = 50.0, c = 3.0, m = 2.0, dt = 1.0 / 60.0;
    World w = massOnSpring(m, 30.0, 4.0, k, c, 20.0);  // stretched by 10, moving outward at 4
    w.step();
    const double F = -k * (30.0 - 20.0) - c * 4.0;
    CHECK(w.bodies()[1].vel.x == Approx(4.0 + F / m * dt));
    CHECK(w.bodies()[0].vel.x == 0.0);  // static anchor unaffected
    // Compressed spring pushes outward.
    World u = massOnSpring(m, 15.0, 0.0, k, 0.0, 20.0);
    u.step();
    CHECK(u.bodies()[1].vel.x > 0.0);
}

TEST_CASE("spring: period matches $2\\pi\\sqrt{m/k}$ within 0.5% at $\\Delta t = 1/60$") {
    // Semi-implicit Euler's frequency error is $O(\\omega^2\\Delta t^2)$: here
    // $\\omega = 5$ rad/s, $\\omega\\Delta t = 0.083$.
    const double k = 25.0, m = 1.0, L0 = 100.0;
    World w = massOnSpring(m, L0 + 10.0, 0.0, k, 0.0, L0);
    double prev = 10.0, firstCross = -1.0, lastCross = -1.0;
    int crossings = 0;
    for (int s = 1; s <= 60 * 60; ++s) {
        w.step();
        const double x = w.bodies()[1].pos.x - L0;
        if (prev > 0.0 && x <= 0.0) {  // downward crossing, interpolated
            const double t = (s - 1 + prev / (prev - x)) / 60.0;
            if (firstCross < 0.0) firstCross = t;
            lastCross = t;
            ++crossings;
        }
        prev = x;
    }
    REQUIRE(crossings > 10);
    const double period = (lastCross - firstCross) / (crossings - 1);
    const double exact = 2.0 * 3.14159265358979323846 * std::sqrt(m / k);
    MESSAGE("measured period " << period << " s, exact " << exact << " s, rel err " << (period - exact) / exact);
    CHECK(std::abs(period - exact) / exact < 0.005);
}

TEST_CASE("spring: undamped energy stays bounded over 1e5 steps (semi-implicit Euler)") {
    const double k = 100.0, m = 1.0;  // omega = 10 rad/s
    World w = massOnSpring(m, 110.0, 0.0, k, 0.0, 100.0);
    const double e0 = w.totalEnergy();
    double emin = e0, emax = e0;
    for (int s = 0; s < 100000; ++s) {
        w.step();
        const double e = w.totalEnergy();
        emin = std::min(emin, e);
        emax = std::max(emax, e);
    }
    MESSAGE("E/E0 range over 1e5 steps: [" << emin / e0 << ", " << emax / e0 << "]");
    CHECK(emin / e0 > 0.9);
    CHECK(emax / e0 < 1.1);
}

TEST_CASE("spring: damping removes energy") {
    World w = massOnSpring(1.0, 110.0, 0.0, 100.0, 2.0, 100.0);
    const double e0 = w.totalEnergy();
    for (int s = 0; s < 600; ++s) w.step();
    // Underdamped decay of energy: $E \\propto e^{-(c/m) t}$, $c/m = 2$, t = 10 s.
    MESSAGE("E/E0 after 10 s = " << w.totalEnergy() / e0 << " (continuous-time: " << std::exp(-20.0) << ")");
    CHECK(w.totalEnergy() / e0 < 1e-6);
}

namespace {
// N masses hanging below a static anchor on springs of rest length 20 px.
World hangingChain(int N, double k, double c) {
    WorldConfig cfg;
    cfg.walls = false;
    World w(cfg);
    int prev = w.addBody(makeBody(2.0, 0.0, {400.0, 500.0}));
    for (int i = 1; i <= N; ++i) {
        const int b = w.addBody(makeBody(2.0, 1.0, {400.0, 500.0 - i * 20.0}));
        w.addSpring({prev, b, 20.0, k, c});
        prev = b;
    }
    return w;
}
}  // namespace

TEST_CASE("spring chain: hangs at the analytic static extension $\\delta_i = (N - i + 1)\\,m g / k$") {
    // Spring i (from the top) carries the weight of the N - i + 1 masses below.
    // k and c are inside the explicit stability bound (see next test).
    const int N = 6;
    const double k = 2000.0, m = 1.0, g = 980.0;
    World w = hangingChain(N, k, 5.0);
    for (int s = 0; s < 3600; ++s) w.step();
    for (int i = 1; i <= N; ++i) {
        const double len = (w.bodies()[static_cast<std::size_t>(i - 1)].pos - w.bodies()[static_cast<std::size_t>(i)].pos).length();
        CAPTURE(i);
        CHECK(len - 20.0 == Approx((N - i + 1) * m * g / k).epsilon(1e-3));
    }
}

TEST_CASE("spring chain (limitation): explicit damping beyond $(\\omega_{max}\\Delta t)^2 + 2\\gamma_{max}\\Delta t < 4$ diverges") {
    // Springs and dampers are explicit forces. For a uniform chain the fastest
    // mode has $\\omega_{max} = 2\\sqrt{k/m}$ and damping rate $\\gamma_{max} = 4c/m$.
    // k = 2000, c = 20, m = 1, $\\Delta t = 1/60$: $2.22 + 2.67 = 4.9 > 4$, so the
    // damper adds energy instead of removing it. Documented, not fixed.
    World stable = hangingChain(6, 2000.0, 5.0), unstable = hangingChain(6, 2000.0, 20.0);
    const double e0 = stable.totalEnergy();
    for (int s = 0; s < 600; ++s) {
        stable.step();
        unstable.step();
    }
    MESSAGE("E/E0 after 10 s: c = 5 -> " << stable.totalEnergy() / e0 << ", c = 20 -> " << unstable.totalEnergy() / e0);
    CHECK(stable.totalEnergy() < e0);
    CHECK(unstable.totalEnergy() > e0);
}

TEST_CASE("distance constraint: pendulum length stays within 0.5% and energy within 5% over 10 s") {
    WorldConfig c;
    c.walls = false;
    c.friction = 0.0;
    World w(c);
    w.addBody(makeBody(2.0, 0.0, {400, 500}));
    w.addBody(makeBody(5.0, 1.0, {400 + 200.0 * std::sin(1.0), 500 - 200.0 * std::cos(1.0)}));
    w.addDistanceConstraint(0, 1);
    const double e0 = w.totalEnergy();
    double maxErr = 0.0;
    for (int s = 0; s < 600; ++s) {
        w.step();
        maxErr = std::max(maxErr, std::abs((w.bodies()[1].pos - w.bodies()[0].pos).length() - 200.0));
    }
    const double drift = (w.totalEnergy() - e0) / std::abs(e0 - (-1.0 * 980.0 * (500 - 200.0)));
    MESSAGE("max length error " << maxErr << " px over 10 s; energy change / swing energy = " << drift);
    CHECK(maxErr < 0.005 * 200.0);  // within 0.5% of the length (velocity-level rod + split-impulse drift fix)
    CHECK(std::abs(drift) < 0.05);
}

TEST_CASE("substeps: a spring unstable at one step per frame is stable with 4 substeps") {
    // $\omega = 180$ rad/s: $\omega\Delta t = 3 > 2$ diverges; with 4 substeps
    // $\omega\,\Delta t/4 = 0.75$.
    auto run = [](int substeps) {
        WorldConfig c = noGravity();
        c.substeps = substeps;
        World w(c);
        w.addBody(makeBody(1.0, 0.0, {0, 0}));
        w.addBody(makeBody(1.0, 1.0, {110.0, 0}));
        w.addSpring({0, 1, 100.0, 180.0 * 180.0, 0.0});
        const double e0 = w.totalEnergy();
        for (int s = 0; s < 600; ++s) w.step();
        return w.totalEnergy() / e0;
    };
    const double one = run(1), four = run(4);
    MESSAGE("E/E0 after 10 s: 1 substep " << one << ", 4 substeps " << four);
    CHECK((!std::isfinite(one) || one > 10.0));  // bounded only because the bodies collide
    CHECK(std::abs(four - 1.0) < 0.2);
}
