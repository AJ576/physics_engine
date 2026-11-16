// Behavior of the ready-made scenes against analytic expectations.
#include <doctest/doctest.h>

#include "physics/scenes.hpp"

#include <algorithm>
#include <cmath>

using doctest::Approx;
using phys::World;

TEST_CASE("spinning ball: skids, then rolls at the analytic speed $v = \\omega_0 r / 3$") {
    // Disc ($I = \tfrac12 m r^2$) on the floor with spin $\omega_0$ and no linear
    // velocity. Friction acts at the contact point, so angular momentum about
    // that point is conserved: $I\omega_0 = (I + m r^2)\,\omega_f$, giving
    // $\omega_f = \omega_0/3$ and rolling speed $v_f = -\omega_f r$.
    const double r = 20.0, w0 = -20.0;
    World w = phys::scenes::spinningBall(r, w0, 0.4);
    for (int s = 0; s < 120; ++s) w.step();
    const auto& b = w.bodies()[0];
    MESSAGE("v = " << b.vel.x << " px/s (analytic " << -w0 * r / 3.0 << "), slip v + w r = " << b.vel.x + b.angVel * r);
    CHECK(b.vel.x == Approx(-w0 * r / 3.0).epsilon(1e-3));
    CHECK(std::abs(b.vel.x + b.angVel * r) < 1e-6);  // rolling without slipping
    CHECK(b.pos.y == Approx(r).epsilon(1e-3));
}

namespace {
// Returns {max rise of the last ball as a fraction of the release height,
//          max |v| of the other balls / incoming speed}, over the first swing.
std::pair<double, double> cradle(double gap) {
    World w = phys::scenes::newtonsCradle(5, 20.0, gap, 0.6);
    const double rest = w.bodies()[3].pos.y, h0 = w.bodies()[1].pos.y - rest;
    double rise = 0.0, vin = 0.0, others = 0.0;
    bool hit = false;
    for (int s = 0; s < 100; ++s) {
        w.step();
        rise = std::max(rise, w.bodies()[9].pos.y - rest);
        if (!hit) vin = std::max(vin, w.bodies()[1].vel.x);  // first ball's speed up to impact
        if (!hit && w.bodies()[9].vel.x > 1.0) hit = true;
        if (hit && s < 70)
            for (int i = 0; i < 4; ++i) others = std::max(others, std::abs(w.bodies()[static_cast<std::size_t>(2 * i + 1)].vel.x));
    }
    return {rise / h0, others / vin};
}
}  // namespace

TEST_CASE("Newton's cradle (2 px gaps): the last ball rises to >= 95% of the release height, the rest stay nearly still") {
    const auto [rise, others] = cradle(2.0);
    MESSAGE("last ball rise / release height = " << rise << ", max |v| of balls 1-4 / incoming = " << others);
    CHECK(rise > 0.95);
    CHECK(others < 0.05);
}

TEST_CASE("Newton's cradle (limitation): with 0.5 px gaps the impulse spreads over the row") {
    // At impact a ball moves 4.3 px per step, more than the gaps, so all four
    // contacts are active in the same step and sequential impulses share the
    // momentum instead of passing it down the row.
    const auto [rise, others] = cradle(0.5);
    MESSAGE("last ball rise / release height = " << rise << ", max |v| of balls 1-4 / incoming = " << others);
    CHECK(rise < 0.5);
}

TEST_CASE("pile scene: 2,000 bodies settle inside the box within 10 s") {
    World w = phys::scenes::pile(2000, 1);
    for (int s = 0; s < 600; ++s) w.step();
    double v2 = 0.0;
    for (const auto& b : w.bodies()) {
        v2 += b.vel.lengthSq();
        REQUIRE((b.pos.x >= 0 && b.pos.x <= 800 && b.pos.y >= 0 && b.pos.y <= 600));
    }
    const double rms = std::sqrt(v2 / 2000.0);
    MESSAGE("RMS speed after 10 s: " << rms << " px/s");
    CHECK(rms < 0.25 * 980.0 / 60.0);
}

TEST_CASE("spring chain scene: swings down and loses energy, stays finite") {
    World w = phys::scenes::springChain();
    const double e0 = w.totalEnergy();
    for (int s = 0; s < 1800; ++s) w.step();
    for (const auto& b : w.bodies()) REQUIRE(std::isfinite(b.pos.x));
    MESSAGE("E/E0 after 30 s: " << w.totalEnergy() / e0);
    CHECK(w.totalEnergy() < e0);
}

TEST_CASE("soft blob scene: lands, keeps its shape (strain at rest < 10%), loses energy") {
    World w = phys::scenes::softBlob();
    const double e0 = w.totalEnergy();
    double peak = 0.0;
    auto strain = [&] {
        double m = 0.0;
        for (const auto& sp : w.springs()) {
            const double l = (w.bodies()[static_cast<std::size_t>(sp.b)].pos - w.bodies()[static_cast<std::size_t>(sp.a)].pos).length();
            m = std::max(m, std::abs(l - sp.restLength) / sp.restLength);
        }
        return m;
    };
    for (int s = 0; s < 600; ++s) {
        w.step();
        peak = std::max(peak, strain());
    }
    MESSAGE("peak strain " << peak << ", strain after 10 s " << strain() << ", E/E0 " << w.totalEnergy() / e0);
    CHECK(strain() < 0.10);
    CHECK(peak < 1.0);
    CHECK(w.totalEnergy() < e0);
}
