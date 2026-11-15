// Tests for phys::World (the sequential-impulse engine). Where a test
// carries over from the old engine's suite its name says so.
#include <doctest/doctest.h>

#include "physics/scene.hpp"
#include "physics/timestep.hpp"
#include "physics/world.hpp"
#include "world_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <utility>

using doctest::Approx;
using phys::Body;
using phys::makeBody;
using phys::Vec2;
using phys::World;
using phys::WorldConfig;

namespace {
WorldConfig freeSpace(double e = 1.0, double mu = 0.0) {
    WorldConfig c;
    c.gravity = {0.0, 0.0};
    c.walls = false;
    c.restitution = e;
    c.friction = mu;
    return c;
}
}  // namespace

// ---------------------------------------------------------------- narrowphase
TEST_CASE("narrowphase: exactly touching circles produce no contact") {
    Body a = makeBody(2.0, 1.0, {100, 100}), b = makeBody(3.0, 1.0, {105, 100});
    Vec2 n; double s;
    CHECK_FALSE(phys::collideCircles(a, b, 1.0 / 60.0, false, n, s));
    CHECK_FALSE(phys::collideCircles(a, b, 1.0 / 60.0, true, n, s));  // not approaching
}

TEST_CASE("narrowphase: overlap along a diagonal gives unit normal and depth") {
    Body a = makeBody(5.0, 1.0, {0, 0}), c = makeBody(6.0, 1.0, {6, 8});  // d = 10, R = 11
    Vec2 n; double s;
    REQUIRE(phys::collideCircles(a, c, 1.0 / 60.0, false, n, s));
    CHECK(n.x == Approx(0.6));
    CHECK(n.y == Approx(0.8));
    CHECK(s == Approx(-1.0));
}

TEST_CASE("narrowphase: coincident centres give a finite unit normal and full depth") {
    Body a = makeBody(2.0, 1.0, {50, 50}), b = makeBody(3.0, 1.0, {50, 50});
    Vec2 n; double s;
    REQUIRE(phys::collideCircles(a, b, 1.0 / 60.0, false, n, s));
    CHECK(std::isfinite(n.x));
    CHECK(n.lengthSq() == Approx(1.0));
    CHECK(s == Approx(-5.0));
    World w(freeSpace());  // and the solver separates them without NaNs
    w.addBody(a);
    w.addBody(b);
    for (int i = 0; i < 120; ++i) w.step();
    const Vec2 d = w.bodies()[1].pos - w.bodies()[0].pos;
    CHECK(std::isfinite(d.x));
    CHECK(d.length() >= 5.0 - w.config().slop - 1e-9);
}

TEST_CASE("narrowphase: swept test finds the time-of-impact normal for a fast pair") {
    // Separated by 10 px, closing at 1200 px/s (20 px per step): a speculative contact.
    Body a = makeBody(1.0, 1.0, {0, 0}, {600, 0}), b = makeBody(1.0, 1.0, {12, 0}, {-600, 0});
    Vec2 n; double s;
    REQUIRE(phys::collideCircles(a, b, 1.0 / 60.0, true, n, s));
    CHECK(n.x == Approx(1.0));
    CHECK(s == Approx(10.0));
    CHECK_FALSE(phys::collideCircles(a, b, 1.0 / 60.0, false, n, s));
    Body c = makeBody(1.0, 1.0, {12, 5}, {-600, 0});  // passes 5 px off-axis: miss (R = 2)
    CHECK_FALSE(phys::collideCircles(a, c, 1.0 / 60.0, true, n, s));
}

// ------------------------------------------------------------------- response
TEST_CASE("response (ported): elastic equal masses swap velocities") {
    World w(freeSpace(1.0));
    w.addBody(makeBody(5.0, 2.0, {95.5, 100}, {20, 0}));
    w.addBody(makeBody(5.0, 2.0, {104.5, 100}, {-20, 0}));
    w.step();
    CHECK(w.bodies()[0].vel.x == Approx(-20.0));
    CHECK(w.bodies()[1].vel.x == Approx(20.0));
}

TEST_CASE("response (ported): elastic unequal masses match the 1D analytic solution") {
    World w(freeSpace(1.0));
    w.addBody(makeBody(5.0, 3.0, {95.5, 200}, {10, 0}));
    w.addBody(makeBody(5.0, 1.0, {104.5, 200}, {0, 0}));
    w.step();
    // $v_1' = \frac{m_1 - m_2}{m_1 + m_2} v_1 = 5$, $v_2' = \frac{2 m_1}{m_1 + m_2} v_1 = 15$
    CHECK(w.bodies()[0].vel.x == Approx(5.0));
    CHECK(w.bodies()[1].vel.x == Approx(15.0));
}

TEST_CASE("response (ported): inelastic keeps momentum, separation speed = e * approach speed") {
    World w(freeSpace(0.5));
    w.addBody(makeBody(5.0, 2.0, {95.5, 100}, {20, 0}));
    w.addBody(makeBody(5.0, 2.0, {104.5, 100}, {-20, 0}));
    w.step();
    const double v1 = w.bodies()[0].vel.x, v2 = w.bodies()[1].vel.x;
    CHECK(2.0 * v1 + 2.0 * v2 == Approx(0.0));
    CHECK(v2 - v1 == Approx(20.0));
}

TEST_CASE("response: impulse magnitude matches $j = -(1+e)\\,v_n / (m_1^{-1} + m_2^{-1})$ (frictionless)") {
    World w(freeSpace(0.5, 0.0));
    // Oblique contact along (0.6, 0.8), unequal masses, slightly overlapping.
    w.addBody(makeBody(5.0, 2.0, {0, 0}, {3, 4}));
    w.addBody(makeBody(5.0, 6.0, {5.94, 7.92}, {-1, 1}));  // distance 9.9 < 10
    const double nx = 0.6, ny = 0.8;
    const double vn = (-1.0 - 3.0) * nx + (1.0 - 4.0) * ny;       // -4.8
    const double j = -(1.0 + 0.5) * vn / (1.0 / 2.0 + 1.0 / 6.0);  // 10.8
    w.step();
    const auto& a = w.bodies()[0];
    const auto& b = w.bodies()[1];
    CHECK(a.vel.x == Approx(3.0 - j / 2.0 * nx));
    CHECK(a.vel.y == Approx(4.0 - j / 2.0 * ny));
    CHECK(b.vel.x == Approx(-1.0 + j / 6.0 * nx));
    CHECK(b.vel.y == Approx(1.0 + j / 6.0 * ny));
    CHECK(std::abs(a.angVel) < 1e-12);  // frictionless: normal impulse passes through the centres
}

TEST_CASE("response: overlapping pair sliding tangentially gets no velocity change") {
    World w(freeSpace(1.0, 0.5));
    w.addBody(makeBody(5.0, 1.0, {100, 100}, {0, 10}));
    w.addBody(makeBody(5.0, 1.0, {109, 100}, {0, -10}));
    w.step();
    CHECK(w.bodies()[0].vel.x == 0.0);
    CHECK(w.bodies()[0].vel.y == Approx(10.0));
    CHECK(w.bodies()[1].vel.y == Approx(-10.0));
}

TEST_CASE("response (ported): overlap is removed by position correction without changing velocity") {
    World w(freeSpace());
    w.addBody(makeBody(10.0, 1.0, {100, 100}));
    w.addBody(makeBody(10.0, 1.0, {110, 100}));  // 10 px deep
    for (int i = 0; i < 120; ++i) w.step();
    const double d = (w.bodies()[1].pos - w.bodies()[0].pos).length();
    CHECK(d >= 20.0 - w.config().slop - 1e-6);
    CHECK(d <= 20.0 + 1e-6);
    CHECK(w.bodies()[0].vel.lengthSq() == 0.0);  // split impulse adds no velocity
}

TEST_CASE("response (ported): static body (mass 0) never moves") {
    World w(freeSpace(1.0));
    w.addBody(makeBody(10.0, 0.0, {100, 100}));
    w.addBody(makeBody(10.0, 2.0, {119, 100}, {-20, 0}));
    for (int i = 0; i < 30; ++i) w.step();
    CHECK(w.bodies()[0].pos == Vec2{100, 100});
    CHECK(w.bodies()[0].vel == Vec2{0, 0});
    CHECK(w.bodies()[1].vel.x > 0.0);
}

// ---------------------------------------------------------------------- walls
TEST_CASE("walls (ported): rebound with restitution") {
    WorldConfig c;
    c.gravity = {0, 0};
    c.restitution = 0.8;
    World w(c);
    w.addBody(makeBody(5.0, 1.0, {790.0, 300}, {100, 0}));
    for (int i = 0; i < 10 && w.bodies()[0].vel.x > 0.0; ++i) w.step();
    CHECK(w.bodies()[0].vel.x == Approx(-80.0));
}

TEST_CASE("walls (ported): a body inside a wall but moving out keeps its velocity") {
    World w;  // gravity on, e = 0.7
    w.addBody(makeBody(5.0, 1.0, {400, 3.0}, {0, 50}));
    w.step();
    CHECK(w.bodies()[0].vel.y == Approx(50.0 - 980.0 / 60.0));  // only gravity acted
}

TEST_CASE("walls (ported): a dropped ball comes to rest on the floor and never sinks") {
    World w;
    w.addBody(makeBody(5.0, 1.0, {400, 300}));
    double minY = 1e9;
    for (int s = 0; s < 1200; ++s) {
        w.step();
        minY = std::min(minY, w.bodies()[0].pos.y);
    }
    CHECK(minY >= 5.0 - w.config().slop - 1e-9);
    CHECK(w.bodies()[0].pos.y == Approx(5.0).epsilon(0.01));
    CHECK(std::abs(w.bodies()[0].vel.y) < 1e-6);
}

TEST_CASE("walls (ported): ball settles for all four gravity directions") {
    const Vec2 gs[4] = {{0, -980}, {0, 980}, {-980, 0}, {980, 0}};
    for (Vec2 g : gs) {
        WorldConfig c;
        c.gravity = g;
        World w(c);
        w.addBody(makeBody(5.0, 1.0, {400, 300}, {30, -20}));
        for (int s = 0; s < 1500; ++s) w.step();
        const Body& b = w.bodies()[0];
        const bool xAxis = g.x != 0.0;
        const double pos = xAxis ? b.pos.x : b.pos.y, vel = xAxis ? b.vel.x : b.vel.y;
        const double limit = xAxis ? c.width : c.height;
        const double expected = (xAxis ? g.x : g.y) < 0.0 ? 5.0 : limit - 5.0;
        CHECK(pos == Approx(expected).epsilon(0.01));
        CHECK(std::abs(vel) < 1e-6);
    }
}

TEST_CASE("walls (ported): 5,000-body pile under gravity stays inside the box") {
    World w;
    testing::load(w, phys::makeFixedArenaScene(5000, 42));
    for (int s = 0; s < 600; ++s) w.step();
    for (const Body& b : w.bodies())
        REQUIRE((b.pos.x >= 0.0 && b.pos.x <= 800.0 && b.pos.y >= 0.0 && b.pos.y <= 600.0));
}

// ----------------------------------------------------------------- broadphase
TEST_CASE("broadphase: grid contact set equals brute force for any cell size") {
    // Dense random scene; compare the set of (a, b) contact pairs after one
    // step's collision detection. Cell sizes below the body diameter used to
    // miss pairs in the old engine.
    auto scene = phys::makeConstantDensityScene(3000, 5, 20.0);
    auto pairsFor = [&](phys::Broadphase bp, double cell) {
        WorldConfig c;
        c.width = scene.width;
        c.height = scene.height;
        c.broadphase = bp;
        c.cellSize = cell;
        World w(c);
        testing::load(w, scene);
        w.step();
        std::set<std::pair<int, int>> s;
        for (const auto& k : w.contacts())
            if (k.b >= 0) REQUIRE(s.insert({k.a, k.b}).second);  // no duplicates
        return s;
    };
    const auto brute = pairsFor(phys::Broadphase::BruteForce, 0.0);
    REQUIRE(brute.size() > 1000);
    for (double cell : {1.5, 3.0, 6.0, 10.0, 40.0}) {
        CAPTURE(cell);
        CHECK(pairsFor(phys::Broadphase::Grid, cell) == brute);
    }
}

TEST_CASE("broadphase: huge, infinite-adjacent coordinates do not overflow cell indices") {
    // The old engine converted floor(x / cell) to int without a range check
    // (undefined behavior, caught by UBSan). The asan preset runs this test.
    World w;
    w.config().walls = false;
    w.addBody(makeBody(2.0, 1.0, {1.0e12, 300.0}));
    w.addBody(makeBody(2.0, 1.0, {-1.0e15, 300.0}, {1.0e14, 0}));
    w.addBody(makeBody(2.0, 1.0, {400.0, 300.0}));
    for (int i = 0; i < 3; ++i) w.step();
    for (const Body& b : w.bodies()) CHECK(std::isfinite(b.pos.x));
    CHECK(w.bodies()[2].pos.x == Approx(400.0));
}

// ------------------------------------------------------------------- timestep
namespace {
int stepsFor(double frame) {
    phys::FixedTimestep t;
    t.advance(frame);
    int n = 0;
    while (t.consume()) ++n;
    return n;
}
}  // namespace

TEST_CASE("timestep: steps per frame are capped and hitch handling is monotonic") {
    CHECK(stepsFor(1.0 / 60.0) == 1);
    CHECK(stepsFor(0.05) == 3);
    CHECK(stepsFor(0.24) <= phys::FixedTimestep::kMaxStepsPerFrame);
    CHECK(stepsFor(0.26) >= stepsFor(0.24));
    CHECK(stepsFor(10.0) == phys::FixedTimestep::kMaxStepsPerFrame);
    CHECK(stepsFor(-1.0) == 0);
    CHECK(stepsFor(std::nan("")) == 0);
}

// ---------------------------------------------------------------- determinism
TEST_CASE("determinism: same seed gives an identical state hash after 300 steps") {
    auto run = [](std::uint64_t seed) {
        auto scene = phys::makeConstantDensityScene(2000, seed);
        WorldConfig c;
        c.width = scene.width;
        c.height = scene.height;
        World w(c);
        testing::load(w, scene);
        for (int s = 0; s < 300; ++s) w.step();
        return testing::stateHash(w);
    };
    const auto h1 = run(7), h2 = run(7), h3 = run(8);
    CHECK(h1 == h2);
    CHECK(h1 != h3);
}

TEST_CASE("scene generator: reproducible, constant density, bodies inside the arena") {
    auto a = phys::makeConstantDensityScene(5000, 1), b = phys::makeConstantDensityScene(5000, 1);
    REQUIRE(a.bodies.size() == 5000);
    CHECK(a.width == Approx(800.0));
    CHECK(a.height == Approx(600.0));
    auto big = phys::makeConstantDensityScene(20000, 1);
    CHECK(big.width * big.height / 20000.0 == Approx(96.0));
    for (size_t i = 0; i < a.bodies.size(); ++i) {
        REQUIRE(a.bodies[i].x == b.bodies[i].x);
        REQUIRE(a.bodies[i].vy == b.bodies[i].vy);
        REQUIRE(a.bodies[i].x - a.bodies[i].radius >= 0.0);
        REQUIRE(a.bodies[i].x + a.bodies[i].radius <= a.width);
    }
    CHECK(phys::SplitMix64(0).next() == 0xE220A8397B1DCDAFull);
}

// --------------------------------------------------------------- conservation
TEST_CASE("conservation: momentum through many elastic collisions (gravity off, no walls)") {
    auto scene = phys::makeConstantDensityScene(1000, 3, 30.0);  // dense: many contacts
    World w(freeSpace(1.0, 0.0));
    testing::load(w, scene);
    w.config().walls = false;
    const Vec2 p0 = w.linearMomentum();
    double sumAbsP = 0.0;
    for (const Body& b : w.bodies()) sumAbsP += b.mass * b.vel.length();
    std::size_t contacts = 0;
    for (int s = 0; s < 600; ++s) {
        w.step();
        contacts += w.lastStepStats().contacts;
    }
    REQUIRE(contacts > 1000);
    const Vec2 p1 = w.linearMomentum();
    const double rel = (p1 - p0).length() / sumAbsP;
    MESSAGE("contacts=" << contacts << "  |dp| / sum|p| = " << rel);
    CHECK(rel < 1e-12);
}
