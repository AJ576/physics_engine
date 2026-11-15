#include <doctest/doctest.h>

#include "helpers.hpp"

#include <cmath>
#include <vector>

using doctest::Approx;
using testing::RestitutionGuard;

// ---------------------------------------------------------------- narrowphase
TEST_CASE("narrowphase: exactly touching circles do not collide") {
    WorldPhysics w({800, 600});
    RigidBody a(2.0, 1.0, {100.0, 100.0}), b(3.0, 1.0, {105.0, 100.0});  // d = r1 + r2
    double nx, ny, overlap;
    CHECK_FALSE(w.areColliding(a, b, nx, ny, overlap));
}

TEST_CASE("narrowphase: overlapping circles along a diagonal give a unit normal") {
    WorldPhysics w({800, 600});
    RigidBody a(5.0, 1.0, {0.0, 0.0}), b(5.0, 1.0, {6.0, 8.0});  // d = 10 = R: touching
    RigidBody c(6.0, 1.0, {6.0, 8.0});                           // R = 11, overlap 1
    double nx, ny, overlap;
    CHECK_FALSE(w.areColliding(a, b, nx, ny, overlap));
    REQUIRE(w.areColliding(a, c, nx, ny, overlap));
    CHECK(nx == Approx(0.6));
    CHECK(ny == Approx(0.8));
    CHECK(overlap == Approx(1.0));
    CHECK(nx * nx + ny * ny == Approx(1.0));
}

TEST_CASE("narrowphase: coincident centres give a finite normal and full overlap") {
    WorldPhysics w({800, 600});
    RigidBody a(2.0, 1.0, {50.0, 50.0}), b(3.0, 1.0, {50.0, 50.0});
    double nx, ny, overlap;
    REQUIRE(w.areColliding(a, b, nx, ny, overlap));
    CHECK(std::isfinite(nx));
    CHECK(std::isfinite(ny));
    CHECK(nx * nx + ny * ny == Approx(1.0));
    CHECK(overlap == Approx(5.0));
    w.resolveCollision(a, b);
    CHECK(std::isfinite(a.getPosition()[0]));
    CHECK(std::hypot(b.getPosition()[0] - a.getPosition()[0], b.getPosition()[1] - a.getPosition()[1]) ==
          Approx(5.0));
}

TEST_CASE("response: tangential relative motion gets no normal impulse") {
    WorldPhysics w({800, 600});
    RigidBody a(5.0, 1.0, {100.0, 100.0}, {0.0, 10.0});
    RigidBody b(5.0, 1.0, {109.0, 100.0}, {0.0, -10.0});  // overlapping, sliding past
    w.resolveCollision(a, b);
    CHECK(a.getVelocity()[0] == 0.0);
    CHECK(a.getVelocity()[1] == 10.0);
    CHECK(b.getVelocity()[1] == -10.0);
}

TEST_CASE("response: impulse magnitude matches $j = -(1+e)\\,v_n / (m_1^{-1} + m_2^{-1})$") {
    RestitutionGuard e(0.5);
    WorldPhysics w({800, 600});
    // Oblique contact, normal (0.6, 0.8), unequal masses.
    RigidBody a(5.0, 2.0, {0.0, 0.0}, {3.0, 4.0});
    RigidBody b(6.0, 6.0, {6.0, 8.0}, {-1.0, 1.0});
    const double nx = 0.6, ny = 0.8;
    const double vn = (-1.0 - 3.0) * nx + (1.0 - 4.0) * ny;  // = -4.8
    const double j = -(1.0 + 0.5) * vn / (1.0 / 2.0 + 1.0 / 6.0);  // = 10.8
    w.calculateImpulse(a, b, nx, ny);
    CHECK(a.getVelocity()[0] == Approx(3.0 - j / 2.0 * nx));
    CHECK(a.getVelocity()[1] == Approx(4.0 - j / 2.0 * ny));
    CHECK(b.getVelocity()[0] == Approx(-1.0 + j / 6.0 * nx));
    CHECK(b.getVelocity()[1] == Approx(1.0 + j / 6.0 * ny));
    // Post-impact normal relative velocity is $-e\,v_n$.
    const double vn2 = (b.getVelocity()[0] - a.getVelocity()[0]) * nx + (b.getVelocity()[1] - a.getVelocity()[1]) * ny;
    CHECK(vn2 == Approx(-0.5 * vn));
}

// ---------------------------------------------------------------- broadphase
namespace {
// K isolated overlapping pairs, far apart from each other, each straddling a
// cell boundary in a different direction. With zero velocity and no gravity
// the exact number of resolved collisions in one step must be K.
size_t resolvedIsolatedPairs(double cellSize, double radius, int K) {
    WorldPhysics w({100000.0, 100000.0});
    w.setGravity({0.0, 0.0});
    w.setGridSize(cellSize);
    const double d = 1.9 * radius;  // centre distance < 2r
    for (int k = 0; k < K; ++k) {
        const double angle = 0.7 * k;
        const double cx = 1000.0 + 200.0 * (k % 50), cy = 1000.0 + 200.0 * (k / 50);
        w.addBody(RigidBody(radius, 1.0, {cx, cy}, {0.0, 0.0}));
        w.addBody(RigidBody(radius, 1.0, {cx + d * std::cos(angle), cy + d * std::sin(angle)}, {0.0, 0.0}));
    }
    TimeManager t;
    return w.runPhysicsProfiled(t).collisionsResolved;
}
}  // namespace

TEST_CASE("broadphase: grid finds every overlapping pair exactly once (cell >= diameter)") {
    CHECK(resolvedIsolatedPairs(10.0, 4.0, 400) == 400u);
    CHECK(resolvedIsolatedPairs(8.0, 4.0, 400) == 400u);
}

// ---------------------------------------------------------------- conservation
TEST_CASE("conservation: momentum through many elastic collisions, gravity off, no walls touched") {
    RestitutionGuard e(1.0);
    WorldPhysics w({1.0e6, 1.0e6});
    w.setGravity({0.0, 0.0});
    auto scene = phys::makeConstantDensityScene(1000, 3, 30.0);  // dense, many contacts
    for (auto& b : scene.bodies) { b.x += 5.0e5; b.y += 5.0e5; }
    for (const auto& b : scene.bodies) w.addBody(RigidBody(b.radius, b.mass, {b.x, b.y}, {b.vx, b.vy}));

    const auto p0 = w.calculateTotalMomentum();
    const double ke0 = w.calculateTotalKineticEnergy();
    double sumAbsP = 0.0;
    for (const auto& b : w.getBodies()) sumAbsP += b.getMass() * std::hypot(b.getVelocity()[0], b.getVelocity()[1]);

    TimeManager t;
    size_t collisions = 0;
    for (int s = 0; s < 600; ++s) collisions += w.runPhysicsProfiled(t).collisionsResolved;
    REQUIRE(collisions > 1000);

    const auto p1 = w.calculateTotalMomentum();
    const double ke1 = w.calculateTotalKineticEnergy();
    MESSAGE("collisions=" << collisions << " |dp|/sum|p| = " << std::hypot(p1[0] - p0[0], p1[1] - p0[1]) / sumAbsP
                          << "  dKE/KE0 = " << (ke1 - ke0) / ke0);
    CHECK(std::hypot(p1[0] - p0[0], p1[1] - p0[1]) / sumAbsP < 1e-12);
    CHECK(std::abs(ke1 - ke0) / ke0 < 1e-9);
}

// ---------------------------------------------------------------- determinism
TEST_CASE("determinism: same seed gives an identical state hash after 300 steps") {
    auto run = [](std::uint64_t seed) {
        WorldPhysics w({1, 1});
        w.setGravity({0.0, -980.0});
        testing::load(w, phys::makeConstantDensityScene(2000, seed));
        TimeManager t;
        for (int s = 0; s < 300; ++s) w.runPhysics(t);
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
    // Pin the first value so a silent RNG change is caught (same on every platform).
    CHECK(phys::SplitMix64(0).next() == 0xE220A8397B1DCDAFull);
}
