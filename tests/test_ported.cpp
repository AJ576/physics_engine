// The 16 tests from the original tests/test_physics.cpp, ported to doctest
// with the same inputs and assertions.
#include <doctest/doctest.h>

#include "helpers.hpp"
#include "physics/spring.hpp"

#include <algorithm>
#include <cmath>

using testing::RestitutionGuard;
using doctest::Approx;

TEST_CASE("integration: constant velocity") {
    RigidBody body(5.0, 2.0, {10.0, 20.0}, {15.0, -10.0});
    body.numericalIntegration(0.1);
    CHECK(body.getPosition()[0] == Approx(11.5).epsilon(1e-12));
    CHECK(body.getPosition()[1] == Approx(19.0).epsilon(1e-12));
    CHECK(body.getVelocity()[0] == Approx(15.0));
    CHECK(body.getVelocity()[1] == Approx(-10.0));
}

TEST_CASE("integration: constant force matches exact kinematics, force cleared") {
    RigidBody body(5.0, 4.0, {0.0, 0.0}, {0.0, 0.0});
    body.setForce({40.0, -20.0});  // a = (10, -5)
    body.numericalIntegration(0.5);
    CHECK(body.getPosition()[0] == Approx(1.25));
    CHECK(body.getPosition()[1] == Approx(-0.625));
    CHECK(body.getVelocity()[0] == Approx(5.0));
    CHECK(body.getVelocity()[1] == Approx(-2.5));
    CHECK(body.getForce()[0] == 0.0);
    CHECK(body.getForce()[1] == 0.0);
}

TEST_CASE("narrowphase: overlap depth and normal; separated pair") {
    WorldPhysics world({800.0, 600.0});
    RigidBody b1(10.0, 1.0, {100.0, 100.0});
    RigidBody b2(10.0, 1.0, {115.0, 100.0});
    RigidBody b3(10.0, 1.0, {130.0, 100.0});
    double nx = 0, ny = 0, overlap = 0;
    REQUIRE(world.areColliding(b1, b2, nx, ny, overlap));
    CHECK(overlap == Approx(5.0));
    CHECK(nx == Approx(1.0));
    CHECK(ny == Approx(0.0));
    CHECK_FALSE(world.areColliding(b1, b3, nx, ny, overlap));
    CHECK(overlap == 0.0);
}

TEST_CASE("response: elastic equal masses swap velocities") {
    RestitutionGuard e(1.0);
    WorldPhysics world({800.0, 600.0});
    world.setGravity({0.0, 0.0});
    RigidBody b1(5.0, 2.0, {98.0, 100.0}, {20.0, 0.0});
    RigidBody b2(5.0, 2.0, {102.0, 100.0}, {-20.0, 0.0});
    REQUIRE(world.resolveCollision(b1, b2));
    CHECK(b1.getVelocity()[0] == Approx(-20.0));
    CHECK(b2.getVelocity()[0] == Approx(20.0));
    CHECK(2.0 * b1.getVelocity()[0] + 2.0 * b2.getVelocity()[0] == Approx(0.0));
}

TEST_CASE("response: elastic unequal masses match the 1D analytic solution") {
    RestitutionGuard e(1.0);
    WorldPhysics world({800.0, 600.0});
    RigidBody b1(5.0, 3.0, {98.0, 200.0}, {10.0, 0.0});
    RigidBody b2(5.0, 1.0, {102.0, 200.0}, {0.0, 0.0});
    REQUIRE(world.resolveCollision(b1, b2));
    // $v_1' = \frac{m_1 - m_2}{m_1 + m_2} v_1 = 5$, $v_2' = \frac{2 m_1}{m_1 + m_2} v_1 = 15$
    CHECK(b1.getVelocity()[0] == Approx(5.0));
    CHECK(b2.getVelocity()[0] == Approx(15.0));
    const double ke = 0.5 * 3.0 * 25.0 + 0.5 * 1.0 * 225.0;
    CHECK(ke == Approx(150.0));
}

TEST_CASE("response: inelastic conserves momentum, separation speed = e * approach speed") {
    RestitutionGuard e(0.5);
    WorldPhysics world({800.0, 600.0});
    RigidBody b1(5.0, 2.0, {98.0, 100.0}, {20.0, 0.0});
    RigidBody b2(5.0, 2.0, {102.0, 100.0}, {-20.0, 0.0});
    world.resolveCollision(b1, b2);
    const double v1 = b1.getVelocity()[0], v2 = b2.getVelocity()[0];
    CHECK(2.0 * v1 + 2.0 * v2 == Approx(0.0));
    CHECK(0.5 * 2.0 * (v1 * v1 + v2 * v2) < 800.0);
    CHECK(v2 - v1 == Approx(20.0));
}

TEST_CASE("response: positional correction removes the full overlap") {
    WorldPhysics world({800.0, 600.0});
    RigidBody b1(10.0, 1.0, {100.0, 100.0});
    RigidBody b2(10.0, 1.0, {110.0, 100.0});
    world.resolveCollision(b1, b2);
    auto p1 = b1.getPosition(), p2 = b2.getPosition();
    CHECK(std::hypot(p2[0] - p1[0], p2[1] - p1[1]) == Approx(20.0));
}

TEST_CASE("walls: rebound with restitution") {
    RestitutionGuard e(0.8);
    WorldPhysics world({800.0, 600.0});
    world.setGravity({0.0, 0.0});
    RigidBody b(5.0, 1.0, {798.0, 300.0}, {100.0, 0.0});
    world.borderCheck(b, 1.0 / 60.0);
    CHECK(b.getPosition()[0] == Approx(795.0));
    CHECK(b.getVelocity()[0] == Approx(-80.0));
}

TEST_CASE("walls: slow ball settles on the floor") {
    WorldPhysics world({800.0, 600.0});
    world.setGravity({0.0, -980.0});
    RigidBody b(5.0, 1.0, {400.0, 3.0}, {0.0, -5.0});
    world.borderCheck(b, 1.0 / 60.0);
    CHECK(b.getPosition()[1] == Approx(5.0));
    CHECK(b.getVelocity()[1] == 0.0);
}

TEST_CASE("walls: a body pushed into a wall but moving out is not flipped back in") {
    RestitutionGuard e(0.7);
    WorldPhysics world({800.0, 600.0});
    world.setGravity({0.0, -980.0});
    RigidBody b(5.0, 1.0, {400.0, 3.0}, {0.0, 50.0});
    world.borderCheck(b, 1.0 / 60.0);
    CHECK(b.getPosition()[1] == Approx(5.0));
    CHECK(b.getVelocity()[1] == Approx(50.0));
    RigidBody s(5.0, 1.0, {798.0, 300.0}, {-30.0, 0.0});
    world.borderCheck(s, 1.0 / 60.0);
    CHECK(s.getPosition()[0] == Approx(795.0));
    CHECK(s.getVelocity()[0] == Approx(-30.0));
}

TEST_CASE("walls: a dropped ball comes to rest on the floor") {
    WorldPhysics world({800.0, 600.0});
    world.setGravity({0.0, -980.0});
    world.addBody(RigidBody(5.0, 1.0, {400.0, 300.0}, {0.0, 0.0}));
    TimeManager time;
    double minY = 1e9;
    for (int s = 0; s < 1200; s++) {
        world.runPhysics(time);
        minY = std::min(minY, world.getBodies()[0].getPosition()[1]);
    }
    CHECK(minY >= 5.0 - 1e-9);
    CHECK(world.getBodies()[0].getPosition()[1] == Approx(5.0));
    CHECK(world.getBodies()[0].getVelocity()[1] == Approx(0.0));
}

TEST_CASE("walls: ball settles for all four gravity directions") {
    const std::array<std::array<double, 2>, 4> gravities = {
        {{0.0, -980.0}, {0.0, 980.0}, {-980.0, 0.0}, {980.0, 0.0}}};
    for (const auto& g : gravities) {
        WorldPhysics world({800.0, 600.0});
        world.setGravity(g);
        world.addBody(RigidBody(5.0, 1.0, {400.0, 300.0}, {30.0, -20.0}));
        TimeManager time;
        for (int s = 0; s < 1500; s++) world.runPhysics(time);
        const auto& b = world.getBodies()[0];
        const int axis = (g[0] != 0.0) ? 0 : 1;
        const double expected = (g[axis] < 0.0) ? 5.0 : world.getBorder()[axis] - 5.0;
        CHECK(b.getPosition()[axis] == Approx(expected));
        CHECK(b.getVelocity()[axis] == Approx(0.0));
    }
}

TEST_CASE("walls: 5,000-body pile under gravity stays inside the box") {
    WorldPhysics world({800.0, 600.0});
    world.setGravity({0.0, -980.0});
    testing::load(world, phys::makeFixedArenaScene(5000, 42));
    TimeManager time;
    for (int s = 0; s < 600; s++) world.runPhysics(time);
    for (const auto& b : world.getBodies()) {
        auto p = b.getPosition();
        REQUIRE((p[0] >= 0.0 && p[0] <= 800.0 && p[1] >= 0.0 && p[1] <= 600.0));
    }
}

TEST_CASE("immovable body (mass 0) is never moved by a collision") {
    WorldPhysics world({800.0, 600.0});
    RigidBody staticBody(10.0, 0.0, {100.0, 100.0}, {0.0, 0.0});
    RigidBody moving(10.0, 2.0, {115.0, 100.0}, {-20.0, 0.0});
    CHECK(staticBody.getInvMass() == 0.0);
    world.resolveCollision(staticBody, moving);
    CHECK(staticBody.getPosition()[0] == 100.0);
    CHECK(staticBody.getVelocity()[0] == 0.0);
    CHECK(moving.getVelocity()[0] > 0.0);
}

TEST_CASE("spring pad: ball is detected and launched back up") {
    Spring spring(200.0, 0.0, 100.0, 20.0, 500.0);
    RigidBody ball(5.0, 1.0, {250.0, 22.0}, {0.0, -30.0});
    bool fromAbove = false;
    REQUIRE(detectSpringHit(ball, spring, fromAbove));
    CHECK(fromAbove);
    applySpringImpulse(ball, spring, fromAbove);
    CHECK(ball.getVelocity()[1] > 0.0);
    CHECK(ball.getPosition()[1] > 24.9);
}
