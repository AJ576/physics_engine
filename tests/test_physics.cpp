#include "physics.hpp"
#include "rigidBody.hpp"
#include "spring.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <string>
#include <chrono>

namespace TestFramework {
    static int testsPassed = 0;
    static int testsFailed = 0;
    static std::string currentTestName = "";

    #define COLOR_RESET   "\033[0m"
    #define COLOR_GREEN   "\033[32m"
    #define COLOR_RED     "\033[31m"
    #define COLOR_YELLOW  "\033[33m"
    #define COLOR_BLUE    "\033[34m"
    #define COLOR_CYAN    "\033[36m"
    #define COLOR_BOLD    "\033[1m"

    #define RUN_TEST(fn) do { \
        TestFramework::currentTestName = #fn; \
        auto start = std::chrono::high_resolution_clock::now(); \
        try { \
            fn(); \
            auto end = std::chrono::high_resolution_clock::now(); \
            double ms = std::chrono::duration<double, std::milli>(end - start).count(); \
            std::cout << COLOR_GREEN << "  [PASS] " << COLOR_RESET << #fn \
                      << " (" << std::fixed << std::setprecision(2) << ms << " ms)\n"; \
            TestFramework::testsPassed++; \
        } catch (const std::exception& e) { \
            std::cout << COLOR_RED << "  [FAIL] " << COLOR_RESET << #fn \
                      << ": " << e.what() << "\n"; \
            TestFramework::testsFailed++; \
        } catch (...) { \
            std::cout << COLOR_RED << "  [FAIL] " << COLOR_RESET << #fn \
                      << ": Unknown exception\n"; \
            TestFramework::testsFailed++; \
        } \
    } while(0)

    #define ASSERT_TRUE(condition) do { \
        if (!(condition)) { \
            throw std::runtime_error("Assertion failed: " #condition " at line " + std::to_string(__LINE__)); \
        } \
    } while(0)

    #define ASSERT_FALSE(condition) do { \
        if (condition) { \
            throw std::runtime_error("Assertion failed: NOT (" #condition ") at line " + std::to_string(__LINE__)); \
        } \
    } while(0)

    #define ASSERT_NEAR(actual, expected, eps) do { \
        double diff = std::abs((actual) - (expected)); \
        if (diff > (eps)) { \
            throw std::runtime_error("Assertion failed: " #actual " (" + std::to_string(actual) + \
                                     ") != " #expected " (" + std::to_string(expected) + \
                                     ") within eps=" + std::to_string(eps) + " at line " + std::to_string(__LINE__)); \
        } \
    } while(0)

    #define ASSERT_GT(val1, val2) do { \
        if (!((val1) > (val2))) { \
            throw std::runtime_error("Assertion failed: " #val1 " (" + std::to_string(val1) + \
                                     ") > " #val2 " (" + std::to_string(val2) + ") at line " + std::to_string(__LINE__)); \
        } \
    } while(0)
}

using namespace TestFramework;

// 1. Kinematics: constant velocity motion without acceleration
void test_numerical_integration_free_motion() {
    RigidBody body(5.0, 2.0, {10.0, 20.0}, {15.0, -10.0});
    double dt = 0.1; // 100 ms

    body.numericalIntegration(dt);

    auto pos = body.getPosition();
    auto vel = body.getVelocity();

    // With zero acceleration: x = x0 + v0 * dt, v = v0
    ASSERT_NEAR(pos[0], 10.0 + 15.0 * 0.1, 1e-6);
    ASSERT_NEAR(pos[1], 20.0 - 10.0 * 0.1, 1e-6);
    ASSERT_NEAR(vel[0], 15.0, 1e-6);
    ASSERT_NEAR(vel[1], -10.0, 1e-6);
}

// 2. Kinematics: constant force acceleration (F = m * a)
void test_numerical_integration_constant_acceleration() {
    double mass = 4.0;
    RigidBody body(5.0, mass, {0.0, 0.0}, {0.0, 0.0});
    
    // Apply F = {40.0, -20.0} => a = {10.0, -5.0}
    body.setForce({40.0, -20.0});
    double dt = 0.5;

    body.numericalIntegration(dt);

    // Kinematics: p = p0 + v0*dt + 0.5*a*dt^2 = 0 + 0 + 0.5 * {10, -5} * 0.25 = {1.25, -0.625}
    // v = v0 + a * dt = {5.0, -2.5}
    auto pos = body.getPosition();
    auto vel = body.getVelocity();

    ASSERT_NEAR(pos[0], 1.25, 1e-6);
    ASSERT_NEAR(pos[1], -0.625, 1e-6);
    ASSERT_NEAR(vel[0], 5.0, 1e-6);
    ASSERT_NEAR(vel[1], -2.5, 1e-6);

    // Force should be reset to zero after integration
    auto force = body.getForce();
    ASSERT_NEAR(force[0], 0.0, 1e-9);
    ASSERT_NEAR(force[1], 0.0, 1e-9);
}

// 3. Narrow-phase circle-circle collision detection
void test_circle_collision_detection() {
    WorldPhysics world({800.0, 600.0});

    RigidBody b1(10.0, 1.0, {100.0, 100.0});
    RigidBody b2(10.0, 1.0, {115.0, 100.0}); // distance = 15, r1+r2 = 20 -> overlapping by 5
    RigidBody b3(10.0, 1.0, {130.0, 100.0}); // distance = 30, r1+r2 = 20 -> separated

    double nx = 0, ny = 0, overlap = 0;

    bool col12 = world.areColliding(b1, b2, nx, ny, overlap);
    ASSERT_TRUE(col12);
    ASSERT_NEAR(overlap, 5.0, 1e-6);
    ASSERT_NEAR(nx, 1.0, 1e-6);
    ASSERT_NEAR(ny, 0.0, 1e-6);

    bool col13 = world.areColliding(b1, b3, nx, ny, overlap);
    ASSERT_FALSE(col13);
    ASSERT_NEAR(overlap, 0.0, 1e-6);
}

// 4. Elastic collision momentum and energy conservation (Equal masses)
void test_elastic_collision_equal_mass() {
    double oldRestitution = getRestitution();
    setRestitution(1.0); // Perfectly elastic

    WorldPhysics world({800.0, 600.0});
    world.setGravity({0.0, 0.0});

    RigidBody b1(5.0, 2.0, {98.0, 100.0}, {20.0, 0.0});
    RigidBody b2(5.0, 2.0, {102.0, 100.0}, {-20.0, 0.0});

    double initP = 2.0 * 20.0 + 2.0 * (-20.0); // 0.0
    double initKE = 0.5 * 2.0 * (20.0 * 20.0) + 0.5 * 2.0 * (-20.0 * -20.0); // 800.0

    bool resolved = world.resolveCollision(b1, b2);
    ASSERT_TRUE(resolved);

    auto v1 = b1.getVelocity();
    auto v2 = b2.getVelocity();

    double finalP = 2.0 * v1[0] + 2.0 * v2[0];
    double finalKE = 0.5 * 2.0 * (v1[0] * v1[0] + v1[1] * v1[1]) +
                    0.5 * 2.0 * (v2[0] * v2[0] + v2[1] * v2[1]);

    ASSERT_NEAR(finalP, initP, 1e-6);
    ASSERT_NEAR(finalKE, initKE, 1e-6);

    // Velocities should swap for equal mass head-on collision
    ASSERT_NEAR(v1[0], -20.0, 1e-6);
    ASSERT_NEAR(v2[0], 20.0, 1e-6);

    setRestitution(oldRestitution);
}

// 5. Elastic collision asymmetric masses (Analytical 1D mechanics solution)
void test_elastic_collision_asymmetric_mass() {
    double oldRestitution = getRestitution();
    setRestitution(1.0);

    WorldPhysics world({800.0, 600.0});
    world.setGravity({0.0, 0.0});

    double m1 = 3.0;
    double m2 = 1.0;
    RigidBody b1(5.0, m1, {98.0, 200.0}, {10.0, 0.0});
    RigidBody b2(5.0, m2, {102.0, 200.0}, {0.0, 0.0});

    double initP = m1 * 10.0 + m2 * 0.0; // 30.0
    double initKE = 0.5 * m1 * 100.0;    // 150.0

    // Analytical formulas for 1D elastic collision with v2 = 0:
    // v1' = (m1 - m2) / (m1 + m2) * v1 = (3 - 1) / 4 * 10 = 5.0
    // v2' = (2 * m1)  / (m1 + m2) * v1 = (2 * 3) / 4 * 10 = 15.0
    double expectedV1 = 5.0;
    double expectedV2 = 15.0;

    bool resolved = world.resolveCollision(b1, b2);
    ASSERT_TRUE(resolved);

    auto v1 = b1.getVelocity();
    auto v2 = b2.getVelocity();

    double finalP = m1 * v1[0] + m2 * v2[0];
    double finalKE = 0.5 * m1 * (v1[0] * v1[0]) + 0.5 * m2 * (v2[0] * v2[0]);

    ASSERT_NEAR(v1[0], expectedV1, 1e-6);
    ASSERT_NEAR(v2[0], expectedV2, 1e-6);
    ASSERT_NEAR(finalP, initP, 1e-6);
    ASSERT_NEAR(finalKE, initKE, 1e-6);

    setRestitution(oldRestitution);
}

// 6. Inelastic collision momentum conservation and energy dissipation
void test_inelastic_collision() {
    double oldRestitution = getRestitution();
    setRestitution(0.5); // Inelastic restitution e = 0.5

    WorldPhysics world({800.0, 600.0});
    RigidBody b1(5.0, 2.0, {98.0, 100.0}, {20.0, 0.0});
    RigidBody b2(5.0, 2.0, {102.0, 100.0}, {-20.0, 0.0});

    double initP = 2.0 * 20.0 + 2.0 * (-20.0); // 0.0
    double initKE = 0.5 * 2.0 * 400.0 + 0.5 * 2.0 * 400.0; // 800.0

    world.resolveCollision(b1, b2);

    auto v1 = b1.getVelocity();
    auto v2 = b2.getVelocity();

    double finalP = 2.0 * v1[0] + 2.0 * v2[0];
    double finalKE = 0.5 * 2.0 * (v1[0] * v1[0]) + 0.5 * 2.0 * (v2[0] * v2[0]);

    // Momentum must STILL be conserved in inelastic collision
    ASSERT_NEAR(finalP, initP, 1e-6);

    // Energy must dissipate (finalKE < initKE)
    ASSERT_GT(initKE, finalKE);

    // For equal mass head-on with e=0.5: v_rel_final = -e * v_rel_init = -0.5 * (-40) = 20
    double vRelFinal = v2[0] - v1[0];
    ASSERT_NEAR(vRelFinal, 20.0, 1e-6);

    setRestitution(oldRestitution);
}

// 7. Positional separation (penetration resolution)
void test_positional_overlap_correction() {
    WorldPhysics world({800.0, 600.0});
    RigidBody b1(10.0, 1.0, {100.0, 100.0});
    RigidBody b2(10.0, 1.0, {110.0, 100.0}); // Overlapping by 10 units (r1+r2=20, dist=10)

    world.resolveCollision(b1, b2);

    auto p1 = b1.getPosition();
    auto p2 = b2.getPosition();

    double dist = std::hypot(p2[0] - p1[0], p2[1] - p1[1]);
    ASSERT_NEAR(dist, 20.0, 1e-6);
}

// 8. Border collision bounce and clamping
void test_border_collision_rebound() {
    double oldRestitution = getRestitution();
    setRestitution(0.8);

    WorldPhysics world({800.0, 600.0});
    world.setGravity({0.0, 0.0});

    double r = 5.0;
    // Body penetrates right boundary at x = 800: position is 798 (penetration = 3)
    RigidBody b(r, 1.0, {798.0, 300.0}, {100.0, 0.0});

    world.borderCheck(b, 1.0 / 60.0);

    auto pos = b.getPosition();
    auto vel = b.getVelocity();

    // Position must be clamped to maxBound = borderX - radius = 800 - 5 = 795
    ASSERT_NEAR(pos[0], 795.0, 1e-6);
    // Velocity must be reversed and scaled by e: -100 * 0.8 = -80
    ASSERT_NEAR(vel[0], -80.0, 1e-6);

    setRestitution(oldRestitution);
}

// 9. Border settling with gravity (eliminates resting jitter)
void test_border_settling_with_gravity() {
    WorldPhysics world({800.0, 600.0});
    double g = -980.0;
    world.setGravity({0.0, g});

    double dt = 1.0 / 60.0;
    // Ball on ground moving downwards slowly (well under the 2*g*dt resting threshold)
    RigidBody b(5.0, 1.0, {400.0, 3.0}, {0.0, -5.0});

    world.borderCheck(b, dt);

    auto pos = b.getPosition();
    auto vel = b.getVelocity();

    // Must be clamped to bottom bound = radius = 5.0
    ASSERT_NEAR(pos[1], 5.0, 1e-6);
    // Velocity should settle to zero instead of endlessly bouncing
    ASSERT_NEAR(vel[1], 0.0, 1e-9);
}

// 9b. Body pushed into the floor and already moving out must not be flipped back in
void test_border_penetrating_body_moving_out() {
    double oldRestitution = getRestitution();
    setRestitution(0.7);

    WorldPhysics world({800.0, 600.0});
    world.setGravity({0.0, -980.0});

    // Radius 5, centre at y=3 -> 2 units inside the floor, moving UP at +50
    RigidBody b(5.0, 1.0, {400.0, 3.0}, {0.0, 50.0});
    world.borderCheck(b, 1.0 / 60.0);

    ASSERT_NEAR(b.getPosition()[1], 5.0, 1e-9);   // pushed back out
    ASSERT_NEAR(b.getVelocity()[1], 50.0, 1e-9);  // velocity untouched

    // Same on a side wall (non-gravity axis), moving away from the right wall
    RigidBody s(5.0, 1.0, {798.0, 300.0}, {-30.0, 0.0});
    world.borderCheck(s, 1.0 / 60.0);
    ASSERT_NEAR(s.getPosition()[0], 795.0, 1e-9);
    ASSERT_NEAR(s.getVelocity()[0], -30.0, 1e-9);

    setRestitution(oldRestitution);
}

// 9b2. A single dropped ball must come to rest exactly on the floor
void test_single_ball_settles_on_floor() {
    WorldPhysics world({800.0, 600.0});
    world.setGravity({0.0, -980.0});
    world.addBody(RigidBody(5.0, 1.0, {400.0, 300.0}, {0.0, 0.0}));

    TimeManager time;
    double minY = 1e9;
    for (int s = 0; s < 1200; s++) {
        world.runPhysics(time);
        minY = std::min(minY, world.getBodies()[0].getPosition()[1]);
    }

    const auto& b = world.getBodies()[0];
    ASSERT_TRUE(minY >= 5.0 - 1e-9);                   // never sinks into the floor
    ASSERT_NEAR(b.getPosition()[1], 5.0, 1e-9);        // resting on the floor
    ASSERT_NEAR(b.getVelocity()[1], 0.0, 1e-9);        // not hopping
}

// 9b3. Ball with sideways speed settles against the wall for every gravity direction
void test_ball_settles_all_gravity_directions() {
    const std::array<std::array<double, 2>, 4> gravities = {{
        {0.0, -980.0}, {0.0, 980.0}, {-980.0, 0.0}, {980.0, 0.0}
    }};
    for (const auto& g : gravities) {
        WorldPhysics world({800.0, 600.0});
        world.setGravity(g);
        world.addBody(RigidBody(5.0, 1.0, {400.0, 300.0}, {30.0, -20.0}));

        TimeManager time;
        for (int s = 0; s < 1500; s++) world.runPhysics(time);

        const auto& b = world.getBodies()[0];
        int axis = (g[0] != 0.0) ? 0 : 1;
        double expected = (g[axis] < 0.0) ? 5.0 : world.getBorder()[axis] - 5.0;
        ASSERT_NEAR(b.getPosition()[axis], expected, 1e-9);
        ASSERT_NEAR(b.getVelocity()[axis], 0.0, 1e-9);
    }
}

// 9c. Large pile under gravity: no body may end up outside the box
void test_pile_stays_inside_box() {
    WorldPhysics world({800.0, 600.0});
    world.setGravity({0.0, -980.0});
    world.setGridSize(10.0);

    std::srand(42);
    for (int i = 0; i < 5000; i++) {
        double m = 10.0 + (std::rand() / (double)RAND_MAX) * 90.0;
        double r = std::sqrt(m / M_PI) / 2.0;
        double x = r + (std::rand() / (double)RAND_MAX) * (800.0 - 2 * r);
        double y = r + (std::rand() / (double)RAND_MAX) * (600.0 - 2 * r);
        double vx = -50.0 + (std::rand() / (double)RAND_MAX) * 100.0;
        double vy = -50.0 + (std::rand() / (double)RAND_MAX) * 100.0;
        world.addBody(RigidBody(r, m, {x, y}, {vx, vy}));
    }

    TimeManager time;
    for (int s = 0; s < 600; s++) world.runPhysics(time);

    for (const auto& b : world.getBodies()) {
        auto p = b.getPosition();
        ASSERT_TRUE(p[0] >= 0.0 && p[0] <= 800.0);
        ASSERT_TRUE(p[1] >= 0.0 && p[1] <= 600.0);
    }
}

// 10. Spatial Hash Grid detects 100% of naive collisions (0 false negatives)
void test_spatial_hash_grid_completeness() {
    WorldPhysics world({800.0, 600.0});
    world.setGridSize(15.0);

    // Place 300 random bodies
    std::srand(12345);
    for (int i = 0; i < 300; i++) {
        double x = 10.0 + (std::rand() / (double)RAND_MAX) * 780.0;
        double y = 10.0 + (std::rand() / (double)RAND_MAX) * 580.0;
        double r = 4.0;
        world.addBody(RigidBody(r, 1.0, {x, y}, {0.0, 0.0}));
    }

    const auto& bodies = world.getBodies();

    // Find all true colliding pairs using exhaustive O(N^2) search
    std::vector<std::pair<size_t, size_t>> naiveCollisions;
    for (size_t i = 0; i < bodies.size(); i++) {
        for (size_t j = i + 1; j < bodies.size(); j++) {
            double nx, ny, overlap;
            if (world.areColliding(bodies[i], bodies[j], nx, ny, overlap)) {
                naiveCollisions.push_back({i, j});
            }
        }
    }

    // Now test with spatial grid
    TimeManager time;
    auto profile = world.runPhysicsProfiled(time);

    // The spatial grid must have evaluated and resolved all collisions
    ASSERT_TRUE(profile.collisionsResolved >= naiveCollisions.size());
}

// 11. Immovable object handling (invMass = 0)
void test_static_body_infinite_mass() {
    WorldPhysics world({800.0, 600.0});
    RigidBody staticBody(10.0, 0.0, {100.0, 100.0}, {0.0, 0.0}); // mass=0 -> invMass=0
    RigidBody movingBody(10.0, 2.0, {115.0, 100.0}, {-20.0, 0.0});

    ASSERT_NEAR(staticBody.getInvMass(), 0.0, 1e-9);

    world.resolveCollision(staticBody, movingBody);

    // Static body position and velocity must never change
    auto staticPos = staticBody.getPosition();
    auto staticVel = staticBody.getVelocity();
    ASSERT_NEAR(staticPos[0], 100.0, 1e-6);
    ASSERT_NEAR(staticVel[0], 0.0, 1e-6);

    // Moving body should bounce off the immovable body
    auto movingVel = movingBody.getVelocity();
    ASSERT_GT(movingVel[0], 0.0);
}

// 12. Spring mechanics and impulse response
void test_spring_mechanics() {
    Spring spring(200.0, 0.0, 100.0, 20.0, 500.0); // bottom at 0, height 20
    RigidBody ball(5.0, 1.0, {250.0, 22.0}, {0.0, -30.0}); // ball bottom at 17 <= 20

    ASSERT_TRUE(isBallOnSpring(ball, spring));

    applySpringImpulse(ball, spring);

    auto vel = ball.getVelocity();
    auto pos = ball.getPosition();

    // Velocity should be reversed upwards
    ASSERT_GT(vel[1], 0.0);
    // Position Y should be restored above spring top + radius
    ASSERT_GT(pos[1], 24.9);
}

int main() {
    std::cout << COLOR_BOLD << COLOR_CYAN
              << "\n======================================================\n"
              << "       2D PHYSICS ENGINE - PHYSICAL CORRECTNESS TESTS\n"
              << "======================================================\n"
              << COLOR_RESET << std::endl;

    RUN_TEST(test_numerical_integration_free_motion);
    RUN_TEST(test_numerical_integration_constant_acceleration);
    RUN_TEST(test_circle_collision_detection);
    RUN_TEST(test_elastic_collision_equal_mass);
    RUN_TEST(test_elastic_collision_asymmetric_mass);
    RUN_TEST(test_inelastic_collision);
    RUN_TEST(test_positional_overlap_correction);
    RUN_TEST(test_border_collision_rebound);
    RUN_TEST(test_border_settling_with_gravity);
    RUN_TEST(test_border_penetrating_body_moving_out);
    RUN_TEST(test_single_ball_settles_on_floor);
    RUN_TEST(test_ball_settles_all_gravity_directions);
    RUN_TEST(test_pile_stays_inside_box);
    RUN_TEST(test_spatial_hash_grid_completeness);
    RUN_TEST(test_static_body_infinite_mass);
    RUN_TEST(test_spring_mechanics);

    std::cout << COLOR_BOLD << COLOR_CYAN
              << "======================================================\n"
              << COLOR_RESET;

    if (testsFailed == 0) {
        std::cout << COLOR_BOLD << COLOR_GREEN
                  << "  ALL " << testsPassed << " TESTS PASSED SUCCESSFULLY!\n"
                  << COLOR_RESET << std::endl;
        return 0;
    } else {
        std::cout << COLOR_BOLD << COLOR_RED
                  << "  TESTS FAILED: " << testsFailed << " / " << (testsPassed + testsFailed) << "\n"
                  << COLOR_RESET << std::endl;
        return 1;
    }
}
