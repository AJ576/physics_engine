// Headless physics benchmark.
// Links physics.cpp + rigidBody.cpp + spring.cpp. No SDL.
// The brute-force collision path lives in THIS file only; WorldPhysics::resolveCollision
// is private. Algorithm is a direct copy of physics.cpp (areColliding / impulse /
// positional correction / borderCheck).
//
// Integrator energy comparison is a STANDALONE spring-mass test, labeled as such:
// body-body springs are not wired into WorldPhysics.

#include "physics.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

extern double e;

namespace {

using Clock = std::chrono::high_resolution_clock;

constexpr double kDt = 1.0 / 60.0;
constexpr double kBudgetMs = 16.6;
constexpr uint32_t kSeed = 42;
constexpr double kRefW = 800.0;
constexpr double kRefH = 600.0;
constexpr int kRefN = 5000;
constexpr double kGridSize = 10.0; // must match WorldPhysics::grid_size
constexpr double kHugeBorder = 1.0e12;

const int kNs[] = {500, 1000, 2000, 5000, 10000, 20000};
const int kExtraBudgetNs[] = {40000, 80000, 160000};

struct StepStats {
    double mean_ms = 0.0;
    double p50_ms = 0.0;
    double p99_ms = 0.0;
    double max_ms = 0.0;
};

struct AcrossRuns {
    double median_mean = 0.0;
    double std_mean = 0.0;
    double median_p50 = 0.0;
    double std_p50 = 0.0;
    double median_p99 = 0.0;
    double std_p99 = 0.0;
    double median_max = 0.0;
    double std_max = 0.0;
};

double percentile(std::vector<double> v, double p) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    if (p <= 0.0) return v.front();
    if (p >= 1.0) return v.back();
    const double idx = p * static_cast<double>(v.size() - 1);
    const size_t lo = static_cast<size_t>(std::floor(idx));
    const size_t hi = static_cast<size_t>(std::ceil(idx));
    const double t = idx - static_cast<double>(lo);
    return v[lo] * (1.0 - t) + v[hi] * t;
}

double mean_of(const std::vector<double>& v) {
    if (v.empty()) return 0.0;
    return std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(v.size());
}

double stddev_of(const std::vector<double>& v) {
    if (v.size() < 2) return 0.0;
    const double m = mean_of(v);
    double acc = 0.0;
    for (double x : v) {
        const double d = x - m;
        acc += d * d;
    }
    return std::sqrt(acc / static_cast<double>(v.size() - 1));
}

double median_of(std::vector<double> v) {
    return percentile(std::move(v), 0.5);
}

StepStats stats_from_steps(const std::vector<double>& steps_ms) {
    StepStats s;
    s.mean_ms = mean_of(steps_ms);
    s.p50_ms = percentile(steps_ms, 0.50);
    s.p99_ms = percentile(steps_ms, 0.99);
    s.max_ms = steps_ms.empty() ? 0.0 : *std::max_element(steps_ms.begin(), steps_ms.end());
    return s;
}

AcrossRuns summarize_runs(const std::vector<StepStats>& runs) {
    std::vector<double> means, p50s, p99s, maxs;
    means.reserve(runs.size());
    p50s.reserve(runs.size());
    p99s.reserve(runs.size());
    maxs.reserve(runs.size());
    for (const auto& r : runs) {
        means.push_back(r.mean_ms);
        p50s.push_back(r.p50_ms);
        p99s.push_back(r.p99_ms);
        maxs.push_back(r.max_ms);
    }
    AcrossRuns a;
    a.median_mean = median_of(means);
    a.std_mean = stddev_of(means);
    a.median_p50 = median_of(p50s);
    a.std_p50 = stddev_of(p50s);
    a.median_p99 = median_of(p99s);
    a.std_p99 = stddev_of(p99s);
    a.median_max = median_of(maxs);
    a.std_max = stddev_of(maxs);
    return a;
}

void arena_for(int n, bool constant_density, double& w, double& h) {
    if (!constant_density) {
        w = kRefW;
        h = kRefH;
        return;
    }
    const double density = static_cast<double>(kRefN) / (kRefW * kRefH);
    const double area = static_cast<double>(n) / density;
    const double aspect = kRefW / kRefH;
    h = std::sqrt(area / aspect);
    w = aspect * h;
}

void populate_world(WorldPhysics& world, int n, double w, double h, uint32_t seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> mass_dist(10.0, 100.0);
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    for (int i = 0; i < n; ++i) {
        const double mass = mass_dist(rng);
        const double radius = std::sqrt(mass / M_PI) / 2.0;
        const double posX = u01(rng) * w;
        const double posY = u01(rng) * h;
        const double velX = -5.0 + u01(rng) * 100.0;
        const double velY = -5.0 + u01(rng) * 100.0;
        world.addBody(RigidBody(radius, mass, {posX, posY}, {velX, velY}));
    }
}

WorldPhysics make_world(int n, double w, double h, uint32_t seed, bool gravity_on) {
    WorldPhysics world({w, h});
    world.setGravity(gravity_on ? std::array<double, 2>{0.0, -980.0}
                                : std::array<double, 2>{0.0, 0.0});
    populate_world(world, n, w, h, seed);
    return world;
}

// --- brute-force path (bench-only copy of physics.cpp collision + borders) ---

bool are_colliding(const RigidBody& body1, const RigidBody& body2,
                   double& nx, double& ny, double& overlap) {
    std::array<double, 2> pos1 = body1.getPosition();
    std::array<double, 2> pos2 = body2.getPosition();
    const double dx = pos2[0] - pos1[0];
    const double dy = pos2[1] - pos1[1];
    const double distanceSq = dx * dx + dy * dy;
    const double minDistance = body1.getRadius() + body2.getRadius();
    const double minDistanceSq = minDistance * minDistance;
    if (distanceSq >= minDistanceSq) {
        overlap = 0.0;
        nx = 0.0;
        ny = 0.0;
        return false;
    }
    const double distance = std::sqrt(distanceSq);
    if (distance == 0.0) {
        nx = 1.0;
        ny = 0.0;
        overlap = minDistance;
    } else {
        nx = dx / distance;
        ny = dy / distance;
        overlap = minDistance - distance;
    }
    return true;
}

void calculate_impulse(RigidBody& body1, RigidBody& body2, double nx, double ny) {
    auto vel1 = body1.getVelocity();
    auto vel2 = body2.getVelocity();
    const double v_rel_x = vel2[0] - vel1[0];
    const double v_rel_y = vel2[1] - vel1[1];
    const double v_rel_normal = (v_rel_x * nx) + (v_rel_y * ny);
    if (v_rel_normal >= 0) return;
    const double numerator = -1.0 * (1.0 + e) * v_rel_normal;
    const double invMassSum = body1.getInvMass() + body2.getInvMass();
    if (invMassSum == 0) return;
    const double j = numerator / invMassSum;
    body1.setVelocityX(vel1[0] - ((j * body1.getInvMass()) * nx));
    body1.setVelocityY(vel1[1] - ((j * body1.getInvMass()) * ny));
    body2.setVelocityX(vel2[0] + ((j * body2.getInvMass()) * nx));
    body2.setVelocityY(vel2[1] + ((j * body2.getInvMass()) * ny));
}

void resolve_collision(RigidBody& b1, RigidBody& b2) {
    double nx = 0.0, ny = 0.0, overlap = 0.0;
    if (!are_colliding(b1, b2, nx, ny, overlap)) return;
    std::array<double, 2> pos1 = b1.getPosition();
    std::array<double, 2> pos2 = b2.getPosition();
    const double invMassSum = b1.getInvMass() + b2.getInvMass();
    if (invMassSum == 0) return;
    const double move1 = overlap * (b1.getInvMass() / invMassSum);
    const double move2 = overlap * (b2.getInvMass() / invMassSum);
    b1.setPositionX(pos1[0] - nx * move1);
    b1.setPositionY(pos1[1] - ny * move1);
    b2.setPositionX(pos2[0] + nx * move2);
    b2.setPositionY(pos2[1] + ny * move2);
    calculate_impulse(b1, b2, nx, ny);
}

void border_check(RigidBody& body1, std::array<double, 2> border,
                  std::array<double, 2> gravity, double dt) {
    const double radius = body1.getRadius();
    std::array<double, 2> pos = body1.getPosition();
    std::array<double, 2> vel = body1.getVelocity();
    for (int i = 0; i < 2; ++i) {
        const double minBound = radius;
        const double maxBound = border[i] - radius;
        double bound = 0.0;
        if (pos[i] < minBound) {
            bound = minBound;
        } else if (pos[i] > maxBound) {
            bound = maxBound;
        } else {
            continue;
        }
        const bool hasGravity = gravity[i] != 0.0;
        if (hasGravity) {
            const bool movingWithGravity = vel[i] * gravity[i] > 0.0;
            const double restingVelocityThreshold = std::abs(gravity[i]) * dt;
            const bool slowEnoughToRest = std::abs(vel[i]) <= restingVelocityThreshold;
            if (movingWithGravity && slowEnoughToRest) {
                pos[i] = bound;
                vel[i] = 0.0;
            } else {
                vel[i] = -vel[i] * e;
            }
        } else {
            pos[i] = bound;
            vel[i] = -vel[i] * e;
        }
    }
    body1.setPosition(pos);
    body1.setVelocity(vel);
}

void brute_force_step(WorldPhysics& world, double dt) {
    auto& bodies = world.getBodies();
    const auto gravity = world.getGravity();
    const auto border = world.getBorder();

    for (size_t i = 0; i < bodies.size(); ++i) {
        if (bodies[i].getInvMass() == 0.0) continue;
        auto force = bodies[i].getForce();
        force[0] += bodies[i].getMass() * gravity[0];
        force[1] += bodies[i].getMass() * gravity[1];
        bodies[i].setForce(force);
    }
    for (size_t i = 0; i < bodies.size(); ++i) {
        bodies[i].numericalIntegration(dt);
    }
    for (size_t i = 0; i < bodies.size(); ++i) {
        for (size_t j = i + 1; j < bodies.size(); ++j) {
            resolve_collision(bodies[i], bodies[j]);
        }
    }
    for (size_t i = 0; i < bodies.size(); ++i) {
        border_check(bodies[i], border, gravity, dt);
    }
}

long long cell_key(int cell_x, int cell_y) {
    return (static_cast<long long>(cell_x) << 32) | static_cast<unsigned int>(cell_y);
}

// Same neighbor set as physics.cpp runPhysics.
uint64_t count_grid_candidate_pairs(const std::vector<RigidBody>& bodies) {
    std::unordered_map<long long, std::vector<int>> grid;
    grid.reserve(bodies.size() * 2);
    for (int i = 0; i < static_cast<int>(bodies.size()); ++i) {
        const auto p = bodies[static_cast<size_t>(i)].getPosition();
        const int cell_x = static_cast<int>(std::floor(p[0] / kGridSize));
        const int cell_y = static_cast<int>(std::floor(p[1] / kGridSize));
        grid[cell_key(cell_x, cell_y)].push_back(i);
    }
    uint64_t pairs = 0;
    const int offsets[4][2] = {{1, 0}, {0, 1}, {1, 1}, {1, -1}};
    for (const auto& entry : grid) {
        const long long key = entry.first;
        const auto& cell = entry.second;
        const int n = static_cast<int>(cell.size());
        pairs += static_cast<uint64_t>(n) * static_cast<uint64_t>(n - 1) / 2;
        const int x = static_cast<int>(key >> 32);
        const int y = static_cast<int>(key & 0xFFFFFFFF);
        for (const auto& off : offsets) {
            const long long nkey = cell_key(x + off[0], y + off[1]);
            auto it = grid.find(nkey);
            if (it == grid.end()) continue;
            pairs += static_cast<uint64_t>(cell.size()) * static_cast<uint64_t>(it->second.size());
        }
    }
    return pairs;
}

uint64_t count_brute_pairs(size_t n) {
    return static_cast<uint64_t>(n) * static_cast<uint64_t>(n - 1) / 2;
}

void timed_grid_steps(WorldPhysics& world, TimeManager& time, int warmup, int timed,
                      std::vector<double>& steps_ms) {
    steps_ms.clear();
    steps_ms.reserve(static_cast<size_t>(timed));
    for (int i = 0; i < warmup; ++i) world.runPhysics(time);
    for (int i = 0; i < timed; ++i) {
        const auto t0 = Clock::now();
        world.runPhysics(time);
        const auto t1 = Clock::now();
        steps_ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
    }
}

void timed_brute_steps(WorldPhysics& world, int warmup, int timed,
                       std::vector<double>& steps_ms) {
    steps_ms.clear();
    steps_ms.reserve(static_cast<size_t>(timed));
    for (int i = 0; i < warmup; ++i) brute_force_step(world, kDt);
    for (int i = 0; i < timed; ++i) {
        const auto t0 = Clock::now();
        brute_force_step(world, kDt);
        const auto t1 = Clock::now();
        steps_ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
    }
}

int brute_timed_steps_for(int n) {
    if (n >= 20000) return 5;
    if (n >= 10000) return 10;
    if (n >= 5000) return 30;
    return 120;
}

int brute_warmup_for(int n) {
    if (n >= 10000) return 2;
    if (n >= 5000) return 5;
    return 20;
}

void print_across(const char* label, int n, double w, double h, const AcrossRuns& a) {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << label << " N=" << n
              << " arena=" << std::setprecision(1) << w << "x" << h
              << std::setprecision(3)
              << "  mean " << a.median_mean << " ± " << a.std_mean << " ms"
              << "  p50 " << a.median_p50 << " ± " << a.std_p50 << " ms"
              << "  p99 " << a.median_p99 << " ± " << a.std_p99 << " ms"
              << "  max " << a.median_max << " ± " << a.std_max << " ms"
              << "\n";
}

AcrossRuns run_grid_scaling(int n, bool constant_density, int runs, int warmup, int timed,
                            std::ostream& csv) {
    double w = 0, h = 0;
    arena_for(n, constant_density, w, h);
    TimeManager time;
    std::vector<StepStats> run_stats;
    run_stats.reserve(static_cast<size_t>(runs));
    std::vector<double> steps;
    for (int r = 0; r < runs; ++r) {
        WorldPhysics world = make_world(n, w, h, kSeed, false);
        timed_grid_steps(world, time, warmup, timed, steps);
        run_stats.push_back(stats_from_steps(steps));
    }
    const AcrossRuns a = summarize_runs(run_stats);
    csv << (constant_density ? "const_density" : "fixed_arena") << ",grid," << n << ","
        << w << "," << h << "," << runs << "," << warmup << "," << timed << ","
        << a.median_mean << "," << a.std_mean << ","
        << a.median_p50 << "," << a.std_p50 << ","
        << a.median_p99 << "," << a.std_p99 << ","
        << a.median_max << "," << a.std_max << "\n";
    print_across(constant_density ? "GRID const-density" : "GRID fixed-arena", n, w, h, a);
    return a;
}

AcrossRuns run_brute_scaling(int n, bool constant_density, int runs,
                             std::ostream& csv) {
    double w = 0, h = 0;
    arena_for(n, constant_density, w, h);
    const int warmup = brute_warmup_for(n);
    const int timed = brute_timed_steps_for(n);
    std::vector<StepStats> run_stats;
    run_stats.reserve(static_cast<size_t>(runs));
    std::vector<double> steps;
    for (int r = 0; r < runs; ++r) {
        WorldPhysics world = make_world(n, w, h, kSeed, false);
        timed_brute_steps(world, warmup, timed, steps);
        run_stats.push_back(stats_from_steps(steps));
    }
    const AcrossRuns a = summarize_runs(run_stats);
    csv << (constant_density ? "const_density" : "fixed_arena") << ",brute," << n << ","
        << w << "," << h << "," << runs << "," << warmup << "," << timed << ","
        << a.median_mean << "," << a.std_mean << ","
        << a.median_p50 << "," << a.std_p50 << ","
        << a.median_p99 << "," << a.std_p99 << ","
        << a.median_max << "," << a.std_max << "\n";
    print_across(constant_density ? "BRUTE const-density" : "BRUTE fixed-arena", n, w, h, a);
    return a;
}

void run_pair_counts(int n, bool constant_density, std::ostream& csv) {
    double w = 0, h = 0;
    arena_for(n, constant_density, w, h);
    WorldPhysics world = make_world(n, w, h, kSeed, false);
    TimeManager time;
    // Count after one physics step so the layout is post-integration, same as the
    // engine's collision pass (grid is rebuilt after numericalIntegration).
    world.runPhysics(time);
    const auto& bodies = world.getBodies();
    const uint64_t grid_pairs = count_grid_candidate_pairs(bodies);
    const uint64_t brute_pairs = count_brute_pairs(bodies.size());
    csv << (constant_density ? "const_density" : "fixed_arena") << "," << n << ","
        << w << "," << h << "," << grid_pairs << "," << brute_pairs << ","
        << (grid_pairs == 0 ? 0.0 : static_cast<double>(brute_pairs) / static_cast<double>(grid_pairs))
        << "\n";
    std::cout << "PAIRS " << (constant_density ? "const-density" : "fixed-arena")
              << " N=" << n
              << " grid=" << grid_pairs
              << " brute=" << brute_pairs
              << " brute/grid=" << std::fixed << std::setprecision(2)
              << (grid_pairs == 0 ? 0.0 : static_cast<double>(brute_pairs) / static_cast<double>(grid_pairs))
              << "\n";
}

struct Totals {
    double px = 0.0, py = 0.0, ke = 0.0;
};

Totals totals_of(const std::vector<RigidBody>& bodies) {
    Totals t;
    for (const auto& b : bodies) {
        const auto v = b.getVelocity();
        const double m = b.getMass();
        t.px += m * v[0];
        t.py += m * v[1];
        t.ke += 0.5 * m * (v[0] * v[0] + v[1] * v[1]);
    }
    return t;
}

void run_conservation(const std::string& out_path) {
    std::ofstream out(out_path);
    const double saved_e = e;
    e = 1.0;

    // Non-overlapping lattice so the first-step pile-up does not dominate drift.
    constexpr int nx = 10;
    constexpr int ny = 10;
    constexpr double spacing = 12.0;
    constexpr double mass = 20.0;
    constexpr double radius = 2.0;
    // WorldPhysics always clamps to [r, border-r]; a "huge" max bound does not
    // disable the min walls at x=r and y=r. Spawn far from the origin so bodies
    // cannot reach those walls in 5000 steps (v_rms ~ tens of units, travel
    // << 1e5).
    constexpr double origin = 1.0e5;
    WorldPhysics world({kHugeBorder, kHugeBorder});
    world.setGravity({0.0, 0.0});
    std::mt19937 rng(kSeed);
    std::uniform_real_distribution<double> vel(-40.0, 40.0);
    for (int iy = 0; iy < ny; ++iy) {
        for (int ix = 0; ix < nx; ++ix) {
            const double x = origin + static_cast<double>(ix) * spacing;
            const double y = origin + static_cast<double>(iy) * spacing;
            world.addBody(RigidBody(radius, mass, {x, y}, {vel(rng), vel(rng)}));
        }
    }

    TimeManager time;
    const Totals t0 = totals_of(world.getBodies());
    constexpr int kSteps = 5000;
    Totals tmax_mom = t0;
    double max_rel_mom = 0.0;
    double max_rel_ke = 0.0;
    Totals t_end = t0;
    for (int i = 0; i < kSteps; ++i) {
        world.runPhysics(time);
        t_end = totals_of(world.getBodies());
        const double mom0 = std::hypot(t0.px, t0.py);
        const double mom = std::hypot(t_end.px, t_end.py);
        const double rel_mom = mom0 > 0.0 ? std::abs(mom - mom0) / mom0 : std::abs(mom - mom0);
        const double rel_ke = t0.ke > 0.0 ? std::abs(t_end.ke - t0.ke) / t0.ke : std::abs(t_end.ke - t0.ke);
        if (rel_mom > max_rel_mom) {
            max_rel_mom = rel_mom;
            tmax_mom = t_end;
        }
        if (rel_ke > max_rel_ke) max_rel_ke = rel_ke;
    }

    std::ostringstream body;
    body << std::scientific << std::setprecision(10);
    body << "CONSERVATION TEST (engine WorldPhysics::runPhysics)\n";
    body << "Setup: 100 non-overlapping circles, e=1, gravity=(0,0), "
            "border=1e12, spawned at ~1e5 so min-walls at x=r,y=r are unreachable\n";
    body << "Steps: " << kSteps << " at dt=1/60, seed=" << kSeed << "\n";
    body << "Initial px=" << t0.px << " py=" << t0.py << " |p|=" << std::hypot(t0.px, t0.py) << "\n";
    body << "Initial KE=" << t0.ke << "\n";
    body << "Final   px=" << t_end.px << " py=" << t_end.py << " |p|=" << std::hypot(t_end.px, t_end.py) << "\n";
    body << "Final   KE=" << t_end.ke << "\n";
    body << "Relative |p| drift at end: "
         << (std::hypot(t0.px, t0.py) > 0.0
                 ? std::abs(std::hypot(t_end.px, t_end.py) - std::hypot(t0.px, t0.py)) / std::hypot(t0.px, t0.py)
                 : 0.0)
         << "\n";
    body << "Relative KE drift at end:  " << (t0.ke > 0.0 ? (t_end.ke - t0.ke) / t0.ke : 0.0) << "\n";
    body << "Max relative |p| drift over run: " << max_rel_mom << "\n";
    body << "Max relative KE drift over run:  " << max_rel_ke << "\n";
    body << "px change: " << (t_end.px - t0.px) << "  py change: " << (t_end.py - t0.py) << "\n";
    body << "Note: positional correction does not modify velocity, so KE only changes via impulses.\n";
    body << "With e=1, pairwise elastic impulses conserve KE; measured drift is roundoff if no walls.\n";
    (void)tmax_mom;

    std::cout << body.str();
    out << body.str();
    e = saved_e;
}

void run_integrator_energy(const std::string& out_path) {
    // STANDALONE analytic test. Not WorldPhysics. Not the Spring class.
    constexpr double m = 1.0;
    constexpr double k = 4.0 * M_PI * M_PI; // omega = 2π, period = 1 s
    constexpr double x0 = 1.0;
    constexpr double v0 = 0.0;
    constexpr int steps = 10000;
    const double E0 = 0.5 * k * x0 * x0 + 0.5 * m * v0 * v0;

    auto energy = [&](double x, double v) {
        return 0.5 * m * v * v + 0.5 * k * x * x;
    };

    double x_ex = x0, v_ex = v0;
    double x_si = x0, v_si = v0;
    double x_eng = x0, v_eng = v0;
    double max_ex = 0.0, max_si = 0.0, max_eng = 0.0;
    for (int i = 0; i < steps; ++i) {
        // Explicit Euler: x += v dt, then v += a(x_old) dt.
        {
            const double a = -(k / m) * x_ex;
            x_ex += v_ex * kDt;
            v_ex += a * kDt;
            max_ex = std::max(max_ex, std::abs(energy(x_ex, v_ex) - E0) / E0);
        }
        // Semi-implicit Euler: v += a(x) dt, then x += v_new dt.
        {
            const double a = -(k / m) * x_si;
            v_si += a * kDt;
            x_si += v_si * kDt;
            max_si = std::max(max_si, std::abs(energy(x_si, v_si) - E0) / E0);
        }
        // Engine formula (RigidBody::numericalIntegration): x += v dt + 0.5 a dt^2, v += a dt.
        {
            const double a = -(k / m) * x_eng;
            x_eng += v_eng * kDt + 0.5 * a * kDt * kDt;
            v_eng += a * kDt;
            max_eng = std::max(max_eng, std::abs(energy(x_eng, v_eng) - E0) / E0);
        }
    }

    // Also drive a real RigidBody with F = -k x to confirm the formula match.
    RigidBody body(1.0, m, {x0, 0.0}, {v0, 0.0});
    double max_rb = 0.0;
    for (int i = 0; i < steps; ++i) {
        const double x = body.getPosition()[0];
        body.setForce({-(k) * x, 0.0});
        body.numericalIntegration(kDt);
        const double xx = body.getPosition()[0];
        const double vv = body.getVelocity()[0];
        max_rb = std::max(max_rb, std::abs(energy(xx, vv) - E0) / E0);
    }

    std::ostringstream body_txt;
    body_txt << std::scientific << std::setprecision(8);
    body_txt << "STANDALONE spring-mass energy test (NOT an engine WorldPhysics measurement)\n";
    body_txt << "System: 1D harmonic oscillator m=1, k=4*pi^2, x0=1, v0=0, dt=1/60, steps=" << steps << "\n";
    body_txt << "E0=" << E0 << "\n";
    body_txt << "Explicit Euler max relative |E-E0|/E0:              " << max_ex << "\n";
    body_txt << "Semi-implicit Euler max relative |E-E0|/E0:          " << max_si << "\n";
    body_txt << "Engine-style kinematics (formula copy) max rel drift: " << max_eng << "\n";
    body_txt << "RigidBody::numericalIntegration + F=-k x max rel:    " << max_rb << "\n";
    body_txt << "Final E/E0 explicit:      " << energy(x_ex, v_ex) / E0 << "\n";
    body_txt << "Final E/E0 semi-implicit: " << energy(x_si, v_si) / E0 << "\n";
    body_txt << "Final E/E0 engine-style:  " << energy(x_eng, v_eng) / E0 << "\n";
    body_txt << "Final E/E0 RigidBody:     "
             << energy(body.getPosition()[0], body.getVelocity()[0]) / E0 << "\n";
    body_txt << "Label: standalone analytic test. WorldPhysics springs are not a mass-spring network.\n";

    std::cout << body_txt.str();
    std::ofstream out(out_path);
    out << body_txt.str();
}

void profile_loop(int n, int seconds) {
    double w = 0, h = 0;
    arena_for(n, true, w, h);
    WorldPhysics world = make_world(n, w, h, kSeed, false);
    TimeManager time;
    const auto deadline = Clock::now() + std::chrono::seconds(seconds);
    uint64_t steps = 0;
    while (Clock::now() < deadline) {
        world.runPhysics(time);
        ++steps;
    }
    std::cout << "PROFILE N=" << n << " steps=" << steps << " seconds=" << seconds << "\n";
}

void print_usage() {
    std::cout
        << "physics_bench [--all] [--scaling] [--grid-vs-brute] [--conservation]\n"
        << "              [--integrator] [--profile N SECONDS] [--quick]\n"
        << "              [--out-dir DIR]\n";
}

} // namespace

int main(int argc, char** argv) {
    bool do_all = true;
    bool do_scaling = false;
    bool do_grid_vs_brute = false;
    bool do_conservation = false;
    bool do_integrator = false;
    bool do_profile = false;
    bool quick = false;
    int profile_n = 5000;
    int profile_seconds = 20;
    std::string out_dir = "bench/results";

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--help" || a == "-h") {
            print_usage();
            return 0;
        }
        if (a == "--quick") {
            quick = true;
            continue;
        }
        if (a == "--out-dir" && i + 1 < argc) {
            out_dir = argv[++i];
            continue;
        }
        if (a == "--profile" && i + 2 < argc) {
            do_all = false;
            do_profile = true;
            profile_n = std::stoi(argv[++i]);
            profile_seconds = std::stoi(argv[++i]);
            continue;
        }
        do_all = false;
        if (a == "--all") do_all = true;
        else if (a == "--scaling") do_scaling = true;
        else if (a == "--grid-vs-brute") do_grid_vs_brute = true;
        else if (a == "--conservation") do_conservation = true;
        else if (a == "--integrator") do_integrator = true;
        else {
            std::cerr << "unknown arg: " << a << "\n";
            print_usage();
            return 1;
        }
    }
    if (do_all) {
        do_scaling = do_grid_vs_brute = do_conservation = do_integrator = true;
    }

    const int runs = quick ? 3 : 5;
    const int warmup = quick ? 20 : 60;
    const int timed = quick ? 60 : 180;
    std::vector<int> ns;
    if (quick) ns = {2000, 5000, 10000};
    else ns.assign(std::begin(kNs), std::end(kNs));

    std::cout << "physics_bench seed=" << kSeed
              << " e=" << e
              << " dt=" << kDt
              << " runs=" << runs
              << " warmup=" << warmup
              << " timed=" << timed
              << " out=" << out_dir << "\n";

    if (do_scaling) {
        std::ofstream csv(out_dir + "/scaling.csv");
        csv << "mode,path,n,w,h,runs,warmup,timed,"
               "median_mean_ms,std_mean_ms,median_p50_ms,std_p50_ms,"
               "median_p99_ms,std_p99_ms,median_max_ms,std_max_ms\n";
        std::cout << "\n=== SCALING (engine grid path, gravity off) ===\n";
        std::vector<std::pair<int, double>> const_means;
        std::vector<std::pair<int, double>> fixed_means;
        for (int n : ns) {
            auto a = run_grid_scaling(n, true, runs, warmup, timed, csv);
            const_means.push_back({n, a.median_mean});
        }
        for (int n : ns) {
            auto a = run_grid_scaling(n, false, runs, warmup, timed, csv);
            fixed_means.push_back({n, a.median_mean});
        }
        auto largest_ok = [](const std::vector<std::pair<int, double>>& v) {
            int best = -1;
            for (auto [n, ms] : v) if (ms <= kBudgetMs) best = n;
            return best;
        };
        std::cout << "Largest listed N with median mean step <= " << kBudgetMs
                  << " ms (const density): " << largest_ok(const_means) << "\n";
        std::cout << "Largest listed N with median mean step <= " << kBudgetMs
                  << " ms (fixed arena): " << largest_ok(fixed_means) << "\n";

        if (!quick) {
            std::cout << "\n=== EXTRA N (grid only, 3 runs x 60 steps) to locate 16.6 ms ===\n";
            const int extra_runs = 3;
            const int extra_warmup = 30;
            const int extra_timed = 60;
            for (int n : kExtraBudgetNs) {
                auto a = run_grid_scaling(n, true, extra_runs, extra_warmup, extra_timed, csv);
                const_means.push_back({n, a.median_mean});
            }
            for (int n : kExtraBudgetNs) {
                auto a = run_grid_scaling(n, false, extra_runs, extra_warmup, extra_timed, csv);
                fixed_means.push_back({n, a.median_mean});
            }
        }

        std::cout << "Largest N with median mean step <= " << kBudgetMs << " ms (const density): "
                  << largest_ok(const_means) << "\n";
        std::cout << "Largest N with median mean step <= " << kBudgetMs << " ms (fixed arena): "
                  << largest_ok(fixed_means) << "\n";
        std::ofstream summary(out_dir + "/budget.txt");
        summary << "const_density_max_n " << largest_ok(const_means) << "\n";
        summary << "fixed_arena_max_n " << largest_ok(fixed_means) << "\n";
        for (auto [n, ms] : const_means) summary << "const " << n << " " << ms << "\n";
        for (auto [n, ms] : fixed_means) summary << "fixed " << n << " " << ms << "\n";
    }

    if (do_grid_vs_brute) {
        std::ofstream pairs(out_dir + "/pairs.csv");
        pairs << "mode,n,w,h,grid_pairs,brute_pairs,brute_over_grid\n";
        std::ofstream csv(out_dir + "/grid_vs_brute.csv");
        csv << "mode,path,n,w,h,runs,warmup,timed,"
               "median_mean_ms,std_mean_ms,median_p50_ms,std_p50_ms,"
               "median_p99_ms,std_p99_ms,median_max_ms,std_max_ms\n";
        std::cout << "\n=== GRID VS BRUTE (constant density, gravity off) ===\n";
        std::cout << "Brute-force narrowphase is a bench-local copy of physics.cpp.\n";
        for (int n : ns) run_pair_counts(n, true, pairs);
        for (int n : ns) run_pair_counts(n, false, pairs);
        for (int n : ns) {
            auto g = run_grid_scaling(n, true, runs, warmup, timed, csv);
            auto b = run_brute_scaling(n, true, runs, csv);
            const double speedup = (g.median_mean > 0.0) ? (b.median_mean / g.median_mean) : 0.0;
            std::cout << std::fixed << std::setprecision(2)
                      << "SPEEDUP const-density N=" << n << " brute/grid=" << speedup << "x\n";
        }
    }

    if (do_conservation) {
        std::cout << "\n=== CONSERVATION ===\n";
        run_conservation(out_dir + "/conservation.txt");
    }

    if (do_integrator) {
        std::cout << "\n=== INTEGRATOR ENERGY (STANDALONE) ===\n";
        run_integrator_energy(out_dir + "/integrator_energy.txt");
    }

    if (do_profile) {
        std::cout << "\n=== PROFILE LOOP ===\n";
        profile_loop(profile_n, profile_seconds);
    }

    return 0;
}
