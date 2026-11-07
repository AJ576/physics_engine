#include "physics.hpp"
#include "rigidBody.hpp"
#include "spring.hpp"

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <fstream>
#include <sstream>

#define COLOR_RESET   "\033[0m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_RED     "\033[31m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_BLUE    "\033[34m"
#define COLOR_MAGENTA "\033[35m"
#define COLOR_CYAN    "\033[36m"
#define COLOR_BOLD    "\033[1m"
#define COLOR_DIM     "\033[2m"

struct BenchmarkStats {
    double meanUs = 0.0;
    double minUs = 0.0;
    double maxUs = 0.0;
    double medianUs = 0.0;
    double p95Us = 0.0;
    double p99Us = 0.0;
    double stddevUs = 0.0;
    double effectiveFps = 0.0;
    double throughputBodiesPerSec = 0.0;
};

BenchmarkStats computeStats(std::vector<double>& samplesUs, size_t bodyCount) {
    BenchmarkStats stats;
    if (samplesUs.empty()) return stats;

    std::sort(samplesUs.begin(), samplesUs.end());
    size_t n = samplesUs.size();

    double sum = std::accumulate(samplesUs.begin(), samplesUs.end(), 0.0);
    stats.meanUs = sum / n;
    stats.minUs = samplesUs.front();
    stats.maxUs = samplesUs.back();
    stats.medianUs = samplesUs[n / 2];
    stats.p95Us = samplesUs[static_cast<size_t>(n * 0.95)];
    stats.p99Us = samplesUs[static_cast<size_t>(n * 0.99)];

    double varianceSum = 0.0;
    for (double v : samplesUs) {
        varianceSum += (v - stats.meanUs) * (v - stats.meanUs);
    }
    stats.stddevUs = std::sqrt(varianceSum / n);

    double meanSec = stats.meanUs / 1e6;
    stats.effectiveFps = (meanSec > 0.0) ? (1.0 / meanSec) : 0.0;
    stats.throughputBodiesPerSec = (meanSec > 0.0) ? (static_cast<double>(bodyCount) / meanSec) : 0.0;

    return stats;
}

// Helper to seed and populate world with deterministic pseudo-random bodies
void populateWorld(WorldPhysics& world, size_t count, double width = 800.0, double height = 600.0, unsigned int seed = 42) {
    world.clearBodies();
    std::srand(seed);

    for (size_t i = 0; i < count; i++) {
        double mass = 10.0 + (std::rand() / (double)RAND_MAX) * 90.0;
        double radius = std::sqrt(mass / M_PI) / 2.0;

        double posX = radius + (std::rand() / (double)RAND_MAX) * (width - 2 * radius);
        double posY = radius + (std::rand() / (double)RAND_MAX) * (height - 2 * radius);

        double velX = -50.0 + (std::rand() / (double)RAND_MAX) * 100.0;
        double velY = -50.0 + (std::rand() / (double)RAND_MAX) * 100.0;

        world.addBody(RigidBody(radius, mass, {posX, posY}, {velX, velY}));
    }
}

// Data structures to collect summary results for reporting
struct ScalabilityResult {
    size_t bodyCount;
    BenchmarkStats stats;
};

struct BroadphaseComparisonResult {
    size_t bodyCount;
    double naiveMs;
    double gridMs;
    double speedup;
    size_t naivePairs;
    size_t gridPairs;
    double pairReductionPct;
};

struct PipelineResult {
    double forcesUs;
    double integrationUs;
    double gridBuildUs;
    double collisionUs;
    double borderUs;
    double springUs;
    double totalUs;
};

struct GridSweepResult {
    double cellSize;
    double stepTimeUs;
    size_t pairsChecked;
};

struct StabilityResult {
    int step;
    double totalEnergy;
    double energyDriftPct;
};

int main(int argc, char* argv[]) {
    bool quickMode = false;
    bool stressMode = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--quick" || arg == "-q") quickMode = true;
        if (arg == "--stress" || arg == "-s") stressMode = true;
    }

    std::cout << COLOR_BOLD << COLOR_CYAN
              << "=========================================================================\n"
              << "            2D PHYSICS ENGINE - BENCHMARK & PROFILING SUITE              \n"
              << "=========================================================================\n"
              << COLOR_RESET;
    std::cout << COLOR_DIM << "Fixed timestep: 1/60 s | Times the physics step only (no rendering)\n"
              << (quickMode ? "Mode: QUICK BENCHMARK\n" : (stressMode ? "Mode: STRESS BENCHMARK\n" : "Mode: STANDARD BENCHMARK\n"))
              << COLOR_RESET << "\n";

    TimeManager timeManager;

    // -------------------------------------------------------------------------
    // BENCHMARK 1: Body Count Scalability & Throughput
    // -------------------------------------------------------------------------
    std::cout << COLOR_BOLD << COLOR_YELLOW
              << "--- [1/5] Scalability & Throughput vs Body Count ---\n"
              << COLOR_RESET;

    std::vector<size_t> bodyCounts;
    if (quickMode) {
        bodyCounts = {250, 500, 1000, 2500, 5000};
    } else if (stressMode) {
        bodyCounts = {500, 1000, 2500, 5000, 10000, 15000, 20000};
    } else {
        bodyCounts = {500, 1000, 2500, 5000, 7500, 10000};
    }

    std::vector<ScalabilityResult> scalabilityResults;
    const int warmupSteps = 10;
    const int measuredSteps = quickMode ? 40 : 100;

    std::cout << std::left
              << std::setw(10) << "Bodies"
              << std::setw(14) << "Mean Step"
              << std::setw(12) << "Min / Max"
              << std::setw(12) << "P95 Latency"
              << std::setw(16) << "Steps / sec"
              << std::setw(18) << "Throughput (b/s)"
              << std::setw(12) << "< 16.67 ms?"
              << "\n";
    std::cout << std::string(84, '-') << "\n";

    for (size_t count : bodyCounts) {
        WorldPhysics world({800.0, 600.0});
        world.setGridSize(10.0);
        world.setGravity({0.0, -980.0});
        populateWorld(world, count);

        // Warmup steps
        for (int w = 0; w < warmupSteps; w++) {
            world.runPhysics(timeManager);
        }

        // Measured steps
        std::vector<double> samplesUs;
        samplesUs.reserve(measuredSteps);

        for (int s = 0; s < measuredSteps; s++) {
            auto t0 = Clock::now();
            world.runPhysics(timeManager);
            auto t1 = Clock::now();
            double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
            samplesUs.push_back(us);
        }

        BenchmarkStats stats = computeStats(samplesUs, count);
        scalabilityResults.push_back({count, stats});

        bool isRealTime = (stats.meanUs <= 16666.7); // 60 Hz = 16.67 ms
        std::string rtString = isRealTime ? (COLOR_GREEN + std::string("yes") + COLOR_RESET)
                                          : (COLOR_RED   + std::string("no")  + COLOR_RESET);

        std::ostringstream minMaxStr;
        minMaxStr << std::fixed << std::setprecision(1) << (stats.minUs / 1000.0) << "/" << (stats.maxUs / 1000.0) << "ms";

        std::cout << std::left
                  << std::setw(10) << count
                  << std::setw(14) << (std::to_string(stats.meanUs / 1000.0).substr(0, 5) + " ms")
                  << std::setw(12) << minMaxStr.str()
                  << std::setw(12) << (std::to_string(stats.p95Us / 1000.0).substr(0, 5) + " ms")
                  << std::setw(16) << std::to_string(static_cast<int>(stats.effectiveFps))
                  << std::setw(18) << static_cast<long long>(stats.throughputBodiesPerSec)
                  << rtString
                  << "\n";
    }
    std::cout << "\n";

    // -------------------------------------------------------------------------
    // BENCHMARK 2: Algorithmic Complexity: Spatial Hash Grid vs Naive O(N^2)
    // -------------------------------------------------------------------------
    std::cout << COLOR_BOLD << COLOR_YELLOW
              << "--- [2/5] Broad-Phase Collision Speedup: Spatial Grid vs Naive O(N^2) ---\n"
              << COLOR_RESET;

    std::vector<size_t> comparisonCounts = {100, 250, 500, 1000, 2000};
    if (!quickMode) {
        comparisonCounts.push_back(3500);
        comparisonCounts.push_back(5000);
    }

    std::vector<BroadphaseComparisonResult> comparisonResults;

    std::cout << std::left
              << std::setw(10) << "Bodies"
              << std::setw(16) << "Naive O(N^2)"
              << std::setw(16) << "Spatial Grid"
              << std::setw(14) << "Speedup"
              << std::setw(20) << "Naive vs Grid Pairs"
              << std::setw(14) << "Pruned Pairs"
              << "\n";
    std::cout << std::string(90, '-') << "\n";

    for (size_t count : comparisonCounts) {
        WorldPhysics worldNaive({800.0, 600.0});
        worldNaive.setGravity({0.0, 0.0});
        populateWorld(worldNaive, count, 800.0, 600.0, 999);

        WorldPhysics worldGrid({800.0, 600.0});
        worldGrid.setGravity({0.0, 0.0});
        worldGrid.setGridSize(10.0);
        populateWorld(worldGrid, count, 800.0, 600.0, 999);

        // Run profiled steps
        int compSteps = quickMode ? 10 : 25;
        double naiveTotalUs = 0.0;
        double gridTotalUs = 0.0;
        size_t lastNaivePairs = 0;
        size_t lastGridPairs = 0;

        for (int s = 0; s < compSteps; s++) {
            auto profNaive = worldNaive.runPhysicsNaiveProfiled(timeManager);
            naiveTotalUs += profNaive.totalTimeUs;
            lastNaivePairs = profNaive.collisionPairsChecked;

            auto profGrid = worldGrid.runPhysicsProfiled(timeManager);
            gridTotalUs += profGrid.totalTimeUs;
            lastGridPairs = profGrid.collisionPairsChecked;
        }

        double naiveAvgMs = (naiveTotalUs / compSteps) / 1000.0;
        double gridAvgMs = (gridTotalUs / compSteps) / 1000.0;
        double speedup = (gridAvgMs > 0.0) ? (naiveAvgMs / gridAvgMs) : 1.0;
        double prunePct = (lastNaivePairs > 0) ? (100.0 * (1.0 - (double)lastGridPairs / lastNaivePairs)) : 0.0;

        comparisonResults.push_back({
            count, naiveAvgMs, gridAvgMs, speedup,
            lastNaivePairs, lastGridPairs, prunePct
        });

        std::ostringstream speedupStr;
        speedupStr << std::fixed << std::setprecision(1) << speedup << "x";

        std::ostringstream pairsStr;
        pairsStr << lastNaivePairs << " -> " << lastGridPairs;

        std::ostringstream pruneStr;
        pruneStr << std::fixed << std::setprecision(1) << prunePct << "%";

        std::cout << std::left
                  << std::setw(10) << count
                  << std::setw(16) << (std::to_string(naiveAvgMs).substr(0, 6) + " ms")
                  << std::setw(16) << (std::to_string(gridAvgMs).substr(0, 6) + " ms")
                  << COLOR_GREEN << std::setw(14) << speedupStr.str() << COLOR_RESET
                  << std::setw(20) << pairsStr.str()
                  << COLOR_CYAN << std::setw(14) << pruneStr.str() << COLOR_RESET
                  << "\n";
    }
    std::cout << "\n";

    // -------------------------------------------------------------------------
    // BENCHMARK 3: Physics Subsystem Latency Breakdown
    // -------------------------------------------------------------------------
    size_t breakdownBodies = 5000;
    std::cout << COLOR_BOLD << COLOR_YELLOW
              << "--- [3/5] Subsystem Execution Breakdown (" << breakdownBodies << " bodies) ---\n"
              << COLOR_RESET;

    WorldPhysics worldProf({800.0, 600.0});
    worldProf.setGridSize(10.0);
    worldProf.setGravity({0.0, -980.0});
    worldProf.addSpring(Spring(300, 0, 150, 15, 800.0));
    populateWorld(worldProf, breakdownBodies);

    // Warmup
    for (int i = 0; i < 10; i++) worldProf.runPhysics(timeManager);

    const int profSteps = quickMode ? 30 : 100;
    PipelineResult pRes = {0, 0, 0, 0, 0, 0, 0};

    for (int i = 0; i < profSteps; i++) {
        auto prof = worldProf.runPhysicsProfiled(timeManager);
        pRes.forcesUs += prof.forcesTimeUs;
        pRes.integrationUs += prof.integrationTimeUs;
        pRes.gridBuildUs += prof.gridBuildTimeUs;
        pRes.collisionUs += prof.collisionTimeUs;
        pRes.borderUs += prof.borderTimeUs;
        pRes.springUs += prof.springTimeUs;
        pRes.totalUs += prof.totalTimeUs;
    }

    pRes.forcesUs /= profSteps;
    pRes.integrationUs /= profSteps;
    pRes.gridBuildUs /= profSteps;
    pRes.collisionUs /= profSteps;
    pRes.borderUs /= profSteps;
    pRes.springUs /= profSteps;
    pRes.totalUs /= profSteps;

    struct PhaseRow {
        std::string name;
        double us;
    };
    std::vector<PhaseRow> phases = {
        {"Numerical Integration (Kinematics)", pRes.integrationUs},
        {"Spatial Grid Construction & Hash",   pRes.gridBuildUs},
        {"Collision Detection & Impulse Res",  pRes.collisionUs},
        {"Boundary Handling & Settling",       pRes.borderUs},
        {"Global Forces Accumulation",         pRes.forcesUs},
        {"Spring Dynamics",                    pRes.springUs}
    };

    std::cout << std::left
              << std::setw(38) << "Subsystem Phase"
              << std::setw(14) << "Time (us)"
              << std::setw(12) << "Time (ms)"
              << std::setw(12) << "% of Step"
              << "Visual Bar\n";
    std::cout << std::string(90, '-') << "\n";

    for (const auto& ph : phases) {
        double pct = (pRes.totalUs > 0) ? (ph.us / pRes.totalUs * 100.0) : 0.0;
        int barWidth = static_cast<int>(pct / 2.5); // 100% = 40 chars
        std::string bar(barWidth, '#');

        std::cout << std::left
                  << std::setw(38) << ph.name
                  << std::setw(14) << std::fixed << std::setprecision(1) << ph.us
                  << std::setw(12) << std::fixed << std::setprecision(3) << (ph.us / 1000.0)
                  << std::setw(12) << (std::to_string(pct).substr(0, 4) + "%")
                  << COLOR_CYAN << bar << COLOR_RESET
                  << "\n";
    }
    std::cout << std::string(90, '-') << "\n";
    std::cout << std::left
              << std::setw(38) << COLOR_BOLD + std::string("Total Physics Step") + COLOR_RESET
              << std::setw(14) << std::fixed << std::setprecision(1) << pRes.totalUs
              << std::setw(12) << std::fixed << std::setprecision(3) << (pRes.totalUs / 1000.0)
              << std::setw(12) << "100.0%"
              << "\n\n";

    // -------------------------------------------------------------------------
    // BENCHMARK 4: Spatial Grid Cell Size Sweep Optimization
    // -------------------------------------------------------------------------
    std::cout << COLOR_BOLD << COLOR_YELLOW
              << "--- [4/5] Spatial Grid Cell Size Sweep Sensitivity (2,500 bodies) ---\n"
              << COLOR_RESET;

    std::vector<double> cellSizes = {5.0, 8.0, 10.0, 12.0, 15.0, 20.0, 25.0, 30.0, 40.0, 50.0};
    std::vector<GridSweepResult> gridSweepResults;

    std::cout << std::left
              << std::setw(14) << "Cell Size (px)"
              << std::setw(16) << "Step Time (ms)"
              << std::setw(18) << "Pairs Checked"
              << "Relative Efficiency\n";
    std::cout << std::string(68, '-') << "\n";

    double bestTimeUs = 1e9;
    double bestCellSize = 10.0;

    for (double cs : cellSizes) {
        WorldPhysics world({800.0, 600.0});
        world.setGridSize(cs);
        populateWorld(world, 2500, 800.0, 600.0, 777);

        for (int i = 0; i < 5; i++) world.runPhysics(timeManager);

        double totalUs = 0.0;
        size_t pairs = 0;
        int steps = quickMode ? 10 : 30;
        for (int i = 0; i < steps; i++) {
            auto prof = world.runPhysicsProfiled(timeManager);
            totalUs += prof.totalTimeUs;
            pairs = prof.collisionPairsChecked;
        }
        double avgUs = totalUs / steps;
        gridSweepResults.push_back({cs, avgUs, pairs});

        if (avgUs < bestTimeUs) {
            bestTimeUs = avgUs;
            bestCellSize = cs;
        }
    }

    for (const auto& res : gridSweepResults) {
        double relativeToBest = res.stepTimeUs / bestTimeUs;
        std::ostringstream effStr;
        if (res.cellSize == bestCellSize) {
            effStr << COLOR_GREEN << "★ OPTIMAL (" << std::fixed << std::setprecision(2) << relativeToBest << "x)" << COLOR_RESET;
        } else {
            effStr << "+" << std::fixed << std::setprecision(1) << ((relativeToBest - 1.0) * 100.0) << "% slower";
        }

        std::cout << std::left
                  << std::setw(14) << res.cellSize
                  << std::setw(16) << std::fixed << std::setprecision(3) << (res.stepTimeUs / 1000.0)
                  << std::setw(18) << res.pairsChecked
                  << effStr.str()
                  << "\n";
    }
    std::cout << "\n";

    // -------------------------------------------------------------------------
    // BENCHMARK 5: Numerical Stability & Energy Conservation Stress Test
    // -------------------------------------------------------------------------
    std::cout << COLOR_BOLD << COLOR_YELLOW
              << "--- [5/5] Elastic Sanity Check: e = 1, no gravity, energy should stay constant ---\n"
              << COLOR_RESET;

    double oldRest = getRestitution();
    setRestitution(1.0); // Perfectly elastic closed box

    WorldPhysics worldStab({800.0, 600.0});
    worldStab.setGravity({0.0, 0.0});
    worldStab.setGridSize(10.0);
    populateWorld(worldStab, 500, 800.0, 600.0, 1234);

    double initialEnergy = worldStab.calculateTotalEnergy();
    std::vector<StabilityResult> stabilityResults;

    int simSteps = quickMode ? 300 : 1000;
    int checkInterval = simSteps / 4;

    std::cout << "Initial total kinetic energy: " << std::fixed << std::setprecision(2) << initialEnergy << " J\n";
    std::cout << std::left
              << std::setw(10) << "Step"
              << std::setw(18) << "Total Energy (J)"
              << std::setw(18) << "Energy Drift (%)"
              << "System Status\n";
    std::cout << std::string(60, '-') << "\n";

    for (int s = 1; s <= simSteps; s++) {
        worldStab.runPhysics(timeManager);

        if (s % checkInterval == 0 || s == simSteps) {
            double currentEnergy = worldStab.calculateTotalEnergy();
            double driftPct = 100.0 * (currentEnergy - initialEnergy) / initialEnergy;
            stabilityResults.push_back({s, currentEnergy, driftPct});

            bool isStable = !std::isnan(currentEnergy) && !std::isinf(currentEnergy);

            std::cout << std::left
                      << std::setw(10) << s
                      << std::setw(18) << currentEnergy
                      << std::setw(18) << (std::to_string(driftPct).substr(0, 5) + "%")
                      << (isStable ? COLOR_GREEN + std::string("OK") + COLOR_RESET
                                   : COLOR_RED   + std::string("NaN/Inf") + COLOR_RESET)
                      << "\n";
        }
    }
    setRestitution(oldRest);
    std::cout << "\n";

    // -------------------------------------------------------------------------
    // WRITE OUTPUT FILES: BENCHMARK_RESULTS.md and BENCHMARK_RESULTS.csv
    // -------------------------------------------------------------------------
    std::ofstream mdFile("BENCHMARK_RESULTS.md");
    if (mdFile.is_open()) {
        mdFile << "# 2D Physics Engine - Performance Benchmark Report\n\n";
        mdFile << "Times the physics step only (no rendering). Fixed timestep 1/60 s. "
               << "Bodies: radius ~0.9 to 2.8 in an 800 x 600 box, random velocities, fixed seed.\n\n";

        mdFile << "## 1. Step Time vs Body Count (gravity on, grid cell size 10)\n\n";
        mdFile << "| Body Count | Mean Step (ms) | Min / Max (ms) | P95 (ms) | Steps / sec | Bodies / sec | Under 16.67 ms? |\n";
        mdFile << "|:---|:---|:---|:---|:---|:---|:---|\n";
        for (const auto& r : scalabilityResults) {
            bool isRt = r.stats.meanUs <= 16666.7;
            mdFile << "| " << r.bodyCount
                   << " | " << std::fixed << std::setprecision(3) << (r.stats.meanUs / 1000.0)
                   << " | " << std::fixed << std::setprecision(2) << (r.stats.minUs / 1000.0) << " / " << (r.stats.maxUs / 1000.0)
                   << " | " << std::fixed << std::setprecision(3) << (r.stats.p95Us / 1000.0)
                   << " | " << static_cast<int>(r.stats.effectiveFps)
                   << " | " << static_cast<long long>(r.stats.throughputBodiesPerSec)
                   << " | " << (isRt ? "yes" : "no")
                   << " |\n";
        }
        mdFile << "\n";

        mdFile << "## 2. Spatial Grid vs Brute-Force O(N²) (no gravity)\n\n";
        mdFile << "| Body Count | Brute Force (ms) | Grid (ms) | Speedup | Brute-Force Pairs | Grid Pairs | Pairs Skipped |\n";
        mdFile << "|:---|:---|:---|:---|:---|:---|:---|\n";
        for (const auto& r : comparisonResults) {
            mdFile << "| " << r.bodyCount
                   << " | " << std::fixed << std::setprecision(3) << r.naiveMs
                   << " | " << std::fixed << std::setprecision(3) << r.gridMs
                   << " | " << std::fixed << std::setprecision(1) << r.speedup << "x"
                   << " | " << r.naivePairs
                   << " | " << r.gridPairs
                   << " | " << std::fixed << std::setprecision(2) << r.pairReductionPct << "%"
                   << " |\n";
        }
        mdFile << "\n";

        mdFile << "## 3. Where the Time Goes (5,000 bodies, gravity on)\n\n";
        mdFile << "| Phase | Time (µs) | Time (ms) | % of Step |\n";
        mdFile << "|:---|:---|:---|:---|\n";
        for (const auto& ph : phases) {
            double pct = (pRes.totalUs > 0) ? (ph.us / pRes.totalUs * 100.0) : 0.0;
            mdFile << "| " << ph.name
                   << " | " << std::fixed << std::setprecision(1) << ph.us
                   << " | " << std::fixed << std::setprecision(3) << (ph.us / 1000.0)
                   << " | " << std::fixed << std::setprecision(1) << pct << "% |\n";
        }
        mdFile << "| **Total** | **" << std::fixed << std::setprecision(1) << pRes.totalUs
               << "** | **" << std::fixed << std::setprecision(3) << (pRes.totalUs / 1000.0)
               << "** | **100.0%** |\n\n";

        mdFile << "## 4. Grid Cell Size Sweep (2,500 bodies, no gravity)\n\n";
        mdFile << "Default cell size used everywhere else is 10. Lower is better.\n\n";
        mdFile << "| Cell Size | Step Time (ms) | Pairs Checked |\n";
        mdFile << "|:---|:---|:---|\n";
        for (const auto& res : gridSweepResults) {
            mdFile << "| " << std::fixed << std::setprecision(0) << res.cellSize
                   << " | " << std::fixed << std::setprecision(3) << (res.stepTimeUs / 1000.0)
                   << " | " << res.pairsChecked << " |\n";
        }
        mdFile << "\n";

        mdFile << "## 5. Elastic Sanity Check\n\n";
        mdFile << "Restitution e = 1 and no gravity, so total kinetic energy should not change. "
               << "This is a regression check that collisions and walls do not add or remove energy, "
               << "not a measure of integrator accuracy.\n\n";
        mdFile << "| Step | Total Energy | Drift (%) |\n";
        mdFile << "|:---|:---|:---|\n";
        for (const auto& r : stabilityResults) {
            mdFile << "| " << r.step
                   << " | " << std::fixed << std::setprecision(2) << r.totalEnergy
                   << " | " << std::setprecision(6) << r.energyDriftPct << " |\n";
        }
        mdFile << "\n";
        mdFile.close();
        std::cout << COLOR_GREEN << "-> Detailed Markdown report written to: BENCHMARK_RESULTS.md\n" << COLOR_RESET;
    }

    std::ofstream csvFile("BENCHMARK_RESULTS.csv");
    if (csvFile.is_open()) {
        csvFile << "body_count,mean_step_ms,min_step_ms,max_step_ms,p95_step_ms,effective_fps,throughput_bodies_sec\n";
        for (const auto& r : scalabilityResults) {
            csvFile << r.bodyCount << ","
                    << (r.stats.meanUs / 1000.0) << ","
                    << (r.stats.minUs / 1000.0) << ","
                    << (r.stats.maxUs / 1000.0) << ","
                    << (r.stats.p95Us / 1000.0) << ","
                    << r.stats.effectiveFps << ","
                    << r.stats.throughputBodiesPerSec << "\n";
        }
        csvFile.close();
        std::cout << COLOR_GREEN << "-> Scalability raw metrics written to:   BENCHMARK_RESULTS.csv\n" << COLOR_RESET;
    }

    std::cout << "\n";
    return 0;
}
