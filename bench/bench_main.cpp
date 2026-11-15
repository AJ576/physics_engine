// Headless benchmark harness for the physics step.
//
// Method (same for every engine version):
//  * Scene: seeded generator (phys::makeConstantDensityScene, 96 px^2 per body).
//  * For each run: build the scene from the same seed, run `warmup` untimed
//    steps, then time `steps` individual steps with std::chrono::steady_clock.
//  * Per run: mean, p50, p99 and max step time. Across runs: the median of
//    each statistic and the sample standard deviation of the per-run means.
//  * Every run uses the same seed, so runs do identical work: the spread
//    across runs is measurement noise, not workload variation.
//
// Usage: physics_bench [--scene gas|pile] [--n 500,1000,...] [--runs R]
//                      [--warmup W] [--steps S] [--brute] [--brute-steps S]
//                      [--search] [--csv FILE] [--seed X]
#include "adapter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <numeric>
#include <string>
#include <vector>

#ifndef PHYS_BENCH_BUILD_INFO
#define PHYS_BENCH_BUILD_INFO "unknown"
#endif

namespace {

using Clock = std::chrono::steady_clock;

struct Options {
    std::string scene = "gas";
    std::vector<std::size_t> ns = {500, 1000, 2000, 5000, 10000, 20000};
    int runs = 7;
    int warmup = 120;
    int steps = 300;
    bool brute = false;
    int bruteSteps = 30;
    bool search = false;
    std::string csv;
    std::uint64_t seed = 42;
};

struct RunStats {
    double mean, p50, p99, max;
};

struct Summary {  // across runs
    double mean, p50, p99, max;  // medians of the per-run statistics (ms)
    double meanStd;              // sample stddev of per-run means (ms)
    double meanMin, meanMax;     // range of per-run means (ms)
};

double percentile(std::vector<double> v, double q) {  // nearest-rank
    std::sort(v.begin(), v.end());
    std::size_t k = static_cast<std::size_t>(std::ceil(q * static_cast<double>(v.size())));
    if (k == 0) k = 1;
    return v[k - 1];
}

double median(std::vector<double> v) {
    std::sort(v.begin(), v.end());
    const std::size_t n = v.size();
    return n % 2 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

RunStats statsOf(const std::vector<double>& ms) {
    RunStats s{};
    s.mean = std::accumulate(ms.begin(), ms.end(), 0.0) / static_cast<double>(ms.size());
    s.p50 = percentile(ms, 0.50);
    s.p99 = percentile(ms, 0.99);
    s.max = *std::max_element(ms.begin(), ms.end());
    return s;
}

Summary summarize(const std::vector<RunStats>& runs) {
    std::vector<double> mean, p50, p99, mx;
    for (const auto& r : runs) {
        mean.push_back(r.mean); p50.push_back(r.p50); p99.push_back(r.p99); mx.push_back(r.max);
    }
    Summary s{};
    s.mean = median(mean); s.p50 = median(p50); s.p99 = median(p99); s.max = median(mx);
    const double mu = std::accumulate(mean.begin(), mean.end(), 0.0) / static_cast<double>(mean.size());
    double ss = 0.0;
    for (double m : mean) ss += (m - mu) * (m - mu);
    s.meanStd = mean.size() > 1 ? std::sqrt(ss / static_cast<double>(mean.size() - 1)) : 0.0;
    s.meanMin = *std::min_element(mean.begin(), mean.end());
    s.meanMax = *std::max_element(mean.begin(), mean.end());
    return s;
}

bench::SimConfig configFor(const Options& o) {
    bench::SimConfig c;
    if (o.scene == "pile") { c.gravityY = -980.0; c.restitution = 0.7; }
    else { c.gravityY = 0.0; c.restitution = 1.0; }  // "gas": steady workload
    return c;
}

template <class StepFn>
RunStats timedRun(std::size_t n, const Options& o, int steps, StepFn stepFn) {
    bench::Sim sim(phys::makeConstantDensityScene(n, o.seed), configFor(o));
    for (int i = 0; i < o.warmup; ++i) sim.step();
    std::vector<double> ms(static_cast<std::size_t>(steps));
    for (int i = 0; i < steps; ++i) {
        const auto t0 = Clock::now();
        stepFn(sim);
        const auto t1 = Clock::now();
        ms[static_cast<std::size_t>(i)] = std::chrono::duration<double, std::milli>(t1 - t0).count();
    }
    return statsOf(ms);
}

Summary measure(std::size_t n, const Options& o) {
    std::vector<RunStats> runs;
    for (int r = 0; r < o.runs; ++r) runs.push_back(timedRun(n, o, o.steps, [](bench::Sim& s) { s.step(); }));
    return summarize(runs);
}

Summary measureBrute(std::size_t n, const Options& o) {
    std::vector<RunStats> runs;
    for (int r = 0; r < o.runs; ++r)
        runs.push_back(timedRun(n, o, o.bruteSteps, [](bench::Sim& s) { s.stepBruteForce(); }));
    return summarize(runs);
}

// Average candidate pairs and contacts per step, over the same steps as the
// timed runs (separate untimed pass).
std::pair<double, double> countPairs(std::size_t n, const Options& o) {
    bench::Sim sim(phys::makeConstantDensityScene(n, o.seed), configFor(o));
    for (int i = 0; i < o.warmup; ++i) sim.step();
    double pairs = 0, contacts = 0;
    for (int i = 0; i < o.steps; ++i) {
        auto c = sim.stepCounted();
        pairs += static_cast<double>(c.candidatePairs);
        contacts += static_cast<double>(c.contacts);
    }
    return {pairs / o.steps, contacts / o.steps};
}

std::vector<std::size_t> parseList(const char* s) {
    std::vector<std::size_t> v;
    for (const char* p = s; *p;) {
        v.push_back(std::strtoull(p, const_cast<char**>(&p), 10));
        if (*p == ',') ++p;
    }
    return v;
}

}  // namespace

int main(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() -> const char* { return i + 1 < argc ? argv[++i] : ""; };
        if (a == "--scene") o.scene = next();
        else if (a == "--n") o.ns = parseList(next());
        else if (a == "--runs") o.runs = std::atoi(next());
        else if (a == "--warmup") o.warmup = std::atoi(next());
        else if (a == "--steps") o.steps = std::atoi(next());
        else if (a == "--brute") o.brute = true;
        else if (a == "--brute-steps") o.bruteSteps = std::atoi(next());
        else if (a == "--search") o.search = true;
        else if (a == "--csv") o.csv = next();
        else if (a == "--seed") o.seed = std::strtoull(next(), nullptr, 10);
        else { std::fprintf(stderr, "unknown option %s\n", a.c_str()); return 2; }
    }
    if (o.scene != "gas" && o.scene != "pile") { std::fprintf(stderr, "scene must be gas or pile\n"); return 2; }

    const auto cfg = configFor(o);
    std::printf("engine: %s\nbuild: %s\n", bench::Sim::engineName(), PHYS_BENCH_BUILD_INFO);
    std::printf("scene: %s (constant density 96 px^2/body, gravity %.0f px/s^2, e=%.2f, seed %llu)\n",
                o.scene.c_str(), cfg.gravityY, cfg.restitution, static_cast<unsigned long long>(o.seed));
    std::printf("runs=%d warmup=%d steps/run=%d dt=1/60 s; times in ms, median over runs; sd = stddev of run means\n\n",
                o.runs, o.warmup, o.steps);

    std::FILE* csv = o.csv.empty() ? nullptr : std::fopen(o.csv.c_str(), "w");
    if (csv) std::fprintf(csv, "scene,n,mean_ms,p50_ms,p99_ms,max_ms,mean_sd_ms,mean_min_ms,mean_max_ms,cand_pairs,contacts,brute_pairs,brute_mean_ms,brute_sd_ms\n");

    std::printf("%7s %9s %9s %9s %9s %8s %7s %13s %11s %14s", "N", "mean", "p50", "p99", "max", "sd", "sd/mean",
                "cand.pairs", "contacts", "brute pairs");
    if (o.brute) std::printf(" %11s %8s", "brute mean", "speedup");
    std::printf("\n");

    for (std::size_t n : o.ns) {
        const Summary s = measure(n, o);
        const auto [pairs, contacts] = countPairs(n, o);
        const double brutePairs = 0.5 * static_cast<double>(n) * static_cast<double>(n - 1);
        std::printf("%7zu %9.3f %9.3f %9.3f %9.3f %8.4f %6.2f%% %13.0f %11.0f %14.0f", n, s.mean, s.p50, s.p99, s.max,
                    s.meanStd, 100.0 * s.meanStd / s.mean, pairs, contacts, brutePairs);
        Summary b{};
        if (o.brute) {
            b = measureBrute(n, o);
            std::printf(" %11.3f %7.1fx", b.mean, b.mean / s.mean);
        }
        std::printf("\n");
        std::fflush(stdout);
        if (csv)
            std::fprintf(csv, "%s,%zu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.1f,%.1f,%.0f,%.6f,%.6f\n", o.scene.c_str(), n,
                         s.mean, s.p50, s.p99, s.max, s.meanStd, s.meanMin, s.meanMax, pairs, contacts, brutePairs,
                         o.brute ? b.mean : 0.0, o.brute ? b.meanStd : 0.0);
    }

    if (o.search) {
        // Largest N whose median p99 step time is <= 16.667 ms (60 Hz), and
        // separately whose median mean step time is. Doubling, then bisection
        // to 1% of N.
        const double budget = 1000.0 / 60.0;
        for (int crit = 0; crit < 2; ++crit) {
            auto ok = [&](std::size_t n) {
                Summary s = measure(n, o);
                const double v = crit == 0 ? s.p99 : s.mean;
                std::printf("  search[%s] N=%zu -> %.3f ms\n", crit == 0 ? "p99" : "mean", n, v);
                std::fflush(stdout);
                return v <= budget;
            };
            std::size_t lo = 1000, hi = 2000;
            while (ok(hi)) { lo = hi; hi *= 2; }
            while (hi - lo > lo / 100) {
                std::size_t mid = lo + (hi - lo) / 2;
                (ok(mid) ? lo : hi) = mid;
            }
            std::printf("largest N with %s step <= 16.667 ms: %zu (next tested N=%zu failed)\n",
                        crit == 0 ? "p99" : "mean", lo, hi);
            if (csv) std::fprintf(csv, "# largest_n_%s_under_16.667ms,%zu,%zu\n", crit == 0 ? "p99" : "mean", lo, hi);
        }
    }
    if (csv) std::fclose(csv);
    return 0;
}
