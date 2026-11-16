// Integrator study (standalone: does not use phys::World except in part 2).
//
// Part 1: $\ddot{x} = -\omega^2 x$, $x_0 = 1$, $v_0 = 0$, $h = 1/60$ s, N steps, for
// explicit Euler, semi-implicit Euler, velocity Verlet and the old engine's
// constant-acceleration update. Energy $E = \tfrac12 v^2 + \tfrac12\omega^2 x^2$.
//   * explicit Euler: checks the claim $E_{n+1} = (1 + \omega^2 h^2)\,E_n$ every step
//   * semi-implicit Euler: checks that $\tilde H = v^2 + \omega^2 x^2 - h\omega^2 x v$ is
//     conserved and that $E/E_0$ stays in $[1/(1+\omega h/2),\ 1/(1-\omega h/2)]$
// Part 2: the same oscillator built in phys::World (static anchor + Hooke
// spring), for each engine integrator, and an undamped 12-link spring chain.
//
// Usage: physics_integrators [--steps N]
#include "physics/world.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>

namespace {

enum Method { ExplicitEuler, SemiImplicit, VelocityVerlet, ConstantAccel };
const char* kNames[] = {"explicit Euler", "semi-implicit Euler", "velocity Verlet", "old engine (const. accel.)"};

struct Result {
    double finalRatio = 1.0;  // E_N / E_0 (inf if it overflowed)
    double minRatio = 1.0, maxRatio = 1.0;
    long overflowStep = -1;   // first step with E = inf, or -1
    double maxFactorDev = 0.0;   // explicit Euler: max |(E_{n+1}/E_n) / (1 + w^2 h^2) - 1|
    double meanLogFactor = 0.0;  // mean of log(E_{n+1}/E_n) over finite steps
    double maxModifiedDev = 0.0; // semi-implicit: max |H~_n / H~_0 - 1|
};

Result run(Method m, double w, double h, long steps) {
    double x = 1.0, v = 0.0;
    const double w2 = w * w;
    auto energy = [&] { return 0.5 * v * v + 0.5 * w2 * x * x; };
    auto modified = [&] { return v * v + w2 * x * x - h * w2 * x * v; };
    const double e0 = energy(), h0 = modified();
    Result r;
    double prev = e0, sumLog = 0.0;
    long finite = 0;
    for (long n = 1; n <= steps; ++n) {
        switch (m) {
            case ExplicitEuler: {
                const double a = -w2 * x;
                x += v * h;
                v += a * h;
                break;
            }
            case SemiImplicit:
                v += -w2 * x * h;
                x += v * h;
                break;
            case VelocityVerlet: {
                const double a0 = -w2 * x;
                x += v * h + 0.5 * a0 * h * h;
                v += 0.5 * (a0 + -w2 * x) * h;
                break;
            }
            case ConstantAccel: {
                const double a = -w2 * x;
                x += v * h + 0.5 * a * h * h;
                v += a * h;
                break;
            }
        }
        const double e = energy();
        if (!std::isfinite(e)) {
            r.overflowStep = n;
            r.finalRatio = std::numeric_limits<double>::infinity();
            r.maxRatio = r.finalRatio;
            break;
        }
        const double ratio = e / e0;
        r.minRatio = std::min(r.minRatio, ratio);
        r.maxRatio = std::max(r.maxRatio, ratio);
        if (m == ExplicitEuler) r.maxFactorDev = std::max(r.maxFactorDev, std::abs((e / prev) / (1.0 + w2 * h * h) - 1.0));
        if (m == SemiImplicit) r.maxModifiedDev = std::max(r.maxModifiedDev, std::abs(modified() / h0 - 1.0));
        sumLog += std::log(e / prev);
        ++finite;
        prev = e;
        r.finalRatio = ratio;
    }
    r.meanLogFactor = finite ? sumLog / static_cast<double>(finite) : 0.0;
    return r;
}

// Part 2: the oscillator inside phys::World.
struct EngineResult {
    double finalRatio, minRatio, maxRatio;
    long overflowStep;
    std::size_t contacts;  // body-body contacts over the run (0 = springs only)
    double meanLogFactor;
    double ratioAt100;     // E_100 / E_0
};

EngineResult track(phys::World& world, long steps) {
    const double e0 = world.totalEnergy();
    EngineResult r{1.0, 1.0, 1.0, -1, 0, 0.0, 1.0};
    double prev = e0, sumLog = 0.0;
    long finite = 0;
    for (long n = 1; n <= steps; ++n) {
        world.step();
        r.contacts += world.lastStepStats().contacts;
        const double e = world.totalEnergy();
        if (!std::isfinite(e)) {
            r.finalRatio = r.maxRatio = std::numeric_limits<double>::infinity();
            r.overflowStep = n;
            break;
        }
        const double ratio = e / e0;
        r.minRatio = std::min(r.minRatio, ratio);
        r.maxRatio = std::max(r.maxRatio, ratio);
        r.finalRatio = ratio;
        if (n == 100) r.ratioAt100 = ratio;
        sumLog += std::log(e / prev);
        prev = e;
        ++finite;
    }
    r.meanLogFactor = finite ? sumLog / static_cast<double>(finite) : 0.0;
    return r;
}

EngineResult engineOscillator(phys::Integrator integ, double w, long steps) {
    phys::WorldConfig c;
    c.gravity = {0, 0};
    c.walls = false;
    c.integrator = integ;
    phys::World world(c);
    // Rest length 1e9 px so a growing oscillation can never make the two
    // bodies collide (which would cap the energy and hide the growth).
    const double L0 = 1.0e9;
    world.addBody(phys::makeBody(0.5, 0.0, {0.0, 0.0}));
    world.addBody(phys::makeBody(0.5, 1.0, {L0 + 1.0, 0.0}));  // stretched by 1 px
    world.addSpring({0, 1, L0, w * w, 0.0});
    return track(world, steps);
}

EngineResult engineChain(phys::Integrator integ, long steps) {
    // 12 discs in a line joined by undamped springs (k = 400, rest 20 px),
    // gravity and walls off, first link kicked sideways. Radius 0.5 px,
    // e = 1 and no friction so that links rarely touch; the contact count is
    // reported to show whether collisions took part.
    phys::WorldConfig c;
    c.gravity = {0, 0};
    c.walls = false;
    c.restitution = 1.0;
    c.friction = 0.0;
    c.integrator = integ;
    phys::World world(c);
    for (int i = 0; i < 12; ++i)
        world.addBody(phys::makeBody(0.5, 1.0, {400.0 + 20.0 * i, 300.0}, i == 0 ? phys::Vec2{0, 60} : phys::Vec2{}));
    for (int i = 0; i + 1 < 12; ++i) world.addSpring({i, i + 1, 20.0, 400.0, 0.0});
    return track(world, steps);
}

std::string fmt(double v) {
    char b[64];
    if (!std::isfinite(v)) return "overflow";
    std::snprintf(b, sizeof b, "%.6g", v);
    return b;
}

}  // namespace

int main(int argc, char** argv) {
    long steps = 100000;
    for (int i = 1; i < argc; ++i)
        if (!std::strcmp(argv[i], "--steps") && i + 1 < argc) steps = std::atol(argv[++i]);
    const double h = 1.0 / 60.0;
    const double omegas[] = {1.0, 5.0, 10.0, 30.0, 60.0};

    std::printf("## Part 1 (standalone): x'' = -w^2 x, x0 = 1, v0 = 0, h = 1/60 s, %ld steps (%.0f s)\n\n", steps, steps * h);
    std::printf("| w (rad/s) | w h | method | E_N/E_0 - 1 | min E/E_0 | max E/E_0 | overflow step |\n");
    std::printf("|---:|---:|---|---:|---:|---:|---:|\n");
    for (double w : omegas)
        for (int m = 0; m < 4; ++m) {
            const Result r = run(static_cast<Method>(m), w, h, steps);
            std::printf("| %g | %.4f | %s | %s | %s | %s | %s |\n", w, w * h, kNames[m],
                        std::isinf(r.finalRatio) ? "overflow" : fmt(r.finalRatio - 1.0).c_str(), fmt(r.minRatio).c_str(),
                        fmt(r.maxRatio).c_str(), r.overflowStep < 0 ? "-" : std::to_string(r.overflowStep).c_str());
        }

    std::printf("\n### Explicit Euler: measured per-step energy factor vs $1 + \\omega^2 h^2$\n\n");
    std::printf("| w | predicted factor | measured mean factor exp(mean log) | max per-step relative deviation | steps until E = inf |\n|---:|---:|---:|---:|---:|\n");
    for (double w : omegas) {
        const Result r = run(ExplicitEuler, w, h, steps);
        std::printf("| %g | %.12f | %.12f | %.2e | %s |\n", w, 1.0 + w * w * h * h, std::exp(r.meanLogFactor),
                    r.maxFactorDev, r.overflowStep < 0 ? "none" : std::to_string(r.overflowStep).c_str());
    }

    std::printf("\n### Semi-implicit Euler: bounds and the conserved modified energy\n\n");
    std::printf("| w | predicted E/E_0 range [1/(1+wh/2), 1/(1-wh/2)] | measured range | max abs(H~_n/H~_0 - 1) |\n|---:|---|---|---:|\n");
    for (double w : omegas) {
        const Result r = run(SemiImplicit, w, h, steps);
        std::printf("| %g | [%.6f, %.6f] | [%.6f, %.6f] | %.2e |\n", w, 1.0 / (1.0 + 0.5 * w * h), 1.0 / (1.0 - 0.5 * w * h),
                    r.minRatio, r.maxRatio, r.maxModifiedDev);
    }

    std::printf("\n### Old engine integrator: mean per-step energy factor vs det = 1 + w^2 h^2 / 2\n\n");
    std::printf("| w | det of step matrix | measured mean factor | steps until E = inf |\n|---:|---:|---:|---:|\n");
    for (double w : omegas) {
        const Result r = run(ConstantAccel, w, h, steps);
        std::printf("| %g | %.12f | %.12f | %s |\n", w, 1.0 + 0.5 * w * w * h * h, std::exp(r.meanLogFactor),
                    r.overflowStep < 0 ? "none" : std::to_string(r.overflowStep).c_str());
    }

    std::printf("\n## Part 2 (engine): phys::World, static anchor + Hooke spring (k = w^2, m = 1), %ld steps\n\n", steps);
    std::printf("Explicit Euler is predicted to give exactly $(1 + \\omega^2 h^2)^{100}$ after 100 steps; the other columns cover the whole run.\n\n");
    std::printf("| w | engine integrator | E_100/E_0 | predicted (explicit) | E_N/E_0 | min E/E_0 | max E/E_0 | overflow step | contacts |\n|---:|---|---:|---:|---:|---:|---:|---:|---:|\n");
    const phys::Integrator integs[] = {phys::Integrator::SemiImplicitEuler, phys::Integrator::ExplicitEuler,
                                       phys::Integrator::ConstantAccel};
    const char* integNames[] = {"SemiImplicitEuler (default)", "ExplicitEuler", "ConstantAccel (old engine)"};
    for (double w : {1.0, 10.0, 30.0})
        for (int k = 0; k < 3; ++k) {
            const EngineResult r = engineOscillator(integs[k], w, steps);
            std::printf("| %g | %s | %.9g | %.9g | %s | %s | %s | %s | %zu |\n", w, integNames[k], r.ratioAt100,
                        std::pow(1.0 + w * w * h * h, 100.0), fmt(r.finalRatio).c_str(), fmt(r.minRatio).c_str(),
                        fmt(r.maxRatio).c_str(), r.overflowStep < 0 ? "-" : std::to_string(r.overflowStep).c_str(),
                        r.contacts);
        }
    std::printf("\n12-link undamped spring chain (k = 400, m = 1, gravity and walls off), %ld steps\n\n", steps);
    std::printf("| engine integrator | E_N/E_0 | min E/E_0 | max E/E_0 | overflow step | contacts |\n|---|---:|---:|---:|---:|---:|\n");
    for (int k = 0; k < 3; ++k) {
        const EngineResult r = engineChain(integs[k], steps);
        std::printf("| %s | %s | %s | %s | %s | %zu |\n", integNames[k], fmt(r.finalRatio).c_str(), fmt(r.minRatio).c_str(),
                    fmt(r.maxRatio).c_str(), r.overflowStep < 0 ? "-" : std::to_string(r.overflowStep).c_str(), r.contacts);
    }
    return 0;
}
