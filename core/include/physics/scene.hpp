#pragma once
// Deterministic, platform-independent scene generation.
//
// std::rand and std::uniform_real_distribution are implementation-defined, so
// the same seed gave different scenes on macOS (libc++) and Linux (libstdc++).
// SplitMix64 plus a 53-bit mantissa conversion uses only integer ops and
// correctly rounded IEEE arithmetic (+, *, /, sqrt), so a seed produces
// bit-identical scenes everywhere.
#include <cstddef>
#include <cstdint>
#include <vector>

namespace phys {

class SplitMix64 {
public:
    explicit SplitMix64(std::uint64_t seed) : state_(seed) {}
    std::uint64_t next() {
        std::uint64_t z = (state_ += 0x9E3779B97F4A7C15ull);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }
    // Uniform in [0, 1): top 53 bits scaled by $2^{-53}$.
    double uniform() { return static_cast<double>(next() >> 11) * 0x1.0p-53; }
    double uniform(double lo, double hi) { return lo + (hi - lo) * uniform(); }

private:
    std::uint64_t state_;
};

struct BodySpec {
    double x, y;    // position (px)
    double vx, vy;  // velocity (px/s)
    double radius;  // px
    double mass;    // kg
};

struct SceneSpec {
    double width = 0.0, height = 0.0;  // arena is [0, width] x [0, height]
    std::vector<BodySpec> bodies;
};

// Arena area per body for the constant-density scenes: the demo's default
// of 5,000 bodies in 800 x 600 px gives $480000 / 5000 = 96\ \mathrm{px}^2$.
inline constexpr double kDefaultAreaPerBody = 96.0;

// N bodies in a 4:3 arena whose area grows linearly with N, so the number
// density stays $1/96\ \mathrm{px}^{-2}$ for every N (N = 5000 gives 800 x 600).
// Mass $m \sim U[10, 100]$, radius $r = \tfrac12\sqrt{m/\pi}$ (0.89 to 2.82 px),
// position uniform inside the walls, velocity $\sim U[-50, 50]$ px/s per axis.
SceneSpec makeConstantDensityScene(std::size_t n, std::uint64_t seed,
                                   double areaPerBody = kDefaultAreaPerBody);

// Same body distribution in a fixed arena (default 800 x 600, the demo window).
SceneSpec makeFixedArenaScene(std::size_t n, std::uint64_t seed,
                              double width = 800.0, double height = 600.0);

}  // namespace phys
