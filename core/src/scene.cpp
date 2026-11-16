#include "physics/scene.hpp"

#include <cmath>

namespace phys {
namespace {
constexpr double kPi = 3.14159265358979323846;

void fill(SceneSpec& s, std::size_t n, std::uint64_t seed) {
    SplitMix64 rng(seed);
    s.bodies.clear();
    s.bodies.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        BodySpec b{};
        b.mass = rng.uniform(10.0, 100.0);
        b.radius = 0.5 * std::sqrt(b.mass / kPi);
        b.x = rng.uniform(b.radius, s.width - b.radius);
        b.y = rng.uniform(b.radius, s.height - b.radius);
        b.vx = rng.uniform(-50.0, 50.0);
        b.vy = rng.uniform(-50.0, 50.0);
        s.bodies.push_back(b);
    }
}
}  // namespace

SceneSpec makeConstantDensityScene(std::size_t n, std::uint64_t seed, double areaPerBody) {
    SceneSpec s;
    const double area = static_cast<double>(n) * areaPerBody;
    s.width = std::sqrt(area * 4.0 / 3.0);
    s.height = 0.75 * s.width;
    fill(s, n, seed);
    return s;
}

SceneSpec makeFixedArenaScene(std::size_t n, std::uint64_t seed, double width, double height) {
    SceneSpec s;
    s.width = width;
    s.height = height;
    fill(s, n, seed);
    return s;
}

}  // namespace phys
