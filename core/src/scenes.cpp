#include "physics/scenes.hpp"

#include "physics/scene.hpp"

#include <cmath>
#include <vector>

namespace phys::scenes {

World pile(std::size_t n, std::uint64_t seed) {
    WorldConfig c;
    World w(c);
    SplitMix64 rng(seed);
    for (std::size_t i = 0; i < n; ++i) {
        const double m = rng.uniform(10.0, 100.0);
        const double r = 0.5 * std::sqrt(m / 3.14159265358979323846);
        w.addBody(makeBody(r, m, {rng.uniform(r, c.width - r), rng.uniform(c.height * 0.3, c.height - r)}));
    }
    return w;
}

World newtonsCradle(int balls, double radius, double gap, double liftAngle) {
    WorldConfig c;
    c.restitution = 1.0;
    c.friction = 0.0;
    World w(c);
    const double L = 200.0, top = 500.0;
    const double x0 = 400.0 - 0.5 * (balls - 1) * (2.0 * radius + gap);
    for (int i = 0; i < balls; ++i) {
        const double ax = x0 + i * (2.0 * radius + gap);
        const int anchor = w.addBody(makeBody(2.0, 0.0, {ax, top}));
        Vec2 p{ax, top - L};
        if (i == 0) p = {ax - L * std::sin(liftAngle), top - L * std::cos(liftAngle)};
        const int ball = w.addBody(makeBody(radius, 1.0, p));
        w.addDistanceConstraint(anchor, ball);
    }
    return w;
}

World spinningBall(double radius, double w0, double mu) {
    WorldConfig c;
    c.friction = mu;
    c.restitution = 0.0;
    World w(c);
    w.addBody(makeBody(radius, 1.0, {200.0, radius}, {0.0, 0.0}, w0));
    return w;
}

World springChain(int links, double stiffness, double damping) {
    WorldConfig c;
    World w(c);
    const double spacing = 20.0;
    int prev = w.addBody(makeBody(4.0, 0.0, {400.0, 560.0}));
    for (int i = 1; i <= links; ++i) {
        // Start horizontally so the chain swings down.
        const int b = w.addBody(makeBody(5.0, 1.0, {400.0 + i * spacing, 560.0}));
        w.addSpring({prev, b, spacing, stiffness, damping});
        prev = b;
    }
    return w;
}

World softBlob(int rings, double spacing, double stiffness, double damping, double height) {
    WorldConfig c;
    c.substeps = 4;
    World w(c);
    std::vector<Vec2> pts;
    for (int q = -rings; q <= rings; ++q)
        for (int r = -rings; r <= rings; ++r) {
            const int s = -q - r;
            if (std::abs(s) > rings) continue;
            pts.push_back({400.0 + spacing * (q + 0.5 * r), height + spacing * (std::sqrt(3.0) / 2.0) * r});
        }
    const int first = static_cast<int>(w.bodies().size());
    for (const Vec2& p : pts) w.addBody(makeBody(0.48 * spacing, 1.0, p));
    for (std::size_t i = 0; i < pts.size(); ++i)
        for (std::size_t j = i + 1; j < pts.size(); ++j) {
            const double d = (pts[j] - pts[i]).length();
            if (d < 1.01 * spacing)
                w.addSpring({first + static_cast<int>(i), first + static_cast<int>(j), d, stiffness, damping});
        }
    return w;
}

World stress(std::size_t n, std::uint64_t seed) {
    const SceneSpec s = makeConstantDensityScene(n, seed);
    WorldConfig c;
    c.width = s.width;
    c.height = s.height;
    c.gravity = {0.0, 0.0};
    c.restitution = 1.0;
    World w(c);
    for (const auto& b : s.bodies) w.addBody(makeBody(b.radius, b.mass, {b.x, b.y}, {b.vx, b.vy}));
    return w;
}

}  // namespace phys::scenes
