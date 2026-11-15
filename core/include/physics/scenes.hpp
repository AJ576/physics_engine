#pragma once
// Ready-made worlds used by the demo, the tests and the benchmarks.
// All in pixels / seconds; arena 800 x 600 unless stated.
#include "physics/world.hpp"

#include <cstddef>
#include <cstdint>

namespace phys::scenes {

// N random discs (r 0.89..2.82 px) dropped into the box under gravity.
World pile(std::size_t n = 2000, std::uint64_t seed = 1);

// Newton's cradle: `balls` discs of radius r hanging from static anchors on
// distance constraints (length 200 px) with `gap` px between neighbours;
// the first ball is pulled out to `liftAngle` radians. e = 1, no friction.
// The gap must exceed the distance a ball travels in one step at impact
// (4.3 px here): with 0.5 px gaps all contacts are active in the same step
// and the sequential-impulse solver spreads the impulse over the row (the
// last ball then rises to 33% of the release height instead of 98%).
World newtonsCradle(int balls = 5, double radius = 20.0, double gap = 2.0, double liftAngle = 0.6);

// One disc on the floor with spin w0 (rad/s, counter-clockwise) and no
// linear velocity, friction mu. It skids until it rolls without slipping.
World spinningBall(double radius = 20.0, double w0 = -20.0, double mu = 0.4);

// `links` discs hanging below a static anchor, joined by damped Hooke springs.
World springChain(int links = 12, double stiffness = 400.0, double damping = 2.0);

// A soft body: discs on a hexagonal lattice joined to their neighbours by
// damped springs, dropped from `height`. Runs 4 substeps per step: the
// springs must be stiff enough to carry the blob's weight (sag per row
// $\approx g\,m/k$) and explicit springs need $(\omega\,\Delta t)^2 < 4$.
World softBlob(int rings = 4, double spacing = 12.0, double stiffness = 8000.0, double damping = 4.0,
               double height = 250.0);

// Constant-density scene from makeConstantDensityScene (gravity off, e = 1),
// the benchmark "gas" workload.
World stress(std::size_t n = 5000, std::uint64_t seed = 42);

}  // namespace phys::scenes
