# 2D Physics Engine (C++17)

A 2D rigid-body engine for circles: impulse-based contacts with friction and
rotation, continuous collision detection, springs and distance constraints,
a uniform-grid broadphase, and an SDL2 demo. The physics core is a static
library with no dependencies beyond the C++ standard library.

Every performance number in this README was measured with the harness in
`bench/` and is reproducible with the commands below. Numbers are from one
machine (a shared cloud VM, see [Environment](#environment)); expect different
absolute values elsewhere.

<!-- Demo GIFs go here:
![pile](docs/media/pile.gif) ![cradle](docs/media/cradle.gif) ![blob](docs/media/blob.gif)
-->
*Demo GIFs: to be recorded.*

## What it does

* **Bodies**: discs with mass, moment of inertia $I = \tfrac12 m r^2$, angle and
  angular velocity. Mass 0 makes a body static.
* **Contacts**: sequential impulses (8 iterations) with accumulated-impulse
  clamping ($j_n \ge 0$), Coulomb friction clamped to the cone
  $|j_t| \le \mu j_n$, restitution applied in a separate pass using the approach
  speed from the start of the step, warm starting, and split-impulse position
  correction (penetration is removed without adding velocity).
* **Continuous collision detection**: a swept-circle test
  ($|d + v_r t| = r_a + r_b$, first root in $[0, \Delta t]$) creates speculative
  contacts; only the earliest impact per body is kept; after the solve, any body
  moving more than half its radius is re-tested along its final path and
  advanced only to its first contact.
* **Springs**: Hooke springs with damping, $F = -k(\ell - \ell_0) - c\,\dot\ell$,
  and rigid distance constraints solved inside the impulse loop.
* **Integrator**: semi-implicit Euler, chosen and measured in
  [docs/INTEGRATORS.md](docs/INTEGRATORS.md).
* **Walls**: four planes around the arena, handled as contacts.
* **Determinism**: same inputs give a bit-identical state (tested by hashing
  the state after N steps), including across 1, 2 and 4 broadphase threads.
* **No heap allocation inside `World::step()`** once buffers have grown
  (tested with a counting `operator new`).

## Architecture

```
core/   physics_core (static library, no SDL)
  world.hpp/.cpp   World: bodies, springs, joints, step()
  scenes.hpp/.cpp  ready-made worlds (pile, cradle, spinning ball, chain, blob, stress)
  scene.hpp/.cpp   seeded, platform-independent random scene generator
  timestep.hpp     fixed-timestep accumulator for render loops
  thread_pool.*    optional broadphase threads
demo/   physics_demo (SDL2 + SDL2_ttf)
tests/  physics_tests, alloc_tests (doctest)
bench/  physics_bench (timing harness), physics_integrators (integrator study),
        compare.py (paired A/B runs), run_suite.sh, env.sh, build_ref.sh
docs/   BENCHMARKS.md, INTEGRATORS.md, OPTIMIZATION_LOG.md, bench/ (raw outputs)
```

**Main loop.** The demo advances a fixed $\Delta t = 1/60$ s with an
accumulator. A frame's elapsed time is clamped to 0.25 s and at most 8 steps are
queued per frame, so a slow frame slows the simulation down instead of
queueing ever more steps.

**One step** (`World::step`, optionally split into substeps):

1. forces: gravity and springs
2. velocities: $v \mathrel{+}= a\,\Delta t$, $\omega \mathrel{+}= \alpha\,\Delta t$
3. broadphase: uniform grid over each body's swept AABB; narrowphase creates
   touching, margin (gap below 1 px) and swept contacts, plus wall contacts
4. solve: warm start, 8 iterations of friction + normal impulses and distance
   constraints, 8 restitution passes
5. positions: split-impulse correction, CCD motion clamp, then
   $x \mathrel{+}= v\,\Delta t$

**Data layout.** Bodies are an array of structs (`std::vector<Body>`, 120 bytes
each). Contacts are a flat array rebuilt each step. The warm-start impulses
live in an open-addressing hash table keyed by body pair. A structure-of-arrays
layout was considered and not adopted: per-body passes are about 1% of the
profile and the solver reads bodies in random order (see
[OPTIMIZATION_LOG.md](docs/OPTIMIZATION_LOG.md), item 3).

**Broadphase.** Each body is inserted into every grid cell its swept AABB
touches, using a counting sort into flat arrays (count, prefix sum, fill). A
pair is tested only in the cell containing the min corner of the two AABBs'
intersection, so each pair is tested once, and any cell size is exact (a test
compares the contact set with brute force for cells of 1.5 to 40 px).

## Results

Physics step only, single thread, gas scene (no gravity, e = 1, constant
density of one body per 96 px²), median over 7 runs of 300 steps. Full tables,
method and raw files: [docs/BENCHMARKS.md](docs/BENCHMARKS.md).

| Bodies | p50 step | p99 step | grid candidate pairs per step | all pairs |
|---:|---:|---:|---:|---:|
| 1,000 | 0.211 ms | 0.255 ms | 338 | 499,500 |
| 5,000 | 1.127 ms | 1.649 ms | 1,736 | 12,497,500 |
| 10,000 | 2.507 ms | 3.731 ms | 3,472 | 49,995,000 |
| 20,000 | 5.440 ms | 8.168 ms | 6,922 | 199,990,000 |

* **Largest N inside a 16.667 ms (60 Hz) step**: 38,000 by p99, 50,000 by
  mean (gas scene, one thread).
* **Grid vs brute force** at 20,000 bodies: 6,922 candidate pairs per step
  instead of 199,990,000; the brute-force step took 5,155 ms against 5.9 ms
  (its CCD re-check also runs without the grid).
* **Pile scene** (gravity, e = 0.7, 3-5 contacts per body): 7.570 ms p50 at
  5,000 bodies, 20.508 ms at 10,000. Solver-bound.
* **Broadphase threads** (4 vCPUs): 6-12% faster at 20,000 and 50,000 bodies,
  no gain at 5,000; results are bit-identical for any thread count.
* **Optimizations, paired A/B** ([OPTIMIZATION_LOG.md](docs/OPTIMIZATION_LOG.md)):
  the counting-sort grid cut the gas step from 9.62 to 5.30 ms at 20,000
  bodies; a compact solver-body array, a cheaper CCD re-check and a sorted
  impulse cache were measured and rejected.
* **Compared with the old engine** (same machine and harness): the old
  single-pass solver was 1.2-1.6x faster per step on the gas scene and
  2.8-3.8x faster on the pile, because it did one projection and one impulse
  per overlapping pair with no friction, rotation, iterations or CCD (it also
  let fast bodies tunnel and piles never came to rest).

#### Environment

Intel Xeon @ 2.10 GHz cloud VM (4 vCPU, shared host), Ubuntu 24.04, GCC 13.3,
`-O3 -DNDEBUG`, benchmark pinned to one core. Run-to-run variation of the
median step time across processes: 6.7% (CV), 13% for p99. Full details in
[docs/bench/final/env.txt](docs/bench/final/env.txt).

## Integrators (summary)

From [docs/INTEGRATORS.md](docs/INTEGRATORS.md), harmonic oscillator
$\ddot x = -\omega^2 x$ at $\Delta t = 1/60$ s (standalone harness, then the
same oscillator inside the engine):

* Explicit Euler multiplies the energy by exactly $1 + \omega^2\Delta t^2$ per
  step (measured factor matches to $7\times10^{-16}$); at $\omega = 10$ rad/s
  the energy overflows a double after 25,763 steps.
* Semi-implicit Euler conserves the modified energy
  $\tilde H = v^2 + \omega^2 x^2 - \Delta t\,\omega^2 x v$ to $3\times10^{-13}$
  over $10^6$ steps, so the energy stays in
  $[1/(1+\omega\Delta t/2),\ 1/(1-\omega\Delta t/2)]$: at $\omega = 10$ rad/s,
  between 0.923 and 1.091 of the start, never drifting.
* The engine's integrator before this branch (second-order position update)
  grows energy by its step-matrix determinant $1 + \tfrac12\omega^2\Delta t^2$
  per step.

## Correctness checks

45 doctest cases (44 in `physics_tests`, 1 in `alloc_tests`), including:

* Narrowphase: touching, overlapping, coincident centres, tangential motion,
  swept time of impact. Impulse magnitude against
  $j = -(1+e)\,v_n / (m_1^{-1} + m_2^{-1})$; 1D elastic collision against the
  analytic result.
* Conservation at $e = 1$ (2,000 bodies, 10 s): linear momentum to
  $|\Delta p| / \sum |p| = 5\times10^{-17}$; kinetic energy $-0.28\%$ with no
  walls and no friction.
* Scenes against analytic answers: a spinning disc rolls at
  $v = \omega_0 r / 3$ (measured 133.333 px/s); a hanging spring chain matches
  the static extension $(N - i + 1)\,mg/k$ within 0.1%; spring period within
  0.03% of $2\pi\sqrt{m/k}$; Newton's cradle hands 98% of the release height to
  the last ball.
* One regression test per bug found in the audit (tunneling, energy injection
  by position correction, integrator energy growth, timestep spiral,
  out-of-range cell index, grid missing pairs).
* Determinism (state hash), zero allocation per step.

All tests also run under AddressSanitizer + UndefinedBehaviorSanitizer
(`asan` preset); the threaded test was run under ThreadSanitizer.

## Limitations

* **Circles only**, in 2D. No polygons, no joints other than distance
  constraints.
* **Energy is not conserved with contacts.** At $e = 1$ without friction,
  2,000 bodies lose 0.28% of their kinetic energy in 10 s with no walls, 2.4%
  with walls, and 8.5% in a dense packing (30 px² per body). Simultaneous
  contacts are the cause; sequential impulses resolve them approximately.
* **Newton's cradle needs gaps.** With balls 0.5 px apart (less than one step
  of travel at impact) the impulse spreads over the row and the last ball rises
  to 33% of the release height instead of 98%.
* **Springs are explicit forces**, so stiff or strongly damped springs are
  unstable unless $(\omega_{max}\Delta t)^2 + 2\gamma_{max}\Delta t < 4$; use
  `WorldConfig::substeps` for stiffer springs.
* **Angular momentum** is not exactly conserved: position-only corrections
  (split impulse, CCD clamp) move bodies without a matching velocity change
  (measured $-1.9\%$ in 10 s in the dense test).
* **The new solver is slower per body than the old one** at small N, because it
  does more work (8 iterations, friction, rotation, CCD). See the results table.
* **Benchmarks come from one shared cloud VM** with 7% run-to-run variation in
  the median step time; differences under about 10% are not resolvable there.
* **Single-threaded solver.** Only the broadphase can use threads, and only
  helps above about 20,000 bodies.

## Build, test, run

Requirements: CMake 3.21+ (for presets; 3.16 without), a C++17 compiler,
optionally SDL2 and SDL2_ttf for the demo.

```bash
cmake --preset release
```

```bash
cmake --build --preset release -j
```

```bash
ctest --preset release
```

Sanitizers:

```bash
cmake --preset asan && cmake --build --preset asan -j && ctest --preset asan
```

Demo (if SDL2 was found at configure time):

```bash
./build/release/physics_demo
```

| Key | Action |
|---|---|
| `1`-`5` | scene: pile, Newton's cradle, spring chain, soft blob, stress (5,000 bodies) |
| `G` | gravity on/off |
| arrows | gravity direction |
| `O` | grid overlay |
| `T` | broadphase threads 1 / 4 |
| `Space` | pause |
| `R` | reset scene |
| `Esc` | quit |

Options: `--scene N` (start scene), `--frames K` (run K frames with one fixed
step per frame, then quit), `--screenshot out.bmp` (save the last frame).

Install SDL2: `brew install sdl2 sdl2_ttf` (macOS) or
`sudo apt install libsdl2-dev libsdl2-ttf-dev` (Debian/Ubuntu).

## Reproducing the benchmarks

```bash
bench/run_suite.sh docs/bench/mymachine
```

This writes `env.txt`, the gas and pile sweeps, the 60 Hz search and a noise
study (about 1 hour, mostly brute force at 20,000 bodies). For a single run:

```bash
./build/release/physics_bench --scene gas --n 5000,20000 --runs 7
```

To compare two builds (paired, interleaved):

```bash
bench/build_ref.sh HEAD~1 /tmp/ref && bench/compare.py /tmp/ref/physics_bench build/release/physics_bench
```

## History

This repository started as a two-person project (GitHub users AJ576 and
ArshAnan). The `engine-v2` branch replaced the original single-pass solver,
data layout and tests; `git log` and `git blame` show who wrote what.
