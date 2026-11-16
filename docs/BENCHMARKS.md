# Benchmarks

All numbers on this page come from files in [`docs/bench/`](bench/), written by
the harness (`bench/physics_bench`, `bench/run_suite.sh`). The tables below are
generated from the CSV files by `bench/tables.py`; nothing is typed in by hand.

## Environment

Both runs used the same machine ([baseline env](bench/baseline/env.txt),
[final env](bench/final/env.txt)):

* Intel Xeon @ 2.10 GHz cloud VM (Firecracker), 4 vCPU, 1 thread per core,
  15.7 GiB RAM, shared host
* Ubuntu 24.04.5, kernel 6.18
* GCC 13.3, `-O3 -DNDEBUG -std=c++17` (CMake Release), no `-march=native`
* benchmark process pinned to one core (`taskset -c 3`); single-threaded

This is not the Apple M4 the old README quoted; numbers from the two machines
are not comparable.

## Method

* **Scenes** (`phys::makeConstantDensityScene`, seed 42): N discs, mass
  $U[10, 100]$, radius $r = \tfrac12\sqrt{m/\pi}$ (0.89-2.82 px), velocity
  $U[-50, 50]$ px/s per axis, in a 4:3 box whose area grows with N so that each
  body has 96 px² (N = 5,000 gives the demo's 800 x 600). The generator uses its
  own RNG, so the scene is bit-identical on every platform.
  * **gas**: no gravity, e = 1, the primary workload (steady, it never settles)
  * **pile**: gravity 980 px/s² downward, e = 0.7
* **Timing**: per run, build the scene, 120 untimed warm-up steps, then 300
  steps each timed with `std::chrono::steady_clock`. Per run: mean, p50, p99,
  max. 7 runs from the same seed; tables show the median of each statistic over
  the runs and the standard deviation of the run means.
* **60 Hz search**: largest N whose median p99 (or mean) step time is at most
  16.667 ms; doubling from 2,000, then bisection to 1% of N.
* **Brute force**: the same engine with the grid replaced by an all-pairs
  loop (`Broadphase::BruteForce`), 30 timed steps per run. In the new engine
  this also removes the grid from the CCD re-check, so "brute force" means no
  spatial structure anywhere; narrowphase and solver work are the same.
  "Candidates" are pairs that reached the narrowphase test.
* **Noise**: 10 separate processes on the same workload (gas, N = 5,000).
* Only the physics step is timed (no rendering).

## What changed between the two runs

The **old engine** (baseline, commit `c1bab18`) resolved each overlapping pair
once per step (position projection + one impulse), with no friction,
rotation, CCD or iterations, and a hash-map grid. The **new engine** (final)
runs 8 sequential-impulse iterations with friction and rotation, 8 restitution
passes, split-impulse position correction, warm starting, speculative contacts
within 1 px, swept CCD with a post-solve motion clamp, and a counting-sort
grid. It does more work per contact by design, so the comparison below is
"old physics vs new physics", not a like-for-like speedup. The like-for-like
effect of each optimization is in
[OPTIMIZATION_LOG.md](OPTIMIZATION_LOG.md).

### Gas scene: step time vs N (ms; median over 7 runs of 300 steps)

| N | old engine p50 | old p99 | new engine p50 | new mean | new p99 | new max | new run-to-run sd |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 500 | 0.089 | 0.124 | 0.108 | 0.108 | 0.152 | 0.252 | 0.014 |
| 1,000 | 0.174 | 0.212 | 0.211 | 0.214 | 0.255 | 0.286 | 0.006 |
| 2,000 | 0.282 | 0.367 | 0.445 | 0.472 | 0.705 | 1.241 | 0.061 |
| 5,000 | 0.799 | 1.198 | 1.127 | 1.191 | 1.649 | 1.882 | 0.068 |
| 10,000 | 1.556 | 1.863 | 2.507 | 2.757 | 3.731 | 4.912 | 0.140 |
| 20,000 | 4.323 | 6.148 | 5.440 | 5.933 | 8.168 | 9.403 | 0.438 |

### Grid vs brute force (gas scene)

| N | brute-force pairs | old grid candidates | new grid candidates | old: brute / grid time | new: brute / grid time |
|---:|---:|---:|---:|---:|---:|
| 500 | 124,750 | 2,153 | 173 | 2.3x (0.207 / 0.089 ms) | 28.3x (3.070 / 0.108 ms) |
| 1,000 | 499,500 | 4,369 | 338 | 4.6x (0.822 / 0.177 ms) | 58.0x (12.440 / 0.214 ms) |
| 2,000 | 1,999,000 | 8,739 | 682 | 14.6x (4.306 / 0.294 ms) | 103.0x (48.616 / 0.472 ms) |
| 5,000 | 12,497,500 | 22,167 | 1,736 | 25.0x (20.854 / 0.835 ms) | 263.0x (313.211 / 1.191 ms) |
| 10,000 | 49,995,000 | 44,420 | 3,472 | 55.4x (86.111 / 1.553 ms) | 471.0x (1298.548 / 2.757 ms) |
| 20,000 | 199,990,000 | 89,196 | 6,922 | 87.2x (390.883 / 4.481 ms) | 868.9x (5155.220 / 5.933 ms) |

### Largest N with the physics step inside 16.667 ms (60 Hz), gas scene

| criterion | old engine | new engine |
|---|---:|---:|
| p99 step time | 44,500 (next tested 44,750 failed) | 38,000 (next tested 38,250 failed) |
| mean step time | 59,000 (next tested 59,500 failed) | 50,000 (next tested 50,500 failed) |

### Pile scene (gravity, e = 0.7): step time vs N (ms)

| N | old engine p50 | old p99 | new engine p50 | new p99 | new contacts per step |
|---:|---:|---:|---:|---:|---:|
| 500 | 0.148 | 0.183 | 0.483 | 0.642 | 1,239 |
| 1,000 | 0.350 | 0.544 | 1.155 | 2.162 | 2,630 |
| 2,000 | 0.839 | 0.965 | 2.373 | 3.407 | 5,598 |
| 5,000 | 2.589 | 3.012 | 7.570 | 10.715 | 15,512 |
| 10,000 | 6.469 | 9.521 | 20.508 | 30.065 | 36,703 |
| 20,000 | 15.480 | 23.237 | 58.636 | 101.493 | 95,837 |

How to read these:

* **Gas, new vs old**: the new engine is 1.2-1.6x slower per step at the same
  N (p50 0.108 vs 0.089 ms at 500, 5.440 vs 4.323 ms at 20,000) because it does
  more work per contact (see "What changed"). Against the end of the physics
  rewrite, before any optimization, the optimizations in
  [OPTIMIZATION_LOG.md](OPTIMIZATION_LOG.md) cut the gas step from 9.62 to
  5.30 ms at N = 20,000 (paired runs, item 1).
* **Tail latency**: p99 is 1.2-1.6x the p50 at every N, and the max is up to
  2.8x (N = 2,000). On this shared VM a single step can be delayed by the host;
  the noise study below shows p99 varying with a CV of 13%.
* **Brute force**: the grid tests 6,922 candidate pairs per step at N = 20,000
  instead of 199,990,000. The new engine's brute-force mode is much slower than
  the old one's because its CCD re-check also loses the grid (see Method), so
  its 869x speedup at 20,000 overstates what the grid alone buys; the
  candidate-pair counts are the structure-only comparison.
* **60 Hz**: 38,000 bodies by p99 and 50,000 by mean in the gas scene, single
  thread, physics step only. In the pile scene the step stays under 16.667 ms
  at N = 5,000 (p99 10.7 ms) and exceeds it at N = 10,000 (p50 20.5 ms).
* **Pile, new vs old**: 2.8-3.8x slower. A pile has 3-5 contacts per body
  (15,512 at N = 5,000, 95,837 at N = 20,000), each solved 8 times plus friction and restitution
  passes; the old engine visited each overlapping pair once. The solver is
  42% of the pile profile and is single-threaded.

## Noise

10 separate processes, gas N = 5,000, 3 runs x 300 steps each
([baseline](bench/baseline/noise.txt), [final](bench/final/noise.txt)):

| Statistic | old engine: mean, CV, range | new engine: mean, CV, range |
|---|---|---|
| mean step | 0.811 ms, 9.8%, 0.736-0.969 | 1.234 ms, 7.1%, 1.145-1.394 |
| p50 step | 0.769 ms, 7.3%, 0.723-0.909 | 1.193 ms, 6.7%, 1.122-1.366 |
| p99 step | 1.086 ms, 22.2%, 0.843-1.470 | 1.747 ms, 13.3%, 1.404-2.135 |

So a single number from one process can be off by about 7% (p50) to 13-22%
(p99). That is why optimizations were judged only by paired, interleaved runs
(`bench/compare.py`): an A/A run of identical binaries gave paired ratios of
0.94-1.15 ([raw](bench/opt/00_aa_noise.md)).

## Thread scaling

The broadphase can use a thread pool (`WorldConfig::threads`, results
bit-identical). Gas scene, 4 vCPUs, same binary, paired runs (from
[OPTIMIZATION_LOG.md](OPTIMIZATION_LOG.md), item 6):

| N | 1 thread p50 | 4 threads p50 | median ratio | ratio range | verdict |
|---:|---:|---:|---:|---|---|
| 5,000 | 1.215 ms | 1.236 ms | 1.009 | 0.930-1.211 | within noise |
| 20,000 | 5.792 ms | 5.092 ms | 0.885 | 0.833-0.952 | faster |
| 50,000 | 15.928 ms | 14.756 ms | 0.936 | 0.870-0.971 | faster |

The solver is single-threaded, which limits the gain (Amdahl).

## Conservation (tests/test_conservation.cpp)

2,000 bodies, gravity off, e = 1, 600 steps (10 s), new engine:

| Case | Contacts in 10 s | Result |
|---|---:|---|
| linear momentum, no walls, $\mu = 0$, dense (30 px²/body) | 189,756 | $\lvert\Delta p\rvert / \sum\lvert p\rvert = 5.5\times10^{-17}$ |
| linear momentum, no walls, $\mu = 0.3$, dense | 258,949 | $6.6\times10^{-17}$ |
| kinetic energy, no walls, $\mu = 0$, demo density (96 px²/body) | 74,215 | $-0.28\%$ |
| kinetic energy, walls, $\mu = 0$, demo density | 174,935 | $-2.4\%$ |
| kinetic energy, no walls, $\mu = 0$, dense | 189,756 | $-8.5\%$ |
| kinetic energy, walls, $\mu = 0.3$ | 187,892 | $-84\%$ (friction converts and dissipates) |
| angular momentum, no walls, $\mu = 0$, dense | 189,756 | $-1.9\%$ |

With 1 restitution pass instead of 8, the demo-density energy loss was
$-9.5\%$ ([OPTIMIZATION_LOG](OPTIMIZATION_LOG.md) and commit history). The old
engine kept kinetic energy exactly at e = 1 because it resolved each pair on
its own; the new solver trades that for stable stacking and friction.

## Reproduce

```bash
cmake --preset release && cmake --build --preset release -j
bench/run_suite.sh docs/bench/mymachine
python3 bench/tables.py docs/bench/baseline docs/bench/mymachine
```
