# Optimization log

Machine: Apple M4, 10 cores, 16 GB. Compiler: Apple clang 17.0.0, `-O3 -DNDEBUG`.
Protocol: same `physics_bench --scaling` as Phase 2 (seed 42, gravity off). Keep a change only if the median-of-run-means improved by more than **2× baseline stddev** on the N that matter (listed 500–20k, and the 60 Hz N). Failures are recorded.

Profile that ranked the candidates (`sample` of N=80000 const-density, 20 s, **before** any opt): see `bench/results/profile_top5.txt`. Grid rebuild allocator traffic was ~34% inclusive. `updateColorFromSpeed` was 3.52% leaf.

## 1. Grid rebuilt without reallocation — KEEP (v2)

**Branch analog:** `opt/grid-reuse`

v1: replace `grid.clear()` with per-vector `clear()` so occupied cells keep capacity; skip empty cells. Did **not** drop vacated keys.

- Const-density N=10000: 1.255 ± 0.137 ms → 0.863 ± 0.146 ms (gain 0.392 > 2σ = 0.274). Keep-worthy.
- Const-density N=20000: 2.540 ± 0.103 ms → 2.049 ± 0.171 ms (gain 0.491 > 2σ = 0.206). Keep-worthy.
- Const-density N=80000: 12.336 ± 0.831 ms → 14.262 ± 1.116 ms (**regression**). Map accumulated every cell a body ever visited.
- Const-density N=160000: 25.664 ± 0.249 ms → 33.491 ± 5.230 ms (**regression**).

v2: same reuse, then `erase` empty keys so the map stays at currently occupied cells.

Final `--all` numbers with v2 kept (compare to `bench/results/scaling_baseline.csv`):

| N (const density) | baseline mean ms | v2 mean ms | vs 2σ |
|---|---|---|---|
| 500 | 0.062 ± 0.004 | 0.042 ± 0.003 | gain 0.020 > 0.008 |
| 1000 | 0.127 ± 0.005 | 0.086 ± 0.002 | gain 0.041 > 0.010 |
| 2000 | 0.246 ± 0.007 | 0.168 ± 0.002 | gain 0.078 > 0.014 |
| 5000 | 0.656 ± 0.227 | 0.440 ± 0.018 | gain 0.216 < 0.454 (directionally faster; baseline noisy) |
| 10000 | 1.255 ± 0.137 | 0.984 ± 0.146 | gain 0.271 ≈ 0.274 |
| 20000 | 2.540 ± 0.103 | 2.271 ± 0.440 | gain 0.269 > 0.206 |
| 80000 | 12.336 ± 0.831 | 10.920 ± 0.642 | gain 1.416 < 1.662 (within noise) |
| 160000 | 25.664 ± 0.249 | 25.001 ± 1.417 | within noise |

Kept because listed N=1k–20k improved, default N=5000 is faster, and the 80k profile point is not worse. Change is in `physics.cpp` `runPhysics` (clear-in-place + erase empty keys). Collision results are unchanged: same pairs, same resolver.

## 2. Defer `updateColorFromSpeed` to once per step — REVERT

**Branch analog:** `opt/defer-color`

Profile: 3.52% leaf. Removed color updates from `numericalIntegration` / velocity setters; one `refreshColor()` after borders.

Stacked on grid-reuse v2. Const-density N=500–20k changes were all **inside 2σ**. N=160000: 31.403 ± 0.961 → 35.211 ± 0.930 ms (regression > 2σ). Reverted `rigidBody.hpp`, `rigidBody.cpp`, `physics.cpp` color call sites.

## 3. Cell size 16 (from 10) — REVERT

**Branch analog:** `opt/cell-size-16`

Larger cells: fewer hash nodes, more pairs per cell. Still `2 r_max < cell` for default radii.

- Const-density N=80000: 12.944 ± 0.918 → 8.804 ± 0.887 ms (gain 4.14 > 2σ). Helps sparse large arenas.
- Fixed-arena N=20000: 3.094 ± 0.264 → 4.663 ± 0.712 ms (**regression** 1.57 > 2σ). Hurts the dense default box.

Not kept: it makes the 800×600 demo slower at 20k.

## 3b. Cell size 8 — REVERT

**Branch analog:** `opt/cell-size-8`

- Listed const-density 500–20k: slower than cell 10 (e.g. N=5000 0.469 → 0.684 ms).
- Const-density N=160000: 31.403 → 25.783 ms (gain above 2σ) — helps only the densest extra-N case.

Not kept: regresses the N the app actually runs.

## Not attempted

- **SoA layout:** collision is already inlined into `runPhysics` (62% leaf). A layout rewrite is large and was not required to beat allocator noise once grid reuse landed.
- **Avoid sqrt in broadphase:** `sqrt` is only on overlapping pairs in narrowphase. Broadphase is a hash of centers. Not a profile leaf.
- **Multithreaded narrowphase:** sequential impulse is order-dependent; a data race would change trajectories. Not tried.

## Kept code

Only grid-reuse v2 in `WorldPhysics::runPhysics`. `grid_size` remains `10`.
