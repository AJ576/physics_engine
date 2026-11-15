# Resume facts (measured or read from source)

Do not quote anything that is not in this file. Numbers are from `bench/physics_bench` on the machine in `bench/results/env.txt`, Release `-O3 -DNDEBUG`, seed 42. Each listed-N row is 5 runs, 60 warmup steps, 180 timed steps; extra-N rows are 3 runs × 60 steps. Values are **median across runs of that run’s mean/p50/p99/max**, with sample stddev.

**Machine:** Apple M4, 10 cores, 16 GB, Darwin 27.0.0 arm64.
**Compiler:** Apple clang 17.0.0 (`clang-1700.6.3.2`).

## Architecture

- **Loop:** fixed `dt = 1/60` s accumulator; render and physics decoupled; frame-time `dt > 0.25` replaced with `0.016` (not a max-step cap).
- **Layout:** array-of-structs `std::vector<RigidBody>`. Not SoA, not ECS. No virtual calls. Grid stores `RigidBody*`.
- **Bodies:** circles / point masses. No orientation, angular velocity, or inertia. Not 2D rigid-body dynamics in the mechanics sense.
- **Integrator:** `x += v dt + ½ a dt²` then `v += a dt` (old velocity). This is **not** semi-implicit Euler.
- **Broadphase:** uniform spatial hash, cell size **10**, rebuilt every step. Bodies inserted by **center only**. Half-neighborhood offsets `(1,0),(0,1),(1,1),(1,-1)`. No AABB before narrowphase.
- **Solver:** one pass. Circle–circle `distance²` test, mass-weighted positional projection of the full overlap, then frictionless impulse `j = -(1+e) (v_rel·n) / (invM1+invM2)`. Global `e = 0.7`. No solver iterations, no friction.
- **Springs:** platform class exists; default app sets `enableSprings = false`. Body–body Hooke `calculateForce` is dead (commented). Not spring constraints.
- **Concurrency:** single-threaded.
- **Kept opt:** cell vectors reused across steps; empty keys erased (`physics.cpp` `runPhysics`).

## Step time — constant density (arena scales with N; density of 5000 bodies in 800×600)

| N | arena | mean ms | p50 ms | p99 ms | max ms |
|---|---|---|---|---|---|
| 500 | 253×190 | 0.042 ± 0.003 | 0.041 ± 0.001 | 0.098 ± 0.032 | 0.107 ± 0.048 |
| 1000 | 358×268 | 0.086 ± 0.002 | 0.084 ± 0.002 | 0.152 ± 0.024 | 0.181 ± 0.037 |
| 2000 | 506×379 | 0.168 ± 0.002 | 0.162 ± 0.002 | 0.257 ± 0.022 | 0.280 ± 0.087 |
| 5000 | 800×600 | 0.440 ± 0.018 | 0.418 ± 0.010 | 0.673 ± 0.148 | 0.762 ± 0.248 |
| 10000 | 1131×849 | 0.984 ± 0.146 | 0.889 ± 0.067 | 2.275 ± 1.907 | 2.897 ± 2.541 |
| 20000 | 1600×1200 | 2.271 ± 0.440 | 2.093 ± 0.094 | 3.583 ± 6.227 | 4.074 ± 25.730 |
| 40000* | 2263×1697 | 3.648 ± 0.193 | 3.612 ± 0.143 | 4.469 ± 0.748 | 4.838 ± 1.000 |
| 80000* | 3200×2400 | 10.920 ± 0.642 | 10.618 ± 0.111 | 15.596 ± 11.857 | 18.100 ± 11.991 |
| 160000* | 4525×3394 | 25.001 ± 1.417 | 23.197 ± 0.255 | 45.242 ± 16.148 | 45.773 ± 19.582 |

\*extra N, 3 runs × 60 steps.

## Step time — fixed arena 800×600

| N | mean ms | p50 ms | p99 ms | max ms |
|---|---|---|---|---|
| 500 | 0.031 ± 0.000 | 0.031 ± 0.000 | 0.037 ± 0.008 | 0.057 ± 0.010 |
| 1000 | 0.069 ± 0.002 | 0.068 ± 0.001 | 0.102 ± 0.020 | 0.137 ± 0.033 |
| 2000 | 0.200 ± 0.001 | 0.196 ± 0.001 | 0.297 ± 0.012 | 0.335 ± 0.023 |
| 5000 | 0.431 ± 0.005 | 0.417 ± 0.003 | 0.605 ± 0.033 | 0.709 ± 0.027 |
| 10000 | 1.040 ± 0.037 | 1.045 ± 0.029 | 1.322 ± 0.385 | 1.445 ± 0.501 |
| 20000 | 3.030 ± 0.210 | 2.838 ± 0.190 | 4.846 ± 3.068 | 5.714 ± 8.596 |
| 40000* | 7.380 ± 0.548 | 7.153 ± 0.182 | 12.318 ± 8.100 | 13.415 ± 13.776 |
| 80000* | 22.352 ± 0.203 | 22.207 ± 0.063 | 23.704 ± 2.851 | 23.838 ± 4.874 |
| 160000* | 82.774 ± 2.887 | 80.288 ± 3.100 | 104.105 ± 14.026 | 121.907 ± 8.938 |

## Largest N inside 16.6 ms (median of run-mean step time)

- Constant density: **80000** (10.920 ± 0.642 ms). 160000 is 25.001 ms.
- Fixed 800×600: **40000** (7.380 ± 0.548 ms). 80000 is 22.352 ms.
- Every listed N (500–20000) is inside budget on both layouts.
- p99 at N=80000 const-density is 15.596 ms (still ≤ 16.6) but stddev is 11.9 ms — do not claim “every frame < 16.6 ms” at 80k.

## Grid vs brute force (constant density; brute path is a bench-local copy of the engine resolver)

Paired step times from `bench/results/grid_vs_brute.csv`:

| N | grid pairs | brute pairs | brute/grid pairs | grid mean ms | brute mean ms | speedup |
|---|---|---|---|---|---|---|
| 500 | 2170 | 124750 | 57.5 | 0.039 ± 0.001 | 0.109 ± 0.002 | **2.82×** |
| 1000 | 4528 | 499500 | 110 | 0.083 ± 0.001 | 0.404 ± 0.002 | **4.90×** |
| 2000 | 8999 | 1999000 | 222 | 0.165 ± 0.001 | 1.895 ± 0.022 | **11.5×** |
| 5000 | 22592 | 12497500 | 553 | 0.416 ± 0.002 | 12.867 ± 0.042 | **30.9×** |
| 10000 | 45936 | 49995000 | 1088 | 0.832 ± 0.002 | 54.344 ± 3.194 | **65.3×** |
| 20000 | 92372 | 199990000 | 2165 | 1.952 ± 0.004 | 258.144 ± 32.229 | **132×** |

## Energy / momentum

**Engine conservation** (`WorldPhysics::runPhysics`, 100 non-overlapping circles, `e=1`, gravity off, spawned at ~1e5 so walls are unreachable, 5000 steps):

- Max relative `|p|` drift: **2.73×10⁻¹⁵**
- Max relative KE drift: **4.51×10⁻¹⁶**
- End-of-run relative `|p|` drift: 1.12×10⁻¹⁵; KE: 2.26×10⁻¹⁶

**Standalone 1D harmonic oscillator** (NOT WorldPhysics, NOT the Spring class). `m=1`, `k=4π²`, `x0=1`, `v0=0`, `dt=1/60`, 10000 steps, `E0=½kx0²`:

| Integrator | max \|E−E0\|/E0 | final E/E0 |
|---|---|---|
| Explicit Euler | 2.33×10⁴⁷ | 2.33×10⁴⁷ |
| Semi-implicit Euler | **5.53×10⁻²** | 1.000825 |
| Engine kinematics (`x += v dt + ½ a dt²`, `v += a dt`) | 5.67×10²³ | 5.67×10²³ |
| `RigidBody::numericalIntegration` + `F=−kx` | 5.67×10²³ | 5.67×10²³ |

The engine integrator matches the unstable kinematic formula, not semi-implicit Euler.

## Profile (N=80000, before grid-reuse)

See `bench/results/profile_top5.txt`. Top 5 leaf: `runPhysics` 62.5%, inlined helpers 13.1%, malloc 5.05%, free 5.00%, `updateColorFromSpeed` 3.52%.

## Claims you must NOT make

- “Rigid body dynamics” (no rotation / inertia).
- “Spring constraints” or “integrating a spring force model” as a live feature.
- “Semi-implicit Euler chosen over explicit Euler to prevent energy drift” — the body integrator is not semi-implicit, and the engine formula **gains** energy on a harmonic oscillator.
- “Multithreaded.”
- “Deterministic” (`srand(time(0))`; collision order follows `unordered_map`).
- “SoA / ECS.”
- “Swept circle CCD” (`prevPosition_` is unused for circle–circle).
- “Real-time at 160k bodies” (25 ms/step const-density; 83 ms fixed arena).
- “p99 < 16.6 ms at 80k” (p99 stddev is 12 ms).
- Any number not in this file or `bench/results/`.
- Linux `perf` numbers (this machine is macOS; profile is `sample`).
