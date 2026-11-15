# Optimization log

Every performance change on `engine-v2` after the physics rewrite, kept or
not, with before/after numbers. Machine, OS and compiler are in
[`bench/baseline/env.txt`](bench/baseline/env.txt) (Intel Xeon @ 2.1 GHz cloud
VM, 4 vCPU, Ubuntu 24.04, GCC 13.3, `-O3`). Single-threaded unless stated.

## Method

* `bench/compare.py A B` runs two `physics_bench` builds alternately (AB, BA,
  AB, ...), each as its own process pinned to one core (`taskset -c 3`), one
  run of 300 timed steps after 120 warm-up steps, same seed. Each round gives
  a paired ratio $r_i = p50_B / p50_A$ of the median step time.
* Verdict: **faster** only if every $r_i < 1$; **slower** only if every
  $r_i > 1$; otherwise **within noise**. A change is kept only if it is faster
  on the scene it targets and not slower elsewhere.
* Noise floor (A/A, identical binaries, 9 rounds,
  [raw](bench/opt/00_aa_noise.md)): paired ratios 0.940-1.145 at N = 5,000 and
  0.958-1.067 at N = 20,000, verdict "within noise". Changes smaller than
  about 5-15% cannot be resolved on this machine.
* Scenes: `gas` = constant density (96 px² per body), no gravity, e = 1;
  `pile` = same bodies with gravity 980 px/s², e = 0.7.
* Profiles: `perf record -e cpu-clock -g` on the `profile` preset (Release +
  frame pointers), `physics_bench --scene gas --n 20000 --runs 1 --steps 600`.

## Starting point (end of Phase 3)

Profile, gas N = 20,000, p50 9.6 ms (top functions by self time):

| Function | Self time |
|---|---:|
| `World::collide()` (grid build + pair loop, `std::unordered_map` iteration inlined) | 44.6% |
| `World::clampFastMotion()` + `sweptFraction()` + its `std::sort` | 19.1% |
| `malloc` / `free` / `_int_malloc` / `_int_free` / `vector::_M_realloc_insert` | 14.4% |
| `World::solveVelocities()` | 5.3% |
| `World::wallContacts()` | 4.0% |

## 1. Flat uniform grid by counting sort (kept)

Replaced `std::unordered_map<uint64_t, std::vector<int>>` (cleared every
step, so every cell vector was freed and re-allocated) with flat arrays:
count entries per cell, prefix sum, fill `items_` in body order. Buffers keep
their capacity, so the grid does no allocation after the first step. Cell
iteration is row-major, which also makes the contact order (and so the
results) independent of the standard library's hash table.

| Scene | N | before p50 | after p50 | median ratio | ratio range | verdict |
|---|---:|---:|---:|---:|---|---|
| gas | 5,000 | 1.714 ms | 1.145 ms | 0.666 | 0.583-0.910 | faster |
| gas | 20,000 | 9.623 ms | 5.303 ms | 0.545 | 0.535-0.576 | faster |
| pile | 5,000 | 8.229 ms | 7.781 ms | 0.957 | 0.898-1.004 | within noise |

Raw: [gas](bench/opt/01_flat_grid_gas.md), [pile](bench/opt/01_flat_grid_pile.md).
The pile scene is solver-bound (see the next profile), so the grid barely
shows there.

## 2. Grid cell size sweep (no change; default stays 10 px)

Bodies have radius 0.89-2.82 px (max diameter 5.64 px). Because bodies are
inserted into every cell their swept AABB touches and pairs are AABB-tested
before the narrowphase, the candidate-pair count does not depend on the cell
size (3,069-3,103 per step at gas N = 20,000). Only the bookkeeping cost does.
`physics_bench --cell C --runs 5 --steps 300`, median p50 ([raw](bench/opt/02_cell_size_sweep.txt)):

| cell (px) | cell / max radius | gas N = 20,000 p50 | pile N = 5,000 p50 |
|---:|---:|---:|---:|
| 3 | 1.1 | 5.324 ms | 8.447 ms |
| 4 | 1.4 | 5.539 ms | 8.116 ms |
| 5 | 1.8 | 5.340 ms | 7.992 ms |
| 6 | 2.1 | 5.142 ms | 7.661 ms |
| 8 | 2.8 | 5.604 ms | 7.589 ms |
| 10 (default) | 3.5 | 5.229 ms | 7.727 ms |
| 12 | 4.3 | 5.301 ms | 8.105 ms |
| 15 | 5.3 | 5.292 ms | 7.795 ms |
| 20 | 7.1 | 5.476 ms | 8.200 ms |
| 30 | 10.6 | 5.969 ms | 8.822 ms |

From 3 to 20 px the spread (5.14-5.60 ms gas, 7.59-8.45 ms pile) is inside
this machine's run-to-run noise, so no cell size in that range is measurably
better than 10 px. At 30 px the cost rises (+14% gas, +14% pile vs 10 px)
because more AABB tests happen per cell. Unlike the old engine, a cell smaller
than a body is still exact (test "grid contact set equals brute force for any
cell size").

## 3. Compact solver-body array (rejected, reverted)

Idea (Box2D's solver bodies; a hot/cold split rather than full SoA): copy
velocity, spin and inverse mass/inertia into a 40-byte record per body before
the solve, plus a static dummy record for the walls so the inner loop has no
wall branch, and write velocities back afterwards. Target: `solveVelocities()`,
42% of the pile profile.

| Scene | N | before p50 | after p50 | median ratio | ratio range | verdict |
|---|---:|---:|---:|---:|---|---|
| pile | 5,000 | 7.937 ms | 7.809 ms | 0.992 | 0.937-1.038 | within noise |
| gas | 5,000 | 1.150 ms | 1.141 ms | 0.989 | 0.935-1.190 | within noise |
| gas | 20,000 | 5.316 ms | 5.477 ms | 1.047 | 0.972-1.088 | within noise |

Raw: [pile](bench/opt/03_solverbodies_pile.md), [gas](bench/opt/03_solverbodies_gas.md).
Why it did not help: the bodies already fit in cache (5,000 x 120 B = 600 KB
against 2 MB of L2 per core on this VM; 20,000 bodies = 2.4 MB is borderline),
so the solver is bound by the dependent arithmetic of each impulse, not by
loads. Reverted.

**Full structure-of-arrays body storage: not attempted.** The per-body
passes that SoA speeds up (integration, force accumulation) are about 1% of
both profiles, and the contact solver reads bodies in contact order (random
access), where one record per body touches one cache line and separate arrays
would touch five. Item 3 suggests even the cheaper version of the layout
change does not pay on this working-set size.

## 4. Cheaper CCD motion clamp (rejected, reverted)

Target: `clampFastMotion()` + `sweptFraction()` + its `std::sort`, 24% of the
gas profile. Changes: displacements computed once per body instead of inside
every comparison, an exact path-AABB prefilter before the swept test, a visit
stamp so a body in several cells is tested once, and precomputed sort keys.
Results were bit-identical (`physics_bench --hash`: same state hash for gas
and pile at N = 5,000 and 20,000).

| Scene | N | before p50 | after p50 | median ratio | ratio range | verdict |
|---|---:|---:|---:|---:|---|---|
| gas | 5,000 | 1.165 ms | 1.159 ms | 1.039 | 0.880-1.049 | within noise |
| gas | 20,000 | 5.343 ms | 5.380 ms | 1.002 | 0.939-1.237 | within noise |
| pile | 5,000 | 7.999 ms | 8.079 ms | 1.013 | 0.950-1.048 | within noise |

Raw: [gas](bench/opt/04_ccd_gas.md), [pile](bench/opt/04_ccd_pile.md).
Why it did not help: `sweptFraction()` already exits after a few flops for
almost every candidate (overlapping or separating), so skipping those calls
saves little, while building path boxes for every body adds an O(N) pass.
The cost is in scanning the cells around each fast body (about 9% of bodies
per step in the gas scene). Reverted.
