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
