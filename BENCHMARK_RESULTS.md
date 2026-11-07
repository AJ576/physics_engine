# 2D Physics Engine - Performance Benchmark Report

Times the physics step only (no rendering). Fixed timestep 1/60 s. Bodies: radius ~0.9 to 2.8 in an 800 x 600 box, random velocities, fixed seed.

## 1. Step Time vs Body Count (gravity on, grid cell size 10)

| Body Count | Mean Step (ms) | Min / Max (ms) | P95 (ms) | Steps / sec | Bodies / sec | Under 16.67 ms? |
|:---|:---|:---|:---|:---|:---|:---|
| 500 | 0.123 | 0.11 / 0.13 | 0.127 | 8155 | 4077761 | yes |
| 1000 | 0.243 | 0.22 / 0.27 | 0.261 | 4107 | 4107141 | yes |
| 2500 | 0.746 | 0.62 / 0.88 | 0.835 | 1340 | 3350362 | yes |
| 5000 | 1.783 | 1.19 / 2.33 | 2.304 | 560 | 2803497 | yes |
| 7500 | 3.161 | 1.70 / 4.37 | 4.262 | 316 | 2372668 | yes |
| 10000 | 4.820 | 2.22 / 6.80 | 6.757 | 207 | 2074730 | yes |

## 2. Spatial Grid vs Brute-Force O(N²) (no gravity)

| Body Count | Brute Force (ms) | Grid (ms) | Speedup | Brute-Force Pairs | Grid Pairs | Pairs Skipped |
|:---|:---|:---|:---|:---|:---|:---|
| 100 | 0.008 | 0.021 | 0.4x | 4950 | 7 | 99.86% |
| 250 | 0.044 | 0.054 | 0.8x | 31125 | 51 | 99.84% |
| 500 | 0.166 | 0.106 | 1.6x | 124750 | 220 | 99.82% |
| 1000 | 0.647 | 0.214 | 3.0x | 499500 | 846 | 99.83% |
| 2000 | 2.925 | 0.530 | 5.5x | 1999000 | 3507 | 99.82% |
| 3500 | 9.524 | 0.985 | 9.7x | 6123250 | 10953 | 99.82% |
| 5000 | 19.735 | 1.192 | 16.6x | 12497500 | 22629 | 99.82% |

## 3. Where the Time Goes (5,000 bodies, gravity on)

| Phase | Time (µs) | Time (ms) | % of Step |
|:---|:---|:---|:---|
| Numerical Integration (Kinematics) | 58.3 | 0.058 | 3.2% |
| Spatial Grid Construction & Hash | 440.8 | 0.441 | 24.4% |
| Collision Detection & Impulse Res | 1201.1 | 1.201 | 66.4% |
| Boundary Handling & Settling | 61.2 | 0.061 | 3.4% |
| Global Forces Accumulation | 10.6 | 0.011 | 0.6% |
| Spring Dynamics | 35.8 | 0.036 | 2.0% |
| **Total** | **1808.0** | **1.808** | **100.0%** |

## 4. Grid Cell Size Sweep (2,500 bodies, no gravity)

Default cell size used everywhere else is 10. Lower is better.

| Cell Size | Step Time (ms) | Pairs Checked |
|:---|:---|:---|
| 5 | 0.649 | 1222 |
| 8 | 0.689 | 3650 |
| 10 | 0.673 | 5801 |
| 12 | 0.657 | 8380 |
| 15 | 0.534 | 12838 |
| 20 | 0.451 | 22752 |
| 25 | 0.420 | 35321 |
| 30 | 0.386 | 50100 |
| 40 | 0.349 | 87098 |
| 50 | 0.369 | 132958 |

## 5. Elastic Sanity Check

Restitution e = 1 and no gravity, so total kinetic energy should not change. This is a regression check that collisions and walls do not add or remove energy, not a measure of integrator accuracy.

| Step | Total Energy | Drift (%) |
|:---|:---|:---|
| 250 | 22102649.40 | 0.000000 |
| 500 | 22102649.40 | 0.000000 |
| 750 | 22102649.40 | 0.000000 |
| 1000 | 22102649.40 | 0.000000 |

