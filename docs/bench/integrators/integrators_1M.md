## Part 1 (standalone): x'' = -w^2 x, x0 = 1, v0 = 0, h = 1/60 s, 1000000 steps (16667 s)

| w (rad/s) | w h | method | E_N/E_0 - 1 | min E/E_0 | max E/E_0 | overflow step |
|---:|---:|---|---:|---:|---:|---:|
| 1 | 0.0167 | explicit Euler | 4.1745e+120 | 1 | 4.1745e+120 | - |
| 1 | 0.0167 | semi-implicit Euler | -0.00818132 | 0.991736 | 1.0084 | - |
| 1 | 0.0167 | velocity Verlet | -2.95436e-05 | 0.999931 | 1 | - |
| 1 | 0.0167 | old engine (const. accel.) | 2.06317e+60 | 1 | 2.06317e+60 | - |
| 5 | 0.0833 | explicit Euler | overflow | 1 | overflow | 102199 |
| 5 | 0.0833 | semi-implicit Euler | 0.00482588 | 0.96 | 1.04348 | - |
| 5 | 0.0833 | velocity Verlet | -0.00173566 | 0.998264 | 1 | - |
| 5 | 0.0833 | old engine (const. accel.) | overflow | 1 | overflow | 204050 |
| 10 | 0.1667 | explicit Euler | overflow | 1 | overflow | 25763 |
| 10 | 0.1667 | semi-implicit Euler | -0.076863 | 0.923077 | 1.09091 | - |
| 10 | 0.1667 | velocity Verlet | -0.00331403 | 0.993056 | 1 | - |
| 10 | 0.1667 | old engine (const. accel.) | overflow | 1 | overflow | 51177 |
| 30 | 0.5000 | explicit Euler | overflow | 1 | overflow | 3154 |
| 30 | 0.5000 | semi-implicit Euler | -0.192932 | 0.8 | 1.33333 | - |
| 30 | 0.5000 | velocity Verlet | -0.0305649 | 0.9375 | 1 | - |
| 30 | 0.5000 | old engine (const. accel.) | overflow | 1 | overflow | 5975 |
| 60 | 1.0000 | explicit Euler | overflow | 1 | overflow | 1014 |
| 60 | 1.0000 | semi-implicit Euler | 0 | 1 | 2 | - |
| 60 | 1.0000 | velocity Verlet | -0.1875 | 0.8125 | 1 | - |
| 60 | 1.0000 | old engine (const. accel.) | overflow | 1 | overflow | 1732 |

### Explicit Euler: measured per-step energy factor vs $1 + \omega^2 h^2$

| w | predicted factor | measured mean factor exp(mean log) | max per-step relative deviation | steps until E = inf |
|---:|---:|---:|---:|---:|
| 1 | 1.000277777778 | 1.000277777778 | 6.66e-16 | none |
| 5 | 1.006944444444 | 1.006944444444 | 4.44e-16 | 102199 |
| 10 | 1.027777777778 | 1.027777777778 | 6.66e-16 | 25763 |
| 30 | 1.250000000000 | 1.250000000000 | 5.55e-16 | 3154 |
| 60 | 2.000000000000 | 2.000000000000 | 0.00e+00 | 1014 |

### Semi-implicit Euler: bounds and the conserved modified energy

| w | predicted E/E_0 range [1/(1+wh/2), 1/(1-wh/2)] | measured range | max abs(H~_n/H~_0 - 1) |
|---:|---|---|---:|
| 1 | [0.991736, 1.008403] | [0.991736, 1.008403] | 5.54e-14 |
| 5 | [0.960000, 1.043478] | [0.960000, 1.043478] | 6.42e-14 |
| 10 | [0.923077, 1.090909] | [0.923077, 1.090909] | 3.09e-13 |
| 30 | [0.800000, 1.333333] | [0.800000, 1.333333] | 2.03e-13 |
| 60 | [0.666667, 2.000000] | [1.000000, 2.000000] | 0.00e+00 |

### Old engine integrator: mean per-step energy factor vs det = 1 + w^2 h^2 / 2

| w | det of step matrix | measured mean factor | steps until E = inf |
|---:|---:|---:|---:|
| 1 | 1.000138888889 | 1.000138888996 | none |
| 5 | 1.003472222222 | 1.003472121959 | 204050 |
| 10 | 1.013888888889 | 1.013888475489 | 51177 |
| 30 | 1.125000000000 | 1.125004573495 | 5975 |
| 60 | 1.500000000000 | 1.499921990304 | 1732 |

## Part 2 (engine): phys::World, static anchor + Hooke spring (k = w^2, m = 1), 1000000 steps

Explicit Euler is predicted to give exactly $(1 + \omega^2 h^2)^{100}$ after 100 steps; the other columns cover the whole run.

| w | engine integrator | E_100/E_0 | predicted (explicit) | E_N/E_0 | min E/E_0 | max E/E_0 | overflow step | contacts |
|---:|---|---:|---:|---:|---:|---:|---:|---:|
| 1 | SemiImplicitEuler (default) | 1.00172627 | 1.02816321 | 0.991734 | 0.991651 | 1.0084 | - | 0 |
| 1 | ExplicitEuler | 1.02816339 | 1.02816321 | 1.10353e+18 | 1 | 1.41003e+18 | - | 2404 |
| 1 | ConstantAccel (old engine) | 1.0148244 | 1.02816321 | 1.07678e+18 | 1 | 1.20518e+18 | - | 1974 |
| 10 | SemiImplicitEuler (default) | 0.932129592 | 15.4853965 | 0.923088 | 0.923016 | 1.09093 | - | 0 |
| 10 | ExplicitEuler | 15.4853972 | 15.4853965 | overflow | 1 | overflow | 1797 | 16 |
| 10 | ConstantAccel (old engine) | 3.81995735 | 15.4853965 | overflow | 1 | overflow | 3365 | 24 |
| 30 | SemiImplicitEuler (default) | 0.876523612 | 4.90909347e+09 | 0.807077 | 0.799991 | 1.33334 | - | 0 |
| 30 | ExplicitEuler | 4.90909346e+09 | 4.90909347e+09 | overflow | 1 | overflow | 242 | 8 |
| 30 | ConstantAccel (old engine) | 133701.91 | 4.90909347e+09 | overflow | 1 | overflow | 457 | 21 |

12-link undamped spring chain (k = 400, m = 1, gravity and walls off), 1000000 steps

| engine integrator | E_N/E_0 | min E/E_0 | max E/E_0 | overflow step | contacts |
|---|---:|---:|---:|---:|---:|
| SemiImplicitEuler (default) | 0.96415 | 0.860294 | 1.13547 | - | 25611 |
| ExplicitEuler | overflow | 1 | overflow | 1014 | 21 |
| ConstantAccel (old engine) | overflow | 1 | overflow | 1875 | 20 |
