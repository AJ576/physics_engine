# Integrator study

Why the engine integrates with semi-implicit (symplectic) Euler, and what that
choice does and does not guarantee. Every number below is printed by
`physics_integrators` (source: `bench/integrator_study.cpp`); the raw outputs
are in [`docs/bench/integrators/`](bench/integrators/).

```bash
cmake --preset release && cmake --build --preset release
./build/release/physics_integrators                    # 100,000 steps
./build/release/physics_integrators --steps 1000000    # 1,000,000 steps
```

The program is deterministic (no timing, no randomness), so these numbers
reproduce exactly with the same compiler and flags.

## Setup

Part 1 is **standalone**: a scalar harmonic oscillator integrated directly,
without `phys::World`:

$$\ddot{x} = -\omega^2 x,\qquad x_0 = 1,\ v_0 = 0,\qquad h = \Delta t = \tfrac{1}{60}\ \mathrm{s},$$

with energy $E = \tfrac12 v^2 + \tfrac12 \omega^2 x^2$, for
$\omega \in \{1, 5, 10, 30, 60\}$ rad/s ($\omega h$ from 0.017 to 1), over
$10^5$ steps (1,667 s simulated) and $10^6$ steps.

| Method | Update |
|---|---|
| explicit Euler | $x_{n+1} = x_n + h v_n,\quad v_{n+1} = v_n + h a_n$ |
| semi-implicit Euler (engine default) | $v_{n+1} = v_n + h a_n,\quad x_{n+1} = x_n + h v_{n+1}$ |
| velocity Verlet | $x_{n+1} = x_n + h v_n + \tfrac12 h^2 a_n,\quad v_{n+1} = v_n + \tfrac12 h (a_n + a_{n+1})$ |
| old engine (until this branch) | $x_{n+1} = x_n + h v_n + \tfrac12 h^2 a_n,\quad v_{n+1} = v_n + h a_n$ |

Part 2 runs the **engine** (`phys::World`): a static anchor and one body of
mass 1 on a Hooke spring of stiffness $k = \omega^2$, once per engine
integrator (`WorldConfig::integrator`), plus an undamped 12-link spring chain.

## Claim 1: explicit Euler multiplies the energy by $(1 + \omega^2 h^2)$ every step. Confirmed.

Algebraically, $\omega^2 x_{n+1}^2 + v_{n+1}^2 = (\omega^2 x_n^2 + v_n^2)(1 + \omega^2 h^2)$
exactly. The harness checks the ratio $E_{n+1}/E_n$ at every step:

| $\omega$ | predicted factor $1 + \omega^2 h^2$ | measured mean factor | max per-step relative deviation | steps until $E$ overflows |
|---:|---:|---:|---:|---:|
| 1 | 1.000277777778 | 1.000277777778 | $6.7\times10^{-16}$ | none in $10^5$ |
| 5 | 1.006944444444 | 1.006944444444 | $4.4\times10^{-16}$ | none in $10^5$ ($E/E_0 = 3.6\times10^{300}$) |
| 10 | 1.027777777778 | 1.027777777778 | $6.7\times10^{-16}$ | 25,763 |
| 30 | 1.250000000000 | 1.250000000000 | $5.6\times10^{-16}$ | 3,154 |
| 60 | 2.000000000000 | 2.000000000000 | 0 | 1,014 |

The deviation is at the level of double-precision rounding: the factor is
exact, not an average. Even at $\omega = 1$ ($\omega h = 0.017$) energy grows
by $1.15\times10^{12}$ over $10^5$ steps.

## Claim 2: semi-implicit Euler keeps the energy bounded. Confirmed, with an exact bound.

Semi-implicit Euler exactly conserves a modified energy (verified by
expanding one step):

$$\tilde H = v^2 + \omega^2 x^2 - h\,\omega^2\,x\,v .$$

$E$ therefore moves on a fixed ellipse and is confined to

$$\frac{E}{E_0} \in \left[\frac{1}{1 + \omega h / 2},\ \frac{1}{1 - \omega h / 2}\right]
\qquad (\omega h < 2).$$

| $\omega$ | $\omega h$ | predicted range | measured range, $10^6$ steps | max $\lvert \tilde H_n/\tilde H_0 - 1\rvert$, $10^6$ steps |
|---:|---:|---|---|---:|
| 1 | 0.017 | [0.991736, 1.008403] | [0.991736, 1.008403] | $5.5\times10^{-14}$ |
| 5 | 0.083 | [0.960000, 1.043478] | [0.960000, 1.043478] | $6.4\times10^{-14}$ |
| 10 | 0.167 | [0.923077, 1.090909] | [0.923077, 1.090909] | $3.1\times10^{-13}$ |
| 30 | 0.5 | [0.800000, 1.333333] | [0.800000, 1.333333] | $2.0\times10^{-13}$ |
| 60 | 1.0 | [0.666667, 2.000000] | [1.000000, 2.000000] | 0 |

The bound is reached and never exceeded; $\tilde H$ drifts only by rounding.
At $\omega h = 1$ the step is a rotation by exactly $\pi/3$, so the orbit has
period 6 steps and visits only two energy levels, which is why the lower end
of the range is not reached there.

Note what "bounded" means: the energy *error* is $O(\omega h)$, not zero.
At $\omega = 10$ the energy reads 7.7% low or 9.1% high depending on the phase.
For $\omega h \ge 2$ semi-implicit Euler is unstable too.

## The other two schemes

| $\omega$ | velocity Verlet: $E/E_0$ range over $10^5$ steps | old engine: mean per-step factor | old engine: $\det$ of step matrix $1 + \tfrac12\omega^2h^2$ | old engine: overflow step |
|---:|---|---:|---:|---:|
| 1 | [0.999931, 1] | 1.000138889130 | 1.000138888889 | none |
| 5 | [0.998264, 1] | 1.003472199713 | 1.003472222222 | none |
| 10 | [0.993056, 1] | 1.013888475489 | 1.013888888889 | 51,177 |
| 30 | [0.9375, 1] | 1.125004573495 | 1.125000000000 | 5,975 |
| 60 | [0.8125, 1] | 1.499921990304 | 1.500000000000 | 1,732 |

* **Velocity Verlet** is also symplectic and second order: its energy error is
  $O(\omega^2 h^2)$, about 10x smaller than semi-implicit Euler's at
  $\omega = 10$ (0.7% vs 7.7%). It is not used in the engine because it needs
  the forces at the new positions before the velocity update, which does not
  fit a velocity-level contact solver that runs between the two half updates.
  Semi-implicit Euler needs one force evaluation per step and is the standard
  choice for sequential-impulse engines (Box2D does the same).
* **The old engine's scheme** (second-order position, first-order velocity)
  has step-matrix determinant $1 + \tfrac12\omega^2 h^2 > 1$: phase-space area
  grows, and the measured average energy growth per step matches the
  determinant to 4-6 digits. It grows at half the rate of explicit Euler but
  still diverges. This branch replaced it (commit "Use semi-implicit Euler in
  the integrator").

## Part 2: the same oscillator inside the engine

Rest length $10^9$ px so the growing oscillation never makes the two bodies
collide. $10^5$ steps.

| $\omega$ | engine integrator | $E_{100}/E_0$ | predicted for explicit Euler $(1+\omega^2h^2)^{100}$ | $E/E_0$ range over the run | contacts |
|---:|---|---:|---:|---|---:|
| 1 | SemiImplicitEuler (default) | 1.00172627 | | [0.991708, 1.0084] | 0 |
| 1 | ExplicitEuler | 1.02816339 | 1.02816321 | grows to $1.15362\times10^{12}$ | 0 |
| 1 | ConstantAccel (old engine) | 1.0148244 | | grows to $1.07512\times10^{6}$ | 0 |
| 10 | SemiImplicitEuler (default) | 0.932129592 | | [0.923077, 1.09093] | 0 |
| 10 | ExplicitEuler | 15.4853972 | 15.4853965 | overflow at step 1,797 | 16 |
| 10 | ConstantAccel (old engine) | 3.81995735 | | overflow at step 3,365 | 24 |
| 30 | SemiImplicitEuler (default) | 0.876523612 | | [0.799991, 1.33334] | 0 |
| 30 | ExplicitEuler | $4.90909346\times10^{9}$ | $4.90909347\times10^{9}$ | overflow at step 242 | 8 |
| 30 | ConstantAccel (old engine) | 133,701.91 | | overflow at step 457 | 21 |

The engine reproduces the standalone results: explicit Euler matches
$(1+\omega^2h^2)^{100}$ to 7 significant digits (the remaining difference is
the precision of a spring with a $10^9$ px rest length), and semi-implicit
Euler stays inside the standalone bounds. The engine overflows earlier than
the standalone runs because once the amplitude reaches the $10^9$ px rest
length the bodies cross (the contact column), and the spring geometry breaks.

**12-link undamped spring chain** ($k = 400$, $m = 1$, gravity and walls off,
first link kicked sideways, $10^5$ steps):

| engine integrator | $E/E_0$ range | final $E/E_0$ | contacts |
|---|---|---:|---:|
| SemiImplicitEuler (default) | [0.887168, 1.1265] | 0.984436 | 2,161 |
| ExplicitEuler | overflow at step 1,014 | | 21 |
| ConstantAccel (old engine) | overflow at step 1,875 | | 20 |

The chain is a nonlinear system (springs in 2D), and its links touch 2,161
times (e = 1, no friction), so the linear-oscillator bound does not apply. The
energy stayed within $-11\%$ / $+13\%$ of the start over 1,667 s.

## What this does not show

* Bounded energy for *springs*. Contacts are not energy-conserving: the
  sequential-impulse solver, restitution below 1, friction and speculative
  contacts all change energy, independent of the integrator. See
  `docs/BENCHMARKS.md` (conservation section) for measured contact-energy
  numbers.
* Stability for stiff or strongly damped springs. Springs and dampers are
  explicit forces. A uniform chain is stable only when
  $(\omega_{max}\Delta t)^2 + 2\gamma_{max}\Delta t < 4$ with
  $\omega_{max} = 2\sqrt{k/m}$ and $\gamma_{max} = 4c/m$. At $k = 2000$, $c = 20$,
  $m = 1$, $\Delta t = 1/60$ the sum is 4.9 and the measured energy grows 4.5x in
  10 s (test "spring chain (limitation)"). `WorldConfig::substeps` divides the
  step: $n$ substeps allow $n^2$ times stiffer springs at $n$ times the cost (the
  soft-blob scene uses 4).
* Accuracy. Semi-implicit Euler is first order: the energy error at
  $\omega h = 0.167$ is up to 9%. The oscillation period is more accurate:
  measured 1.25627 s against the exact $2\pi\sqrt{m/k} = 1.25664$ s at
  $\omega h = 0.083$ (0.03%, test "spring: period").
