# Physics Engine

A real-time 2D physics sandbox in C++17 with SDL2. It simulates thousands of
circles with impulse-based collisions, a spatial hash grid for broad-phase
collision detection, and gravity you can switch on and steer at runtime.

The physics core (`physics_core`) is a standalone static library with no SDL
dependency. The GUI, the unit tests and the benchmark suite all link against it.

## Features

- **5,000 circles by default.** Mass is random in 10–100 and radius scales with
  mass (`r = sqrt(m / π) / 2`, so about 0.9–2.8 px).
- **Fixed 60 Hz timestep** using an accumulator, so physics runs at the same
  rate regardless of frame rate. Frame gaps over 0.25 s (e.g. while resizing)
  are clamped so the simulation doesn't jump.
- **Circle-circle collisions** with positional correction (overlap split by
  inverse mass) and an impulse along the contact normal using a coefficient of
  restitution (`e = 0.7` by default).
- **Spatial hash grid broad-phase.** Each step the bodies are bucketed into
  grid cells (default size 10). Each cell is checked against itself and 4 of
  its 8 neighbours, so every pair is tested once.
- **Immovable bodies.** A mass of 0 gives an inverse mass of 0, so the body
  is never moved by collisions.
- **Walls with settling.** Bodies are pushed back inside the window and only
  bounce if moving into a wall. On the gravity axis, bounces use the speed at
  the moment of contact, and slow impacts come to rest instead of hopping
  forever.
- **Runtime gravity**: on/off, and direction (up, down, left, right).
- **Resizable window.** The physics walls follow the window size.
- **Speed-based colouring**: purple (slow) → blue → red (fast).
- **HUD**: render FPS, total kinetic energy, total potential energy and their
  sum.
- **Spring pads** (`spring.hpp`): rectangular pads that launch a ball back up
  with a boosted bounce (up to 2× its incoming speed). They are implemented and
  tested but **turned off in the GUI** (`enableSprings = false` in `main.cpp`).

## Controls

| Key | Action |
|---|---|
| `G` | Toggle gravity on/off (starts off) |
| Arrow keys | Point gravity up/down/left/right (only while gravity is on) |
| `Esc` | Quit |
| Resize window | Moves the physics walls to the new window edges |

## How a Physics Step Works

`WorldPhysics::runPhysics` runs once per 1/60 s:

1. **Forces**: add gravity (`m · g`) to every movable body.
2. **Integrate**: `x += v·dt + ½·a·dt²`, then `v += a·dt`. This is exact for
   constant acceleration, which gravity is.
3. **Build the grid**: hash each body's cell `(floor(x/size), floor(y/size))`
   into a 64-bit key in an `unordered_map`.
4. **Spring pads**: bounce any ball touching a pad (no-op when there are none).
5. **Collisions**: for each cell, test pairs inside it and against its
   neighbours at offsets (1,0), (0,1), (1,1), (1,−1). Overlapping pairs are
   separated, then an impulse is applied if they are moving towards each other.
6. **Walls**: push bodies back inside, bounce or settle.

Units are pixels and seconds; gravity is 980 px/s².

## Build

Dependencies: CMake 3.16+, a C++17 compiler, SDL2 and SDL2_ttf.

```bash
# macOS
brew install sdl2 sdl2_ttf cmake
# Ubuntu/Debian
sudo apt install -y cmake g++ libsdl2-dev libsdl2-ttf-dev
# Fedora
sudo dnf install cmake gcc-c++ SDL2-devel SDL2_ttf-devel
```

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

This builds three programs:

| Target | What it is |
|---|---|
| `build/physics_engine` | The interactive SDL simulation |
| `build/physics_tests` | Unit tests for the physics core |
| `build/benchmark_suite` | Timing benchmarks for the physics step |

## Tests

```bash
./build/physics_tests
# or
(cd build && ctest --output-on-failure)
```

16 tests, written with a small assert framework in `tests/test_physics.cpp`:

- **Integration**: constant velocity, and constant force (`F = m·a`) against
  the exact kinematics formula.
- **Collision detection**: overlap depth and contact normal for touching and
  separated circles.
- **Collision response**:
  - Elastic, equal masses: velocities swap; momentum and energy conserved.
  - Elastic, unequal masses: matches the analytic 1D solution.
  - Inelastic: momentum conserved, energy lost, separation speed = `e ×`
    approach speed.
  - Overlap fully corrected.
  - Immovable body never moves.
- **Walls**:
  - Rebound with restitution.
  - Slow ball settles on the floor.
  - A ball pushed into the floor that is already moving out isn't flipped back
    in.
  - A dropped ball comes to rest exactly on the floor in all four gravity
    directions.
  - A 5,000-body pile under gravity stays inside the box.
- **Spatial grid**: finds at least as many collisions as brute force on a
  random scene.
- **Spring pads**: ball is detected on the pad and launched back up.

## Benchmarks

```bash
./build/benchmark_suite            # standard: 500 to 10,000 bodies
./build/benchmark_suite --quick    # shorter run
./build/benchmark_suite --stress   # up to 20,000 bodies
```

This times **only the physics step** (no rendering) and writes
`BENCHMARK_RESULTS.md` and `BENCHMARK_RESULTS.csv` to the current directory.

Results from one run on an Apple M4 (16 GB), Apple clang 17, Release (`-O3`).
Full tables are in [`BENCHMARK_RESULTS.md`](BENCHMARK_RESULTS.md). Expect some
run-to-run variation.

**Step time vs body count** (gravity on, so bodies pile up on the floor):

| Bodies | Mean step | P95 step |
|---:|---:|---:|
| 1,000 | 0.24 ms | 0.26 ms |
| 2,500 | 0.75 ms | 0.84 ms |
| 5,000 | 1.78 ms | 2.30 ms |
| 10,000 | 4.82 ms | 6.76 ms |

Up to 10,000 bodies, the physics step fits comfortably inside a 16.67 ms
(60 Hz) frame.

**Spatial grid vs brute force** (no gravity, bodies spread out):

| Bodies | Brute force | Grid | Speedup |
|---:|---:|---:|---:|
| 100 | 0.008 ms | 0.021 ms | 0.4× (grid slower) |
| 500 | 0.17 ms | 0.11 ms | 1.6× |
| 1,000 | 0.65 ms | 0.21 ms | 3.0× |
| 5,000 | 19.7 ms | 1.19 ms | 16.6× |

The grid skips about 99.8% of candidate pairs here, but that figure depends on
how spread out the scene is. Below a few hundred bodies, building the grid
costs more than it saves.

**Where the time goes** at 5,000 bodies: collision checks and response about
66%, building the grid about 24%, everything else under 10%.

**Grid cell size**: in the sparse, no-gravity sweep, larger cells (30–40) were
about 2× faster than the default of 10. The default hasn't been retuned
because a dense pile under gravity behaves differently.

## Known Limitations

- **No friction or rotation.** Circles slide along walls forever and don't
  spin.
- **Single-pass collision solver.** Large piles stay inside the box but keep
  jostling instead of coming fully to rest. Proper stacking would need an
  iterative solver.
- **Cell size must be at least the largest diameter**, or the grid will miss
  collisions. The default (10) covers the default bodies (max diameter ≈ 5.6).
- **Energy isn't conserved in the GUI.** With `e = 0.7` every collision loses
  energy, so the HUD's total energy drops over time. That's expected.
- **HUD FPS is capped** by VSync at the display refresh rate. It's the render
  rate, not the physics speed.
- **HUD font is macOS-only.** It loads from macOS system font paths, so on
  Linux the simulation runs but the HUD text won't appear.
- **Bodies spawn in the left 600×600 px** of the 800×600 window with mostly
  up-and-right velocities.

## Project Layout

| Path | Purpose |
|---|---|
| `main.cpp` | Spawns bodies, input handling, main loop |
| `physics.hpp` / `physics.cpp` | Timestep manager, world step, spatial grid, collisions, walls, energy/momentum helpers, profiled step variants |
| `rigidBody.hpp` / `rigidBody.cpp` | Body state, integration, speed-based colour |
| `spring.hpp` / `spring.cpp` | Spring pads |
| `graphics.hpp` / `graphics.cpp` | SDL window, cached circle textures, HUD |
| `tests/test_physics.cpp` | Unit tests |
| `benchmarks/benchmark_suite.cpp` | Benchmark runner |
| `CMakeLists.txt` | Builds `physics_core`, the GUI, the tests and the benchmarks |
