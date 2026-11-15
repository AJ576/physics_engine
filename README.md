# Physics Engine

A real-time 2D circle physics sandbox built with C++17, SDL2, and SDL2_ttf.

This project simulates circular bodies with fixed-timestep integration,
impulse-based collisions, a uniform-grid broadphase, border interactions,
runtime gravity control, and live on-screen metrics (FPS, kinetic energy,
potential energy, total energy). Measured step times are in `docs/RESUME_FACTS.md`.

## Current Feature Set (Spring Features Ignored)

- High body-count simulation (default: `5000` circles)
- Circle-circle collision detection and impulse resolution
- Spatial grid broad-phase for faster collision checks
- Fixed `60 Hz` physics stepping via accumulator
- Runtime gravity toggle and direction switching
- Border collision handling with restitution and gravity-aware settling
- Window resize support with physics-border + renderer sync
- Per-body speed-based color gradient rendering
- Real-time HUD: FPS, total KE, total PE, total energy

## Controls

- `Esc` - Quit
- `G` - Toggle gravity ON/OFF
- Arrow keys (`Up`, `Down`, `Left`, `Right`) - Change gravity direction (when gravity is ON)
- Window resize - Updates simulation bounds dynamically

## Dependencies

- `CMake` (3.16+)
- `C++17` compiler (`clang++`/`g++`)
- `SDL2`
- `SDL2_ttf`

### Install Dependencies

#### macOS (Homebrew)

```bash
brew install sdl2 sdl2_ttf cmake
```

#### Ubuntu/Debian

```bash
sudo apt update
sudo apt install -y cmake g++ libsdl2-dev libsdl2-ttf-dev
```

#### Fedora

```bash
sudo dnf install cmake gcc-c++ SDL2-devel SDL2_ttf-devel
```

## Build

From project root:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

## Run

```bash
./build/physics_engine
```

## Notes on Simulation Behavior

- Coordinate system uses physics-space `+Y` upward (renderer flips Y for SDL draw coordinates)
- Gravity is off at startup (`{0, 0}`) and can be enabled at runtime
- Collision response uses global restitution `e = 0.7` (`rigidBody.cpp`)
- Circle radius is derived from mass in `main.cpp`

## Project Layout

| File | Purpose |
|---|---|
| `main.cpp` | App setup, random body generation, input handling, game loop |
| `physics.hpp` / `physics.cpp` | Time manager, world update, collision system, border handling |
| `rigidBody.hpp` / `rigidBody.cpp` | Body state, force integration, speed-based coloring |
| `spring.hpp` / `spring.cpp` | Platform springs (disabled in `main.cpp`) |
| `graphics.hpp` / `graphics.cpp` | SDL window/renderer setup, circle rendering, HUD text |
| `bench/physics_bench.cpp` | Headless benchmark (no SDL) |
| `CMakeLists.txt` | Build configuration |

## Benchmarks

Headless target, no SDL. Release only; Debug is not a performance number.

```bash
cmake -S . -B build_release -DCMAKE_BUILD_TYPE=Release
cmake --build build_release --target physics_bench -j
./build_release/physics_bench --all --out-dir bench/results
```

Useful subsets: `--scaling`, `--grid-vs-brute`, `--conservation`, `--integrator`, `--profile N SECONDS`.

Measured on Apple M4, Apple clang 17, `-O3 -DNDEBUG`, seed 42. Full tables: `docs/RESUME_FACTS.md` and `bench/results/`.

**Default N=5000, 800×600, gravity off:** median mean step **0.431 ± 0.005 ms**.

**Largest N with median mean step ≤ 16.6 ms:** **80000** at constant density (10.92 ± 0.64 ms); **40000** in a fixed 800×600 arena (7.38 ± 0.55 ms).

**Grid vs brute force** (constant density, N=5000): 30.9× step-time (0.416 ms vs 12.87 ms). N=20000: 132× (1.95 ms vs 258 ms).

**Elastic conservation** (100 circles, e=1, no gravity, no wall hits, 5000 steps): relative `|p|` drift ≤ 2.73×10⁻¹⁵, KE drift ≤ 4.51×10⁻¹⁶.

The integrator energy comparison is a **standalone** harmonic oscillator, not WorldPhysics springs. Semi-implicit Euler max relative energy drift 5.53×10⁻²; the engine’s `x += v dt + ½ a dt²` formula is **not** that integrator.

## Performance Tips

- Start with fewer bodies (edit `randomBodyCount` in `main.cpp`) if your machine struggles
- Release builds (`-DCMAKE_BUILD_TYPE=Release`) are much faster than debug builds
- VSync is enabled in renderer creation, so displayed FPS may be display-limited
- Verified step times are from `physics_bench`, not from the SDL window FPS counter
