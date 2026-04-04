# Physics Engine

A real-time 2D rigid-body physics sandbox built with C++17, SDL2, and SDL2_ttf.

This project simulates thousands of circular bodies with fixed-timestep integration,
impulse-based collisions, border interactions, runtime gravity control, and live
on-screen metrics (FPS, kinetic energy, potential energy, total energy).

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
- Collision response uses global restitution `e = 0.5`
- Circle radius is derived from mass in `main.cpp`

## Project Layout

| File | Purpose |
|---|---|
| `main.cpp` | App setup, random body generation, input handling, game loop |
| `physics.hpp` / `physics.cpp` | Time manager, world update, collision system, border handling |
| `rigidBody.hpp` / `rigidBody.cpp` | Rigid body state, force integration, speed-based coloring |
| `graphics.hpp` / `graphics.cpp` | SDL window/renderer setup, circle rendering, HUD text |
| `CMakeLists.txt` | Build configuration and SDL dependency linking |

## Performance Tips

- Start with fewer bodies (edit `randomBodyCount` in `main.cpp`) if your machine struggles
- Release builds (`-DCMAKE_BUILD_TYPE=Release`) are much faster than debug builds
- VSync is enabled in renderer creation, so FPS may be display-limited
