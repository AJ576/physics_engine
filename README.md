# Physics Engine

A 2D physics simulation built with C++ and SDL2. It simulates rigid bodies (circles) with gravity, collision detection, and spring-based platforms. The window displays real-time physics stats like total energy and momentum.

## Features

- **Rigid body dynamics** — Circles with mass, velocity, and elastic collisions
- **Spring platforms** — Bouncy ground surfaces that compress under impact
- **Wall collisions** — Balls bounce off the window borders
- **Fixed timestep physics** — 60 Hz simulation for stable, deterministic behavior
- **Resizable window** — Physics and graphics update when you resize

## Dependencies

- **SDL2** — Windowing and rendering
- **SDL2_ttf** — Text rendering for on-screen physics info

### Installing Dependencies

**macOS (Homebrew):**
```bash
brew install sdl2 sdl2_ttf
```

**Ubuntu/Debian:**
```bash
sudo apt install libsdl2-dev libsdl2-ttf-dev
```

**Fedora:**
```bash
sudo dnf install SDL2-devel SDL2_ttf-devel
```

## Building with CMake

### Quick start

```bash
mkdir -p build
cd build
cmake ..
cmake --build .
```

### What this does

1. **`mkdir -p build`** — Creates a build directory so generated files (object files, Makefiles, etc.) stay separate from source code.

2. **`cmake ..`** — Configures the project. CMake finds your compiler, SDL2, SDL2_ttf, and generates native build files (Makefiles on Unix, or project files for IDEs).

3. **`cmake --build .`** — Compiles the project. Only changed files are recompiled on subsequent runs.

### Alternative: one-line build

```bash
cd build && cmake .. && cmake --build .
```

### Clean rebuild

To start from scratch (e.g. after dependency changes):

```bash
rm -rf build
mkdir build && cd build
cmake ..
cmake --build .
```

## Running the engine

After building, run:

```bash
./physics_engine
```

Or from the project root:

```bash
./build/physics_engine
```

### Controls

- **ESC** — Quit
- **Close window** — Quit
- **Resize window** — Physics world resizes; balls bounce off the new borders

## Project structure

| File | Purpose |
|------|---------|
| `main.cpp` | Entry point, main loop, event handling |
| `physics.hpp` / `physics.cpp` | World physics, collision, fixed timestep |
| `rigidBody.hpp` / `rigidBody.cpp` | Circle rigid bodies with mass and velocity |
| `spring.hpp` / `spring.cpp` | Spring platforms for bouncing |
| `graphics.hpp` / `graphics.cpp` | SDL2 rendering and text overlay |
| `CMakeLists.txt` | CMake build configuration |
