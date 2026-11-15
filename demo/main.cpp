#include "graphics.hpp"
#include "physics/scene.hpp"
#include "physics/timestep.hpp"
#include "physics/world.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>

namespace {
// Speed-based colour (was computed inside the physics core on every velocity
// write): purple (slow) -> blue -> red (>= 500 px/s).
std::array<int, 4> speedColor(const phys::Body& b) {
    const double t = std::min(b.vel.length() / 500.0, 1.0);
    auto lerp = [](int a, int c, double u) { return static_cast<int>(std::lround(a + (c - a) * u)); };
    if (t <= 0.5) {
        const double u = t / 0.5;
        return {lerp(170, 40, u), lerp(60, 190, u), lerp(255, 255, u), 255};
    }
    const double u = (t - 0.5) / 0.5;
    return {lerp(40, 255, u), lerp(190, 40, u), lerp(255, 40, u), 255};
}
}  // namespace

int main() {
    Graphics graphics(800, 600);

    phys::WorldConfig cfg;
    cfg.gravity = {0.0, 0.0};
    phys::World world(cfg);
    const auto seed = static_cast<std::uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());
    for (const auto& b : phys::makeFixedArenaScene(5000, seed).bodies)
        world.addBody(phys::makeBody(b.radius, b.mass, {b.x, b.y}, {b.vx, b.vy}));

    phys::FixedTimestep timestep(cfg.dt);
    bool gravityOn = false;
    phys::Vec2 gravityDirection{0.0, -980.0};
    auto last = std::chrono::steady_clock::now();
    bool running = true;
    SDL_Event event;

    while (running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) running = false;
            if (event.type == SDL_KEYDOWN) {
                switch (event.key.keysym.sym) {
                    case SDLK_ESCAPE: running = false; break;
                    case SDLK_g:
                        gravityOn = !gravityOn;
                        std::cout << "Gravity: " << (gravityOn ? "ON" : "OFF") << std::endl;
                        break;
                    case SDLK_UP: gravityDirection = {0.0, 980.0}; break;
                    case SDLK_DOWN: gravityDirection = {0.0, -980.0}; break;
                    case SDLK_LEFT: gravityDirection = {-980.0, 0.0}; break;
                    case SDLK_RIGHT: gravityDirection = {980.0, 0.0}; break;
                    default: break;
                }
                world.config().gravity = gravityOn ? gravityDirection : phys::Vec2{0.0, 0.0};
            }
            if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_RESIZED) {
                world.config().width = event.window.data1;
                world.config().height = event.window.data2;
                graphics.updateSize(event.window.data1, event.window.data2);
                timestep.reset();  // only on resize (was: on every window event)
            }
        }

        const auto now = std::chrono::steady_clock::now();
        timestep.advance(std::chrono::duration<double>(now - last).count());
        last = now;
        while (timestep.consume()) world.step();

        graphics.clear();
        for (const auto& b : world.bodies()) graphics.drawCircle(b, speedColor(b));
        graphics.printPhysicsInfo(world.kineticEnergy(), world.potentialEnergy());
        graphics.present();
    }
    return 0;
}
