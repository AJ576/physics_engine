// Interactive demo. Keys:
//   1 pile   2 Newton's cradle   3 spring chain   4 soft blob   5 stress (5,000 bodies)
//   G gravity on/off   arrows: gravity direction   O grid overlay   T 1/4 broadphase threads
//   Space pause   R reset scene   Esc quit
// Options: --scene N (start scene, 1-5), --frames K (quit after K frames; for
// smoke tests and recordings; one fixed step per frame), --screenshot FILE.bmp
// (save the last frame).
#include "graphics.hpp"
#include "physics/scenes.hpp"
#include "physics/timestep.hpp"
#include "physics/world.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace {

const char* kSceneNames[] = {"", "pile (2,000)", "Newton's cradle", "spring chain", "soft blob", "stress (5,000, no gravity)"};

phys::World makeScene(int id) {
    switch (id) {
        case 1: return phys::scenes::pile(2000, 1);
        case 2: return phys::scenes::newtonsCradle();
        case 3: return phys::scenes::springChain();
        case 4: return phys::scenes::softBlob();
        default: return phys::scenes::stress(5000, 42);
    }
}

// Speed colour: purple (slow) -> blue -> red (>= 500 px/s).
Color speedColor(const phys::Body& b) {
    if (b.isStatic()) return {150, 150, 150, 255};
    const double t = std::min(b.vel.length() / 500.0, 1.0);
    auto lerp = [](int a, int c, double u) { return static_cast<int>(std::lround(a + (c - a) * u)); };
    if (t <= 0.5) {
        const double u = t / 0.5;
        return {lerp(170, 40, u), lerp(60, 190, u), 255, 255};
    }
    const double u = (t - 0.5) / 0.5;
    return {lerp(40, 255, u), lerp(190, 40, u), lerp(255, 40, u), 255};
}

std::string fmt(const char* f, double a, double b = 0.0, double c = 0.0) {
    char buf[160];
    std::snprintf(buf, sizeof buf, f, a, b, c);
    return buf;
}

}  // namespace

int main(int argc, char** argv) {
    int sceneId = 1;
    long maxFrames = -1;
    std::string screenshot;
    for (int i = 1; i + 1 < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--scene") sceneId = std::clamp(std::atoi(argv[++i]), 1, 5);
        else if (a == "--frames") maxFrames = std::atol(argv[++i]);
        else if (a == "--screenshot") screenshot = argv[++i];
    }
    Graphics gfx(800, 600);
    phys::World world = makeScene(sceneId);
    phys::FixedTimestep timestep(world.config().dt);
    bool running = true, paused = false, grid = false;
    int threads = 1;
    phys::Vec2 gravityDir{0.0, -980.0};
    bool gravityOn = true;
    double stepMs = 0.0;  // smoothed physics time per step
    auto last = std::chrono::steady_clock::now();
    auto lastPrint = last;

    auto load = [&](int id) {
        sceneId = id;
        world = makeScene(id);
        world.config().threads = threads;
        gravityOn = world.config().gravity.lengthSq() > 0.0;
        if (gravityOn) world.config().gravity = gravityDir;
        timestep.reset();
    };
    load(sceneId);

    SDL_Event ev;
    long frame = 0;
    while (running && (maxFrames < 0 || frame++ < maxFrames)) {
        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_QUIT) running = false;
            if (ev.type == SDL_KEYDOWN) {
                const SDL_Keycode k = ev.key.keysym.sym;
                if (k == SDLK_ESCAPE) running = false;
                if (k >= SDLK_1 && k <= SDLK_5) load(static_cast<int>(k - SDLK_0));
                if (k == SDLK_r) load(sceneId);
                if (k == SDLK_SPACE) paused = !paused;
                if (k == SDLK_o) grid = !grid;
                if (k == SDLK_t) world.config().threads = threads = (threads == 1 ? 4 : 1);
                if (k == SDLK_g) gravityOn = !gravityOn;
                if (k == SDLK_UP) gravityDir = {0.0, 980.0};
                if (k == SDLK_DOWN) gravityDir = {0.0, -980.0};
                if (k == SDLK_LEFT) gravityDir = {-980.0, 0.0};
                if (k == SDLK_RIGHT) gravityDir = {980.0, 0.0};
                world.config().gravity = gravityOn ? gravityDir : phys::Vec2{};
            }
            if (ev.type == SDL_WINDOWEVENT && ev.window.event == SDL_WINDOWEVENT_RESIZED) {
                world.config().width = ev.window.data1;
                world.config().height = ev.window.data2;
                gfx.updateSize(ev.window.data1, ev.window.data2);
                timestep.reset();
            }
        }

        const auto now = std::chrono::steady_clock::now();
        // With --frames, run exactly one fixed step per frame (deterministic,
        // independent of how fast frames render): for smoke tests and recordings.
        const double frameTime = maxFrames >= 0 ? world.config().dt : std::chrono::duration<double>(now - last).count();
        timestep.advance(paused ? 0.0 : frameTime);
        last = now;
        while (timestep.consume()) {
            const auto t0 = std::chrono::steady_clock::now();
            world.step();
            const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            stepMs = stepMs == 0.0 ? ms : 0.9 * stepMs + 0.1 * ms;
        }

        gfx.clear();
        if (grid) gfx.drawGrid(world.config().cellSize, {60, 60, 80, 255});
        const auto& bodies = world.bodies();
        for (const auto& s : world.springs())
            gfx.drawLine(bodies[static_cast<std::size_t>(s.a)].pos, bodies[static_cast<std::size_t>(s.b)].pos, {120, 200, 120, 255});
        for (const auto& j : world.distanceConstraints())
            gfx.drawLine(bodies[static_cast<std::size_t>(j.a)].pos, bodies[static_cast<std::size_t>(j.b)].pos, {200, 200, 200, 255});
        for (const auto& b : bodies) gfx.drawCircle(b, speedColor(b));

        const double ke = world.kineticEnergy(), pe = world.potentialEnergy();
        const auto& st = world.lastStepStats();
        std::vector<std::string> hud = {
            std::string("scene ") + std::to_string(sceneId) + ": " + kSceneNames[sceneId] + (paused ? "  [paused]" : ""),
            fmt("bodies %.0f   contacts %.0f   threads %.0f", static_cast<double>(bodies.size()),
                static_cast<double>(st.contacts + st.wallContacts), threads),
            fmt("physics step %.3f ms   render %.0f fps", stepMs, gfx.fps()),
            fmt("kinetic %.4g   potential %.4g", ke, pe),
            fmt("total energy %.6g", ke + pe),
            std::string("gravity ") + (gravityOn ? "on" : "off") + (grid ? "   grid overlay on" : ""),
            "1-5 scene  G gravity  arrows dir  O grid  T threads  Space pause  R reset",
        };
        gfx.drawHud(hud);
        if (!gfx.hasFont() && now - lastPrint > std::chrono::seconds(1)) {
            lastPrint = now;
            for (const auto& l : hud) std::cout << l << "\n";
            std::cout << "\n";
        }
        if (!screenshot.empty() && maxFrames >= 0 && frame == maxFrames) gfx.saveScreenshot(screenshot);
        gfx.present();
    }
    return 0;
}
