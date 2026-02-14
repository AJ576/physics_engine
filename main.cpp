#include "rigidBody.hpp"
#include "graphics.hpp"
#include <iostream>

int main()
{
    // Initialize Graphics (800x600 window)
    Graphics graphics(800, 600);

    RigidBody body1;
    RigidBody body2;
    TimeManager TIME;

    body1.radius = 10.0f;  // Increased for visibility
    body1.mass = 1.0f;
    body1.position = {100.0f, 100.0f};  // Starting position in pixels
    body1.velocity = {30.0f, 30.0f};
    body1.acceleration = {0.0f, 0.0f};
    body1.force = {0.0f, 0.0f};

    body2.radius = 15.0f;  // Larger radius
    body2.mass = 5.0f;
    body2.position = {300.0f, 300.0f};
    body2.velocity = {-30.0f, -30.0f};
    body2.acceleration = {0.0f, 0.0f};
    body2.force = {0.0f, 0.0f};

    bool running = true;
    SDL_Event event;

    while (running)
    {
        // Handle events
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            }
            if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE) {
                running = false;
            }
        }

        TIME.tick();

        while(TIME.physicsTime())
        {
            runPhysics(body1, body2, TIME);
        }

        // Clear screen
        graphics.clear();

        // Draw body1 in red
        std::array<int, 4> red = {255, 0, 0, 255};
        graphics.drawCircle(body1, red);

        // Draw body2 in blue
        std::array<int, 4> blue = {0, 0, 255, 255};
        graphics.drawCircle(body2, blue);

        // Present the frame
        graphics.present();

        // Small delay to control frame rate
        SDL_Delay(16);  // ~60 FPS
    }

    return 0;
}