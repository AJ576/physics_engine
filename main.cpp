#include "rigidBody.hpp"
#include "graphics.hpp"
#include <cmath>
#include <cstdio>
#include <iostream>

int main()
{
    
    Graphics graphics(800, 600);

    // Create rigid bodies using constructor
    RigidBody body1(20.0, 2.0, {100.0, 100.0}, {100.0, 100.0});
    RigidBody body2(10.0, 5.0, {300.0, 300.0}, {200.0, 20.0});
    TimeManager TIME;

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

        // 1. Calculate Momentum (p = mv)
        std::array<double, 2> vel1 = body1.getVelocity();
        std::array<double, 2> vel2 = body2.getVelocity();
        double p1x = body1.getMass() * vel1[0];
        double p1y = body1.getMass() * vel1[1];
        double p2x = body2.getMass() * vel2[0];
        double p2y = body2.getMass() * vel2[1];

        // 2. Calculate Kinetic Energy (KE = 0.5 * m * v^2)
     
        double ke1 = 0.5 * body1.getMass() * (vel1[0] * vel1[0] + vel1[1] * vel1[1]);
        double ke2 = 0.5 * body2.getMass() * (vel2[0] * vel2[0] + vel2[1] * vel2[1]);

        double totalPX = p1x + p2x;
        double totalPY = p1y + p2y;
        double totalKE = ke1 + ke2;

        std::array<int, 4> white = {255, 255, 255, 255};
        char buf[256];

        // Display Energy 
        snprintf(buf, sizeof(buf), "Total Kinetic Energy: %.2f J", totalKE);
        graphics.drawText(buf, 400, 10, white);

        // Display Momentum 
        snprintf(buf, sizeof(buf), "Total Momentum: x=%.1f y=%.1f", totalPX, totalPY);
        graphics.drawText(buf, 400, 35, white);

        // Present the frame
        graphics.present();


    }

    return 0;
}