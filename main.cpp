#include "rigidBody.hpp"
#include "graphics.hpp"
#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>

int main()
{
    
    Graphics graphics(800, 600);

    // Bodies are now stored in a vector for easier management.
    std::vector<RigidBody> bodies;

    bodies.push_back(RigidBody(20.0, 2.0, {100.0, 100.0}, {100.0, 100.0}));
    bodies.push_back(RigidBody(10.0, 5.0, {300.0, 300.0}, {200.0, 20.0}));
    bodies.push_back(RigidBody(15.0, 3.0, {400.0, 200.0}, {-50.0, 80.0}));  
   
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
            runPhysics(bodies, TIME);
        }

        // Clear screen
        graphics.clear();

        std::vector<std::array<int, 4>> colors = {{255,0,0,255}, {0,0,255,255}, {0,255,0,255}, {255,255,0,255}};
        for (size_t i = 0; i < bodies.size(); i++) {
            graphics.drawCircle(bodies[i], colors[i%colors.size()]);
        }

        // 1. Calculate Momentum (p = mv)
        // Calculate total momentum for each body and sum them up.
        double totalPX = 0.0;
        double totalPY = 0.0;
        for (size_t i = 0; i < bodies.size(); i++) {
            totalPX += bodies[i].getMass() * bodies[i].getVelocity()[0];
            totalPY += bodies[i].getMass() * bodies[i].getVelocity()[1];
        }

        // 2. Calculate Kinetic Energy (KE = 0.5 * m * v^2)
        // Calculate total kinetic energy for each body and sum them up.
        double totalKE = 0.0;
        for (size_t i = 0; i < bodies.size(); i++) {
            totalKE += 0.5 * bodies[i].getMass() * (bodies[i].getVelocity()[0] * bodies[i].getVelocity()[0] + bodies[i].getVelocity()[1] * bodies[i].getVelocity()[1]);
        }

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