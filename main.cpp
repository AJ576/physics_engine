#include "rigidBody.hpp"
#include "graphics.hpp"
#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>

int main()
{
    
    Graphics graphics(800, 600);

    // Bodies are now stored in a vector for easier management.
    std::vector<RigidBody> bodies;

    // Seed random number generator
    srand(time(0));
    
    // Number of bodies to create
    int n = 100;
    
    // Generate n random bodies
    for (int i = 0; i < n; i++) {
        // Random mass: 10 to 100
        double mass = 10.0 + (rand() / (double)RAND_MAX) * 90.0;
        
        // Random position: 0 to 600 for both x and y
        double posX = (rand() / (double)RAND_MAX) * 600.0;
        double posY = (rand() / (double)RAND_MAX) * 600.0;
        
        // Random velocity: -500 to 500 for both x and y
        double velX = -500.0 + (rand() / (double)RAND_MAX) * 1000.0;
        double velY = -500.0 + (rand() / (double)RAND_MAX) * 1000.0;
        
        // Make radius proportional to mass
        // radius = sqrt(mass / π) * scale_factor
        double radius = sqrt(mass / M_PI)*2;
        
        bodies.push_back(RigidBody(radius, mass, {posX, posY}, {velX, velY}));
    }
   
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