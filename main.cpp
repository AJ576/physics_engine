#include "rigidBody.hpp"
#include "graphics.hpp"
#include <cmath>
#include <cstdio>
#include <iostream>

int main()
{
    
    Graphics graphics(800, 600);

    RigidBody body1;
    RigidBody body2;
    TimeManager TIME;

    body1.setRadius(20.0f);  
    body1.setMass(2.0f);
    body1.setPosition({100.0f, 100.0f});  
    body1.setVelocity({10.0f, 10.0f});
    body1.setAcceleration({0.0f, 0.0f});
    body1.setForce({0.0f, 0.0f});

    body2.setRadius(20.0f);  
    body2.setMass(2.0f);
    body2.setPosition({300.0f, 300.0f});
    body2.setVelocity({-10.0f, -10.0f});
    body2.setAcceleration({0.0f, 0.0f});
    body2.setForce({0.0f, 0.0f});

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
        std::array<float, 2> vel1 = body1.getVelocity();
        std::array<float, 2> vel2 = body2.getVelocity();
        float p1x = body1.getMass() * vel1[0];
        float p1y = body1.getMass() * vel1[1];
        float p2x = body2.getMass() * vel2[0];
        float p2y = body2.getMass() * vel2[1];

        // 2. Calculate Kinetic Energy (KE = 0.5 * m * v^2)
     
        float ke1 = 0.5f * body1.getMass() * (vel1[0] * vel1[0] + vel1[1] * vel1[1]);
        float ke2 = 0.5f * body2.getMass() * (vel2[0] * vel2[0] + vel2[1] * vel2[1]);

        float totalPX = p1x + p2x;
        float totalPY = p1y + p2y;
        float totalKE = ke1 + ke2;

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