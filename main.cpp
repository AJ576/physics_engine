#include "physics.hpp"
#include "graphics.hpp"
#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <chrono>

int main()
{
    Graphics graphics(800, 600);

    // Initialize World Physics with the border
    WorldPhysics world({800.0, 600.0});

    // Seed random number generator
    srand(time(0));
    
    // Number of bodies to create
    int n = 10000;
    
    // Generate n random bodies
    for (int i = 0; i < n; i++) {
        // Random mass: 10 to 100
        double mass = 10.0 + (rand() / (double)RAND_MAX) * 90.0;
        
        // Random position: 0 to 600 for both x and y
        double posX = (rand() / (double)RAND_MAX) * 600.0;
        double posY = (rand() / (double)RAND_MAX) * 600.0;
        
        // Random velocity: -500 to 500 for both x and y
        double velX = -5.0 + (rand() / (double)RAND_MAX)*500 ;
        double velY = -5.0 + (rand() / (double)RAND_MAX)*500 ;
        // Make radius proportional to mass
        // radius = sqrt(mass / π) * scale_factor
        double radius = sqrt(mass / M_PI)/2;
        
        world.addBody(RigidBody(radius, mass, {posX, posY}, {velX, velY}));
    }
   
    TimeManager TIME;

    bool running = true;
    SDL_Event event;

    bool gravityOn = false;
    world.setGravity({0.0, 0.0});

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

            // <-- ADD THIS BLOCK
            if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_g) {
                gravityOn = !gravityOn;
                world.setGravity(gravityOn ? std::array<double, 2>{0.0, -980.0}
                                           : std::array<double, 2>{0.0, 0.0});
                std::cout << "Gravity: " << (gravityOn ? "ON" : "OFF") << std::endl;
            }
            // <-- END ADD

            // --- ADD THIS BLOCK FOR RESIZE ---
            if (event.type == SDL_WINDOWEVENT) {
                if (event.window.event == SDL_WINDOWEVENT_RESIZED) {
                    int newW = event.window.data1;
                    int newH = event.window.data2;

                    // 1. Update Physics borders so balls bounce off new walls
                    world.setBorder({(double)newW, (double)newH});

                    // 2. Update Graphics height so the "Y-flip" math stays correct
                    graphics.updateSize(newW, newH);

                    std::cout << "Resized to: " << newW << "x" << newH << std::endl;
                }
                TIME.reset();
            }
        }

        TIME.tick();

        while(TIME.physicsTime())
        {
            world.runPhysics(TIME);
        }

        // Clear screen
        graphics.clear();

        // Get bodies reference for rendering and calculations
        const std::vector<RigidBody>& bodies = world.getBodies();

        std::vector<std::array<int, 4>> colors = {{255,0,0,255}, {0,0,255,255}, {0,255,0,255}, {255,255,0,255}};
        for (size_t i = 0; i < bodies.size(); i++) {
            graphics.drawCircle(bodies[i], colors[i%colors.size()]);
        }

        // Display informational text (Energy, Momentum, etc.)
        graphics.printPhysicsInfo(bodies,world.getGravity()[1]);

        // Present the frame
        graphics.present();
    }

    return 0;
}