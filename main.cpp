#include "physics.hpp"
#include "graphics.hpp"
#include "spring.hpp"
#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>

int main()
{
    Graphics graphics(800, 600);

    // Initialize World Physics with the border
    WorldPhysics world({800.0, 600.0});

<<<<<<< HEAD
    // Seed random number generator
    srand(time(0));
    
    // Number of bodies to create
    int n = 1;
    
    // Generate n random bodies
    for (int i = 0; i < n; i++) {
        // Random mass: 10 to 100
        double mass = 10.0 + (rand() / (double)RAND_MAX) * 90.0;
        
        // Random position: 0 to 600 for both x and y
        double posX = (rand() / (double)RAND_MAX) * 600.0;
        double posY = (rand() / (double)RAND_MAX) * 600.0;
        
        // Random velocity: -500 to 500 for both x and y
        double velX = -5.0 + (rand() / (double)RAND_MAX)*100 ;
        double velY = -5.0 + (rand() / (double)RAND_MAX)*100 ;
        // Make radius proportional to mass
        // radius = sqrt(mass / π) * scale_factor
        double radius = sqrt(mass / M_PI)*2;
        
=======
    // Ground springs: (x, y bottom, width, height, spring constant k)
    world.addSpring(Spring(300, 0, 150, 15, 800.0));
    world.addSpring(Spring(100, 0, 120, 12, 600.0));

    // Few balls, placed above the springs so you can see bounces (y is up, vy < 0 = downward)
    const int n = 4;
    for (int i = 0; i < n; i++) {
        double mass = 30.0 + i * 15.0;
        double radius = sqrt(mass / M_PI) ;
        double posX, posY, velX, velY;
        if (i < 2) {
            // Land on right spring (x ~300–450)
            posX = 330.0 + i * 50.0;
            posY = 220.0 + i * 40.0;
            velX = (i == 0) ? 40.0 : -30.0;
            velY = -180.0;
        } else {
            // Land on left spring (x ~100–220)
            posX = 130.0 + (i - 2) * 45.0;
            posY = 240.0;
            velX = 25.0;
            velY = -160.0;
        }
>>>>>>> main
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

        // Draw springs behind balls
        std::array<int, 4> springColor = {140, 200, 140, 255};
        for (const auto& spring : world.getSprings()) {
            graphics.drawSpring(spring, springColor);
        }

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