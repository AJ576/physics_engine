#include "graphics.hpp"
#include <iostream>

Graphics::Graphics(int width, int height) : windowHeight(height) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cout << "SDL Error: " << SDL_GetError() << std::endl;
    }

    window = SDL_CreateWindow("Physics Engine", SDL_WINDOWPOS_CENTERED, 
                              SDL_WINDOWPOS_CENTERED, width, height, 0);
    
    // The "Accelerated" flag uses your GPU to draw
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
}

Graphics::~Graphics() {
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
}

void Graphics::clear() {
    SDL_SetRenderDrawColor(renderer, 20, 20, 30, 255); // Dark blue background
    SDL_RenderClear(renderer);
}

void Graphics::drawCircle(const RigidBody& body, std::array<int, 4>& color) {
    SDL_SetRenderDrawColor(renderer, color[0], color[1], color[2], color[3]); // White ball
    
    // Midpoint Circle Algorithm (Drawing pixel by pixel)
    for (int w = -body.radius; w < body.radius; w++) {
        for (int h = -body.radius; h < body.radius; h++) {
            if (w*w + h*h <= body.radius * body.radius) {
                // Flip Y coordinate so (0,0) is at the bottom
                int screenX = (int)body.position[0] + w;
                int screenY = windowHeight - ((int)body.position[1] + h);
                SDL_RenderDrawPoint(renderer, screenX, screenY);
            }
        }
    }
}

void Graphics::present() {
    SDL_RenderPresent(renderer);
}

bool Graphics::processEvents() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_QUIT) return false;
    }
    return true;
}