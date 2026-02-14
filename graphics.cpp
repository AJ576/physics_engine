#include "graphics.hpp"
#include <iostream>

Graphics::Graphics(int width, int height) : windowHeight(height) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cout << "SDL Error: " << SDL_GetError() << std::endl;
    }
    if (TTF_Init() < 0) {
        std::cout << "TTF Error: " << TTF_GetError() << std::endl;
    }

    window = SDL_CreateWindow("Physics Engine", SDL_WINDOWPOS_CENTERED, 
                              SDL_WINDOWPOS_CENTERED, width, height, 0);
    
    // The "Accelerated" flag uses your GPU to draw
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    font = TTF_OpenFont("/System/Library/Fonts/Supplemental/Arial.ttf", 24);
    if (!font) {
        font = TTF_OpenFont("/System/Library/Fonts/Helvetica.ttc", 24);
    }
    if (!font) {
        font = TTF_OpenFont("/Library/Fonts/Arial.ttf", 24);
    }
    if (!font) {
        std::cout << "TTF_OpenFont failed: " << TTF_GetError() << std::endl;
    }
}

Graphics::~Graphics() {
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    if (font) TTF_CloseFont(font);
    TTF_Quit();
    SDL_Quit();
}

void Graphics::clear() {
    SDL_SetRenderDrawColor(renderer, 20, 20, 30, 255); // Dark blue background
    SDL_RenderClear(renderer);
}

void Graphics::drawCircle(const RigidBody& body, const std::array<int, 4>& color) {
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

void Graphics::drawText(const std::string& text, int x, int y, const std::array<int, 4>& color) {
    if (!font) return;
    SDL_Color sdlColor = { (Uint8)color[0], (Uint8)color[1], (Uint8)color[2], (Uint8)color[3] };
    SDL_Surface* surface = TTF_RenderText_Blended(font, text.c_str(), sdlColor);
    if (!surface) return;
    SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
    if (!texture) {
        SDL_FreeSurface(surface);
        return;
    }
    SDL_Rect dst = { x, y, surface->w, surface->h };
    SDL_RenderCopy(renderer, texture, nullptr, &dst);
    SDL_DestroyTexture(texture);
    SDL_FreeSurface(surface);
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