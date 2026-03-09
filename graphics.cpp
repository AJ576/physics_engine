#include "graphics.hpp"
#include <iostream>

Graphics::Graphics(int width, int height) : windowHeight(height), windowWidth(width) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cout << "SDL Error: " << SDL_GetError() << std::endl;
    }
    if (TTF_Init() < 0) {
        std::cout << "TTF Error: " << TTF_GetError() << std::endl;
    }

    window = SDL_CreateWindow("Physics Engine", SDL_WINDOWPOS_CENTERED, 
                          SDL_WINDOWPOS_CENTERED, width, height, SDL_WINDOW_RESIZABLE);
    
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
    
    double radius = body.getRadius();
    std::array<double, 2> position = body.getPosition();
    
    // Midpoint Circle Algorithm (Drawing pixel by pixel)
    int r = static_cast<int>(radius);
    for (int w = -r; w <= r; w++) {
        for (int h = -r; h <= r; h++) {
            if (w*w + h*h <= radius * radius) {
                // Flip Y coordinate so (0,0) is at the bottom
                int screenX = (int)position[0] + w;
                int screenY = windowHeight - ((int)position[1] + h);
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

void Graphics::updateSize(int width, int height)
{
    windowHeight = height;
    windowWidth = width;
}

void Graphics::printPhysicsInfo(const std::vector<RigidBody>& bodies) {
    if (windowWidth < 400 || windowHeight < 200) {
        // Screen too small, don't display
        return;
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

    char buf[256];
    
    // Calculate scale and placement relative to screen size
    // Base resolution 800x600.
    double scale = std::min(windowWidth / 800.0, windowHeight / 600.0);
    int xLeft = (int)(windowWidth * 0.45); // Pushed further left from 0.5 to prevent extending off-screen
    int yTopKE = (int)(windowHeight * 0.010); // 10 / 600 ~ 0.016
    int yTopMo = (int)(windowHeight * 0.050); // 35 / 600 ~ 0.058

    if (!font) return;
    SDL_Color sdlColor = { 255, 255, 255, 255 };

    // Display Energy
    snprintf(buf, sizeof(buf), "Total Kinetic Energy: %.2f J", totalKE);
    SDL_Surface* surfaceKE = TTF_RenderText_Blended(font, buf, sdlColor);
    if (surfaceKE) {
        SDL_Texture* textureKE = SDL_CreateTextureFromSurface(renderer, surfaceKE);
        if (textureKE) {
            SDL_Rect dst = { xLeft, yTopKE, (int)(surfaceKE->w * scale), (int)(surfaceKE->h * scale) };
            SDL_RenderCopy(renderer, textureKE, nullptr, &dst);
            SDL_DestroyTexture(textureKE);
        }
        SDL_FreeSurface(surfaceKE);
    }

    // Display Momentum
    snprintf(buf, sizeof(buf), "Total Momentum: x=%.1f y=%.1f", totalPX, totalPY);
    SDL_Surface* surfaceMo = TTF_RenderText_Blended(font, buf, sdlColor);
    if (surfaceMo) {
        SDL_Texture* textureMo = SDL_CreateTextureFromSurface(renderer, surfaceMo);
        if (textureMo) {
            SDL_Rect dst = { xLeft, yTopMo, (int)(surfaceMo->w * scale), (int)(surfaceMo->h * scale) };
            SDL_RenderCopy(renderer, textureMo, nullptr, &dst);
            SDL_DestroyTexture(textureMo);
        }
        SDL_FreeSurface(surfaceMo);
    }
}