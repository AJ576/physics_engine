#include "graphics.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>

Graphics::Graphics(int width, int height)
    : windowHeight(height), windowWidth(width),
      fps_(0.0), fpsAccumTime_(0.0), fpsFrameCount_(0), lastCounter_(0)
{
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cout << "SDL Error: " << SDL_GetError() << std::endl;
    }
    if (TTF_Init() < 0) {
        std::cout << "TTF Error: " << TTF_GetError() << std::endl;
    }

    window = SDL_CreateWindow("Physics Engine", SDL_WINDOWPOS_CENTERED, 
                          SDL_WINDOWPOS_CENTERED, width, height, SDL_WINDOW_RESIZABLE);
    
    // The "Accelerated" flag uses your GPU to draw
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

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

    lastCounter_ = SDL_GetPerformanceCounter();
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
    // FPS update happens once per rendered frame
    Uint64 now = SDL_GetPerformanceCounter();
    double dt = static_cast<double>(now - lastCounter_) /
                static_cast<double>(SDL_GetPerformanceFrequency());
    lastCounter_ = now;

    fpsFrameCount_++;
    fpsAccumTime_ += dt;

    // this basically prevents fps changes every milisec
    // we average out fps over 0.5 to 1sec
    if (fpsAccumTime_ >= 0.5) {
        fps_ = static_cast<double>(fpsFrameCount_) / fpsAccumTime_;
        fpsFrameCount_ = 0;
        fpsAccumTime_ = 0.0;
    }

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
    if (!font) return;

    // hide info text on very small windows
    if (windowWidth < 320 || windowHeight < 240) return;

    // Compute total kinetic energy and total momentum
    double totalKE = 0.0;
    double totalPX = 0.0;
    double totalPY = 0.0;

    for (const auto& body : bodies) {
        const auto v = body.getVelocity();
        const double m = body.getMass();

        totalKE += 0.5 * m * (v[0] * v[0] + v[1] * v[1]);
        totalPX += m * v[0];
        totalPY += m * v[1];
    }

    // Relative placement
    int x = static_cast<int>(windowWidth * 0.02);
    int y = static_cast<int>(windowHeight * 0.02);
    int lineStep = std::max(16, static_cast<int>(windowHeight * 0.04));

    std::array<int, 4> color = {255, 255, 255, 255};

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);

    // FPS
    ss.str("");
    ss.clear();
    ss << "FPS: " << fps_;
    drawText(ss.str(), x, y, color);
    y += lineStep;

    // KE
    ss.str("");
    ss.clear();
    ss << "Total KE: " << totalKE;
    drawText(ss.str(), x, y, color);
    y += lineStep;

    // Momentum
    ss.str("");
    ss.clear();
    ss << "Total Px: " << totalPX;
    drawText(ss.str(), x, y, color);
    y += lineStep;

    ss.str("");
    ss.clear();
    ss << "Total Py: " << totalPY;
    drawText(ss.str(), x, y, color);
}