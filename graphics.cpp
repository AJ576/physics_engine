#include "graphics.hpp"
#include <algorithm>
#include <iostream>
#include <sstream>
#include <iomanip>

Graphics::Graphics(int width, int height): 
    windowHeight(height), windowWidth(width),fps_(0.0), fpsAccumTime_(0.0), fpsFrameCount_(0), lastCounter_(0)
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
    for (auto& kv : circleCache_) {
        SDL_DestroyTexture(kv.second);
    }
    circleCache_.clear();

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
    int r = std::max(1, (int)std::round(body.getRadius()));
    SDL_Texture* tex = getCircleTexture(r);

    SDL_SetTextureColorMod(tex, (Uint8)color[0], (Uint8)color[1], (Uint8)color[2]);
    SDL_SetTextureAlphaMod(tex, (Uint8)color[3]);

    auto p = body.getPosition();
    SDL_Rect dst;
    dst.w = r * 2;
    dst.h = r * 2;
    dst.x = (int)std::round(p[0]) - r;
    dst.y = windowHeight - ((int)std::round(p[1]) + r); // keep your Y-flip convention

    SDL_RenderCopy(renderer, tex, nullptr, &dst);
}

void Graphics::drawSpring(const Spring& spring, const std::array<int, 4>& color) {
    SDL_SetRenderDrawColor(renderer, (Uint8)color[0], (Uint8)color[1], (Uint8)color[2], (Uint8)color[3]);
    SDL_Rect rect;
    rect.x = (int)std::round(spring.getX());
    // Physics y is up; SDL y is down — match drawCircle convention
    rect.y = windowHeight - (int)std::round(spring.getY() + spring.getHeight());
    rect.w = std::max(1, (int)std::round(spring.getWidth()));
    rect.h = std::max(1, (int)std::round(spring.getHeight()));
    SDL_RenderFillRect(renderer, &rect);
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

void Graphics::printPhysicsInfo(const std::vector<RigidBody>& bodies, double gravityY) {
    if (!font) return;

    // hide info text on very small windows
    if (windowWidth < 320 || windowHeight < 240) return;

    // Compute total kinetic energy
    double totalKE = 0.0;
    double totalPE = 0.0;

    for (const auto& body : bodies) {
        const auto v = body.getVelocity();
        const auto p = body.getPosition();
        const double m = body.getMass();

        totalKE += 0.5 * m * (v[0] * v[0] + v[1] * v[1]);
        totalPE += m * (-gravityY) * p[1];  // PE = m*g*h (assuming g points down)
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

    // KE only
    ss.str("");
    ss.clear();
    ss << "Total KE: " << totalKE;
    drawText(ss.str(), x, y, color);
    y += lineStep; // <-- ADD THIS LINE

    //PE only
    ss.str(""); ss.clear();
    ss << "Total PE: " << totalPE;
    drawText(ss.str(), x, y, color);
    y += lineStep;

    //total
    ss.str(""); ss.clear();
    ss << "Total E: " << (totalKE + totalPE);
    drawText(ss.str(), x, y, color);
}

SDL_Texture* Graphics::createCircleTexture(int radius) {
    int d = radius * 2;
    SDL_Texture* tex = SDL_CreateTexture(
        renderer,
        SDL_PIXELFORMAT_RGBA8888,
        SDL_TEXTUREACCESS_TARGET,
        d, d
    );
    SDL_SetTextureBlendMode(tex, SDL_BLENDMODE_BLEND);

    SDL_Texture* oldTarget = SDL_GetRenderTarget(renderer);
    SDL_SetRenderTarget(renderer, tex);

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
    SDL_RenderClear(renderer);

    // draw filled white circle into texture
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    for (int y = -radius; y <= radius; ++y) {
        int xSpan = (int)std::sqrt(radius * radius - y * y);
        SDL_RenderDrawLine(renderer, radius - xSpan, radius + y, radius + xSpan, radius + y);
    }

    SDL_SetRenderTarget(renderer, oldTarget);
    return tex;
}

SDL_Texture* Graphics::getCircleTexture(int radius) {
    auto it = circleCache_.find(radius);
    if (it != circleCache_.end()) return it->second;

    SDL_Texture* tex = createCircleTexture(radius);
    circleCache_[radius] = tex;
    return tex;
}