#include "graphics.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

Graphics::Graphics(int width, int height) : width_(width), height_(height) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) std::cerr << "SDL error: " << SDL_GetError() << "\n";
    if (TTF_Init() < 0) std::cerr << "TTF error: " << TTF_GetError() << "\n";
    window_ = SDL_CreateWindow("Physics Engine", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width, height,
                               SDL_WINDOW_RESIZABLE);
    renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!renderer_) renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_SOFTWARE);  // headless / no GPU
    if (!renderer_) std::cerr << "SDL_CreateRenderer failed: " << SDL_GetError() << "\n";
    SDL_SetRenderDrawBlendMode(renderer_, SDL_BLENDMODE_BLEND);
    const char* fonts[] = {"/System/Library/Fonts/Supplemental/Arial.ttf", "/System/Library/Fonts/Helvetica.ttc",
                           "/Library/Fonts/Arial.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
                           "/usr/share/fonts/TTF/DejaVuSans.ttf", "/usr/share/fonts/dejavu/DejaVuSans.ttf",
                           "C:\\Windows\\Fonts\\arial.ttf"};
    for (const char* f : fonts)
        if ((font_ = TTF_OpenFont(f, 15))) break;
    if (!font_) std::cerr << "No font found: the HUD is printed to the terminal once per second instead.\n";
    lastCounter_ = SDL_GetPerformanceCounter();
}

Graphics::~Graphics() {
    for (auto& kv : circleCache_) SDL_DestroyTexture(kv.second);
    if (font_) TTF_CloseFont(font_);
    SDL_DestroyRenderer(renderer_);
    SDL_DestroyWindow(window_);
    TTF_Quit();
    SDL_Quit();
}

void Graphics::clear() {
    SDL_SetRenderDrawColor(renderer_, 20, 20, 30, 255);
    SDL_RenderClear(renderer_);
}

void Graphics::drawCircle(const phys::Body& body, const Color& color) {
    const int r = std::max(1, static_cast<int>(std::lround(body.radius)));
    SDL_Texture* tex = circleTexture(r);
    SDL_SetTextureColorMod(tex, static_cast<Uint8>(color[0]), static_cast<Uint8>(color[1]), static_cast<Uint8>(color[2]));
    SDL_SetTextureAlphaMod(tex, static_cast<Uint8>(color[3]));
    SDL_Rect dst{sx(body.pos.x) - r, sy(body.pos.y) - r, 2 * r, 2 * r};
    SDL_RenderCopy(renderer_, tex, nullptr, &dst);
    if (r >= 4) {  // spin marker: a radius drawn at the body's angle
        const phys::Vec2 tip = body.pos + phys::Vec2{std::cos(body.angle), std::sin(body.angle)} * body.radius;
        drawLine(body.pos, tip, {20, 20, 30, 255});
    }
}

void Graphics::drawLine(phys::Vec2 a, phys::Vec2 b, const Color& c) {
    SDL_SetRenderDrawColor(renderer_, static_cast<Uint8>(c[0]), static_cast<Uint8>(c[1]), static_cast<Uint8>(c[2]),
                           static_cast<Uint8>(c[3]));
    SDL_RenderDrawLine(renderer_, sx(a.x), sy(a.y), sx(b.x), sy(b.y));
}

void Graphics::drawGrid(double cell, const Color& c) {
    if (cell < 2.0) return;
    SDL_SetRenderDrawColor(renderer_, static_cast<Uint8>(c[0]), static_cast<Uint8>(c[1]), static_cast<Uint8>(c[2]),
                           static_cast<Uint8>(c[3]));
    for (double x = 0.0; x <= width_; x += cell) SDL_RenderDrawLine(renderer_, sx(x), 0, sx(x), height_);
    for (double y = 0.0; y <= height_; y += cell) SDL_RenderDrawLine(renderer_, 0, sy(y), width_, sy(y));
}

void Graphics::drawText(const std::string& text, int x, int y, const Color& c) {
    if (!font_ || text.empty()) return;
    SDL_Color col{static_cast<Uint8>(c[0]), static_cast<Uint8>(c[1]), static_cast<Uint8>(c[2]), static_cast<Uint8>(c[3])};
    SDL_Surface* surface = TTF_RenderText_Blended(font_, text.c_str(), col);
    if (!surface) return;
    if (SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer_, surface)) {
        SDL_Rect dst{x, y, surface->w, surface->h};
        SDL_RenderCopy(renderer_, texture, nullptr, &dst);
        SDL_DestroyTexture(texture);
    }
    SDL_FreeSurface(surface);
}

void Graphics::drawHud(const std::vector<std::string>& lines) {
    if (!font_) return;
    const int lineH = TTF_FontLineSkip(font_);
    SDL_Rect bg{6, 6, 330, lineH * static_cast<int>(lines.size()) + 8};
    SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 150);
    SDL_RenderFillRect(renderer_, &bg);
    int y = 10;
    for (const auto& l : lines) {
        drawText(l, 12, y, {235, 235, 235, 255});
        y += lineH;
    }
}

void Graphics::present() {
    const Uint64 now = SDL_GetPerformanceCounter();
    fpsTime_ += static_cast<double>(now - lastCounter_) / static_cast<double>(SDL_GetPerformanceFrequency());
    lastCounter_ = now;
    if (++fpsFrames_, fpsTime_ >= 0.5) {
        fps_ = fpsFrames_ / fpsTime_;
        fpsFrames_ = 0;
        fpsTime_ = 0.0;
    }
    SDL_RenderPresent(renderer_);
}

void Graphics::updateSize(int width, int height) {
    width_ = width;
    height_ = height;
}

SDL_Texture* Graphics::circleTexture(int radius) {
    auto it = circleCache_.find(radius);
    if (it != circleCache_.end()) return it->second;
    const int d = radius * 2;
    SDL_Texture* tex = SDL_CreateTexture(renderer_, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET, d, d);
    SDL_SetTextureBlendMode(tex, SDL_BLENDMODE_BLEND);
    SDL_Texture* old = SDL_GetRenderTarget(renderer_);
    SDL_SetRenderTarget(renderer_, tex);
    SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 0);
    SDL_RenderClear(renderer_);
    SDL_SetRenderDrawColor(renderer_, 255, 255, 255, 255);
    for (int y = -radius; y <= radius; ++y) {
        const int span = static_cast<int>(std::sqrt(static_cast<double>(radius * radius - y * y)));
        SDL_RenderDrawLine(renderer_, radius - span, radius + y, radius + span, radius + y);
    }
    SDL_SetRenderTarget(renderer_, old);
    circleCache_[radius] = tex;
    return tex;
}

bool Graphics::saveScreenshot(const std::string& path) {
    SDL_Surface* shot = SDL_CreateRGBSurfaceWithFormat(0, width_, height_, 32, SDL_PIXELFORMAT_ARGB8888);
    if (!shot) return false;
    if (!renderer_) {
        SDL_FreeSurface(shot);
        return false;
    }
    const bool ok = SDL_RenderReadPixels(renderer_, nullptr, SDL_PIXELFORMAT_ARGB8888, shot->pixels, shot->pitch) == 0 &&
                    SDL_SaveBMP(shot, path.c_str()) == 0;
    SDL_FreeSurface(shot);
    return ok;
}
