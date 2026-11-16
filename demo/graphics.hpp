#pragma once
// SDL2 rendering for the demo. Physics uses y up; the window uses y down.
#include "physics/world.hpp"

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#include <array>
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

using Color = std::array<int, 4>;

class Graphics {
public:
    Graphics(int width, int height);
    ~Graphics();
    Graphics(const Graphics&) = delete;
    Graphics& operator=(const Graphics&) = delete;

    void clear();
    void drawCircle(const phys::Body& body, const Color& color);  // filled disc plus a spin marker
    void drawLine(phys::Vec2 a, phys::Vec2 b, const Color& color);
    void drawGrid(double cellSize, const Color& color);
    void drawText(const std::string& text, int x, int y, const Color& color);
    void drawHud(const std::vector<std::string>& lines);
    void present();
    bool saveScreenshot(const std::string& bmpPath);
    void updateSize(int width, int height);
    double fps() const { return fps_; }
    bool hasFont() const { return font_ != nullptr; }

private:
    SDL_Texture* circleTexture(int radius);
    int sx(double x) const { return static_cast<int>(std::lround(x)); }
    int sy(double y) const { return height_ - static_cast<int>(std::lround(y)); }

    SDL_Window* window_ = nullptr;
    SDL_Renderer* renderer_ = nullptr;
    TTF_Font* font_ = nullptr;
    int width_, height_;
    double fps_ = 0.0, fpsTime_ = 0.0;
    int fpsFrames_ = 0;
    Uint64 lastCounter_ = 0;
    std::unordered_map<int, SDL_Texture*> circleCache_;
};
