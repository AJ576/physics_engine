#ifndef GRAPHICS_HPP
#define GRAPHICS_HPP

#pragma once
#include <array>
#include <string>
#include <vector>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include "rigidBody.hpp"
#include "spring.hpp"

class Graphics {
private:
    SDL_Window* window;
    SDL_Renderer* renderer;
    TTF_Font* font;
    int windowWidth;
    int windowHeight;

    // FPS state lives in Graphics now
    double fps_;
    double fpsAccumTime_;
    int fpsFrameCount_;
    Uint64 lastCounter_;

    std::unordered_map<int, SDL_Texture*> circleCache_;
    SDL_Texture* getCircleTexture(int radius);
    SDL_Texture* createCircleTexture(int radius); // <- fixed case

public:
    Graphics(int width, int height);
    ~Graphics();

    void clear();
    void drawCircle(const RigidBody& body, const std::array<int, 4>& color);
    void drawSpring(const Spring& spring, const std::array<int, 4>& color);
    void drawText(const std::string& text, int x, int y, const std::array<int, 4>& color);
    void present();
    bool processEvents();
    void updateSize(int width, int height);

    void printPhysicsInfo(const std::vector<RigidBody>& bodies,double gravityY);

    // optional helper
    double getFPS() const { return fps_; }
};

#endif // GRAPHICS_HPP