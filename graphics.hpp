#ifndef GRAPHICS_HPP
#define GRAPHICS_HPP

#pragma once // read once per programam.
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <string>
#include <vector>
#include <array>
#include "rigidBody.hpp"

class Graphics {
public:
    Graphics(int width, int height);
    ~Graphics(); // Destructor to clean up memory

    void clear();
    void drawCircle(const RigidBody& body, const std::array<int, 4>& color);
    void drawText(const std::string& text,int x, int y, const std::array<int, 4>& color);
    void printPhysicsInfo(const std::vector<RigidBody>& bodies);
    void present();
    bool processEvents(); // Handles things like clicking "X" to close
    void updateSize(int width, int height);

private:
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    TTF_Font* font = nullptr;
    int windowHeight;
    int windowWidth;
};

#endif // GRAPHICS_HPP