#ifndef GRAPHICS_HPP
#define GRAPHICS_HPP

#pragma once // read once per programam.
#include <SDL2/SDL.h>
#include <vector>
#include <array>
#include "rigidBody.hpp"

class Graphics {
public:
    Graphics(int width, int height);
    ~Graphics(); // Destructor to clean up memory

    void clear();
    void drawCircle(const RigidBody& body, std::array<int, 4>& color);
    void present();
    bool processEvents(); // Handles things like clicking "X" to close

private:
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    int windowHeight;
};

#endif // GRAPHICS_HPP