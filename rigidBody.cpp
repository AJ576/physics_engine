#include <iostream>
#include <cmath>

class RigidBody {
    public:
        float radius; // m
        float mass; // kg
        std::array<float, 2> position; // m
        std::array<float, 2> velocity; // m/s
        std::array<float, 2> acceleration; // m/s^2
};

bool areColliding(RigidBody body1, RigidBody body2) {
    if ((pow(body1.position[0] - body2.position[0], 2) + pow(body1.position[1] - body2.position[1], 2)) < (body1.radius + body2.radius)) return true;
    else return false;
}

int main() {
    RigidBody body1;
    RigidBody body2;

    body1.radius = 1.0f;
    body1.mass = 1.0f;
    body1.position = {0.0f, 0.0f};
    body1.velocity = {0.0f, 0.0f};
    body1.acceleration = {0.0f, 0.0f};

    body2.radius = 1.0f;
    body2.mass = 2.0f;
    body2.position = {21.0f, 21.0f};
    body2.velocity = {-1.0f, -1.0f};
    body2.acceleration = {0.0f, 0.0f};
}