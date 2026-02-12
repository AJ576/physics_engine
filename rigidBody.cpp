#include <iostream>
#include <cmath>
#include <array>

float springConstant; // N/m

class RigidBody {
    public:
        float radius; // m
        float mass; // kg
        std::array<float, 2> position; // m
        std::array<float, 2> velocity; // m/s
        std::array<float, 2> acceleration; // m/s^2
        std::array<float, 2> force; // N

    void numericalIntegration(float dt) {
        if (mass == 0.0f) return;

        // We are using Semi-Implicit Euler Method for integration. So, we need to update the velocity first and then the position.
        // The update of velocity is based on the new acceleration. So, we need to update the acceleration first and then the velocity.

        // Acceleration
        acceleration[0] = force[0] / mass;
        acceleration[1] = force[1] / mass;

        // Velocity
        velocity[0] += acceleration[0] * dt;
        velocity[1] += acceleration[1] * dt;

        // Position
        position[0] += velocity[0] * dt;
        position[1] += velocity[1] * dt;

        // Reset force
        // We are not using a force accumulator, so we need to reset the force to 0.0f after each integration step.
        // After this we need to calculate the new force based on the new position and velocity.
        force[0] = 0.0f;
        force[1] = 0.0f;
    }
};

bool areColliding(const RigidBody& body1, const RigidBody& body2) {
    float dx = body1.position[0] - body2.position[0];
    float dy = body1.position[1] - body2.position[1];
    float distSq = dx * dx + dy * dy;
    float rSum = body1.radius + body2.radius;
    return distSq < rSum * rSum;
}

void calculateForce(RigidBody& body1, RigidBody& body2) {
    if (areColliding(body1, body2)) {
        // Hooke's Law for Elastic Collision
        // F = -k * (x - x0)
        // Where k is the spring constant and x0 is the equilibrium position.
        // We are not using a spring constant, so we need to calculate the new force based on the new position and velocity.
        body1.force[0] = -springConstant * (body1.position[0] - body2.position[0]);
        body1.force[1] = -springConstant * (body1.position[1] - body2.position[1]);
        body2.force[0] = -springConstant * (body2.position[0] - body1.position[0]);
        body2.force[1] = -springConstant * (body2.position[1] - body1.position[1]);
    }
}

int main() {
    RigidBody body1;
    RigidBody body2;

    body1.radius = 1.0f;
    body1.mass = 1.0f;
    body1.position = {0.0f, 0.0f};
    body1.velocity = {0.0f, 0.0f};
    body1.acceleration = {0.0f, 0.0f};
    body1.force = {0.0f, 0.0f};

    body2.radius = 1.0f;
    body2.mass = 2.0f;
    body2.position = {21.0f, 21.0f};
    body2.velocity = {-1.0f, -1.0f};
    body2.acceleration = {0.0f, 0.0f};
    body2.force = {0.0f, 0.0f};
}