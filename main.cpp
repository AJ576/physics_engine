#include "rigidBody.hpp"
#include<iostream>

int main()
{
    RigidBody body1;
    RigidBody body2;
    TimeManager TIME;

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

    while (true)
    {
        TIME.tick();

        while(TIME.physicsTime())
        {
            runPhysics(body1, body2, TIME);
        }
    }
}