#include <iostream>
#include <cmath>
#include <chrono>

using Clock = std::chrono::high_resolution_clock;
using TimePoint = std::chrono::time_point<Clock>;
using Duration = std::chrono::duration<double>;
#include <array>

float springConstant; // N/m

class RigidBody {
    public:
        float radius; // m
        float mass; // kg
        std::array<float, 2> position; // m
        std::array<float, 2> velocity; // m/s
        std::array<float, 2> acceleration; // m/s^2
};

class TimeManager {
    private:
        double accumulator = 0.0;
        TimePoint last_time;
    
    public:
        const double fixedDeltaTime = 1.0/60.0; //60HZ

        TimeManager(){
            last_time = Clock::now();
        }

        void tick()
        {
            auto current_time = Clock::now();
            Duration passed_time = current_time-last_time;
            last_time = current_time;

            accumulator += passed_time.count();
        }

        bool physicsTime()
        {
            if(accumulator > fixedDeltaTime)
            {
                accumulator-=fixedDeltaTime;
                return true;
            }
            return false;
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
void runPhysics()
{
    std::cout<<"hello_world";
}
int main() {
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
            runPhysics();
        }
    }
}