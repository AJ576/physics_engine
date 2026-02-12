#include <iostream>
#include <cmath>
#include <chrono>

using Clock = std::chrono::high_resolution_clock;
using TimePoint = std::chrono::time_point<Clock>;
using Duration = std::chrono::duration<double>;

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


bool areColliding(RigidBody body1, RigidBody body2) {
    if ((pow(body1.position[0] - body2.position[0], 2) + pow(body1.position[1] - body2.position[1], 2)) < (body1.radius + body2.radius)) return true;
    else return false;
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

    body2.radius = 1.0f;
    body2.mass = 2.0f;
    body2.position = {21.0f, 21.0f};
    body2.velocity = {-1.0f, -1.0f};
    body2.acceleration = {0.0f, 0.0f};

    while (true)
    {
        TIME.tick();

        if(TIME.physicsTime())
        {
            runPhysics();
        }
    }
}