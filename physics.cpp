#include "physics.hpp"
#include <cmath>
#include <algorithm>

extern double springConstant; // N/m
extern double e; // coefficient of restitution

// Pack two signed cell coordinates into one 64-bit grid key. Done through
// unsigned types so negative coordinates (bodies briefly past the left or
// bottom wall) do not left-shift a negative value, which is UB before C++20.
static inline long long cellKey(int x, int y)
{
    return (long long)(((unsigned long long)(unsigned int)x << 32) | (unsigned int)y);
}

// TimeManager method implementations
TimeManager::TimeManager(){
    last_time = Clock::now();
}

void TimeManager::tick()
{
    auto current_time = Clock::now();
    Duration passed_time = current_time-last_time;
    last_time = current_time;

    double dt = passed_time.count();

    // CAP: If the frame took longer than 0.25 seconds (resize/lag), 
    // just pretend it took 0.016 seconds (1 frame at 60fps).
    if (dt > 0.25) {
        dt = 0.016; 
    }

    accumulator += dt;
}

bool TimeManager::physicsTime()
{
    if(accumulator > fixedDeltaTime)
    {
        accumulator-=fixedDeltaTime;
        return true;
    }
    return false;
}

void TimeManager::reset() {
    last_time = Clock::now();
    accumulator = 0.0; // Optional: clear the lag buffer too
}

// WorldPhysics method implementations
WorldPhysics::WorldPhysics(std::array<double, 2> border) : border_(border) {}

void WorldPhysics::applyGlobalForces()
{
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        if (bodies[i].getInvMass() == 0.0) {
            continue;
        }

        auto force = bodies[i].getForce();
        force[0] += bodies[i].getMass() * gravity_[0];
        force[1] += bodies[i].getMass() * gravity_[1];
        bodies[i].setForce(force);
    }
}

// Free function implementations -> WorldPhysics method implementations
bool WorldPhysics::areColliding(const RigidBody& body1, const RigidBody& body2,
                                double& nx, double& ny, double& overlap) {
    std::array<double, 2> pos1 = body1.getPosition();
    std::array<double, 2> pos2 = body2.getPosition();

    double dx = pos2[0] - pos1[0];
    double dy = pos2[1] - pos1[1];

    double distanceSq = dx * dx + dy * dy;
    double minDistance = body1.getRadius() + body2.getRadius();
    double minDistanceSq = minDistance * minDistance;

    if (distanceSq >= minDistanceSq) {
        overlap = 0.0;
        nx = 0.0;
        ny = 0.0;
        return false;
    }

    double distance = std::sqrt(distanceSq);

    // calculate the Normal (Unit Vector)
    // use distance to make sure the vector length is 1.0
    if (distance == 0.0) {
        nx = 1.0;
        ny = 0.0;
        overlap = minDistance;
    } else {
        nx = dx / distance;
        ny = dy / distance;
        overlap = minDistance - distance;
    }

    return true;
}

void WorldPhysics::calculateForce(RigidBody& body1, RigidBody& body2) {
    // Hooke's Law for Elastic Collision
    // F = -k * (x - x0)
    // Where k is the spring constant and x0 is the equilibrium position.
    std::array<double, 2> pos1 = body1.getPosition();
    std::array<double, 2> pos2 = body2.getPosition();
    
    body1.setForceX(springConstant * (pos1[0] - pos2[0]));
    body1.setForceY(springConstant * (pos1[1] - pos2[1]));
    body2.setForceX(springConstant * (pos2[0] - pos1[0]));
    body2.setForceY(springConstant * (pos2[1] - pos1[1]));
}

void WorldPhysics::calculateImpulse(RigidBody& body1, RigidBody& body2, double nx, double ny)
{
    auto vel1 = body1.getVelocity();
    auto vel2 = body2.getVelocity();

    //now find velocity relative to each other
    auto v_rel_x = vel2[0] - vel1[0];
    auto v_rel_y = vel2[1] - vel1[1];

    //find the nomral by doing a dot product
    double v_rel_normal = (v_rel_x * nx) + (v_rel_y * ny);

    //if they are already seperating or sliding no need to apply an oppsite impulse over and over.
    if (v_rel_normal >= 0)
    {
        return;
    }

    //calculate impuse by formula
    //      −(1 + e) * v_rel_normal
    //j = --------------------------------
    //      (1/m1 + 1/m2)

    double numerator = -1.0 * (1.0 + e) * v_rel_normal;
    double invMassSum = body1.getInvMass() + body2.getInvMass();

    //if both inv masses are 0 its imovable vs imovable. this means sim broke, 
    //no need to break it more with calculations
    if (invMassSum == 0)
    {
        return;
    }

    double j = numerator / invMassSum; //impulse

    //now calculate new velcoties and set them
    body1.setVelocityX(vel1[0] - ((j * body1.getInvMass()) * nx));
    body1.setVelocityY(vel1[1] - ((j * body1.getInvMass()) * ny));

    body2.setVelocityX(vel2[0] + ((j * body2.getInvMass()) * nx));
    body2.setVelocityY(vel2[1] + ((j * body2.getInvMass()) * ny));
}

bool WorldPhysics::resolveCollision(RigidBody& b1, RigidBody& b2)
{
    double nx = 0.0, ny = 0.0, overlap = 0.0;

    // if not colliding, return immediately
    if (!areColliding(b1, b2, nx, ny, overlap)) {
        return false;
    }

    std::array<double, 2> pos1 = b1.getPosition();
    std::array<double, 2> pos2 = b2.getPosition();

    //find total inv mass sum
    double invMassSum = b1.getInvMass() + b2.getInvMass();

    //we cooked if this happens
    if(invMassSum == 0)
    {
        return false;
    }

    //now move them along the normal based on their respective masses.
    //find out how much they move by mulaiplying the overlap to the individual InvMass/sum Invmass
    double move1 = overlap * (b1.getInvMass() / invMassSum);
    double move2 = overlap * (b2.getInvMass() / invMassSum);

    //then find components along x and y by multiplying this to the normal
    double dx1 = nx * move1;
    double dx2 = nx * move2;

    double dy1 = ny * move1;
    double dy2 = ny * move2;

    b1.setPositionX(pos1[0] - dx1);
    b1.setPositionY(pos1[1] - dy1);

    b2.setPositionX(pos2[0] + dx2);
    b2.setPositionY(pos2[1] + dy2);

    // after positional correction, resolve velocity with impulse
    calculateImpulse(b1, b2, nx, ny);
    return true;
}

void WorldPhysics::borderCheck(RigidBody& body1, double dt) {
    double radius = body1.getRadius();
    std::array<double, 2> pos = body1.getPosition();
    std::array<double, 2> vel = body1.getVelocity();

    for (int i = 0; i < 2; ++i) {
        double minBound = radius;
        double maxBound = border_[i] - radius;

        double penetration = 0.0;
        double normal = 0.0;
        double bound = 0.0;

        if (pos[i] < minBound) {
            penetration = minBound - pos[i];
            normal = +1.0;
            bound = minBound;
        } else if (pos[i] > maxBound) {
            penetration = pos[i] - maxBound;
            normal = -1.0;
            bound = maxBound;
        } else {
            continue;
        }

        // Check if this axis has gravity
        bool hasGravity = gravity_[i] != 0.0;

        if (hasGravity) {
            // Gravity-aligned axis: use settling logic
            bool movingWithGravity = vel[i] * gravity_[i] > 0.0;
            // With a fixed timestep, a body resting on the floor hits it at
            // roughly g*dt to 1.5*g*dt every step, so a threshold of exactly
            // g*dt leaves it hopping forever. 2*g*dt lets it settle.
            double restingVelocityThreshold = 2.0 * std::abs(gravity_[i]) * dt;
            bool slowEnoughToRest = std::abs(vel[i]) <= restingVelocityThreshold;

            if (movingWithGravity && slowEnoughToRest) {
                pos[i] = bound;
                vel[i] = 0.0;
            } else {
                // Always push back out of the wall, and only bounce if the
                // body is moving INTO the wall. Otherwise a body that was
                // pushed into the floor (e.g. by a pile above it) and is
                // already moving out would get flipped back in.
                pos[i] = bound;
                if (vel[i] * normal < 0.0) {
                    double speed = std::abs(vel[i]);
                    if (movingWithGravity) {
                        // The body overshot the wall by `penetration` and
                        // gained speed doing so. Use the speed it had when it
                        // actually touched the wall (v_c^2 = v^2 - 2 g d);
                        // otherwise every bounce injects a little energy and
                        // the body hops forever instead of settling.
                        double vc2 = speed * speed - 2.0 * std::abs(gravity_[i]) * penetration;
                        speed = std::sqrt(std::max(0.0, vc2));
                    }
                    vel[i] = normal * speed * e;
                }
            }
        } else {
            // Non-gravity axis: snap back, bounce only if moving into the wall
            pos[i] = bound;
            if (vel[i] * normal < 0.0) {
                vel[i] = -vel[i] * e;
            }
        }
    }
    
    body1.setPosition(pos);
    body1.setVelocity(vel);
}
void WorldPhysics::addBody(const RigidBody& body) {
    bodies.push_back(body);
}

void WorldPhysics::clearBodies() {
    bodies.clear();
    grid.clear();
}

void WorldPhysics::addSpring(const Spring& spring) {
    springs_.push_back(spring);
}

void WorldPhysics::clearSprings() {
    springs_.clear();
}

const std::vector<Spring>& WorldPhysics::getSprings() const {
    return springs_;
}

std::vector<RigidBody>& WorldPhysics::getBodies() {
    return bodies;
}

const std::vector<RigidBody>& WorldPhysics::getBodies() const {
    return bodies;
}

double WorldPhysics::calculateTotalKineticEnergy() const {
    double totalKE = 0.0;
    for (const auto& b : bodies) {
        auto v = b.getVelocity();
        totalKE += 0.5 * b.getMass() * (v[0] * v[0] + v[1] * v[1]);
    }
    return totalKE;
}

double WorldPhysics::calculateTotalPotentialEnergy() const {
    double totalPE = 0.0;
    for (const auto& b : bodies) {
        auto p = b.getPosition();
        double m = b.getMass();
        if (gravity_[0] != 0.0) {
            totalPE += m * (-gravity_[0]) * p[0];
        }
        if (gravity_[1] != 0.0) {
            totalPE += m * (-gravity_[1]) * p[1];
        }
    }
    return totalPE;
}

double WorldPhysics::calculateTotalEnergy() const {
    return calculateTotalKineticEnergy() + calculateTotalPotentialEnergy();
}

std::array<double, 2> WorldPhysics::calculateTotalMomentum() const {
    std::array<double, 2> totalP = {0.0, 0.0};
    for (const auto& b : bodies) {
        auto v = b.getVelocity();
        double m = b.getMass();
        totalP[0] += m * v[0];
        totalP[1] += m * v[1];
    }
    return totalP;
}

void WorldPhysics::runPhysics(const TimeManager& TIME)
{
    applyGlobalForces();

    //First step numerical integration
    for (size_t i = 0; i < bodies.size(); i++) {
        bodies[i].numericalIntegration(TIME.fixedDeltaTime);
    }

    //grid collision starts here
    //clear the grid
    grid.clear();

    //rebuild the grid
    for (size_t i = 0; i<bodies.size(); i++)
    {
        //get poistion and thus the cell val
        double x = bodies[i].getPosition()[0];
        double y = bodies[i].getPosition()[1];

        int cell_x = (int)std::floor(x / grid_size);
        int cell_y = (int)std::floor(y / grid_size);

        long long key = cellKey(cell_x, cell_y);

        grid[key].push_back(&bodies[i]);
    }

    //check for spring collisions
    // First relax each spring back toward its resting height, then test the
    // bodies against the (possibly compressed) current top surface.
    for (auto& spring : springs_) {
        spring.update(TIME.fixedDeltaTime);
    }

    for (size_t i = 0; i < bodies.size(); i++) {
        for (auto& spring : springs_) {
            bool hitFromAbove = true;
            if (detectSpringHit(bodies[i], spring, hitFromAbove)) {
                applySpringImpulse(bodies[i], spring, hitFromAbove);
            }
        }
    }

    for (auto& entry: grid)
    {
        long long key = entry.first;
        std::vector<RigidBody*>& cell = entry.second;

        //unpack the coords from key
        int x = key >> 32;
        int y = (int) (key & 0xFFFFFFFF);

        //loop over all bodies inside the cell
        for (size_t i = 0; i < cell.size();i++)
        {
            for(size_t j = i+1; j < cell.size(); j++)
            {
                resolveCollision(*cell[i],*cell[j]);
            }
        }
        //now the neigbors which is a bit weirder.
        //we only chck forward and down neigbors cuz neighbors behidn us and above already checked us

        int offsets[4][2] = {{1, 0}, {0, 1}, {1, 1}, {1, -1}};

        for(auto &off : offsets)
        {
            int nx = x + off[0];
            int ny = y + off[1];
            //make key to access the cell
            long long nkey = cellKey(nx, ny);
            //if key exists
            auto it = grid.find(nkey);
            if (it != grid.end())
            {
                auto& ncell = it->second;

                for (size_t i = 0; i < cell.size();i++)
                {
                    for (size_t j = 0; j < ncell.size();j++)
                    {
                        resolveCollision(*cell[i],*ncell[j]);
                    }
        
                }
            }
        }   
    }

    //Final step, border check
    for (size_t i = 0; i < bodies.size(); i++) {
        borderCheck(bodies[i], TIME.fixedDeltaTime);
    }
}

void WorldPhysics::runPhysicsNaive(const TimeManager& TIME)
{
    applyGlobalForces();

    for (size_t i = 0; i < bodies.size(); i++) {
        bodies[i].numericalIntegration(TIME.fixedDeltaTime);
    }

    for (size_t i = 0; i < bodies.size(); i++) {
        for (const auto& spring : springs_) {
            if (isBallOnSpring(bodies[i], spring)) {
                applySpringImpulse(bodies[i], spring);
            }
        }
    }

    for (size_t i = 0; i < bodies.size(); i++) {
        for (size_t j = i + 1; j < bodies.size(); j++) {
            resolveCollision(bodies[i], bodies[j]);
        }
    }

    for (size_t i = 0; i < bodies.size(); i++) {
        borderCheck(bodies[i], TIME.fixedDeltaTime);
    }
}

StepProfile WorldPhysics::runPhysicsProfiled(const TimeManager& TIME)
{
    StepProfile profile;
    auto startTotal = Clock::now();

    // 1. Global Forces
    auto t0 = Clock::now();
    applyGlobalForces();
    auto t1 = Clock::now();
    profile.forcesTimeUs = std::chrono::duration<double, std::micro>(t1 - t0).count();

    // 2. Numerical Integration
    t0 = Clock::now();
    for (size_t i = 0; i < bodies.size(); i++) {
        bodies[i].numericalIntegration(TIME.fixedDeltaTime);
    }
    t1 = Clock::now();
    profile.integrationTimeUs = std::chrono::duration<double, std::micro>(t1 - t0).count();

    // 3. Grid Build
    t0 = Clock::now();
    grid.clear();
    for (size_t i = 0; i < bodies.size(); i++) {
        double x = bodies[i].getPosition()[0];
        double y = bodies[i].getPosition()[1];

        int cell_x = (int)std::floor(x / grid_size);
        int cell_y = (int)std::floor(y / grid_size);

        long long key = cellKey(cell_x, cell_y);
        grid[key].push_back(&bodies[i]);
    }
    t1 = Clock::now();
    profile.gridBuildTimeUs = std::chrono::duration<double, std::micro>(t1 - t0).count();

    // 4. Springs
    t0 = Clock::now();
    for (size_t i = 0; i < bodies.size(); i++) {
        for (const auto& spring : springs_) {
            if (isBallOnSpring(bodies[i], spring)) {
                applySpringImpulse(bodies[i], spring);
            }
        }
    }
    t1 = Clock::now();
    profile.springTimeUs = std::chrono::duration<double, std::micro>(t1 - t0).count();

    // 5. Grid Collision Checks
    t0 = Clock::now();
    int offsets[4][2] = {{1, 0}, {0, 1}, {1, 1}, {1, -1}};

    for (auto& entry: grid)
    {
        long long key = entry.first;
        std::vector<RigidBody*>& cell = entry.second;

        int x = key >> 32;
        int y = (int) (key & 0xFFFFFFFF);

        for (size_t i = 0; i < cell.size(); i++)
        {
            for(size_t j = i + 1; j < cell.size(); j++)
            {
                profile.collisionPairsChecked++;
                if (resolveCollision(*cell[i], *cell[j])) {
                    profile.collisionsResolved++;
                }
            }
        }

        for (auto &off : offsets)
        {
            int nx = x + off[0];
            int ny = y + off[1];
            long long nkey = cellKey(nx, ny);
            auto it = grid.find(nkey);
            if (it != grid.end())
            {
                auto& ncell = it->second;
                for (size_t i = 0; i < cell.size(); i++)
                {
                    for (size_t j = 0; j < ncell.size(); j++)
                    {
                        profile.collisionPairsChecked++;
                        if (resolveCollision(*cell[i], *ncell[j])) {
                            profile.collisionsResolved++;
                        }
                    }
                }
            }
        }   
    }
    t1 = Clock::now();
    profile.collisionTimeUs = std::chrono::duration<double, std::micro>(t1 - t0).count();

    // 6. Border Check
    t0 = Clock::now();
    for (size_t i = 0; i < bodies.size(); i++) {
        borderCheck(bodies[i], TIME.fixedDeltaTime);
    }
    t1 = Clock::now();
    profile.borderTimeUs = std::chrono::duration<double, std::micro>(t1 - t0).count();

    auto endTotal = Clock::now();
    profile.totalTimeUs = std::chrono::duration<double, std::micro>(endTotal - startTotal).count();

    return profile;
}

StepProfile WorldPhysics::runPhysicsNaiveProfiled(const TimeManager& TIME)
{
    StepProfile profile;
    auto startTotal = Clock::now();

    auto t0 = Clock::now();
    applyGlobalForces();
    auto t1 = Clock::now();
    profile.forcesTimeUs = std::chrono::duration<double, std::micro>(t1 - t0).count();

    t0 = Clock::now();
    for (size_t i = 0; i < bodies.size(); i++) {
        bodies[i].numericalIntegration(TIME.fixedDeltaTime);
    }
    t1 = Clock::now();
    profile.integrationTimeUs = std::chrono::duration<double, std::micro>(t1 - t0).count();

    t0 = Clock::now();
    for (size_t i = 0; i < bodies.size(); i++) {
        for (const auto& spring : springs_) {
            if (isBallOnSpring(bodies[i], spring)) {
                applySpringImpulse(bodies[i], spring);
            }
        }
    }
    t1 = Clock::now();
    profile.springTimeUs = std::chrono::duration<double, std::micro>(t1 - t0).count();

    t0 = Clock::now();
    for (size_t i = 0; i < bodies.size(); i++) {
        for (size_t j = i + 1; j < bodies.size(); j++) {
            profile.collisionPairsChecked++;
            if (resolveCollision(bodies[i], bodies[j])) {
                profile.collisionsResolved++;
            }
        }
    }
    t1 = Clock::now();
    profile.collisionTimeUs = std::chrono::duration<double, std::micro>(t1 - t0).count();

    t0 = Clock::now();
    for (size_t i = 0; i < bodies.size(); i++) {
        borderCheck(bodies[i], TIME.fixedDeltaTime);
    }
    t1 = Clock::now();
    profile.borderTimeUs = std::chrono::duration<double, std::micro>(t1 - t0).count();

    auto endTotal = Clock::now();
    profile.totalTimeUs = std::chrono::duration<double, std::micro>(endTotal - startTotal).count();

    return profile;
}