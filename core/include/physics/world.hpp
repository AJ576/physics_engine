#pragma once
// 2D rigid-body world: circles with rotation, Coulomb friction, restitution,
// Hooke springs and distance constraints, solved with sequential impulses.
//
// Units are pixels, seconds and kilograms. Gravity defaults to
// $980\ \mathrm{px/s^2}$ downward ($y$ is up).
//
// One step of length $\Delta t$ (semi-implicit Euler with a velocity-level
// constraint solver in between):
//   1. forces: gravity, springs $F = -k(\ell - \ell_0) - c\,\dot{\ell}$
//   2. velocities: $v \mathrel{+}= a\,\Delta t$, $\omega \mathrel{+}= \alpha\,\Delta t$
//   3. broadphase (uniform grid over swept AABBs) and narrowphase (circle-circle,
//      circle-wall), creating touching and speculative contacts
//   4. warm start, then sequential impulses: friction clamped to the cone
//      $|j_t| \le \mu j_n$, normal impulse clamped to $j_n \ge 0$ (accumulated),
//      distance constraints; then a restitution pass
//   5. positions: $x \mathrel{+}= \alpha\,v\,\Delta t$ plus split-impulse position correction,
//      where $\alpha < 1$ only for a fast body whose final path hits something
//      (conservative advancement, so nothing passes through anything)
#include "physics/vec2.hpp"

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace phys {

struct Body {
    Vec2 pos, vel;
    double angle = 0.0, angVel = 0.0;  // rad, rad/s (counter-clockwise)
    double radius = 1.0;
    double mass = 1.0, invMass = 1.0;
    double inertia = 0.5, invInertia = 2.0;  // solid disc: $I = \tfrac12 m r^2$
    Vec2 force;            // accumulated external force, cleared every step
    double torque = 0.0;   // accumulated external torque, cleared every step

    bool isStatic() const { return invMass == 0.0; }
};

// A circle of the given radius and mass. mass <= 0 makes it static (infinite
// mass and inertia: it never moves).
Body makeBody(double radius, double mass, Vec2 pos, Vec2 vel = {}, double angVel = 0.0);

struct Spring {           // Hooke spring with damping between two bodies
    int a = 0, b = 0;
    double restLength = 0.0;
    double stiffness = 0.0;  // k (N/px with px units)
    double damping = 0.0;    // c, acts on the rate of change of length
};

struct DistanceConstraint {  // rigid rod: $|p_b - p_a| = L$
    int a = 0, b = 0;
    double length = 0.0;
};

enum class Integrator {
    SemiImplicitEuler,  // $v_{n+1} = v_n + a_n\Delta t$, $x_{n+1} = x_n + v_{n+1}\Delta t$ (default)
    ExplicitEuler,      // $x_{n+1} = x_n + v_n\Delta t$, $v_{n+1} = v_n + a_n\Delta t$
    ConstantAccel,      // the old engine: $x_{n+1} = x_n + v_n\Delta t + \tfrac12 a_n\Delta t^2$
};

enum class Broadphase { Grid, BruteForce };
enum class PositionCorrection { None, Baumgarte, SplitImpulse };

struct WorldConfig {
    double dt = 1.0 / 60.0;
    // Each step() runs this many substeps of dt / substeps. Explicit springs
    // are stable for $(\omega\,\Delta t)^2 < 4$ (undamped), so n substeps allow
    // $n^2$ times stiffer springs at n times the cost.
    int substeps = 1;
    Vec2 gravity{0.0, -980.0};
    double width = 800.0, height = 600.0;  // walls at x = 0, x = width, y = 0, y = height
    bool walls = true;

    double restitution = 0.7;
    double friction = 0.3;
    // Approach speeds below this do not bounce. < 0 means automatic:
    // $2\,|g|\,\Delta t$, so a body resting under gravity (which arrives at
    // $|g|\Delta t$ every step) does not jitter. With $g = 0$ every approach bounces.
    double restitutionThreshold = -1.0;

    int velocityIterations = 8;
    // Passes of the restitution step. The velocity solve treats contacts as
    // inelastic first; with several contacts on one body a single restitution
    // pass (as in Box2D) does not restore the bounce. Measured at e = 1, no
    // friction, 2,000 bodies at demo density, 10 s: KE -9.5% with 1 pass,
    // -0.28% with 8 (docs/BENCHMARKS.md, conservation).
    int restitutionIterations = 8;
    int positionIterations = 4;  // split-impulse iterations
    PositionCorrection positionCorrection = PositionCorrection::SplitImpulse;
    double baumgarte = 0.2;   // fraction of penetration removed per step
    double slop = 0.01;       // px of penetration allowed before correcting
    bool warmStarting = true;
    bool continuous = true;   // body-body swept-circle test: speculative contacts for fast pairs (CCD)
    // Pairs closer than this get a speculative contact even if they are not
    // approaching yet: the solver changes velocities (e.g. the floor stops the
    // lower of two bodies falling together), so a pair with zero relative
    // velocity at detection can still close within the step. Should exceed
    // $|g|\,\Delta t^2$ (0.27 px at the defaults).
    double speculativeDistance = 1.0;  // px

    Integrator integrator = Integrator::SemiImplicitEuler;
    Broadphase broadphase = Broadphase::Grid;
    double cellSize = 10.0;   // px; any value is correct (bodies span cells), speed varies
};

struct Contact {
    int a = 0;            // body index
    int b = 0;            // body index, or wall: -1 left, -2 right, -3 bottom, -4 top
    Vec2 normal;          // unit, from a to b
    double separation = 0.0;  // < 0 penetrating, > 0 speculative gap
    Vec2 ra, rb;          // contact point relative to the centres
    double normalMass = 0.0, tangentMass = 0.0;
    double normalImpulse = 0.0, tangentImpulse = 0.0;  // accumulated
    double maxNormalImpulse = 0.0;
    double relativeVelocity = 0.0;  // normal relative velocity at the start of the step (before forces)
    double pseudoImpulse = 0.0;     // split impulse (position correction)
    double toi = 0.0;               // swept contacts: time of impact (s); 0 for touching / margin
};

// Narrowphase for two circles. Returns true and fills normal (a -> b) and
// separation if they overlap now, if the gap is below `margin`, or, when
// `continuous` is set, if the
// swept-circle test $|d + v_r t| = r_a + r_b$ has a root $t \in [0, \Delta t]$
// (a speculative contact with separation >= 0 and the normal at impact).
// `margin` is the speculative distance (contacts are also made for gaps below it).
// `toi` (optional) receives the time of impact for swept contacts, 0 otherwise.
bool collideCircles(const Body& a, const Body& b, double dt, bool continuous, Vec2& normal, double& separation,
                    double margin = 0.0, double* toi = nullptr);

struct StepStats {
    std::size_t candidatePairs = 0;  // body pairs that reached the narrowphase
    std::size_t contacts = 0;        // body-body contacts (touching + speculative)
    std::size_t speculative = 0;     // of which speculative (not yet touching)
    std::size_t wallContacts = 0;
    std::size_t fastBodies = 0;      // bodies checked by the post-solve motion clamp
    std::size_t clampedBodies = 0;   // of which advanced less than a full step
};

class World {
public:
    explicit World(const WorldConfig& cfg = {});

    int addBody(const Body& b);
    int addSpring(const Spring& s);
    int addDistanceConstraint(int a, int b);  // length = current distance
    int addDistanceConstraint(const DistanceConstraint& c);
    void clear();

    void step();

    WorldConfig& config() { return cfg_; }
    const WorldConfig& config() const { return cfg_; }
    std::vector<Body>& bodies() { return bodies_; }
    const std::vector<Body>& bodies() const { return bodies_; }
    const std::vector<Spring>& springs() const { return springs_; }
    const std::vector<DistanceConstraint>& distanceConstraints() const { return joints_; }
    const std::vector<Contact>& contacts() const { return contacts_; }
    const StepStats& lastStepStats() const { return stats_; }

    // $\sum \tfrac12 m v^2 + \tfrac12 I \omega^2$
    double kineticEnergy() const;
    // gravity $\sum -m\,g\cdot x$ plus springs $\sum \tfrac12 k (\ell - \ell_0)^2$
    double potentialEnergy() const;
    double totalEnergy() const { return kineticEnergy() + potentialEnergy(); }
    Vec2 linearMomentum() const;
    double angularMomentum() const;  // about the origin, including spin

private:
    struct JointState {
        Vec2 u;              // unit axis a -> b
        double error = 0.0;  // current length - target length
        double mass = 0.0;
        double impulse = 0.0;       // accumulated (warm started)
        double pseudoImpulse = 0.0;
    };

    void substep();
    void applyForces();
    void integrateVelocities();
    void collide();
    void narrowphase(int i, int j);
    void filterSweptContacts();
    void wallContacts(int i);
    void prepareContacts();
    void warmStart();
    void solveVelocities();
    void applyRestitution();
    void storeImpulses();
    void solvePositions();
    void clampFastMotion();
    void integratePositions();
    double sweptFraction(int f, int j) const;

    double restitutionThreshold() const;

    WorldConfig cfg_;
    std::vector<Body> bodies_;
    std::vector<Spring> springs_;
    std::vector<DistanceConstraint> joints_;
    std::vector<JointState> jointState_;
    std::vector<Contact> contacts_;
    std::vector<Vec2> pseudoVel_;     // split-impulse velocities (linear only)
    std::vector<Vec2> prevVel_;       // velocities before this step's forces (for restitution)
    std::vector<double> prevAngVel_;
    std::vector<Vec2> velCorrection_; // integrator-specific position term
    std::unordered_map<std::uint64_t, std::vector<int>> grid_;
    std::vector<int> oversized_;
    double h_ = 1.0 / 60.0;  // current substep length
    double cellSizeUsed_ = 10.0;
    double maxRadius_ = 0.0;
    std::vector<double> alpha_;     // fraction of this step's motion each body may advance
    std::vector<double> minToi_;
    std::vector<int> fast_;
    std::vector<char> isFast_;
    std::unordered_map<std::uint64_t, Vec2> impulseCache_;  // (normal, tangent) by contact key
    StepStats stats_;
};

}  // namespace phys
