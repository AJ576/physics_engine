#include "physics/world.hpp"

#include <algorithm>
#include <cmath>

namespace phys {

namespace {

// Cell coordinate of a world coordinate, clamped so the float -> int
// conversion is always defined (huge, infinite or NaN inputs included).
inline std::int32_t cellCoord(double v, double invCell) {
    double c = std::floor(v * invCell);
    if (!(c >= -1.0e9)) c = -1.0e9;  // also catches NaN
    if (c > 1.0e9) c = 1.0e9;
    return static_cast<std::int32_t>(c);
}

inline std::uint64_t cellKey(std::int32_t x, std::int32_t y) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32) | static_cast<std::uint32_t>(y);
}

inline std::uint64_t contactKey(int a, int b) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(a)) << 32) | static_cast<std::uint32_t>(b);
}

struct Aabb {
    double minX, minY, maxX, maxY;
};

// A body whose swept AABB spans more cells than this is tested against
// every body instead of being inserted into the grid.
constexpr int kMaxCellsPerBody = 16;

}  // namespace

Body makeBody(double radius, double mass, Vec2 pos, Vec2 vel, double angVel) {
    Body b;
    b.pos = pos;
    b.vel = vel;
    b.angVel = angVel;
    b.radius = radius;
    if (mass > 0.0) {
        b.mass = mass;
        b.invMass = 1.0 / mass;
        b.inertia = 0.5 * mass * radius * radius;
        b.invInertia = 1.0 / b.inertia;
    } else {
        b.mass = 0.0;
        b.invMass = 0.0;
        b.inertia = 0.0;
        b.invInertia = 0.0;
        b.vel = {};
        b.angVel = 0.0;
    }
    return b;
}

World::World(const WorldConfig& cfg) : cfg_(cfg) {}

int World::addBody(const Body& b) {
    bodies_.push_back(b);
    return static_cast<int>(bodies_.size()) - 1;
}

void World::clear() {
    bodies_.clear();
    contacts_.clear();
    impulseCache_.clear();
    grid_.clear();
}

double World::restitutionThreshold() const {
    if (cfg_.restitutionThreshold >= 0.0) return cfg_.restitutionThreshold;
    return 2.0 * cfg_.gravity.length() * cfg_.dt;
}

// --------------------------------------------------------------------- step
void World::step() {
    stats_ = {};
    integrateVelocities();
    collide();
    prepareContacts();
    if (cfg_.warmStarting) warmStart();
    solveVelocities();
    applyRestitution();
    storeImpulses();
    solvePositions();
    clampFastMotion();
    integratePositions();
}

void World::integrateVelocities() {
    const double dt = cfg_.dt;
    double k = 0.0;  // position update uses $v_{final} - k\,a\,\Delta t$
    if (cfg_.integrator == Integrator::ExplicitEuler) k = 1.0;
    if (cfg_.integrator == Integrator::ConstantAccel) k = 0.5;
    if (k != 0.0) velCorrection_.assign(bodies_.size(), Vec2{});
    prevVel_.resize(bodies_.size());
    prevAngVel_.resize(bodies_.size());
    for (std::size_t i = 0; i < bodies_.size(); ++i) {
        Body& b = bodies_[i];
        prevVel_[i] = b.vel;
        prevAngVel_[i] = b.angVel;
        if (b.isStatic()) continue;
        const Vec2 a = cfg_.gravity + b.force * b.invMass;
        b.vel += a * dt;
        b.angVel += b.torque * b.invInertia * dt;
        if (k != 0.0) velCorrection_[i] = a * (k * dt);
    }
}

// --------------------------------------------------------------- collision
void World::collide() {
    contacts_.clear();
    const int n = static_cast<int>(bodies_.size());
    maxRadius_ = 0.0;
    for (const Body& b : bodies_) maxRadius_ = std::max(maxRadius_, b.radius);

    if (cfg_.walls)
        for (int i = 0; i < n; ++i) wallContacts(i);

    if (cfg_.broadphase == Broadphase::BruteForce) {
        for (int i = 0; i < n; ++i)
            for (int j = i + 1; j < n; ++j) {
                ++stats_.candidatePairs;
                narrowphase(i, j);
            }
        filterSweptContacts();
        return;
    }

    // Uniform grid over swept AABBs: each body is inserted into every cell
    // its AABB (from x to x + v dt, padded by the radius) overlaps, so any
    // cell size finds every pair. A pair is reported only in the cell that
    // contains the min corner of the two AABBs' intersection, which both
    // bodies cover, so each pair is tested exactly once.
    const double cell = cfg_.cellSize > 0.0 ? cfg_.cellSize : 10.0;
    const double inv = 1.0 / cell;
    cellSizeUsed_ = cell;
    const double dt = cfg_.continuous ? cfg_.dt : 0.0;
    const double pad = 0.5 * cfg_.speculativeDistance;  // two pads = one margin
    std::vector<Aabb> boxes(static_cast<std::size_t>(n));
    grid_.clear();
    oversized_.clear();
    for (int i = 0; i < n; ++i) {
        const Body& b = bodies_[static_cast<std::size_t>(i)];
        const Vec2 p1 = b.pos + b.vel * dt;
        const double r = b.radius + pad;
        Aabb box{std::min(b.pos.x, p1.x) - r, std::min(b.pos.y, p1.y) - r, std::max(b.pos.x, p1.x) + r,
                 std::max(b.pos.y, p1.y) + r};
        boxes[static_cast<std::size_t>(i)] = box;
        const std::int32_t x0 = cellCoord(box.minX, inv), x1 = cellCoord(box.maxX, inv);
        const std::int32_t y0 = cellCoord(box.minY, inv), y1 = cellCoord(box.maxY, inv);
        if (static_cast<std::int64_t>(x1 - x0 + 1) * (y1 - y0 + 1) > kMaxCellsPerBody) {
            oversized_.push_back(i);
            continue;
        }
        for (std::int32_t y = y0; y <= y1; ++y)
            for (std::int32_t x = x0; x <= x1; ++x) grid_[cellKey(x, y)].push_back(i);
    }

    auto overlap = [&](int i, int j) {
        const Aabb& a = boxes[static_cast<std::size_t>(i)];
        const Aabb& b = boxes[static_cast<std::size_t>(j)];
        return a.minX <= b.maxX && b.minX <= a.maxX && a.minY <= b.maxY && b.minY <= a.maxY;
    };

    for (auto& entry : grid_) {
        const std::int32_t cx = static_cast<std::int32_t>(entry.first >> 32);
        const std::int32_t cy = static_cast<std::int32_t>(entry.first & 0xFFFFFFFFu);
        const std::vector<int>& ids = entry.second;
        for (std::size_t p = 0; p < ids.size(); ++p) {
            for (std::size_t q = p + 1; q < ids.size(); ++q) {
                const int i = ids[p], j = ids[q];  // i < j: bodies are inserted in index order
                if (!overlap(i, j)) continue;
                const Aabb& a = boxes[static_cast<std::size_t>(i)];
                const Aabb& b = boxes[static_cast<std::size_t>(j)];
                if (cellCoord(std::max(a.minX, b.minX), inv) != cx || cellCoord(std::max(a.minY, b.minY), inv) != cy)
                    continue;
                ++stats_.candidatePairs;
                narrowphase(i, j);
            }
        }
    }

    for (std::size_t k = 0; k < oversized_.size(); ++k) {
        const int i = oversized_[k];
        for (int j = 0; j < n; ++j) {
            if (j == i) continue;
            // Oversized pairs are handled once, by the lower index.
            if (std::find(oversized_.begin(), oversized_.end(), j) != oversized_.end() && j < i) continue;
            if (!overlap(i, j)) continue;
            ++stats_.candidatePairs;
            narrowphase(std::min(i, j), std::max(i, j));
        }
    }

    filterSweptContacts();
}

// A swept contact (two bodies that would meet within the step) is kept only
// if it is the earliest time of impact for both bodies. Otherwise a fast
// body would push every body along its path in the same step, including
// bodies hidden behind the first one it hits.
void World::filterSweptContacts() {
    bool any = false;
    for (const Contact& c : contacts_) any |= c.toi > 0.0;
    if (!any) return;
    // Touching or margin contacts that are approaching count as impact at t = 0.
    minToi_.assign(bodies_.size(), 1e300);
    for (const Contact& c : contacts_) {
        if (c.toi <= 0.0) {
            if (c.b < 0) continue;
            const Body& A = bodies_[static_cast<std::size_t>(c.a)];
            const Body& B = bodies_[static_cast<std::size_t>(c.b)];
            if (dot(B.vel - A.vel, c.normal) >= 0.0) continue;
            minToi_[static_cast<std::size_t>(c.a)] = 0.0;
            minToi_[static_cast<std::size_t>(c.b)] = 0.0;
            continue;
        }
        minToi_[static_cast<std::size_t>(c.a)] = std::min(minToi_[static_cast<std::size_t>(c.a)], c.toi);
        minToi_[static_cast<std::size_t>(c.b)] = std::min(minToi_[static_cast<std::size_t>(c.b)], c.toi);
    }
    std::size_t removed = 0;
    contacts_.erase(std::remove_if(contacts_.begin(), contacts_.end(),
                                   [&](const Contact& c) {
                                       const bool drop = c.toi > 0.0 &&
                                                         (c.toi > minToi_[static_cast<std::size_t>(c.a)] ||
                                                          c.toi > minToi_[static_cast<std::size_t>(c.b)]);
                                       removed += drop && c.b >= 0;
                                       return drop;
                                   }),
                    contacts_.end());
    stats_.contacts -= removed;
    stats_.speculative -= removed;
}

bool collideCircles(const Body& A, const Body& B, double dt, bool continuous, Vec2& normal, double& separation,
                    double margin, double* toi) {
    if (toi) *toi = 0.0;
    const Vec2 d = B.pos - A.pos;
    const double R = A.radius + B.radius;
    const double distSq = d.lengthSq();
    if (distSq < R * R) {
        const double dist = std::sqrt(distSq);
        normal = dist > 1e-12 * R ? d / dist : Vec2{1.0, 0.0};  // coincident centres: any unit normal
        separation = dist - R;
        return true;
    }
    const double Rm = R + margin;
    if (distSq < Rm * Rm) {  // within the speculative margin
        const double dist = std::sqrt(distSq);
        normal = d / dist;
        separation = dist - R;
        return true;
    }
    if (!continuous) return false;
    // Swept circle test: first root of $|d + v_r t|^2 = R^2$, i.e.
    // $|v_r|^2 t^2 + 2 (d\cdot v_r) t + |d|^2 - R^2 = 0$.
    const Vec2 vr = B.vel - A.vel;
    const double dv = dot(d, vr);
    if (dv >= 0.0) return false;  // separating
    const double vr2 = vr.lengthSq();
    const double disc = dv * dv - vr2 * (distSq - R * R);
    if (disc < 0.0) return false;  // closest approach is farther than R
    const double t = (-dv - std::sqrt(disc)) / vr2;
    if (t > dt) return false;  // not reached this step
    normal = (d + vr * t) / R;
    separation = std::max(0.0, dot(d, normal) - R);
    if (toi) *toi = t;
    return true;
}

void World::narrowphase(int i, int j) {
    const Body& A = bodies_[static_cast<std::size_t>(i)];
    const Body& B = bodies_[static_cast<std::size_t>(j)];
    if (A.isStatic() && B.isStatic()) return;
    Contact c;
    c.a = i;
    c.b = j;
    if (!collideCircles(A, B, cfg_.dt, cfg_.continuous, c.normal, c.separation, cfg_.speculativeDistance, &c.toi))
        return;
    if (c.separation > 0.0 || (B.pos - A.pos).lengthSq() >= (A.radius + B.radius) * (A.radius + B.radius))
        ++stats_.speculative;
    c.ra = c.normal * A.radius;
    c.rb = c.normal * (-B.radius);
    contacts_.push_back(c);
    ++stats_.contacts;
}

void World::wallContacts(int i) {
    const Body& b = bodies_[static_cast<std::size_t>(i)];
    if (b.isStatic()) return;
    const double r = b.radius;
    const double gaps[4] = {b.pos.x - r, cfg_.width - b.pos.x - r, b.pos.y - r, cfg_.height - b.pos.y - r};
    const Vec2 normals[4] = {{-1.0, 0.0}, {1.0, 0.0}, {0.0, -1.0}, {0.0, 1.0}};  // body -> wall
    for (int w = 0; w < 4; ++w) {
        const double s = gaps[w];
        const double approach = dot(b.vel, normals[w]);  // > 0 moving toward the wall
        const bool touching = s < 0.0;
        // Walls are always speculative by velocity (cheap: one plane test), so a
        // body can never pass through a wall, as with the old engine's snapping.
        const bool speculative = s < cfg_.speculativeDistance || (approach > 0.0 && s < approach * cfg_.dt);
        if (!touching && !speculative) continue;
        Contact c;
        c.a = i;
        c.b = -1 - w;
        c.normal = normals[w];
        c.separation = s;
        c.ra = normals[w] * r;
        c.rb = {};
        contacts_.push_back(c);
        ++stats_.wallContacts;
    }
}

// ------------------------------------------------------------------ solver

void World::prepareContacts() {
    for (Contact& c : contacts_) {
        const Body& A = bodies_[static_cast<std::size_t>(c.a)];
        const bool wall = c.b < 0;
        const Body* B = wall ? nullptr : &bodies_[static_cast<std::size_t>(c.b)];
        const double imB = wall ? 0.0 : B->invMass, iiB = wall ? 0.0 : B->invInertia;
        const Vec2 t = perp(c.normal);
        const double rnA = cross(c.ra, c.normal), rnB = cross(c.rb, c.normal);
        const double rtA = cross(c.ra, t), rtB = cross(c.rb, t);
        const double kN = A.invMass + imB + A.invInertia * rnA * rnA + iiB * rnB * rnB;
        const double kT = A.invMass + imB + A.invInertia * rtA * rtA + iiB * rtB * rtB;
        c.normalMass = kN > 0.0 ? 1.0 / kN : 0.0;
        c.tangentMass = kT > 0.0 ? 1.0 / kT : 0.0;
        // Approach speed for restitution, from the velocities at the start of
        // the step. Using the velocities after gravity was added made a ball
        // on the floor bounce at $e(v + g\Delta t)$ every hop, a fixed point at
        // $v = e g \Delta t / (1 - e)$, so it never came to rest.
        const auto ia = static_cast<std::size_t>(c.a);
        const Vec2 vA = prevVel_[ia] + cross(prevAngVel_[ia], c.ra);
        Vec2 vB;
        if (!wall) {
            const auto ib = static_cast<std::size_t>(c.b);
            vB = prevVel_[ib] + cross(prevAngVel_[ib], c.rb);
        }
        c.relativeVelocity = dot(vB - vA, c.normal);
    }
}

void World::warmStart() {
    for (Contact& c : contacts_) {
        auto it = impulseCache_.find(contactKey(c.a, c.b));
        if (it == impulseCache_.end()) continue;
        c.normalImpulse = it->second.x;
        c.tangentImpulse = it->second.y;
        const Vec2 P = c.normal * c.normalImpulse + perp(c.normal) * c.tangentImpulse;
        Body& A = bodies_[static_cast<std::size_t>(c.a)];
        A.vel -= P * A.invMass;
        A.angVel -= A.invInertia * cross(c.ra, P);
        if (c.b >= 0) {
            Body& B = bodies_[static_cast<std::size_t>(c.b)];
            B.vel += P * B.invMass;
            B.angVel += B.invInertia * cross(c.rb, P);
        }
    }
}

void World::solveVelocities() {
    const double invDt = 1.0 / cfg_.dt;
    const bool baumgarte = cfg_.positionCorrection == PositionCorrection::Baumgarte;
    const double mu = cfg_.friction;
    Vec2 zeroV;
    double zeroW = 0.0;

    for (int it = 0; it < cfg_.velocityIterations; ++it) {
        for (Contact& c : contacts_) {
            Body& A = bodies_[static_cast<std::size_t>(c.a)];
            const bool wall = c.b < 0;
            Body* B = wall ? nullptr : &bodies_[static_cast<std::size_t>(c.b)];
            Vec2& vB = wall ? zeroV : B->vel;
            double& wB = wall ? zeroW : B->angVel;
            const double imB = wall ? 0.0 : B->invMass, iiB = wall ? 0.0 : B->invInertia;
            const Vec2 t = perp(c.normal);

            // Friction: tangent impulse clamped to the friction cone $|j_t| \le \mu j_n$.
            {
                const Vec2 dv = (vB + cross(wB, c.rb)) - (A.vel + cross(A.angVel, c.ra));
                const double vt = dot(dv, t);
                const double maxF = mu * c.normalImpulse;
                const double newT = std::clamp(c.tangentImpulse - c.tangentMass * vt, -maxF, maxF);
                const double lambda = newT - c.tangentImpulse;
                c.tangentImpulse = newT;
                const Vec2 P = t * lambda;
                A.vel -= P * A.invMass;
                A.angVel -= A.invInertia * cross(c.ra, P);
                vB += P * imB;
                wB += iiB * cross(c.rb, P);
            }
            // Normal: accumulated impulse clamped to $j_n \ge 0$.
            {
                const Vec2 dv = (vB + cross(wB, c.rb)) - (A.vel + cross(A.angVel, c.ra));
                const double vn = dot(dv, c.normal);
                double target = 0.0;
                if (c.separation > 0.0) target = -c.separation * invDt;  // speculative: may close the gap
                else if (baumgarte) target = cfg_.baumgarte * std::max(-c.separation - cfg_.slop, 0.0) * invDt;
                const double newN = std::max(c.normalImpulse - c.normalMass * (vn - target), 0.0);
                const double lambda = newN - c.normalImpulse;
                c.normalImpulse = newN;
                c.maxNormalImpulse = std::max(c.maxNormalImpulse, newN);
                const Vec2 P = c.normal * lambda;
                A.vel -= P * A.invMass;
                A.angVel -= A.invInertia * cross(c.ra, P);
                vB += P * imB;
                wB += iiB * cross(c.rb, P);
            }
            zeroV = {};
            zeroW = 0.0;
        }
    }
}

void World::applyRestitution() {
    // For contacts that were approaching faster than the threshold and
    // actually pushed, drive the normal velocity to $-e\,v_n^{-}$ (the
    // pre-solve approach velocity), keeping $j_n \ge 0$.
    const double e = cfg_.restitution;
    if (e == 0.0) return;
    const double thr = restitutionThreshold();
    for (Contact& c : contacts_) {
        if (c.relativeVelocity > -thr || c.maxNormalImpulse == 0.0) continue;
        Body& A = bodies_[static_cast<std::size_t>(c.a)];
        const bool wall = c.b < 0;
        Body* B = wall ? nullptr : &bodies_[static_cast<std::size_t>(c.b)];
        const Vec2 vB = wall ? Vec2{} : B->vel + cross(B->angVel, c.rb);
        const double vn = dot(vB - (A.vel + cross(A.angVel, c.ra)), c.normal);
        const double newN = std::max(c.normalImpulse - c.normalMass * (vn + e * c.relativeVelocity), 0.0);
        const double lambda = newN - c.normalImpulse;
        c.normalImpulse = newN;
        const Vec2 P = c.normal * lambda;
        A.vel -= P * A.invMass;
        A.angVel -= A.invInertia * cross(c.ra, P);
        if (!wall) {
            B->vel += P * B->invMass;
            B->angVel += B->invInertia * cross(c.rb, P);
        }
    }
}

void World::storeImpulses() {
    if (!cfg_.warmStarting) return;
    impulseCache_.clear();
    for (const Contact& c : contacts_)
        if (c.normalImpulse != 0.0 || c.tangentImpulse != 0.0)
            impulseCache_[contactKey(c.a, c.b)] = Vec2{c.normalImpulse, c.tangentImpulse};
}

void World::solvePositions() {
    // Split impulse: penetration is removed with pseudo
    // velocities that move positions this step but are never added to the
    // real velocities, so position correction does not add kinetic energy.
    if (cfg_.positionCorrection != PositionCorrection::SplitImpulse) return;
    pseudoVel_.assign(bodies_.size(), Vec2{});
    const double invDt = 1.0 / cfg_.dt;
    const double beta = cfg_.baumgarte;
    for (int it = 0; it < cfg_.positionIterations; ++it) {
        for (Contact& c : contacts_) {
            if (c.separation >= -cfg_.slop) continue;
            const auto a = static_cast<std::size_t>(c.a);
            const bool wall = c.b < 0;
            const Vec2 pvB = wall ? Vec2{} : pseudoVel_[static_cast<std::size_t>(c.b)];
            const double vn = dot(pvB - pseudoVel_[a], c.normal);
            const double bias = beta * (-c.separation - cfg_.slop) * invDt;
            const double newP = std::max(c.pseudoImpulse - c.normalMass * (vn - bias), 0.0);
            const double lambda = newP - c.pseudoImpulse;
            c.pseudoImpulse = newP;
            pseudoVel_[a] -= c.normal * (lambda * bodies_[a].invMass);
            if (!wall) pseudoVel_[static_cast<std::size_t>(c.b)] += c.normal * (lambda * bodies_[static_cast<std::size_t>(c.b)].invMass);
        }
    }
}

// First fraction $s \in [0, 1]$ of this step's motion at which bodies f and j
// (moving by their final displacements) touch, or 1 if they do not. Pairs
// already overlapping are left to the contact solver.
double World::sweptFraction(int f, int j) const {
    const auto fi = static_cast<std::size_t>(f), ji = static_cast<std::size_t>(j);
    const Body& F = bodies_[fi];
    const Body& J = bodies_[ji];
    const double dt = cfg_.dt;
    const bool split = cfg_.positionCorrection == PositionCorrection::SplitImpulse;
    const Vec2 df = (F.vel + (split ? pseudoVel_[fi] : Vec2{})) * dt;
    const Vec2 dj = (J.vel + (split ? pseudoVel_[ji] : Vec2{})) * dt;
    const Vec2 d = J.pos - F.pos, m = dj - df;
    const double R = F.radius + J.radius;
    const double c = d.lengthSq() - R * R;
    if (c <= 0.0) return 1.0;  // overlapping now
    const double b = dot(d, m);
    if (b >= 0.0) return 1.0;  // separating
    const double a = m.lengthSq();
    const double disc = b * b - a * c;
    if (disc < 0.0) return 1.0;
    const double t = (-b - std::sqrt(disc)) / a;
    return t < 1.0 ? std::max(t, 0.0) : 1.0;
}

void World::clampFastMotion() {
    // Conservative advancement for bodies whose final (post-solve)
    // displacement exceeds half their radius: the solver can make a body fast
    // after contacts were detected (e.g. a hit hands its speed on), and that
    // body's new path was never tested. Each fast body's path is tested
    // against nearby bodies (grid), other fast bodies (sort and sweep on x)
    // and the walls, and every body involved advances only to the first touch.
    const std::size_t n = bodies_.size();
    alpha_.assign(n, 1.0);
    if (!cfg_.continuous) return;
    const double dt = cfg_.dt;
    const bool split = cfg_.positionCorrection == PositionCorrection::SplitImpulse;
    auto disp = [&](std::size_t i) { return (bodies_[i].vel + (split ? pseudoVel_[i] : Vec2{})) * dt; };

    fast_.clear();
    isFast_.assign(n, 0);
    for (std::size_t i = 0; i < n; ++i) {
        const Body& b = bodies_[i];
        if (b.isStatic()) continue;
        const double half = 0.5 * b.radius;
        if (disp(i).lengthSq() > half * half) {
            fast_.push_back(static_cast<int>(i));
            isFast_[i] = 1;
        }
    }
    stats_.fastBodies = fast_.size();
    if (fast_.empty()) return;

    auto hit = [&](int f, int j) {
        const double s = sweptFraction(f, j);
        if (s < 1.0) {
            alpha_[static_cast<std::size_t>(f)] = std::min(alpha_[static_cast<std::size_t>(f)], s);
            alpha_[static_cast<std::size_t>(j)] = std::min(alpha_[static_cast<std::size_t>(j)], s);
        }
    };

    // Fast vs slow. A slow body moves at most half its radius, so it is found
    // in the cells (built in collide()) around the fast body's path padded by
    // its radius, the largest radius and half the largest radius.
    const double inv = 1.0 / cellSizeUsed_;
    for (int f : fast_) {
        const Body& F = bodies_[static_cast<std::size_t>(f)];
        const Vec2 p1 = F.pos + disp(static_cast<std::size_t>(f));
        const double pad = F.radius + 1.5 * maxRadius_ + cfg_.speculativeDistance;
        const std::int32_t x0 = cellCoord(std::min(F.pos.x, p1.x) - pad, inv), x1 = cellCoord(std::max(F.pos.x, p1.x) + pad, inv);
        const std::int32_t y0 = cellCoord(std::min(F.pos.y, p1.y) - pad, inv), y1 = cellCoord(std::max(F.pos.y, p1.y) + pad, inv);
        if (cfg_.broadphase == Broadphase::BruteForce ||
            static_cast<std::int64_t>(x1 - x0 + 1) * (y1 - y0 + 1) > 4096) {
            for (std::size_t j = 0; j < n; ++j)
                if (!isFast_[j]) hit(f, static_cast<int>(j));
        } else {
            for (std::int32_t y = y0; y <= y1; ++y)
                for (std::int32_t x = x0; x <= x1; ++x) {
                    auto it = grid_.find(cellKey(x, y));
                    if (it == grid_.end()) continue;
                    for (int j : it->second)
                        if (!isFast_[static_cast<std::size_t>(j)]) hit(f, j);
                }
            for (int j : oversized_)
                if (!isFast_[static_cast<std::size_t>(j)]) hit(f, j);
        }
        // Walls: the disc may not cross a wall plane.
        if (cfg_.walls) {
            const Vec2 d = disp(static_cast<std::size_t>(f));
            const double gaps[4] = {F.pos.x - F.radius, cfg_.width - F.pos.x - F.radius, F.pos.y - F.radius,
                                    cfg_.height - F.pos.y - F.radius};
            const double towards[4] = {-d.x, d.x, -d.y, d.y};
            for (int w = 0; w < 4; ++w)
                if (gaps[w] >= 0.0 && towards[w] > gaps[w])
                    alpha_[static_cast<std::size_t>(f)] = std::min(alpha_[static_cast<std::size_t>(f)], gaps[w] / towards[w]);
        }
    }

    // Fast vs fast: sort by the min x of each path's AABB and sweep.
    std::sort(fast_.begin(), fast_.end(), [&](int a, int b) {
        const Body& A = bodies_[static_cast<std::size_t>(a)];
        const Body& B = bodies_[static_cast<std::size_t>(b)];
        const double ax = std::min(A.pos.x, A.pos.x + disp(static_cast<std::size_t>(a)).x) - A.radius;
        const double bx = std::min(B.pos.x, B.pos.x + disp(static_cast<std::size_t>(b)).x) - B.radius;
        return ax < bx || (ax == bx && a < b);
    });
    for (std::size_t p = 0; p < fast_.size(); ++p) {
        const auto ip = static_cast<std::size_t>(fast_[p]);
        const Body& A = bodies_[ip];
        const double maxX = std::max(A.pos.x, A.pos.x + disp(ip).x) + A.radius;
        for (std::size_t q = p + 1; q < fast_.size(); ++q) {
            const auto iq = static_cast<std::size_t>(fast_[q]);
            const Body& B = bodies_[iq];
            if (std::min(B.pos.x, B.pos.x + disp(iq).x) - B.radius > maxX) break;
            hit(fast_[p], fast_[q]);
        }
    }
    for (std::size_t i = 0; i < n; ++i) stats_.clampedBodies += alpha_[i] < 1.0;
}

void World::integratePositions() {
    const double dt = cfg_.dt;
    const bool split = cfg_.positionCorrection == PositionCorrection::SplitImpulse;
    const bool corr = cfg_.integrator != Integrator::SemiImplicitEuler;
    for (std::size_t i = 0; i < bodies_.size(); ++i) {
        Body& b = bodies_[i];
        b.force = {};
        b.torque = 0.0;
        if (b.isStatic()) continue;
        Vec2 v = b.vel;
        if (corr) v -= velCorrection_[i];
        if (split) v += pseudoVel_[i];
        b.pos += v * (alpha_[i] * dt);
        b.angle += b.angVel * dt;
        if (cfg_.walls) {  // safety net: a centre outside the arena is put back on the boundary
            b.pos.x = std::clamp(b.pos.x, 0.0, cfg_.width);
            b.pos.y = std::clamp(b.pos.y, 0.0, cfg_.height);
        }
    }
}

// ----------------------------------------------------------------- metrics
double World::kineticEnergy() const {
    double ke = 0.0;
    for (const Body& b : bodies_) ke += 0.5 * b.mass * b.vel.lengthSq() + 0.5 * b.inertia * b.angVel * b.angVel;
    return ke;
}

double World::potentialEnergy() const {
    double pe = 0.0;
    for (const Body& b : bodies_)
        if (!b.isStatic()) pe -= b.mass * dot(cfg_.gravity, b.pos);
    return pe;
}

Vec2 World::linearMomentum() const {
    Vec2 p;
    for (const Body& b : bodies_) p += b.vel * b.mass;
    return p;
}

double World::angularMomentum() const {
    double L = 0.0;
    for (const Body& b : bodies_) L += b.mass * cross(b.pos, b.vel) + b.inertia * b.angVel;
    return L;
}

}  // namespace phys
