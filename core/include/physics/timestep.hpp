#pragma once
// Fixed-timestep accumulator for a render loop.
//
//   FixedTimestep ts(1.0 / 60.0);
//   every frame: ts.advance(frameSeconds); while (ts.consume()) world.step();
//
// A single frame's elapsed time is clamped to kMaxFrameTime and at most
// kMaxStepsPerFrame steps are ever queued: if the simulation cannot keep up
// it slows down instead of spiralling (each slow frame queuing more steps).
namespace phys {

class FixedTimestep {
public:
    static constexpr double kMaxFrameTime = 0.25;  // s
    static constexpr int kMaxStepsPerFrame = 8;

    explicit FixedTimestep(double dt = 1.0 / 60.0) : dt_(dt) {}

    void advance(double frameSeconds) {
        double t = frameSeconds;
        if (!(t > 0.0)) t = 0.0;  // also rejects NaN
        if (t > kMaxFrameTime) t = kMaxFrameTime;
        acc_ += t;
        const double maxQueued = kMaxStepsPerFrame * dt_;
        if (acc_ > maxQueued) acc_ = maxQueued;
    }
    bool consume() {
        if (acc_ < dt_) return false;
        acc_ -= dt_;
        return true;
    }
    void reset() { acc_ = 0.0; }
    double dt() const { return dt_; }
    double alpha() const { return acc_ / dt_; }  // fraction of a step left over, for interpolation

private:
    double dt_;
    double acc_ = 0.0;
};

}  // namespace phys
