#pragma once
#include <cmath>

namespace phys {

struct Vec2 {
    double x = 0.0, y = 0.0;

    constexpr Vec2() = default;
    constexpr Vec2(double x_, double y_) : x(x_), y(y_) {}

    constexpr Vec2 operator+(Vec2 o) const { return {x + o.x, y + o.y}; }
    constexpr Vec2 operator-(Vec2 o) const { return {x - o.x, y - o.y}; }
    constexpr Vec2 operator-() const { return {-x, -y}; }
    constexpr Vec2 operator*(double s) const { return {x * s, y * s}; }
    constexpr Vec2 operator/(double s) const { return {x / s, y / s}; }
    Vec2& operator+=(Vec2 o) { x += o.x; y += o.y; return *this; }
    Vec2& operator-=(Vec2 o) { x -= o.x; y -= o.y; return *this; }
    Vec2& operator*=(double s) { x *= s; y *= s; return *this; }
    constexpr bool operator==(Vec2 o) const { return x == o.x && y == o.y; }

    double lengthSq() const { return x * x + y * y; }
    double length() const { return std::sqrt(lengthSq()); }
};

constexpr Vec2 operator*(double s, Vec2 v) { return {s * v.x, s * v.y}; }
constexpr double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
// 2D cross product $a \times b = a_x b_y - a_y b_x$ (a scalar).
constexpr double cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
// $\omega \times r$ for a scalar angular velocity: $(-\omega r_y,\ \omega r_x)$.
constexpr Vec2 cross(double w, Vec2 r) { return {-w * r.y, w * r.x}; }
// Left perpendicular, $(-v_y, v_x)$.
constexpr Vec2 perp(Vec2 v) { return {-v.y, v.x}; }

}  // namespace phys
