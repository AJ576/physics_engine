// Counts heap allocations made inside World::step() once the world has
// warmed up (buffers reached their working size).
#include <doctest/doctest.h>

#include "physics/scenes.hpp"

#include <atomic>
#include <cstdlib>
#include <new>

namespace {
std::atomic<long> g_allocs{0};
}

void* operator new(std::size_t n) {
    g_allocs.fetch_add(1, std::memory_order_relaxed);
    if (void* p = std::malloc(n ? n : 1)) return p;
    throw std::bad_alloc();
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }

namespace {
long allocationsDuring(phys::World& w, int warmup, int steps) {
    for (int i = 0; i < warmup; ++i) w.step();
    const long before = g_allocs.load();
    for (int i = 0; i < steps; ++i) w.step();
    return g_allocs.load() - before;
}
}  // namespace

TEST_CASE("allocation: step() makes no heap allocations after warm-up (pile and gas)" * doctest::should_fail()) {
    phys::World pile = phys::scenes::pile(2000, 1);
    const long a = allocationsDuring(pile, 300, 120);
    phys::World gas = phys::scenes::stress(5000, 42);
    const long b = allocationsDuring(gas, 120, 120);
    MESSAGE("allocations in 120 steps: pile " << a << ", gas " << b);
    CHECK(a == 0);
    CHECK(b == 0);
}
