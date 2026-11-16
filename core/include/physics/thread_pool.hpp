#pragma once
// Minimal persistent thread pool with a blocking parallel-for over chunks.
// Workers sleep on a condition variable between calls.
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace phys {

class ThreadPool {
public:
    explicit ThreadPool(int threads);
    ~ThreadPool();
    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    int size() const { return static_cast<int>(workers_.size()) + 1; }  // workers + caller

    // Calls fn(chunk) for chunk = 0 .. chunks-1, spread over the workers and
    // the calling thread; returns when all chunks are done.
    void run(int chunks, const std::function<void(int)>& fn);

private:
    void workerLoop();

    std::vector<std::thread> workers_;
    std::mutex m_;
    std::condition_variable wake_, done_;
    const std::function<void(int)>* job_ = nullptr;
    int chunks_ = 0, next_ = 0, remaining_ = 0;
    unsigned generation_ = 0;
    bool stop_ = false;
};

}  // namespace phys
