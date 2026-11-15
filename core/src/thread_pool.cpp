#include "physics/thread_pool.hpp"

namespace phys {

ThreadPool::ThreadPool(int threads) {
    for (int i = 1; i < threads; ++i) workers_.emplace_back([this] { workerLoop(); });
}

ThreadPool::~ThreadPool() {
    {
        std::lock_guard<std::mutex> lock(m_);
        stop_ = true;
    }
    wake_.notify_all();
    for (auto& t : workers_) t.join();
}

void ThreadPool::run(int chunks, const std::function<void(int)>& fn) {
    if (workers_.empty() || chunks <= 1) {
        for (int c = 0; c < chunks; ++c) fn(c);
        return;
    }
    {
        std::lock_guard<std::mutex> lock(m_);
        job_ = &fn;
        chunks_ = chunks;
        next_ = 0;
        remaining_ = chunks;
        ++generation_;
    }
    wake_.notify_all();
    // The caller takes chunks too.
    for (;;) {
        int c;
        {
            std::lock_guard<std::mutex> lock(m_);
            if (next_ >= chunks_) break;
            c = next_++;
        }
        fn(c);
        std::lock_guard<std::mutex> lock(m_);
        if (--remaining_ == 0) done_.notify_all();
    }
    std::unique_lock<std::mutex> lock(m_);
    done_.wait(lock, [this] { return remaining_ == 0; });
    job_ = nullptr;
}

void ThreadPool::workerLoop() {
    unsigned seen = 0;
    std::unique_lock<std::mutex> lock(m_);
    for (;;) {
        wake_.wait(lock, [&] { return stop_ || (generation_ != seen && job_ && next_ < chunks_); });
        if (stop_) return;
        seen = generation_;
        while (job_ && next_ < chunks_) {
            const int c = next_++;
            const std::function<void(int)>* job = job_;
            lock.unlock();
            (*job)(c);
            lock.lock();
            if (--remaining_ == 0) done_.notify_all();
        }
    }
}

}  // namespace phys
