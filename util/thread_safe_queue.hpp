#ifndef THREAD_SAFE_QUEUE_HPP
#define THREAD_SAFE_QUEUE_HPP

#include <queue>
#include <mutex>
#include <condition_variable>
#include <optional>

template <typename T>
class ThreadSafeQueue {
public:
    ThreadSafeQueue() = default;

    // ?癒?퓠 ?????곕떽?
    void push(const T& item) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.push(item);
        }
        cond_var_.notify_one();
    }

    // ??????곗눖源?(??곸몵筌???疫?
    T pop() {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_var_.wait(lock, [&]() { return !queue_.empty(); });

        T item = queue_.front();
        queue_.pop();
        return item;
    }

    // ??????곗눖沅?? timeout ??됱몵筌?std::nullopt 獄쏆꼹??
    std::optional<T> try_pop_for(std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!cond_var_.wait_for(lock, timeout, [&]() { return !queue_.empty(); })) {
            return std::nullopt;
        }

        T item = queue_.front();
        queue_.pop();
        return item;
    }

    // ????쑴肉?遺? ?類ㅼ뵥
    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }

private:
    mutable std::mutex mutex_;
    std::queue<T> queue_;
    std::condition_variable cond_var_;
};

#endif // THREAD_SAFE_QUEUE_HPP
