#pragma once
#include <atomic>
#include <array>
#include <optional>

template <typename T, size_t Capacity>
class SpscQueue {
    static_assert((Capacity & (Capacity - 1)) == 0, "must be power of 2");
public:
    bool try_push(const T& item) noexcept {
        const auto head = head_.load(std::memory_order_relaxed);
        const auto tail = tail_.load(std::memory_order_acquire);
        if (head - tail == Capacity) return false;
        buf_[head & kMask] = item;
        head_.store(head + 1, std::memory_order_release);
        return true;
    }
    std::optional<T> try_pop() noexcept {
        const auto tail = tail_.load(std::memory_order_relaxed);
        const auto head = head_.load(std::memory_order_acquire);
        if (head == tail) return std::nullopt;
        T item = buf_[tail & kMask];
        tail_.store(tail + 1, std::memory_order_release);
        return item;
    }
private:
    static constexpr size_t kMask = Capacity - 1;
    alignas(64) std::atomic<size_t> head_{0};
    alignas(64) std::atomic<size_t> tail_{0};
    alignas(64) std::array<T, Capacity> buf_{};
};