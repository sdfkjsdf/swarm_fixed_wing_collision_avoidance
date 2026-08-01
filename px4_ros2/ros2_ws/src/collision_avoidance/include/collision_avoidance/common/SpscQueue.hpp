#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <optional>

namespace collision_avoidance::common
{

template <typename T, std::size_t Capacity>
class SpscQueue {
    static_assert(Capacity > 0, "capacity must be greater than zero");
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
    static constexpr std::size_t kMask = Capacity - 1;
    alignas(64) std::atomic<std::size_t> head_{0};
    alignas(64) std::atomic<std::size_t> tail_{0};
    alignas(64) std::array<T, Capacity> buf_{};
};

}  // namespace collision_avoidance::common
