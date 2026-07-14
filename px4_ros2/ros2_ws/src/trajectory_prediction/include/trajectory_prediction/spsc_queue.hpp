#pragma once

/* ════════════════════════════════════════════════════════════════════
   spsc_queue.hpp — Single Producer / Single Consumer lock-free 큐
   ────────────────────────────────────────────────────────────────────
   용도: rt_thread (producer) 가 push, main thread (consumer) 가 pop.
         두 thread 간에 mutex 없이 setpoint 데이터를 안전하게 전달.

   동작:
     - 고정 크기 ring buffer (capacity = power of 2)
     - try_push()  : 가득 차면 false 반환 (non-blocking)
     - try_pop()   : 비어있으면 std::nullopt 반환 (non-blocking)
     - alignas(64) : false sharing 방지 (head/tail/buf 가 다른 캐시라인)

   주의: producer 1개 + consumer 1개일 때만 안전. 둘 이상이면 깨짐.
         이 패키지에서는 OutputQueue_rt2mt 타입으로 사용 (StateType.hpp).
   ════════════════════════════════════════════════════════════════════ */

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
