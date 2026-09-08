#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include <collision_avoidance/common/SpscQueue.hpp>

namespace collision_avoidance::common
{

// ONE producer across all partitions, ONE consumer. Reserved space per stream
// prevents one peer's burst from dropping another stream's inputs. Accepted
// events retain global producer order; this is not a priority/coalescing queue.
template <typename T, std::size_t PartitionCount, std::size_t CapacityPerPartition>
class OrderedSpscInbox
{
    static_assert(PartitionCount > 0);
    struct Entry {
        std::uint64_t sequence{0};
        T value{};
    };
public:
    bool try_push(std::size_t partition, const T & value) noexcept
    {
        if (partition >= PartitionCount) return false;
        const auto sequence = next_sequence_;
        if (!queues_[partition].try_push(Entry{sequence, value})) return false;
        ++next_sequence_;
        published_sequence_.store(sequence, std::memory_order_release);
        return true;
    }

    // Detach a bounded batch BEFORE running expensive handlers. Slots can then
    // accept arrivals while the consumer processes the detached batch. The
    // cutoff also prevents concurrent arrivals from extending this pass.
    template <std::size_t BatchCapacity>
    std::size_t drainTo(std::array<T, BatchCapacity> & batch) noexcept
    {
        const auto cutoff = published_sequence_.load(std::memory_order_acquire);
        std::size_t count = 0;
        while (count < BatchCapacity) {
            std::size_t selected = PartitionCount;
            std::uint64_t first = 0;
            for (std::size_t p = 0; p < PartitionCount; ++p) {
                const auto * entry = queues_[p].peekForConsumer();
                if (entry && entry->sequence <= cutoff
                    && (selected == PartitionCount || entry->sequence < first)) {
                    selected = p;
                    first = entry->sequence;
                }
            }
            if (selected == PartitionCount) break;
            batch[count++] = queues_[selected].try_pop()->value;
        }
        return count;
    }

private:
    std::array<SpscQueue<Entry, CapacityPerPartition>, PartitionCount> queues_{};
    std::uint64_t next_sequence_{1}; // producer-only
    std::atomic<std::uint64_t> published_sequence_{0};
};

} // namespace collision_avoidance::common
