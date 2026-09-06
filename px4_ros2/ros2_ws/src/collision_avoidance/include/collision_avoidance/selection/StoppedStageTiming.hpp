#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <ostream>

namespace collision_avoidance::selection
{
struct StageTimingRecord
{
    std::uint64_t source_us{0}, epoch{0}, period_us{0}, start_ns{0}, end_ns{0};
    std::uint8_t stage{0}, candidate_count{0};
    bool result_available{false}, output_queued{false};
};

// Single writer while running. No reader until the owning worker has joined.
// Allocated/value-initialized once before start; no transport or locking here.
struct StoppedStageTiming
{
    static constexpr std::size_t capacity = 16384;
    std::array<StageTimingRecord, capacity> records{};
    std::size_t size{0};
    std::uint64_t dropped{0};

    static std::uint64_t now() noexcept
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    void append(const StageTimingRecord & record) noexcept
    {
        if (size == capacity) { ++dropped; return; }
        records[size++] = record;
    }
    // Caller must have stopped/joined the worker; not a live logging API.
    void write(std::ostream & out, int vehicle) const
    {
        out << "[stop-stage-begin],1," << vehicle << ',' << size << ',' << dropped << '\n';
        for (std::size_t i = 0; i < size; ++i) {
            const auto & r = records[i];
            out << "[stop-stage]," << unsigned(r.stage) << ',' << r.source_us << ','
                << r.epoch << ',' << r.period_us << ',' << r.start_ns << ',' << r.end_ns
                << ',' << unsigned(r.candidate_count) << ',' << r.result_available
                << ',' << r.output_queued << '\n';
        }
        out << "[stop-stage-end]," << vehicle << ',' << size << '\n';
    }
};
}  // namespace collision_avoidance::selection
