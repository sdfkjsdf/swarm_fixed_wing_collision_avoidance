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

// Diagnostic clocks only, never control inputs.
struct BeliefArrivalTiming
{
    std::uint64_t callback_ns{0}, callback_wall_ns{0}, middleware_received_ns{0};
};
struct BeliefTimingRecord
{
    std::uint64_t source_us{0}, sample_us{0};
    BeliefArrivalTiming arrival{};
    std::uint64_t enqueue_ns{0}, dispatch_ns{0}, accepted_ns{0};
    bool accepted{false};
};
struct PipelineTimingRecord
{
    std::uint64_t source_us{0}, start_ns{0}, drain_end_ns{0}, end_ns{0};
    std::uint64_t remote_processing_ns{0}, belief_processing_ns{0};
    std::uint32_t input_count{0}, remote_count{0}, belief_count{0};
};

// Single writer while running. No reader until the owning worker has joined.
// Allocated/value-initialized once before start; no transport or locking here.
struct StoppedStageTiming
{
    static constexpr std::size_t capacity = 16384;
    std::array<StageTimingRecord, capacity> records{};
    std::size_t size{0};
    std::uint64_t dropped{0};
    // Bounded, pre-touched; idle polls are not retained. No live consumer.
    static constexpr std::size_t pipeline_capacity = 131072;
    static constexpr std::size_t belief_capacity = 32768;
    std::array<PipelineTimingRecord, pipeline_capacity> pipelines{};
    std::array<BeliefTimingRecord, belief_capacity> beliefs{};
    std::size_t pipeline_size{0}, belief_size{0};
    std::uint64_t pipeline_dropped{0}, belief_dropped{0};

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
    void appendPipeline(const PipelineTimingRecord & record) noexcept
    {
        if (pipeline_size == pipeline_capacity) { ++pipeline_dropped; return; }
        pipelines[pipeline_size++] = record;
    }
    void appendBelief(const BeliefTimingRecord & record) noexcept
    {
        if (belief_size == belief_capacity) { ++belief_dropped; return; }
        beliefs[belief_size++] = record;
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
        out << "[stop-pipeline-begin],1," << vehicle << ',' << pipeline_size << ','
            << pipeline_dropped << ',' << belief_size << ',' << belief_dropped << '\n';
        for (std::size_t i = 0; i < pipeline_size; ++i) {
            const auto & r = pipelines[i];
            out << "[stop-pipeline]," << r.source_us << ',' << r.start_ns << ','
                << r.drain_end_ns << ',' << r.end_ns << ',' << r.remote_processing_ns
                << ',' << r.belief_processing_ns << ',' << r.input_count << ','
                << r.remote_count << ',' << r.belief_count << '\n';
        }
        for (std::size_t i = 0; i < belief_size; ++i) {
            const auto & r = beliefs[i];
            out << "[stop-belief]," << r.source_us << ',' << r.sample_us << ','
                << r.arrival.callback_ns << ',' << r.arrival.callback_wall_ns << ','
                << r.arrival.middleware_received_ns << ',' << r.enqueue_ns << ','
                << r.dispatch_ns << ',' << r.accepted_ns << ',' << r.accepted << '\n';
        }
        out << "[stop-pipeline-end]," << vehicle << ',' << pipeline_size << ','
            << belief_size << '\n';
    }
};
}  // namespace collision_avoidance::selection
