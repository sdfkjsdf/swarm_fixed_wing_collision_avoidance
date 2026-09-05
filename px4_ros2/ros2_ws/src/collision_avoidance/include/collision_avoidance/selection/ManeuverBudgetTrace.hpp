#pragma once

#include <chrono>
#include <cstdint>
#include <limits>

namespace collision_avoidance::selection
{
// Observation only. Neither these timestamps nor these fields enter selection.
// Event 1=proposal ready, 2=commit, 3=pair monitor, 4=ROS setpoint publish.
struct ManeuverBudgetTrace
{
    std::uint8_t event{0};
    std::uint64_t dropped_trace_count{0};
    std::uint64_t wall_ns{0};
    std::uint64_t steady_ns{0};
    std::uint64_t publish_end_wall_ns{0};
    std::uint64_t state_timestamp_us{0};
    std::uint64_t state_sample_timestamp_us{0};
    std::uint64_t evaluation_timestamp_us{0};
    std::uint64_t epoch{0};
    int vehicle_id{-1};
    int peer_id{-1};
    std::uint8_t candidate_id{0};
    std::uint8_t peer_candidate_id{0};
    std::uint64_t input_revision{0};
    std::uint64_t peer_input_revision{0};
    std::uint64_t source_timestamp_us{0};
    std::uint64_t peer_source_timestamp_us{0};
    bool active{false};
    double pmr_m{std::numeric_limits<double>::quiet_NaN()};
    double masd_m{std::numeric_limits<double>::quiet_NaN()};
    double u95_m{std::numeric_limits<double>::quiet_NaN()};
    double ad_m{std::numeric_limits<double>::quiet_NaN()};
    double pmr_horizon_s{std::numeric_limits<double>::quiet_NaN()};
    double lateral_acceleration_mps2{std::numeric_limits<double>::quiet_NaN()};
    double ground_speed_command_mps{std::numeric_limits<double>::quiet_NaN()};
    double equivalent_airspeed_command_mps{std::numeric_limits<double>::quiet_NaN()};
};

inline void stampBudgetTrace(ManeuverBudgetTrace & trace)
{
    trace.wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    trace.steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
} // namespace collision_avoidance::selection
