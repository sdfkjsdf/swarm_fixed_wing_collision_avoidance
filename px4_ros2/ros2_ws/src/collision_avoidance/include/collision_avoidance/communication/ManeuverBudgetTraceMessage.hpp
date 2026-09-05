#pragma once
#include <collision_avoidance/msg/maneuver_budget_trace.hpp>
#include <collision_avoidance/selection/ManeuverBudgetTrace.hpp>

namespace collision_avoidance::communication
{
inline msg::ManeuverBudgetTrace budgetTraceMessage(const selection::ManeuverBudgetTrace & t)
{
    msg::ManeuverBudgetTrace m;
    m.event = t.event;
    m.dropped_trace_count = t.dropped_trace_count;
    m.wall_ns = t.wall_ns;
    m.steady_ns = t.steady_ns;
    m.publish_end_wall_ns = t.publish_end_wall_ns;
    m.state_timestamp_us = t.state_timestamp_us;
    m.state_sample_timestamp_us = t.state_sample_timestamp_us;
    m.evaluation_timestamp_us = t.evaluation_timestamp_us;
    m.epoch = t.epoch;
    m.vehicle_id = t.vehicle_id;
    m.peer_id = t.peer_id;
    m.candidate_id = t.candidate_id;
    m.peer_candidate_id = t.peer_candidate_id;
    m.input_revision = t.input_revision;
    m.peer_input_revision = t.peer_input_revision;
    m.source_timestamp_us = t.source_timestamp_us;
    m.peer_source_timestamp_us = t.peer_source_timestamp_us;
    m.active = t.active;
    m.pmr_m = t.pmr_m;
    m.masd_m = t.masd_m;
    m.u95_m = t.u95_m;
    m.ad_m = t.ad_m;
    m.pmr_horizon_s = t.pmr_horizon_s;
    m.lateral_acceleration_mps2 = t.lateral_acceleration_mps2;
    m.ground_speed_command_mps = t.ground_speed_command_mps;
    m.equivalent_airspeed_command_mps = t.equivalent_airspeed_command_mps;
    return m;
}
} // namespace collision_avoidance::communication
