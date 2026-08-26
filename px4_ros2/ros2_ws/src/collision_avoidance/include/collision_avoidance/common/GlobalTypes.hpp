#pragma once

#include <array>
#include <cstddef>
#include <limits>

#include <collision_avoidance/common/SpscQueue.hpp>

/*
 * Production modules and HILS packages share the data-only types in this file.
 * Keep this header free of ROS 2 and PX4 framework dependencies so that the
 * types remain inexpensive to include and usable in pure C++ tests.
 *
 * Thread ownership suffixes belong to object/member names (m_*_mt, m_*_rt),
 * not to reusable type names. The channel direction is expressed by each
 * queue alias.
 */
namespace collision_avoidance::types
{

inline constexpr std::size_t kMaxAgents = 8;
inline constexpr std::size_t kSetpointQueueCapacity = 8;

/* Main-thread vehicle state populated by odometry callbacks. */
struct ControlState
{
    std::array<float, 3> position{0.0f, 0.0f, 0.0f};
    std::array<float, 3> velocity{0.0f, 0.0f, 0.0f};
    std::array<float, 3> position_variance{0.0f, 0.0f, 0.0f};
    std::array<float, 3> velocity_variance{0.0f, 0.0f, 0.0f};
    double timestamp{0.0};
    int check_vehicle_id{0};
    bool update_state{false};
};

/* Immutable main-to-RT snapshot of all active agents. */
struct ControlSnapshot
{
    std::array<ControlState, kMaxAgents> agents{};
    double timestamp{0.0};
    int num_agents{0};
};

using ControlInputQueue =
    collision_avoidance::common::SpscQueue<ControlSnapshot, kMaxAgents>;
using AgentUpdateFlags = std::array<bool, kMaxAgents>;

/* RT-thread representation consumed by the flocking algorithm. */
struct AgentState
{
    float pos_n{0.0f};
    float pos_e{0.0f};
    float pos_d{0.0f};
    float vel_n{0.0f};
    float vel_e{0.0f};
    float vel_d{0.0f};
    float speed{0.0f};
    float psi{0.0f};
    float gamma{0.0f};
};

using AgentStateArray = std::array<AgentState, kMaxAgents>;

/* RT-to-main fixed-wing setpoint shared by production and HILS replay. */
struct FwSetpoint
{
    float course{0.0f};
    /* Legacy field name: carries the ground-speed command until the PX4
       boundary converts it to an EAS command. */
    float airspeed{0.0f};
    float height_rate{0.0f};
    float height_setpoint{std::numeric_limits<float>::quiet_NaN()};
    float lateral_acceleration{0.0f};
    bool is_fallback{true};
};

using FwSetpointQueue = collision_avoidance::common::SpscQueue<
    FwSetpoint, kSetpointQueueCapacity>;

}  // namespace collision_avoidance::types
