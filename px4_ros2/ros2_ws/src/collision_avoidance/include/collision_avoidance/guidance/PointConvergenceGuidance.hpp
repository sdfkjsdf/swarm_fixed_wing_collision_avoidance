#pragma once

#include <collision_avoidance/common/GlobalTypes.hpp>

namespace collision_avoidance::guidance
{

class PointConvergenceGuidance
{
public:
    struct Parameters
    {
        float target_north_m{300.0F};
        float target_east_m{300.0F};
        float ground_speed_command_mps{20.0F};
        float course_error_gain_per_s{1.2F};
        float maximum_roll_degrees{45.0F};
        float gravity_mps2{9.80665F};
    };

    explicit PointConvergenceGuidance(const Parameters & params);

    collision_avoidance::types::FwSetpoint computeFwSetpoint(
        const collision_avoidance::types::AgentState & self,
        float height_setpoint) const noexcept;

private:
    Parameters m_params;
};

}  // namespace collision_avoidance::guidance
