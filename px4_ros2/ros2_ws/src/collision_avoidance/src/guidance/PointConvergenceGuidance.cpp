#include <collision_avoidance/guidance/PointConvergenceGuidance.hpp>

#include <algorithm>
#include <cmath>

namespace collision_avoidance::guidance
{
namespace
{

float wrapPi(float angle) noexcept
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

PointConvergenceGuidance::PointConvergenceGuidance(
    const Parameters & params)
: m_params(params)
{
}

collision_avoidance::types::FwSetpoint
PointConvergenceGuidance::computeFwSetpoint(
    const collision_avoidance::types::AgentState & self,
    float height_setpoint) const noexcept
{
    const float delta_north = m_params.target_north_m - self.pos_n;
    const float delta_east = m_params.target_east_m - self.pos_e;
    const float distance_m = std::hypot(delta_north, delta_east);
    const float desired_course = distance_m > 1.0F
        ? std::atan2(delta_east, delta_north)
        : self.psi;
    const float course_error = wrapPi(desired_course - self.psi);
    const float speed_mps = std::max(
        std::hypot(self.vel_n, self.vel_e), 1.0F);
    const float maximum_lateral_acceleration =
        m_params.gravity_mps2
        * std::tan(
            m_params.maximum_roll_degrees
            * static_cast<float>(M_PI) / 180.0F);

    collision_avoidance::types::FwSetpoint output;
    output.course = self.psi;
    output.airspeed = m_params.ground_speed_command_mps;
    output.height_rate = 0.0F;
    output.height_setpoint = height_setpoint;
    output.lateral_acceleration = std::clamp(
        speed_mps * m_params.course_error_gain_per_s * course_error,
        -maximum_lateral_acceleration,
        maximum_lateral_acceleration);
    output.is_fallback = false;
    return output;
}

}  // namespace collision_avoidance::guidance
