#include <collision_avoidance/control/GroundToEasAdapter.hpp>

#include <cmath>
#include <limits>

namespace collision_avoidance::control
{

float computeRequiredEquivalentAirspeed(
    float desired_ground_speed,
    float desired_ground_course,
    float desired_height_rate_up,
    float wind_north,
    float wind_east,
    float wind_down,
    float air_density) noexcept
{
    const bool inputs_are_valid =
        std::isfinite(desired_ground_speed)
        && std::isfinite(desired_ground_course)
        && std::isfinite(desired_height_rate_up)
        && std::isfinite(wind_north)
        && std::isfinite(wind_east)
        && std::isfinite(wind_down)
        && std::isfinite(air_density)
        && desired_ground_speed >= 0.0F
        && std::abs(desired_height_rate_up) <= desired_ground_speed
        && air_density > 0.0F;
    if (!inputs_are_valid) {
        return std::numeric_limits<float>::quiet_NaN();
    }

    const float horizontal_ground_speed = std::sqrt(std::fmax(
        0.0F,
        desired_ground_speed * desired_ground_speed
            - desired_height_rate_up * desired_height_rate_up));
    const float gs_n = horizontal_ground_speed * std::cos(desired_ground_course);
    const float gs_e = horizontal_ground_speed * std::sin(desired_ground_course);
    const float gs_d = -desired_height_rate_up;
    const float as_n = gs_n - wind_north;
    const float as_e = gs_e - wind_east;
    const float as_d = gs_d - wind_down;
    const float true_airspeed =
        std::sqrt(as_n * as_n + as_e * as_e + as_d * as_d);
    return true_airspeed * std::sqrt(air_density / kSeaLevelAirDensityKgM3);
}

}  // namespace collision_avoidance::control
