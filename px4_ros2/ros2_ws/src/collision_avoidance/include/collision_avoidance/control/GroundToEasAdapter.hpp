#pragma once

namespace collision_avoidance::control
{

inline constexpr float kSeaLevelAirDensityKgM3 = 1.225F;

/* Convert a 3-D ground-speed command to the equivalent-airspeed command at
   the PX4 boundary. Height rate is positive up; wind uses NED signs, so
   wind_down is positive down. This is an instantaneous feed-forward
   conversion and does not guarantee closed-loop ground-speed tracking.
   Returns NaN when the inputs do not define a physically valid command. */
float computeRequiredEquivalentAirspeed(
    float desired_ground_speed,
    float desired_ground_course,
    float desired_height_rate_up,
    float wind_north,
    float wind_east,
    float wind_down,
    float air_density) noexcept;

}  // namespace collision_avoidance::control
