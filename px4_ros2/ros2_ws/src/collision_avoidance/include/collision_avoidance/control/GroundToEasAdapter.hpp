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

/* Apply the accelerated-stall minimum EAS associated with a coordinated
   turn command. The lateral acceleration command represents
   phi_cmd = atan2(a_lat_cmd, gravity), so the turn load factor is
   n = 1 / cos(phi_cmd). The returned command is
   max(raw_equivalent_airspeed, minimum_level_eas * sqrt(n)).
   Returns NaN when an input is invalid. */
float applyTurnMinimumEquivalentAirspeed(
    float raw_equivalent_airspeed,
    float minimum_level_equivalent_airspeed,
    float lateral_acceleration_command,
    float gravity) noexcept;

}  // namespace collision_avoidance::control
