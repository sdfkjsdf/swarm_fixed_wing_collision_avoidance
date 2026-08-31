#include <collision_avoidance/selection/ManeuverActivationController.hpp>

#include <algorithm>
#include <cmath>

namespace collision_avoidance::selection
{

ManeuverActivationController::ManeuverActivationController(
    const ManeuverActivationControllerParams & params)
: m_params(params)
{
}

ManeuverActivationStatus ManeuverActivationController::update(
    const ManeuverActivationSample & sample) noexcept
{
    m_status.just_activated = false;
    m_status.just_deactivated = false;
    m_status.deactivation_reason = ManeuverDeactivationReason::None;

    if (sample.timestamp_us < m_last_timestamp_us) {
        return m_status;
    }
    m_last_timestamp_us = sample.timestamp_us;

    if (m_status.active) {
        if (sample.valid) {
            addNewAffectedThreats(sample);
        }
        const bool future_cpa_clear = sample.valid && futureCpaClear(sample);
        if (future_cpa_clear) {
            m_status.active = false;
            m_status.just_deactivated = true;
            m_status.deactivation_reason =
                ManeuverDeactivationReason::FutureCpaClear;
        }
        return m_status;
    }

    if (!sample.allow_new_activation
        || !sample.valid || !std::isfinite(sample.minimum_ad_m)
        || sample.minimum_ad_m >= 0.0
        || sample.unsafe_threat_mask == 0U) {
        return m_status;
    }

    m_status.active = true;
    m_status.just_activated = true;
    m_status.activation_timestamp_us = sample.timestamp_us;
    m_status.affected_threat_mask = sample.unsafe_threat_mask;
    for (std::size_t aircraft = 0;
         aircraft < sample.activation_criteria_m.size(); ++aircraft) {
        const std::uint32_t bit = std::uint32_t{1} << aircraft;
        if ((sample.unsafe_threat_mask & bit) != 0U) {
            m_status.activation_criteria_m[aircraft] =
                sample.activation_criteria_m[aircraft];
        }
    }
    m_status.latched_candidate_id = sample.selected_candidate_id;
    m_status.latched_candidate_input_revision =
        sample.selected_candidate_input_revision;
    m_status.latched_input = sample.selected_input;
    return m_status;
}

bool ManeuverActivationController::replaceActiveCommand(
    std::uint8_t candidate_id,
    std::uint64_t candidate_input_revision,
    const estimation::PredictInput & input) noexcept
{
    if (!m_status.active) {
        return false;
    }
    m_status.latched_candidate_id = candidate_id;
    m_status.latched_candidate_input_revision = candidate_input_revision;
    m_status.latched_input = input;
    m_status.just_activated = false;
    m_status.just_deactivated = false;
    m_status.deactivation_reason = ManeuverDeactivationReason::None;
    return true;
}

ManeuverActivationStatus ManeuverActivationController::status() const noexcept
{
    return m_status;
}

void ManeuverActivationController::reset() noexcept
{
    m_status = ManeuverActivationStatus{};
    m_last_timestamp_us = 0;
}

void ManeuverActivationController::addNewAffectedThreats(
    const ManeuverActivationSample & sample) noexcept
{
    const std::uint32_t new_threat_mask =
        sample.unsafe_threat_mask & ~m_status.affected_threat_mask;
    for (std::size_t aircraft = 0;
         aircraft < sample.activation_criteria_m.size(); ++aircraft) {
        const std::uint32_t bit = std::uint32_t{1} << aircraft;
        if ((new_threat_mask & bit) != 0U) {
            m_status.activation_criteria_m[aircraft] =
                sample.activation_criteria_m[aircraft];
        }
    }
    m_status.affected_threat_mask |= sample.unsafe_threat_mask;
}

bool ManeuverActivationController::futureCpaClear(
    const ManeuverActivationSample & sample) const noexcept
{
    if (m_status.affected_threat_mask == 0U) {
        return false;
    }

    for (std::size_t aircraft = 0;
         aircraft < sample.activation_criteria_m.size(); ++aircraft) {
        const std::uint32_t bit = std::uint32_t{1} << aircraft;
        if ((m_status.affected_threat_mask & bit) == 0U) {
            continue;
        }
        const double activation_criterion_m =
            m_status.activation_criteria_m[aircraft];
        double cpa_distance_m = 0.0;
        if (!std::isfinite(activation_criterion_m)
            || activation_criterion_m < 0.0
            || !futureCpaDistance(
                sample.relative_positions_ned_m[aircraft],
                sample.relative_velocities_ned_mps[aircraft],
                cpa_distance_m)
            || cpa_distance_m <= activation_criterion_m) {
            return false;
        }
    }
    return true;
}

bool ManeuverActivationController::futureCpaDistance(
    const std::array<double, 3> & relative_position_ned_m,
    const std::array<double, 3> & relative_velocity_ned_mps,
    double & distance_m) const noexcept
{
    double relative_speed_squared_m2ps2 = 0.0;
    double radial_product_m2ps = 0.0;
    for (std::size_t axis = 0; axis < relative_position_ned_m.size(); ++axis) {
        const double position = relative_position_ned_m[axis];
        const double velocity = relative_velocity_ned_mps[axis];
        if (!std::isfinite(position) || !std::isfinite(velocity)) {
            return false;
        }
        relative_speed_squared_m2ps2 += velocity * velocity;
        radial_product_m2ps += position * velocity;
    }
    if (!std::isfinite(relative_speed_squared_m2ps2)
        || !std::isfinite(radial_product_m2ps)) {
        return false;
    }

    double cpa_time_s = 0.0;
    if (relative_speed_squared_m2ps2
        > m_params.relative_speed_squared_epsilon_m2ps2) {
        cpa_time_s = std::max(
            0.0, -radial_product_m2ps / relative_speed_squared_m2ps2);
    }

    double distance_squared_m2 = 0.0;
    for (std::size_t axis = 0; axis < relative_position_ned_m.size(); ++axis) {
        const double cpa_position = relative_position_ned_m[axis]
            + relative_velocity_ned_mps[axis] * cpa_time_s;
        distance_squared_m2 += cpa_position * cpa_position;
    }
    if (!std::isfinite(distance_squared_m2) || distance_squared_m2 < 0.0) {
        return false;
    }
    distance_m = std::sqrt(distance_squared_m2);
    return std::isfinite(distance_m);
}

}  // namespace collision_avoidance::selection
