#include <collision_avoidance/selection/ManeuverActivationController.hpp>

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
        const bool separating = sample.valid
            && allAffectedThreatsSeparating(sample);
        const bool timed_out = sample.timestamp_us
                >= m_status.activation_timestamp_us
            && sample.timestamp_us - m_status.activation_timestamp_us
                >= m_params.maximum_active_duration_us;
        if (separating || timed_out) {
            m_status.active = false;
            m_status.just_deactivated = true;
            // Treat a later penetration as a new conflict only after the
            // current predicted cone has first cleared AD = 0.
            m_rearm_required = true;
            m_status.deactivation_reason = separating
                ? ManeuverDeactivationReason::Separating
                : ManeuverDeactivationReason::Timeout;
        }
        return m_status;
    }

    if (m_rearm_required) {
        if (sample.valid && std::isfinite(sample.minimum_ad_m)
            && sample.minimum_ad_m >= 0.0) {
            m_rearm_required = false;
        }
        return m_status;
    }

    if (!sample.valid || !std::isfinite(sample.minimum_ad_m)
        || sample.minimum_ad_m >= 0.0
        || sample.unsafe_threat_mask == 0U) {
        return m_status;
    }

    m_status.active = true;
    m_status.just_activated = true;
    m_status.activation_timestamp_us = sample.timestamp_us;
    m_status.affected_threat_mask = sample.unsafe_threat_mask;
    m_status.latched_candidate_id = sample.selected_candidate_id;
    m_status.latched_candidate_input_revision =
        sample.selected_candidate_input_revision;
    m_status.latched_input = sample.selected_input;
    return m_status;
}

ManeuverActivationStatus ManeuverActivationController::status() const noexcept
{
    return m_status;
}

void ManeuverActivationController::reset() noexcept
{
    m_status = ManeuverActivationStatus{};
    m_last_timestamp_us = 0;
    m_rearm_required = false;
}

bool ManeuverActivationController::allAffectedThreatsSeparating(
    const ManeuverActivationSample & sample) const noexcept
{
    if (!std::isfinite(m_params.separating_rate_threshold_mps)
        || m_params.separating_rate_threshold_mps < 0.0
        || m_status.affected_threat_mask == 0U) {
        return false;
    }

    for (std::size_t aircraft = 0;
         aircraft < sample.separation_rates_mps.size(); ++aircraft) {
        const std::uint32_t bit = std::uint32_t{1} << aircraft;
        if ((m_status.affected_threat_mask & bit) == 0U) {
            continue;
        }
        const double rate = sample.separation_rates_mps[aircraft];
        if (!std::isfinite(rate)
            || rate < m_params.separating_rate_threshold_mps) {
            return false;
        }
    }
    return true;
}

}  // namespace collision_avoidance::selection
