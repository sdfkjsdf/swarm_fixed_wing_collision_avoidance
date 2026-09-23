#include <collision_avoidance/selection/ManeuverActivationController.hpp>

#include <cmath>

namespace collision_avoidance::selection
{

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
        if (sample.valid && sample.allow_deactivation) {
            m_status.active = false;
            m_status.just_deactivated = true;
            m_status.deactivation_reason =
                ManeuverDeactivationReason::CoordinatedNominalReturnSafe;
        }
        return m_status;
    }

    // Release does not add a cooldown or change the strict AD activation rule.
    const bool local_ad_trigger = std::isfinite(sample.minimum_ad_m)
        && sample.minimum_ad_m < 0.0 && sample.unsafe_threat_mask != 0U;
    if (!sample.allow_new_activation || !sample.valid
        || (!local_ad_trigger && !sample.coordinated_activation_requested)
        || sample.unsafe_threat_mask == 0U) {
        return m_status;
    }

    m_status.active = true;
    m_status.just_activated = true;
    m_status.activation_timestamp_us = sample.timestamp_us;
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

}  // namespace collision_avoidance::selection
