#pragma once

#include <array>
#include <cstdint>

#include <collision_avoidance/estimation/trajectory_prediction/PredictTypes.hpp>
#include <collision_avoidance/selection/ManeuverCombinationEvaluator.hpp>

namespace collision_avoidance::selection
{

enum class ManeuverDeactivationReason : std::uint8_t
{
    None = 0,
    Separating,
    Timeout,
};

struct ManeuverActivationControllerParams
{
    std::uint64_t maximum_active_duration_us{4'500'000};
    // 100 ft/s from the disclosed baseline Auto ACAS termination criterion.
    double separating_rate_threshold_mps{30.48};
    // Experimental early-activation boundary [m]. The disclosed AMAC
    // baseline is recovered with zero; a positive value activates while AD
    // still has that much residual separation margin.
    double activation_threshold_m{0.0};
};

struct ManeuverActivationSample
{
    std::uint64_t timestamp_us{0};
    bool valid{false};
    double minimum_ad_m{0.0};
    std::uint32_t unsafe_threat_mask{0};
    std::array<double, kMaximumSelectionAircraft> separation_rates_mps{};
    std::uint8_t selected_candidate_id{0};
    std::uint64_t selected_candidate_input_revision{0};
    estimation::PredictInput selected_input{};
};

struct ManeuverActivationStatus
{
    bool active{false};
    bool just_activated{false};
    bool just_deactivated{false};
    ManeuverDeactivationReason deactivation_reason{
        ManeuverDeactivationReason::None};
    std::uint64_t activation_timestamp_us{0};
    std::uint32_t affected_threat_mask{0};
    std::uint8_t latched_candidate_id{0};
    std::uint64_t latched_candidate_input_revision{0};
    estimation::PredictInput latched_input{};
};

/* ROS-independent AMAC state boundary. Selection may continue in parallel,
   but an active ownship command remains latched until a disclosed baseline
   deactivation condition is met. */
class ManeuverActivationController
{
public:
    explicit ManeuverActivationController(
        const ManeuverActivationControllerParams & params = {});

    ManeuverActivationStatus update(
        const ManeuverActivationSample & sample) noexcept;
    ManeuverActivationStatus status() const noexcept;
    void reset() noexcept;

private:
    bool allAffectedThreatsSeparating(
        const ManeuverActivationSample & sample) const noexcept;

    ManeuverActivationControllerParams m_params;
    ManeuverActivationStatus m_status{};
    std::uint64_t m_last_timestamp_us{0};
    bool m_rearm_required{false};
};

}  // namespace collision_avoidance::selection
