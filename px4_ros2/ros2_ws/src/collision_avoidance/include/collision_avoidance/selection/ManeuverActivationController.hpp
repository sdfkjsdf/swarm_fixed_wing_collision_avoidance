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
    FutureCpaClear,
};

struct ManeuverActivationControllerParams
{
    // Project reconstruction for the unpublished Lockheed CPA special case.
    // Below this squared relative speed, future relative position is treated
    // as constant instead of dividing by a near-zero denominator.
    double relative_speed_squared_epsilon_m2ps2{1.0e-12};
};

struct ManeuverActivationSample
{
    std::uint64_t timestamp_us{0};
    bool valid{false};
    double minimum_ad_m{0.0};
    std::uint32_t unsafe_threat_mask{0};
    std::array<double, kMaximumSelectionAircraft> activation_criteria_m{};
    std::array<std::array<double, 3>, kMaximumSelectionAircraft>
        relative_positions_ned_m{};
    std::array<std::array<double, 3>, kMaximumSelectionAircraft>
        relative_velocities_ned_mps{};
    std::uint8_t selected_candidate_id{0};
    std::uint64_t selected_candidate_input_revision{0};
    estimation::PredictInput selected_input{};
    // Formation discrimination may inhibit only a new activation. It never
    // terminates or resets an already active avoidance episode.
    bool allow_new_activation{true};
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
    std::array<double, kMaximumSelectionAircraft> activation_criteria_m{};
    std::uint8_t latched_candidate_id{0};
    std::uint64_t latched_candidate_input_revision{0};
    estimation::PredictInput latched_input{};
};

/* ROS-independent AMAC state boundary. Selection continues independently.
   A coordinated replacement may update the active command without ending the
   activation episode. */
class ManeuverActivationController
{
public:
    explicit ManeuverActivationController(
        const ManeuverActivationControllerParams & params = {});

    ManeuverActivationStatus update(
        const ManeuverActivationSample & sample) noexcept;
    bool futureCpaClear(
        const ManeuverActivationSample & sample) const noexcept;
    bool replaceActiveCommand(
        std::uint8_t candidate_id,
        std::uint64_t candidate_input_revision,
        const estimation::PredictInput & input) noexcept;
    ManeuverActivationStatus status() const noexcept;
    void reset() noexcept;

private:
    void addNewAffectedThreats(
        const ManeuverActivationSample & sample) noexcept;
    bool futureCpaDistance(
        const std::array<double, 3> & relative_position_ned_m,
        const std::array<double, 3> & relative_velocity_ned_mps,
        double & distance_m) const noexcept;

    ManeuverActivationControllerParams m_params;
    ManeuverActivationStatus m_status{};
    std::uint64_t m_last_timestamp_us{0};
};

}  // namespace collision_avoidance::selection
