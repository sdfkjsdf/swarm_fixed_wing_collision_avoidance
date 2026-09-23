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
    // Value 1 belonged to the removed CPA termination rule in older bags.
    CoordinatedNominalReturnSafe = 2,
};

struct ManeuverActivationSample
{
    std::uint64_t timestamp_us{0};
    bool valid{false};
    double minimum_ad_m{0.0};
    std::uint32_t unsafe_threat_mask{0};
    // Current pair budgets/kinematics are also used by optional Formation
    // discrimination; they are not a second termination criterion.
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
    // A peer in the same committed interaction-graph component observed the
    // AD activation boundary. This closes the contract between a jointly
    // evaluated component tuple and its execution without changing the legacy
    // local-AD trigger when the interaction graph is disabled.
    bool coordinated_activation_requested{false};
    // Explicit permission from the worker: fresh joint nominal rollout,
    // ownship transition against active peers, and all peer confirmations.
    bool allow_deactivation{false};
};

struct ManeuverActivationStatus
{
    bool active{false};
    bool just_activated{false};
    bool just_deactivated{false};
    ManeuverDeactivationReason deactivation_reason{
        ManeuverDeactivationReason::None};
    std::uint64_t activation_timestamp_us{0};
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
    ManeuverActivationController() = default;

    ManeuverActivationStatus update(
        const ManeuverActivationSample & sample) noexcept;
    bool replaceActiveCommand(
        std::uint8_t candidate_id,
        std::uint64_t candidate_input_revision,
        const estimation::PredictInput & input) noexcept;
    ManeuverActivationStatus status() const noexcept;
    void reset() noexcept;

private:
    ManeuverActivationStatus m_status{};
    std::uint64_t m_last_timestamp_us{0};
};

}  // namespace collision_avoidance::selection
