#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include <collision_avoidance/estimation/reconstruction/ReconstructTrajectory.hpp>
#include <collision_avoidance/estimation/trajectory_prediction/TrajectoryUncertainty.hpp>

namespace collision_avoidance::estimation
{

inline constexpr double kTrajectoryIntentStepSeconds = 0.1;
inline constexpr std::size_t kManeuverCandidateCount = 7;
inline constexpr std::size_t kTrajectoryIntentInputDimension = 4;

enum class ManeuverCandidateId : std::uint8_t
{
    RollMinus50 = 0,
    RollMinus30,
    RollMinus15,
    RollZero,
    RollPlus15,
    RollPlus30,
    RollPlus50,
};

struct ManeuverCandidateTable
{
    std::array<PredictInput, kManeuverCandidateCount> inputs{};

    const PredictInput * find(std::uint8_t candidate_id) const noexcept;
};

ManeuverCandidateTable makeLevelTurnCandidateTable(
    double raw_ground_speed_command,
    double height_command,
    double gravity = 9.80665) noexcept;

/* Fixed-size, ROS-independent A-to-B payload. Predictor parameters, horizon
   and sample times remain a preflight contract.  The selected candidate's
   actual input is dynamic and therefore travels with the compact mean. */
struct TrajectoryIntentPacket
{
    std::uint64_t source_timestamp_us{0};
    std::uint64_t selection_epoch{0};
    std::array<float, kPredictStateDimension> initial_state{};
    std::array<float,
        kPredictStateDimension * kPredictStateDimension> initial_covariance{};
    TrajectorySample compressed_mean{};
    std::uint8_t candidate_id{
        static_cast<std::uint8_t>(ManeuverCandidateId::RollZero)};
    // [V_cmd, h_cmd, h_dot_cmd, a_lat_cmd].  h_cmd may be NaN when unused.
    std::array<float, kTrajectoryIntentInputDimension> candidate_input{};
    // Deterministic identity of candidate_id + transmitted float32 input.
    std::uint64_t candidate_input_revision{0};
};

static_assert(std::is_trivially_copyable_v<TrajectoryIntentPacket>);

struct ReceivedTrajectoryIntent
{
    std::uint64_t source_timestamp_us{0};
    std::uint64_t selection_epoch{0};
    std::uint8_t candidate_id{0};
    PredictInput candidate_input{};
    std::uint64_t candidate_input_revision{0};
    PredictionMeanTrajectory reconstructed_mean{};
    TrajectoryCone cone{};
};

/* A-side module: legacy callers may resolve an ID through the fixed table;
   dynamic callers supply the actual candidate input.  Both paths predict and
   build the same compact payload. */
class TrajectoryIntentSender
{
public:
    TrajectoryIntentSender(
        const TrajectoryPredict & predictor,
        const ManeuverCandidateTable & candidates);

    bool buildForSelectedCandidate(
        std::uint64_t source_timestamp_us,
        std::uint8_t candidate_id,
        const PredictState & initial_state,
        const PredictStateCovariance & initial_covariance,
        TrajectoryIntentPacket & packet,
        std::uint64_t selection_epoch = 0) const;

    bool buildForCandidateInput(
        std::uint64_t source_timestamp_us,
        std::uint8_t candidate_id,
        const PredictInput & candidate_input,
        const PredictState & initial_state,
        const PredictStateCovariance & initial_covariance,
        TrajectoryIntentPacket & packet,
        std::uint64_t selection_epoch = 0) const;

private:
    TrajectoryPredict m_predictor;
    ManeuverCandidateTable m_candidates;
};

/* B-side module: consumes the copied/deserialized packet, reconstructs the
   46-point mean and propagates P0 along that reconstructed mean. */
class TrajectoryIntentReceiver
{
public:
    TrajectoryIntentReceiver(
        const TrajectoryPredict & predictor,
        const UncertaintyParams & uncertainty_params = {});

    bool receive(
        const TrajectoryIntentPacket & packet,
        ReceivedTrajectoryIntent & received);

private:
    TrajectoryPredict m_predictor;
    TrajectoryUncertainty m_uncertainty;
    ReconstructTrajectory m_reconstructor;
};

}  // namespace collision_avoidance::estimation
