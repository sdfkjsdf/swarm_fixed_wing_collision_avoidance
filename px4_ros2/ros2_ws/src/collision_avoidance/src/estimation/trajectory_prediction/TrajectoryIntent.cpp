#include <collision_avoidance/estimation/trajectory_prediction/TrajectoryIntent.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace collision_avoidance::estimation
{
namespace
{

constexpr std::array<double, kManeuverCandidateCount> kCandidateRollDegrees{
    -50.0, -30.0, -15.0, 0.0, 15.0, 30.0, 50.0};

bool finiteState(const PredictState & state) noexcept
{
    return std::isfinite(state.p_n) && std::isfinite(state.p_e)
        && std::isfinite(state.h) && std::isfinite(state.V)
        && std::isfinite(state.psi) && std::isfinite(state.h_dot)
        && std::isfinite(state.phi);
}

bool usableInput(const PredictInput & input) noexcept
{
    return std::isfinite(input.V_cmd)
        && (std::isfinite(input.h_cmd) || std::isnan(input.h_cmd))
        && std::isfinite(input.h_dot_cmd)
        && std::isfinite(input.a_lat_cmd);
}

std::array<float, kPredictStateDimension> encodeState(
    const PredictState & state) noexcept
{
    return {
        static_cast<float>(state.p_n),
        static_cast<float>(state.p_e),
        static_cast<float>(state.h),
        static_cast<float>(state.V),
        static_cast<float>(state.psi),
        static_cast<float>(state.h_dot),
        static_cast<float>(state.phi)};
}

PredictState decodeState(
    const std::array<float, kPredictStateDimension> & state) noexcept
{
    return {
        state[0], state[1], state[2], state[3],
        state[4], state[5], state[6]};
}

PredictStateCovariance decodeCovariance(
    const std::array<float,
        kPredictStateDimension * kPredictStateDimension> & covariance) noexcept
{
    PredictStateCovariance decoded{};
    std::transform(
        covariance.begin(), covariance.end(), decoded.begin(),
        [](float value) { return static_cast<double>(value); });
    return decoded;
}

bool stateFromPoseVelocity(
    const PoseVel & pose_velocity,
    double roll,
    PredictState & state) noexcept
{
    if (!pose_velocity.pos.allFinite() || !pose_velocity.vel.allFinite()
        || !std::isfinite(roll)) {
        return false;
    }

    const double velocity_north = pose_velocity.vel.x();
    const double velocity_east = pose_velocity.vel.y();
    const double velocity_down = pose_velocity.vel.z();
    const double horizontal_speed = std::hypot(velocity_north, velocity_east);
    const double speed = std::hypot(horizontal_speed, velocity_down);
    if (!(horizontal_speed > 1.0e-6) || !std::isfinite(speed)) {
        return false;
    }

    state = {
        pose_velocity.pos.x(),
        pose_velocity.pos.y(),
        -pose_velocity.pos.z(),
        speed,
        std::atan2(velocity_east, velocity_north),
        -velocity_down,
        roll};
    return finiteState(state);
}

}  // namespace

const PredictInput * ManeuverCandidateTable::find(
    std::uint8_t candidate_id) const noexcept
{
    const auto index = static_cast<std::size_t>(candidate_id);
    return index < inputs.size() ? &inputs[index] : nullptr;
}

ManeuverCandidateTable makeLevelTurnCandidateTable(
    double raw_ground_speed_command,
    double height_command,
    double gravity) noexcept
{
    ManeuverCandidateTable table;
    constexpr double degrees_to_radians = M_PI / 180.0;
    for (std::size_t index = 0; index < table.inputs.size(); ++index) {
        const double roll = kCandidateRollDegrees[index] * degrees_to_radians;
        table.inputs[index] = {
            raw_ground_speed_command,
            height_command,
            0.0,
            gravity * std::tan(roll)};
    }
    return table;
}

TrajectoryIntentSender::TrajectoryIntentSender(
    const TrajectoryPredict & predictor,
    const ManeuverCandidateTable & candidates)
: m_predictor(predictor), m_candidates(candidates)
{
}

bool TrajectoryIntentSender::buildForSelectedCandidate(
    std::uint64_t source_timestamp_us,
    std::uint8_t candidate_id,
    const PredictState & initial_state,
    const PredictStateCovariance & initial_covariance,
    TrajectoryIntentPacket & packet,
    std::uint64_t selection_epoch) const
{
    const PredictInput * input = m_candidates.find(candidate_id);
    if (input == nullptr || !usableInput(*input) || !finiteState(initial_state)
        || !TrajectoryUncertainty::covarianceIsFiniteAndPsd(initial_covariance)) {
        return false;
    }

    PredictionMeanTrajectory predicted_mean{};
    m_predictor.predict(
        initial_state, *input, kTrajectoryIntentStepSeconds, predicted_mean);
    const TrajectorySample compressed_mean =
        m_predictor.extractKeySamples(predicted_mean);
    if (!validateKeySamples(compressed_mean)) {
        return false;
    }

    TrajectoryIntentPacket candidate_packet{};
    candidate_packet.source_timestamp_us = source_timestamp_us;
    candidate_packet.selection_epoch = selection_epoch;
    candidate_packet.candidate_id = candidate_id;
    candidate_packet.initial_state = encodeState(initial_state);
    std::transform(
        initial_covariance.begin(),
        initial_covariance.end(),
        candidate_packet.initial_covariance.begin(),
        [](double value) { return static_cast<float>(value); });
    candidate_packet.compressed_mean = compressed_mean;
    packet = candidate_packet;
    return true;
}

TrajectoryIntentReceiver::TrajectoryIntentReceiver(
    const TrajectoryPredict & predictor,
    const ManeuverCandidateTable & candidates,
    const UncertaintyParams & uncertainty_params)
: m_predictor(predictor),
  m_candidates(candidates),
  m_uncertainty(uncertainty_params)
{
}

bool TrajectoryIntentReceiver::receive(
    const TrajectoryIntentPacket & packet,
    ReceivedTrajectoryIntent & received)
{
    const PredictInput * input = m_candidates.find(packet.candidate_id);
    const PredictState initial_state = decodeState(packet.initial_state);
    const PredictStateCovariance initial_covariance =
        decodeCovariance(packet.initial_covariance);
    if (input == nullptr || !usableInput(*input) || !finiteState(initial_state)
        || !validateKeySamples(packet.compressed_mean)
        || !TrajectoryUncertainty::covarianceIsFiniteAndPsd(initial_covariance)) {
        return false;
    }

    m_reconstructor.calculate_clamp_cubic_spline(packet.compressed_mean);
    PredictionMeanTrajectory reconstructed_mean{};
    PredictState roll_state = initial_state;
    for (std::size_t point = 0; point < kTrajectoryPointCount; ++point) {
        const float time_s = static_cast<float>(
            static_cast<double>(point) * kTrajectoryIntentStepSeconds);
        const PoseVel pose_velocity = m_reconstructor.reconstruct(time_s);
        if (!stateFromPoseVelocity(
                pose_velocity, roll_state.phi, reconstructed_mean[point])) {
            return false;
        }
        if (point + 1 < kTrajectoryPointCount) {
            roll_state = m_predictor.stepRK4(
                roll_state, *input, kTrajectoryIntentStepSeconds);
        }
    }

    PredictionInputTrajectory inputs{};
    inputs.fill(*input);
    ReceivedTrajectoryIntent candidate_received{};
    candidate_received.source_timestamp_us = packet.source_timestamp_us;
    candidate_received.selection_epoch = packet.selection_epoch;
    candidate_received.candidate_id = packet.candidate_id;
    candidate_received.reconstructed_mean = reconstructed_mean;
    if (!m_uncertainty.propagateAlongMean(
            m_predictor,
            reconstructed_mean,
            initial_covariance,
            inputs,
            kTrajectoryIntentStepSeconds,
            candidate_received.cone)) {
        return false;
    }

    received = candidate_received;
    return true;
}

}  // namespace collision_avoidance::estimation
