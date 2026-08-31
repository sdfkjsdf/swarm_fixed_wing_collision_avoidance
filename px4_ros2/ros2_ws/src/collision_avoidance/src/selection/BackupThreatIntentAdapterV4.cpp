#include <collision_avoidance/selection/BackupThreatIntentAdapterV4.hpp>

#include <algorithm>
#include <array>
#include <cmath>

namespace collision_avoidance::selection
{
namespace
{

bool finite(double value) noexcept
{
    return std::isfinite(value);
}

bool validState(const estimation::PredictState & state) noexcept
{
    return finite(state.p_n) && finite(state.p_e) && finite(state.h)
        && finite(state.V) && state.V > 0.0 && finite(state.psi)
        && finite(state.h_dot) && finite(state.phi);
}

bool validInput(const estimation::PredictInput & input) noexcept
{
    return finite(input.V_cmd) && input.V_cmd > 0.0
        && (finite(input.h_cmd) || std::isnan(input.h_cmd))
        && finite(input.h_dot_cmd) && finite(input.a_lat_cmd);
}

estimation::PredictState interpolateState(
    const estimation::PredictionMeanTrajectory & trajectory,
    double age_s) noexcept
{
    const double fractional_index = age_s
        / estimation::kTrajectoryIntentStepSeconds;
    std::size_t lower = static_cast<std::size_t>(
        std::floor(fractional_index));
    lower = std::min(lower, estimation::kTrajectoryPointCount - 1);
    const std::size_t upper = std::min(
        lower + 1, estimation::kTrajectoryPointCount - 1);
    const double alpha = lower == upper
        ? 0.0
        : std::clamp(
            fractional_index - static_cast<double>(lower), 0.0, 1.0);
    const auto blend = [alpha](double lhs, double rhs) noexcept {
        return (1.0 - alpha) * lhs + alpha * rhs;
    };
    const auto & first = trajectory[lower];
    const auto & second = trajectory[upper];
    return estimation::PredictState{
        blend(first.p_n, second.p_n),
        blend(first.p_e, second.p_e),
        blend(first.h, second.h),
        blend(first.V, second.V),
        first.psi + alpha * std::remainder(
            second.psi - first.psi, 2.0 * std::acos(-1.0)),
        blend(first.h_dot, second.h_dot),
        blend(first.phi, second.phi)};
}

}  // namespace

BackupThreatIntentAdapterV4::BackupThreatIntentAdapterV4(
    const BackupThreatIntentAdapterV4Params & params)
: m_params(params),
  m_predictor(params.predictor)
{
}

const BackupThreatIntentAdapterV4Params &
BackupThreatIntentAdapterV4::params() const noexcept
{
    return m_params;
}

bool BackupThreatIntentAdapterV4::validParams(
    const BackupThreatIntentAdapterV4Params & params) noexcept
{
    if (!finite(params.horizon_s) || params.horizon_s <= 0.0
        || !finite(params.integration_step_s)
        || params.integration_step_s <= 0.0
        || params.maximum_intent_age_us == 0
        || !finite(params.time_tolerance_s)
        || params.time_tolerance_s <= 0.0) {
        return false;
    }
    const double interval_count = params.horizon_s
        / params.integration_step_s;
    const double rounded_interval_count = std::round(interval_count);
    return finite(interval_count)
        && std::abs(interval_count - rounded_interval_count)
            <= params.time_tolerance_s
        && rounded_interval_count >= 1.0
        && rounded_interval_count + 1.0
            <= static_cast<double>(kMaximumBackupTrajectoryPointsV4);
}

BackupThreatIntentAdapterV4Result
BackupThreatIntentAdapterV4::alignAndPropagate(
    std::uint64_t evaluation_timestamp_us,
    int vehicle_id,
    double physical_clearance_m,
    const estimation::ReceivedTrajectoryIntent & intent) const noexcept
{
    BackupThreatIntentAdapterV4Result result{};
    if (!validParams(m_params)) {
        result.status = BackupThreatIntentStatusV4::InvalidConfiguration;
        return result;
    }
    if (vehicle_id < 0 || !finite(physical_clearance_m)
        || physical_clearance_m < 0.0 || !validInput(intent.candidate_input)) {
        result.status = BackupThreatIntentStatusV4::InvalidIntent;
        return result;
    }
    if (intent.source_timestamp_us > evaluation_timestamp_us) {
        result.status = BackupThreatIntentStatusV4::FutureIntent;
        return result;
    }
    result.source_age_us = evaluation_timestamp_us
        - intent.source_timestamp_us;
    if (result.source_age_us > m_params.maximum_intent_age_us) {
        result.status = BackupThreatIntentStatusV4::StaleIntent;
        return result;
    }
    const double age_s = static_cast<double>(result.source_age_us) * 1.0e-6;
    const double available_horizon_s = static_cast<double>(
        estimation::kTrajectoryPointCount - 1)
        * estimation::kTrajectoryIntentStepSeconds;
    if (!finite(age_s) || age_s < 0.0
        || age_s > available_horizon_s + m_params.time_tolerance_s) {
        result.status = BackupThreatIntentStatusV4::StaleIntent;
        return result;
    }

    const estimation::PredictState aligned_initial = interpolateState(
        intent.reconstructed_mean, age_s);
    if (!validState(aligned_initial)) {
        result.status = BackupThreatIntentStatusV4::InvalidIntent;
        return result;
    }

    std::array<
        estimation::PredictState,
        kMaximumBackupTrajectoryPointsV4> propagated{};
    const std::size_t point_count = static_cast<std::size_t>(
        std::llround(m_params.horizon_s / m_params.integration_step_s)) + 1;
    propagated[0] = aligned_initial;
    for (std::size_t index = 1; index < point_count; ++index) {
        propagated[index] = m_predictor.stepRK4(
            propagated[index - 1],
            intent.candidate_input,
            m_params.integration_step_s);
        if (!validState(propagated[index])) {
            result.status = BackupThreatIntentStatusV4::PropagationFailed;
            return result;
        }
    }

    result.trajectory.vehicle_id = vehicle_id;
    result.trajectory.source_timestamp_us = evaluation_timestamp_us;
    result.trajectory.physical_clearance_m = physical_clearance_m;
    result.trajectory.point_count = point_count;
    for (std::size_t index = 0; index < point_count; ++index) {
        auto & point = result.trajectory.points[index];
        point.time_offset_s = static_cast<double>(index)
            * m_params.integration_step_s;
        point.north_m = propagated[index].p_n;
        point.east_m = propagated[index].p_e;
    }
    result.status = BackupThreatIntentStatusV4::Valid;
    return result;
}

const char * backupThreatIntentStatusName(
    BackupThreatIntentStatusV4 status) noexcept
{
    switch (status) {
    case BackupThreatIntentStatusV4::Valid:
        return "valid";
    case BackupThreatIntentStatusV4::InvalidConfiguration:
        return "invalid_configuration";
    case BackupThreatIntentStatusV4::InvalidIntent:
        return "invalid_intent";
    case BackupThreatIntentStatusV4::FutureIntent:
        return "future_intent";
    case BackupThreatIntentStatusV4::StaleIntent:
        return "stale_intent";
    case BackupThreatIntentStatusV4::PropagationFailed:
        return "propagation_failed";
    }
    return "unknown";
}

}  // namespace collision_avoidance::selection
