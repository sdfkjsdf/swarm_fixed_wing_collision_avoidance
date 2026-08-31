#include <collision_avoidance/selection/SafeControlCandidateAdapter.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace collision_avoidance::selection
{
namespace
{

bool finite(double value) noexcept
{
    return std::isfinite(value);
}

bool validAltitudeCommand(double altitude_command_m) noexcept
{
    return finite(altitude_command_m) || std::isnan(altitude_command_m);
}

bool validInterval(
    const HeadingRateInterval & interval,
    SafeControlDirection direction,
    double maximum_rate_radps) noexcept
{
    if (!interval.feasible) {
        return true;
    }
    if (!finite(interval.lower_radps)
        || !finite(interval.upper_radps)
        || interval.lower_radps > interval.upper_radps) {
        return false;
    }

    if (direction == SafeControlDirection::Left) {
        return interval.lower_radps >= 0.0
            && interval.upper_radps <= maximum_rate_radps;
    }
    return interval.lower_radps >= -maximum_rate_radps
        && interval.upper_radps <= 0.0;
}

double clampToInterval(
    double value,
    const HeadingRateInterval & interval) noexcept
{
    return std::clamp(
        value, interval.lower_radps, interval.upper_radps);
}

struct ProjectionChoice
{
    double rate_radps{0.0};
    double distance_radps{std::numeric_limits<double>::infinity()};
    int direction_rank{0};
    bool available{false};
};

void considerProjection(
    double nominal_rate_radps,
    const HeadingRateInterval & interval,
    int direction_rank,
    double tolerance_radps,
    ProjectionChoice & best) noexcept
{
    if (!interval.feasible) {
        return;
    }

    const double projected = clampToInterval(nominal_rate_radps, interval);
    const double distance = std::abs(projected - nominal_rate_radps);
    const bool smaller_distance =
        distance < best.distance_radps - tolerance_radps;
    const bool same_distance =
        std::abs(distance - best.distance_radps) <= tolerance_radps;
    const bool smaller_magnitude = same_distance
        && std::abs(projected) < std::abs(best.rate_radps) - tolerance_radps;
    const bool same_magnitude = same_distance
        && std::abs(std::abs(projected) - std::abs(best.rate_radps))
            <= tolerance_radps;
    const bool earlier_direction = same_magnitude
        && direction_rank < best.direction_rank;

    if (!best.available || smaller_distance || smaller_magnitude
        || earlier_direction) {
        best.rate_radps = projected;
        best.distance_radps = distance;
        best.direction_rank = direction_rank;
        best.available = true;
    }
}

double robustRepresentative(
    const HeadingRateInterval & interval,
    SafeControlDirection direction,
    double robustness_guard_radps) noexcept
{
    const double width = interval.upper_radps - interval.lower_radps;
    if (width < 2.0 * robustness_guard_radps) {
        return interval.lower_radps + 0.5 * width;
    }
    return direction == SafeControlDirection::Left
        ? interval.upper_radps - robustness_guard_radps
        : interval.lower_radps + robustness_guard_radps;
}

bool isDuplicate(
    const SafeControlCandidateAdapterResult & result,
    double rate_radps,
    double tolerance_radps) noexcept
{
    for (std::size_t index = 0; index < result.candidate_count; ++index) {
        if (std::abs(
                result.candidates[index].heading_rate_v4_radps
                - rate_radps) <= tolerance_radps) {
            return true;
        }
    }
    return false;
}

bool appendCandidate(
    SafeCandidateRole role,
    double rate_radps,
    const SafeControlCandidateAdapterInput & input,
    const SafeControlCandidateAdapterParams & params,
    SafeControlCandidateAdapterResult & result) noexcept
{
    if (result.candidate_count >= result.candidates.size()
        || isDuplicate(
            result, rate_radps, params.duplicate_tolerance_radps)) {
        return true;
    }

    const double lateral_acceleration =
        SafeControlCandidateAdapter::v4HeadingRateToPx4LateralAcceleration(
            input.true_airspeed_mps,
            rate_radps,
            params.speed_tolerance_mps);
    if (!finite(lateral_acceleration)) {
        return false;
    }

    auto & candidate = result.candidates[result.candidate_count++];
    candidate.role = role;
    candidate.heading_rate_v4_radps = rate_radps;
    candidate.predictor_input.V_cmd = input.ground_speed_command_mps;
    candidate.predictor_input.h_cmd = input.altitude_command_m;
    candidate.predictor_input.h_dot_cmd = 0.0;
    candidate.predictor_input.a_lat_cmd = lateral_acceleration;
    return true;
}

bool appendBackupCandidate(
    SafeCandidateRole role,
    double rate_radps,
    const BackupControlCandidateAdapterInputV4 & input,
    const SafeControlCandidateAdapterParams & params,
    SafeControlCandidateAdapterResult & result) noexcept
{
    if (result.candidate_count >= result.candidates.size()
        || isDuplicate(result, rate_radps, params.duplicate_tolerance_radps)) {
        return true;
    }
    const double lateral_acceleration =
        SafeControlCandidateAdapter::v4HeadingRateToPx4LateralAcceleration(
            input.true_airspeed_mps,
            rate_radps,
            params.speed_tolerance_mps);
    if (!finite(lateral_acceleration)) {
        return false;
    }
    auto & candidate = result.candidates[result.candidate_count++];
    candidate.role = role;
    candidate.heading_rate_v4_radps = rate_radps;
    candidate.predictor_input.V_cmd = input.ground_speed_command_mps;
    candidate.predictor_input.h_cmd = input.altitude_command_m;
    candidate.predictor_input.h_dot_cmd = 0.0;
    candidate.predictor_input.a_lat_cmd = lateral_acceleration;
    return true;
}

}  // namespace

SafeControlCandidateAdapter::SafeControlCandidateAdapter(
    const SafeControlCandidateAdapterParams & params)
: m_params(params)
{
}

const SafeControlCandidateAdapterParams &
SafeControlCandidateAdapter::params() const noexcept
{
    return m_params;
}

bool SafeControlCandidateAdapter::validParams(
    const SafeControlCandidateAdapterParams & params) noexcept
{
    return finite(params.robustness_guard_radps)
        && params.robustness_guard_radps > 0.0
        && finite(params.duplicate_tolerance_radps)
        && params.duplicate_tolerance_radps > 0.0
        && finite(params.speed_tolerance_mps)
        && params.speed_tolerance_mps > 0.0;
}

double SafeControlCandidateAdapter::v4HeadingRateToPx4LateralAcceleration(
    double true_airspeed_mps,
    double heading_rate_v4_radps,
    double speed_tolerance_mps) noexcept
{
    if (!finite(true_airspeed_mps)
        || !finite(heading_rate_v4_radps)
        || !finite(speed_tolerance_mps)
        || speed_tolerance_mps <= 0.0
        || true_airspeed_mps <= speed_tolerance_mps) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double acceleration =
        -true_airspeed_mps * heading_rate_v4_radps;
    return finite(acceleration)
        ? acceleration
        : std::numeric_limits<double>::quiet_NaN();
}

double SafeControlCandidateAdapter::px4LateralAccelerationToV4HeadingRate(
    double true_airspeed_mps,
    double lateral_acceleration_px4_mps2,
    double speed_tolerance_mps) noexcept
{
    if (!finite(true_airspeed_mps)
        || !finite(lateral_acceleration_px4_mps2)
        || !finite(speed_tolerance_mps)
        || speed_tolerance_mps <= 0.0
        || true_airspeed_mps <= speed_tolerance_mps) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double heading_rate =
        -lateral_acceleration_px4_mps2 / true_airspeed_mps;
    return finite(heading_rate)
        ? heading_rate
        : std::numeric_limits<double>::quiet_NaN();
}

SafeControlCandidateAdapterResult SafeControlCandidateAdapter::generate(
    const SafeControlCandidateAdapterInput & input) const noexcept
{
    SafeControlCandidateAdapterResult result{};
    if (!validParams(m_params)) {
        result.status =
            SafeControlCandidateAdapterStatus::InvalidConfiguration;
        return result;
    }
    if (input.safe_set.status
        == SafeControlSetStatus::SearchSetInfeasible) {
        result.status =
            SafeControlCandidateAdapterStatus::SearchSetInfeasible;
        return result;
    }
    if (input.safe_set.status != SafeControlSetStatus::Valid
        || !finite(input.safe_set.effective_max_heading_rate_radps)
        || input.safe_set.effective_max_heading_rate_radps <= 0.0
        || !validInterval(
            input.safe_set.left_safe,
            SafeControlDirection::Left,
            input.safe_set.effective_max_heading_rate_radps)
        || !validInterval(
            input.safe_set.right_safe,
            SafeControlDirection::Right,
            input.safe_set.effective_max_heading_rate_radps)
        || (!input.safe_set.left_safe.feasible
            && !input.safe_set.right_safe.feasible)) {
        result.status = SafeControlCandidateAdapterStatus::InvalidSafeSet;
        return result;
    }
    if (!finite(input.true_airspeed_mps)
        || input.true_airspeed_mps <= m_params.speed_tolerance_mps) {
        result.status = SafeControlCandidateAdapterStatus::InvalidAirspeed;
        return result;
    }
    if (!finite(input.ground_speed_command_mps)
        || input.ground_speed_command_mps <= 0.0
        || !validAltitudeCommand(input.altitude_command_m)) {
        result.status =
            SafeControlCandidateAdapterStatus::InvalidPredictorCommand;
        return result;
    }
    if (input.nominal_rate_available
        && !finite(input.nominal_heading_rate_v4_radps)) {
        result.status = SafeControlCandidateAdapterStatus::InvalidNominalRate;
        return result;
    }

    if (input.nominal_rate_available) {
        ProjectionChoice projection;
        // Direction-rank tie break is Left before Right.
        considerProjection(
            input.nominal_heading_rate_v4_radps,
            input.safe_set.left_safe,
            0,
            m_params.duplicate_tolerance_radps,
            projection);
        considerProjection(
            input.nominal_heading_rate_v4_radps,
            input.safe_set.right_safe,
            1,
            m_params.duplicate_tolerance_radps,
            projection);
        if (!projection.available
            || !appendCandidate(
                SafeCandidateRole::NearNominal,
                projection.rate_radps,
                input,
                m_params,
                result)) {
            result.status =
                SafeControlCandidateAdapterStatus::InvalidSafeSet;
            result.candidate_count = 0;
            return result;
        }
    }

    if (input.safe_set.left_safe.feasible) {
        const double representative = robustRepresentative(
            input.safe_set.left_safe,
            SafeControlDirection::Left,
            m_params.robustness_guard_radps);
        if (!appendCandidate(
                SafeCandidateRole::RobustLeft,
                representative,
                input,
                m_params,
                result)) {
            result.status =
                SafeControlCandidateAdapterStatus::InvalidSafeSet;
            result.candidate_count = 0;
            return result;
        }
    }

    if (input.safe_set.right_safe.feasible) {
        const double representative = robustRepresentative(
            input.safe_set.right_safe,
            SafeControlDirection::Right,
            m_params.robustness_guard_radps);
        if (!appendCandidate(
                SafeCandidateRole::RobustRight,
                representative,
                input,
                m_params,
                result)) {
            result.status =
                SafeControlCandidateAdapterStatus::InvalidSafeSet;
            result.candidate_count = 0;
            return result;
        }
    }

    result.status = SafeControlCandidateAdapterStatus::Valid;
    return result;
}

SafeControlCandidateAdapterResult
SafeControlCandidateAdapter::generateFromBackupInterpolation(
    const BackupControlCandidateAdapterInputV4 & input) const noexcept
{
    SafeControlCandidateAdapterResult result{};
    if (!validParams(m_params)) {
        result.status =
            SafeControlCandidateAdapterStatus::InvalidConfiguration;
        return result;
    }
    if (!finite(input.true_airspeed_mps)
        || input.true_airspeed_mps <= m_params.speed_tolerance_mps) {
        result.status = SafeControlCandidateAdapterStatus::InvalidAirspeed;
        return result;
    }
    if (!finite(input.ground_speed_command_mps)
        || input.ground_speed_command_mps <= 0.0
        || !validAltitudeCommand(input.altitude_command_m)) {
        result.status =
            SafeControlCandidateAdapterStatus::InvalidPredictorCommand;
        return result;
    }
    if (input.interpolation.status
            == BackupInterpolationStatusV4::NoCertifiedBranch
        || input.interpolation.status
            == BackupInterpolationStatusV4::InterpolationInfeasible) {
        result.status =
            SafeControlCandidateAdapterStatus::SearchSetInfeasible;
        return result;
    }
    if (input.interpolation.status != BackupInterpolationStatusV4::Valid) {
        result.status = SafeControlCandidateAdapterStatus::InvalidSafeSet;
        return result;
    }

    const auto branchFeasible = [](const auto & branch) noexcept {
        return branch.status == BackupInterpolationBranchStatusV4::Feasible
            && finite(branch.safe_heading_rate_v4_radps)
            && finite(branch.nominal_heading_rate_v4_radps);
    };
    const bool left_feasible = branchFeasible(input.interpolation.left);
    const bool right_feasible = branchFeasible(input.interpolation.right);
    if (!left_feasible && !right_feasible) {
        result.status =
            SafeControlCandidateAdapterStatus::SearchSetInfeasible;
        return result;
    }

    const auto & nearest = !right_feasible
        || (left_feasible
            && std::abs(input.interpolation.left.safe_heading_rate_v4_radps
                    - input.interpolation.left.nominal_heading_rate_v4_radps)
                <= std::abs(
                    input.interpolation.right.safe_heading_rate_v4_radps
                    - input.interpolation.right.nominal_heading_rate_v4_radps))
        ? input.interpolation.left
        : input.interpolation.right;
    if (!appendBackupCandidate(
            SafeCandidateRole::NearNominal,
            nearest.safe_heading_rate_v4_radps,
            input,
            m_params,
            result)) {
        result.status = SafeControlCandidateAdapterStatus::InvalidSafeSet;
        result.candidate_count = 0;
        return result;
    }
    if (left_feasible
        && !appendBackupCandidate(
            SafeCandidateRole::RobustLeft,
            input.interpolation.left.safe_heading_rate_v4_radps,
            input,
            m_params,
            result)) {
        result.status = SafeControlCandidateAdapterStatus::InvalidSafeSet;
        result.candidate_count = 0;
        return result;
    }
    if (right_feasible
        && !appendBackupCandidate(
            SafeCandidateRole::RobustRight,
            input.interpolation.right.safe_heading_rate_v4_radps,
            input,
            m_params,
            result)) {
        result.status = SafeControlCandidateAdapterStatus::InvalidSafeSet;
        result.candidate_count = 0;
        return result;
    }

    result.status = SafeControlCandidateAdapterStatus::Valid;
    return result;
}

const char * safeControlCandidateAdapterStatusName(
    SafeControlCandidateAdapterStatus status) noexcept
{
    switch (status) {
    case SafeControlCandidateAdapterStatus::Valid:
        return "valid";
    case SafeControlCandidateAdapterStatus::SearchSetInfeasible:
        return "search_set_infeasible";
    case SafeControlCandidateAdapterStatus::InvalidConfiguration:
        return "invalid_configuration";
    case SafeControlCandidateAdapterStatus::InvalidSafeSet:
        return "invalid_safe_set";
    case SafeControlCandidateAdapterStatus::InvalidAirspeed:
        return "invalid_airspeed";
    case SafeControlCandidateAdapterStatus::InvalidPredictorCommand:
        return "invalid_predictor_command";
    case SafeControlCandidateAdapterStatus::InvalidNominalRate:
        return "invalid_nominal_rate";
    }
    return "unknown";
}

}  // namespace collision_avoidance::selection
