#pragma once

#include <collision_avoidance/selection/ManeuverSelectionWorker.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace collision_avoidance::selection::worker_detail
{
constexpr std::uint8_t kRollZeroId = static_cast<std::uint8_t>(
    estimation::ManeuverCandidateId::RollZero);
constexpr double kTrajectoryStepSeconds =
    estimation::kTrajectoryIntentStepSeconds;
constexpr double kTrajectoryHorizonSeconds =
    estimation::kTrajectoryIntentHorizonSeconds;
constexpr std::uint64_t kTrajectoryHorizonMicroseconds =
    static_cast<std::uint64_t>(kTrajectoryHorizonSeconds * 1.0e6);
constexpr std::uint64_t kInteractionFnvOffset = 14695981039346656037ULL;
constexpr std::uint64_t kInteractionFnvPrime = 1099511628211ULL;

inline std::uint64_t assembledCandidateHash(
    std::uint64_t graph_hash,
    const std::array<std::uint8_t, kMaximumSelectionAircraft> & candidate_ids,
    std::uint32_t candidate_valid_mask,
    std::size_t aircraft_count) noexcept
{
    std::uint64_t hash = kInteractionFnvOffset;
    const auto mix = [&hash](std::uint8_t byte) {
        hash ^= byte;
        hash *= kInteractionFnvPrime;
    };
    for (std::size_t byte = 0; byte < sizeof(graph_hash); ++byte) {
        mix(static_cast<std::uint8_t>(graph_hash >> (byte * 8U)));
    }
    for (std::size_t index = 0; index < aircraft_count; ++index) {
        mix(static_cast<std::uint8_t>(
            (candidate_valid_mask >> index) & std::uint32_t{1}));
        if ((candidate_valid_mask & (std::uint32_t{1} << index)) == 0U) {
            continue;
        }
        mix(candidate_ids[index]);
    }
    return hash;
}

inline std::uint32_t candidateMaskForAircraftCount(
    const std::size_t aircraft_count) noexcept
{
    return aircraft_count >= 32U
        ? std::numeric_limits<std::uint32_t>::max()
        : (std::uint32_t{1} << aircraft_count) - std::uint32_t{1};
}

inline bool candidateIsValid(
    const std::uint32_t candidate_valid_mask,
    const std::size_t aircraft_index) noexcept
{
    return aircraft_index < 32U
        && (candidate_valid_mask & (std::uint32_t{1} << aircraft_index)) != 0U;
}

struct IntentKinematics
{
    std::array<double, 3> position_ned{};
    std::array<double, 3> velocity_ned{};
};

enum class IntentKinematicsStatus : std::uint8_t
{
    Valid,
    Future,
    Stale,
    Invalid,
};

inline bool finitePositive(double value) noexcept
{
    return std::isfinite(value) && value > 0.0;
}

inline bool nominalPredictInput(
    const ManeuverSelectionNominalSetpointSnapshot & snapshot,
    std::uint64_t now_us,
    std::uint64_t maximum_age_us,
    estimation::PredictInput & input) noexcept
{
    if (!snapshot.valid || snapshot.timestamp_us == 0
        || snapshot.timestamp_us > now_us
        || now_us - snapshot.timestamp_us > maximum_age_us
        || !finitePositive(snapshot.ground_speed_command_mps)
        || (!std::isfinite(snapshot.altitude_command_m)
            && !std::isnan(snapshot.altitude_command_m))
        || !std::isfinite(snapshot.lateral_acceleration_px4_mps2)) {
        return false;
    }
    input.V_cmd = snapshot.ground_speed_command_mps;
    input.h_cmd = snapshot.altitude_command_m;
    input.h_dot_cmd = 0.0;
    input.a_lat_cmd = snapshot.lateral_acceleration_px4_mps2;
    return true;
}

inline bool validParams(const ManeuverSelectionWorkerParams & params) noexcept
{
    const bool v4_execution_policy =
        params.execution_policy == ManeuverExecutionPolicy::ContinuousV4
        || params.execution_policy == ManeuverExecutionPolicy::HorizonGatedV4;
    if (params.total_agent_count < 2
        || params.total_agent_count
            > static_cast<int>(kMaximumSelectionAircraft)
        || params.vehicle_id < 0
        || params.vehicle_id >= params.total_agent_count
        || !finitePositive(params.ground_speed_command_mps)
        || !finitePositive(params.gravity_mps2)
        || params.trajectory_refresh_period_us == 0
        || params.candidate_refresh_period_us == 0
        || params.coordination_delay_us == 0
        || params.maximum_belief_delay_us == 0
        || (params.interaction_graph_params.enabled
            && (!params.exhaustive_test_mode
                || params.execution_policy
                    != ManeuverExecutionPolicy::AmacAdThreshold))
        || (params.interaction_graph_params.enabled
            && (!std::isfinite(
                    params.interaction_graph_params.ad_screen_m)
                || params.interaction_graph_params
                        .trajectory_library_version == 0
                || params.interaction_graph_params
                        .ad_masd_config_version == 0
                || params.interaction_graph_params.config_version == 0))
        || !finitePositive(
            params.activation_params
                .relative_speed_epsilon_mps)
        || !finitePositive(params.activation_params.cpa_horizon_s)
        || (params.formation_discrimination_enabled
            && (params.execution_policy
                    != ManeuverExecutionPolicy::AmacAdThreshold
                || !finitePositive(params.formation_target_separation_m)
                || params.formation_target_separation_m
                    <= params.evaluator_params
                            .desired_separation_distance_m
                        + params.evaluator_params.ownship_half_wingspan_m
                        + params.evaluator_params.threat_half_wingspan_m
                || !formation::FormationDiscriminator::validConfig(
                    params.formation_boundary_config)))
        || (params.active_switching_enabled
            && (!finitePositive(params.active_switch_cost_margin)
                || !finitePositive(
                    params.active_switch_minimum_ad_margin_m)))
        || (v4_execution_policy
            && (!params.v4_safe_control_enabled || params.v4_shadow_only))
        || (params.execution_policy == ManeuverExecutionPolicy::HorizonGatedV4
            && (!finitePositive(params.v4_horizon_trigger_m)
                || !params.evaluator_params.robust_cone_filter_enabled
                || params.v4_control_architecture
                    != V4ControlArchitecture::LegacySafeControlSet))
        || (params.v4_safe_control_enabled
            && (!finitePositive(params.v4_trim_airspeed_mps)
                || params.v4_maximum_airspeed_age_us == 0
                || params.v4_maximum_nominal_age_us == 0
                || !SafeControlCandidateAdapter::validParams(
                    params.v4_candidate_adapter_params)
                || (params.v4_control_architecture
                        == V4ControlArchitecture::LegacySafeControlSet
                    && !SafeControlSetV4::validParams(
                        params.v4_safe_control_params))
                || (params.v4_control_architecture
                        == V4ControlArchitecture::ClosedFormBackupModeB
                    && (!BackupControlInterpolatorV4::validParams(
                            params.mode_b_interpolator_params)
                        || !BackupThreatIntentAdapterV4::validParams(
                            params.mode_b_intent_adapter_params)
                        || std::abs(
                            params.mode_b_interpolator_params.certifier.horizon_s
                            - params.mode_b_intent_adapter_params.horizon_s)
                            > params.mode_b_interpolator_params.certifier
                                .threat_time_tolerance_s
                        || std::abs(
                            params.mode_b_interpolator_params.certifier
                                .integration_step_s
                            - params.mode_b_intent_adapter_params
                                .integration_step_s)
                            > params.mode_b_interpolator_params.certifier
                                .threat_time_tolerance_s))))
        || (params.v4_safe_control_enabled && !params.v4_shadow_only
            && params.exhaustive_test_mode)
        || ((params.evaluator_params.positive_margin_filter_enabled
                || params.evaluator_params.robust_cone_filter_enabled)
            && (!std::isfinite(
                    params.evaluator_params.positive_margin_gamma)
                || params.evaluator_params.positive_margin_gamma <= 0.0
                || params.evaluator_params.positive_margin_gamma > 1.0
                || !finitePositive(
                    params.evaluator_params.positive_margin_reference_m)
                || (params.evaluator_params.positive_margin_filter_enabled
                    && !finitePositive(
                        params.evaluator_params
                            .maximum_lateral_acceleration_mps2))))) {
        return false;
    }

    std::array<bool, estimation::kManeuverCandidateCount> seen{};
    for (const std::uint8_t candidate_id : params.eligible_candidate_ids) {
        if (candidate_id >= estimation::kManeuverCandidateCount
            || seen[candidate_id]) {
            return false;
        }
        seen[candidate_id] = true;
    }
    return seen[kRollZeroId];
}

inline IntentKinematicsStatus interpolateIntentKinematics(
    const estimation::ReceivedTrajectoryIntent & intent,
    std::uint64_t evaluation_timestamp_us,
    IntentKinematics & kinematics) noexcept
{
    if (intent.source_timestamp_us > evaluation_timestamp_us) {
        return IntentKinematicsStatus::Future;
    }
    const double age_s = static_cast<double>(
        evaluation_timestamp_us - intent.source_timestamp_us) * 1.0e-6;
    if (!std::isfinite(age_s) || age_s < 0.0
        || age_s > kTrajectoryHorizonSeconds) {
        return IntentKinematicsStatus::Stale;
    }

    const double fractional_index = age_s / kTrajectoryStepSeconds;
    std::size_t lower = static_cast<std::size_t>(
        std::floor(fractional_index));
    lower = std::min(lower, estimation::kTrajectoryPointCount - 1);
    const std::size_t upper = std::min(
        lower + 1, estimation::kTrajectoryPointCount - 1);
    const double alpha = upper == lower
        ? 0.0
        : std::clamp(
            fractional_index - static_cast<double>(lower), 0.0, 1.0);
    const auto & first = intent.reconstructed_mean[lower];
    const auto & second = intent.reconstructed_mean[upper];
    const auto blend = [alpha](double lhs, double rhs) {
        return (1.0 - alpha) * lhs + alpha * rhs;
    };

    const double p_n = blend(first.p_n, second.p_n);
    const double p_e = blend(first.p_e, second.p_e);
    const double h = blend(first.h, second.h);
    const double speed = blend(first.V, second.V);
    const double climb_rate = blend(first.h_dot, second.h_dot);
    const double course = first.psi + alpha * std::remainder(
        second.psi - first.psi, 2.0 * std::acos(-1.0));
    if (!std::isfinite(p_n) || !std::isfinite(p_e) || !std::isfinite(h)
        || !std::isfinite(speed) || !std::isfinite(climb_rate)
        || !std::isfinite(course)) {
        return IntentKinematicsStatus::Invalid;
    }

    const double horizontal_speed = std::sqrt(std::max(
        0.0, speed * speed - climb_rate * climb_rate));
    kinematics.position_ned = {p_n, p_e, -h};
    kinematics.velocity_ned = {
        horizontal_speed * std::cos(course),
        horizontal_speed * std::sin(course),
        -climb_rate};
    return IntentKinematicsStatus::Valid;
}

inline V4SnapshotStatus classifySnapshot(
    bool present,
    bool value_valid,
    std::uint64_t timestamp_us,
    std::uint64_t evaluation_timestamp_us,
    std::uint64_t maximum_age_us,
    std::uint64_t & age_us) noexcept
{
    age_us = 0;
    if (!present) {
        return V4SnapshotStatus::Missing;
    }
    if (!value_valid) {
        return V4SnapshotStatus::Invalid;
    }
    if (timestamp_us > evaluation_timestamp_us) {
        return V4SnapshotStatus::Future;
    }
    age_us = evaluation_timestamp_us - timestamp_us;
    if (age_us > maximum_age_us) {
        return V4SnapshotStatus::Stale;
    }
    return V4SnapshotStatus::Valid;
}

}  // namespace collision_avoidance::selection::worker_detail

