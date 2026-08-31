#include <collision_avoidance/selection/BackupSafetyCertifierV4.hpp>

#include <algorithm>
#include <cmath>

namespace collision_avoidance::selection
{
namespace
{

bool finite(double value) noexcept
{
    return std::isfinite(value);
}

bool validOwnship(const SafeControlOwnshipState & ownship) noexcept
{
    if (!finite(ownship.north_m)
        || !finite(ownship.east_m)
        || !finite(ownship.heading_ned_rad)
        || !finite(ownship.true_airspeed_mps)
        || !finite(ownship.longitudinal_acceleration_mps2)) {
        return false;
    }
    if (ownship.longitudinal_source
            != LongitudinalDriftSource::ValidatedExternal
        && ownship.longitudinal_source
            != LongitudinalDriftSource::LocalOneStepFreeze) {
        return false;
    }
    return ownship.longitudinal_source
            != LongitudinalDriftSource::LocalOneStepFreeze
        || ownship.longitudinal_acceleration_mps2 == 0.0;
}

bool validThreatMetadata(
    const BackupThreatTrajectoryV4 & threat) noexcept
{
    return threat.vehicle_id >= 0
        && finite(threat.physical_clearance_m)
        && threat.physical_clearance_m >= 0.0
        && threat.point_count > 0
        && threat.point_count <= threat.points.size();
}

BackupBranchClassificationV4 classify(
    bool left_certified,
    bool right_certified) noexcept
{
    if (left_certified && right_certified) {
        return BackupBranchClassificationV4::BothCertified;
    }
    if (left_certified) {
        return BackupBranchClassificationV4::LeftOnly;
    }
    if (right_certified) {
        return BackupBranchClassificationV4::RightOnly;
    }
    return BackupBranchClassificationV4::NeitherCertified;
}

BackupBranchFailureReasonV4 failureReason(
    bool path_safe,
    bool terminal_safe) noexcept
{
    if (path_safe && terminal_safe) {
        return BackupBranchFailureReasonV4::None;
    }
    if (!path_safe && !terminal_safe) {
        return BackupBranchFailureReasonV4::PathAndTerminalMarginViolation;
    }
    return path_safe
        ? BackupBranchFailureReasonV4::TerminalTurnMarginViolation
        : BackupBranchFailureReasonV4::PathMarginViolation;
}

}  // namespace

BackupSafetyCertifierV4::BackupSafetyCertifierV4(
    const BackupSafetyCertifierV4Params & params)
: m_params(params),
  m_model(params.model)
{
}

const BackupSafetyCertifierV4Params &
BackupSafetyCertifierV4::params() const noexcept
{
    return m_params;
}

bool BackupSafetyCertifierV4::validParams(
    const BackupSafetyCertifierV4Params & params) noexcept
{
    if (!BackupControlModelV4::validParams(params.model)
        || !finite(params.horizon_s)
        || params.horizon_s <= params.model.time_tolerance_s
        || !finite(params.integration_step_s)
        || params.integration_step_s <= 0.0
        || !finite(params.reference_margin_m)
        || params.reference_margin_m < 0.0
        || !finite(params.certification_tolerance_m)
        || params.certification_tolerance_m < 0.0
        || !finite(params.threat_time_tolerance_s)
        || params.threat_time_tolerance_s <= 0.0) {
        return false;
    }

    const double required_points = std::ceil(
        (params.horizon_s - params.model.time_tolerance_s)
        / params.integration_step_s) + 1.0;
    return finite(required_points)
        && required_points >= 2.0
        && required_points
            <= static_cast<double>(kMaximumBackupTrajectoryPointsV4);
}

BackupSafetyCertifierV4Result BackupSafetyCertifierV4::evaluate(
    const BackupSafetyCertifierV4Input & input) const noexcept
{
    BackupSafetyCertifierV4Result result{};
    result.left.direction = BackupDirectionV4::Left;
    result.right.direction = BackupDirectionV4::Right;
    if (!validParams(m_params)) {
        result.status = BackupCertificationStatusV4::InvalidConfiguration;
        return result;
    }
    if (!validOwnship(input.ownship)
        || input.ownship.true_airspeed_mps
            <= m_params.model.speed_tolerance_mps) {
        result.status = BackupCertificationStatusV4::InvalidOwnshipState;
        return result;
    }
    if (input.threat_count > input.threats.size()) {
        result.status = BackupCertificationStatusV4::InvalidThreatCount;
        return result;
    }

    result.left.trajectory = m_model.propagate(
        input.ownship,
        BackupDirectionV4::Left,
        m_params.horizon_s,
        m_params.integration_step_s);
    result.right.trajectory = m_model.propagate(
        input.ownship,
        BackupDirectionV4::Right,
        m_params.horizon_s,
        m_params.integration_step_s);
    if (result.left.trajectory.status != BackupPropagationStatusV4::Valid
        || result.right.trajectory.status
            != BackupPropagationStatusV4::Valid) {
        result.status = BackupCertificationStatusV4::PropagationFailed;
        result.propagation_status =
            result.left.trajectory.status != BackupPropagationStatusV4::Valid
            ? result.left.trajectory.status
            : result.right.trajectory.status;
        return result;
    }

    result.effective_max_heading_rate_initial_radps =
        m_model.effectiveMaxHeadingRate(
            input.ownship.true_airspeed_mps);
    result.left.backup_heading_rate_initial_v4_radps =
        result.left.trajectory.points[0]
            .backup_heading_rate_v4_radps;
    result.right.backup_heading_rate_initial_v4_radps =
        result.right.trajectory.points[0]
            .backup_heading_rate_v4_radps;

    for (std::size_t threat_index = 0;
         threat_index < input.threat_count;
         ++threat_index) {
        const auto & threat = input.threats[threat_index];
        result.invalid_threat_vehicle_id = threat.vehicle_id;
        if (!validThreatMetadata(threat)) {
            result.status = BackupCertificationStatusV4::InvalidThreatState;
            return result;
        }
        if (threat.source_timestamp_us > input.ownship.timestamp_us) {
            result.status =
                BackupCertificationStatusV4::FutureThreatTimestamp;
            return result;
        }
        if (threat.source_timestamp_us < input.ownship.timestamp_us) {
            result.status =
                BackupCertificationStatusV4::StaleThreatTimestamp;
            return result;
        }
        if (threat.point_count != result.left.trajectory.point_count) {
            result.status =
                BackupCertificationStatusV4::InvalidThreatTrajectory;
            return result;
        }
        for (std::size_t point_index = 0;
             point_index < threat.point_count;
             ++point_index) {
            const auto & threat_point = threat.points[point_index];
            const double expected_time = result.left.trajectory
                .points[point_index].time_offset_s;
            if (!finite(threat_point.time_offset_s)
                || !finite(threat_point.north_m)
                || !finite(threat_point.east_m)
                || std::abs(threat_point.time_offset_s - expected_time)
                    > m_params.threat_time_tolerance_s) {
                result.status =
                    BackupCertificationStatusV4::InvalidThreatTrajectory;
                return result;
            }
        }
    }
    result.invalid_threat_vehicle_id = -1;

    const auto evaluateBranch = [this, &input](
        BackupBranchResultV4 & branch) noexcept
        -> BackupCertificationStatusV4 {
            for (std::size_t point_index = 0;
                 point_index < branch.trajectory.point_count;
                 ++point_index) {
                const auto & ownship_point =
                    branch.trajectory.points[point_index];
                for (std::size_t threat_index = 0;
                     threat_index < input.threat_count;
                     ++threat_index) {
                    const auto & threat = input.threats[threat_index];
                    const auto & threat_point = threat.points[point_index];
                    const double distance_m = std::hypot(
                        ownship_point.north_m - threat_point.north_m,
                        ownship_point.east_m - threat_point.east_m);
                    const double margin_m = distance_m
                        - threat.physical_clearance_m
                        - m_params.reference_margin_m;
                    if (!finite(margin_m)) {
                        return BackupCertificationStatusV4::InvalidThreatState;
                    }
                    if (margin_m < branch.minimum_path_margin_m) {
                        branch.minimum_path_margin_m = margin_m;
                        branch.limiting_path_threat_id = threat.vehicle_id;
                        branch.limiting_path_time_offset_s =
                            ownship_point.time_offset_s;
                    }
                }
            }

            const auto & terminal = branch.trajectory.points[
                branch.trajectory.point_count - 1];
            const double maximum_rate = m_model.effectiveMaxHeadingRate(
                terminal.true_airspeed_mps);
            if (!finite(maximum_rate)
                || maximum_rate
                    <= m_params.model.heading_rate_tolerance_radps) {
                return BackupCertificationStatusV4::DegenerateTerminalGeometry;
            }
            const double radius_m = terminal.true_airspeed_mps
                / maximum_rate;
            if (!finite(radius_m) || radius_m <= 0.0) {
                return BackupCertificationStatusV4::DegenerateTerminalGeometry;
            }

            // In NED course coordinates the geometric LEFT normal is
            // [sin(chi), -cos(chi)]. sigma=+1 selects LEFT, sigma=-1 RIGHT.
            const double sigma = branch.direction == BackupDirectionV4::Left
                ? 1.0
                : -1.0;
            const double center_north_m = terminal.north_m
                + sigma * radius_m * std::sin(terminal.heading_ned_rad);
            const double center_east_m = terminal.east_m
                - sigma * radius_m * std::cos(terminal.heading_ned_rad);

            for (std::size_t threat_index = 0;
                 threat_index < input.threat_count;
                 ++threat_index) {
                const auto & threat = input.threats[threat_index];
                const auto & threat_terminal = threat.points[
                    threat.point_count - 1];
                const double margin_m = std::hypot(
                    center_north_m - threat_terminal.north_m,
                    center_east_m - threat_terminal.east_m)
                    - (threat.physical_clearance_m + radius_m);
                if (!finite(margin_m)) {
                    return BackupCertificationStatusV4::InvalidThreatState;
                }
                if (margin_m < branch.terminal_turn_margin_m) {
                    branch.terminal_turn_margin_m = margin_m;
                    branch.limiting_terminal_threat_id = threat.vehicle_id;
                }
            }

            const bool path_safe = branch.minimum_path_margin_m
                >= m_params.certification_tolerance_m;
            const bool terminal_safe = branch.terminal_turn_margin_m
                >= m_params.certification_tolerance_m;
            branch.certified = path_safe && terminal_safe;
            branch.failure_reason = failureReason(path_safe, terminal_safe);
            return BackupCertificationStatusV4::Valid;
        };

    result.status = evaluateBranch(result.left);
    if (result.status != BackupCertificationStatusV4::Valid) {
        return result;
    }
    result.status = evaluateBranch(result.right);
    if (result.status != BackupCertificationStatusV4::Valid) {
        return result;
    }

    result.classification = classify(
        result.left.certified, result.right.certified);
    result.evaluated_threat_count = input.threat_count;
    result.status = BackupCertificationStatusV4::Valid;
    return result;
}

const char * backupCertificationStatusName(
    BackupCertificationStatusV4 status) noexcept
{
    switch (status) {
    case BackupCertificationStatusV4::Valid:
        return "valid";
    case BackupCertificationStatusV4::InvalidConfiguration:
        return "invalid_configuration";
    case BackupCertificationStatusV4::InvalidOwnshipState:
        return "invalid_ownship_state";
    case BackupCertificationStatusV4::InvalidThreatCount:
        return "invalid_threat_count";
    case BackupCertificationStatusV4::InvalidThreatState:
        return "invalid_threat_state";
    case BackupCertificationStatusV4::FutureThreatTimestamp:
        return "future_threat_timestamp";
    case BackupCertificationStatusV4::StaleThreatTimestamp:
        return "stale_threat_timestamp";
    case BackupCertificationStatusV4::InvalidThreatTrajectory:
        return "invalid_threat_trajectory";
    case BackupCertificationStatusV4::PropagationFailed:
        return "propagation_failed";
    case BackupCertificationStatusV4::DegenerateTerminalGeometry:
        return "degenerate_terminal_geometry";
    }
    return "unknown";
}

const char * backupBranchFailureReasonName(
    BackupBranchFailureReasonV4 reason) noexcept
{
    switch (reason) {
    case BackupBranchFailureReasonV4::None:
        return "none";
    case BackupBranchFailureReasonV4::PathMarginViolation:
        return "path_margin_violation";
    case BackupBranchFailureReasonV4::TerminalTurnMarginViolation:
        return "terminal_turn_margin_violation";
    case BackupBranchFailureReasonV4::PathAndTerminalMarginViolation:
        return "path_and_terminal_margin_violation";
    }
    return "unknown";
}

}  // namespace collision_avoidance::selection
