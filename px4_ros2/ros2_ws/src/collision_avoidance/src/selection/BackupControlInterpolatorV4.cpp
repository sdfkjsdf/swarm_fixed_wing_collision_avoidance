#include <collision_avoidance/selection/BackupControlInterpolatorV4.hpp>

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

bool validTolerances(
    const BackupInterpolationTolerancesV4 & tolerances) noexcept
{
    return finite(tolerances.coefficient_mps)
        && tolerances.coefficient_mps > 0.0
        && finite(tolerances.residual_mps)
        && tolerances.residual_mps > 0.0
        && finite(tolerances.mu)
        && tolerances.mu > 0.0
        && finite(tolerances.distance_m)
        && tolerances.distance_m > 0.0;
}

double directionSign(BackupDirectionV4 direction) noexcept
{
    return direction == BackupDirectionV4::Left ? 1.0 : -1.0;
}

struct DirectionalVector
{
    double north_mps{0.0};
    double east_mps{0.0};
    double heading_ned_radps{0.0};
    double airspeed_mps2{0.0};
};

DirectionalVector addScaled(
    const DirectionalVector & value,
    const DirectionalVector & derivative,
    double scale) noexcept
{
    return {
        value.north_mps + scale * derivative.north_mps,
        value.east_mps + scale * derivative.east_mps,
        value.heading_ned_radps
            + scale * derivative.heading_ned_radps,
        value.airspeed_mps2 + scale * derivative.airspeed_mps2,
    };
}

DirectionalVector directionalDerivative(
    const DirectionalVector & value,
    double heading_ned_rad,
    double true_airspeed_mps,
    double closed_heading_rate_speed_derivative) noexcept
{
    return {
        -true_airspeed_mps * std::sin(heading_ned_rad)
                * value.heading_ned_radps
            + std::cos(heading_ned_rad) * value.airspeed_mps2,
        true_airspeed_mps * std::cos(heading_ned_rad)
                * value.heading_ned_radps
            + std::sin(heading_ned_rad) * value.airspeed_mps2,
        closed_heading_rate_speed_derivative * value.airspeed_mps2,
        0.0,
    };
}

bool finiteVector(const DirectionalVector & value) noexcept
{
    return finite(value.north_mps)
        && finite(value.east_mps)
        && finite(value.heading_ned_radps)
        && finite(value.airspeed_mps2);
}

struct ConstraintRecord
{
    BackupInterpolationConstraintDiagnosticV4 diagnostic{};
};

}  // namespace

BackupControlInterpolatorV4::BackupControlInterpolatorV4(
    const BackupControlInterpolatorV4Params & params)
: m_params(params),
  m_certifier(params.certifier),
  m_model(params.certifier.model)
{
}

const BackupControlInterpolatorV4Params &
BackupControlInterpolatorV4::params() const noexcept
{
    return m_params;
}

bool BackupControlInterpolatorV4::validParams(
    const BackupControlInterpolatorV4Params & params) noexcept
{
    return BackupSafetyCertifierV4::validParams(params.certifier)
        && finite(params.path_alpha_gain_per_s)
        && params.path_alpha_gain_per_s > 0.0
        && finite(params.terminal_alpha_gain_per_s)
        && params.terminal_alpha_gain_per_s > 0.0
        && validTolerances(params.tolerances);
}

bool BackupControlInterpolatorV4::applyScalarConstraint(
    double a_mps,
    double b_mps,
    const BackupInterpolationTolerancesV4 & tolerances,
    ScalarMuIntervalV4 & interval) noexcept
{
    if (!validTolerances(tolerances)
        || !finite(a_mps)
        || !finite(b_mps)
        || !finite(interval.lower)
        || !finite(interval.upper)
        || interval.lower < 0.0
        || interval.upper > 1.0
        || interval.lower > interval.upper + tolerances.mu
        || !interval.feasible) {
        interval.feasible = false;
        return false;
    }

    if (std::abs(b_mps) <= tolerances.coefficient_mps) {
        interval.feasible = a_mps >= -tolerances.residual_mps;
        return interval.feasible;
    }

    const double bound = -a_mps / b_mps;
    if (!finite(bound)) {
        interval.feasible = false;
        return false;
    }
    if (b_mps > 0.0) {
        interval.lower = std::max(interval.lower, bound);
    } else {
        interval.upper = std::min(interval.upper, bound);
    }

    if (interval.lower > interval.upper + tolerances.mu
        || interval.upper < -tolerances.mu
        || interval.lower > 1.0 + tolerances.mu) {
        interval.feasible = false;
        return false;
    }

    interval.lower = std::clamp(interval.lower, 0.0, 1.0);
    interval.upper = std::clamp(interval.upper, 0.0, 1.0);
    if (interval.lower > interval.upper) {
        const double boundary = std::clamp(
            0.5 * (interval.lower + interval.upper), 0.0, 1.0);
        interval.lower = boundary;
        interval.upper = boundary;
    }
    return true;
}

BackupControlInterpolatorV4Result BackupControlInterpolatorV4::evaluate(
    const BackupSafetyCertifierV4Input & input,
    double nominal_heading_rate_v4_radps) const noexcept
{
    BackupControlInterpolatorV4Result result{};
    result.left.direction = BackupDirectionV4::Left;
    result.right.direction = BackupDirectionV4::Right;
    result.left.nominal_heading_rate_v4_radps =
        nominal_heading_rate_v4_radps;
    result.right.nominal_heading_rate_v4_radps =
        nominal_heading_rate_v4_radps;
    if (!validParams(m_params)) {
        result.status = BackupInterpolationStatusV4::InvalidConfiguration;
        return result;
    }
    if (!finite(nominal_heading_rate_v4_radps)) {
        result.status = BackupInterpolationStatusV4::InvalidNominalRate;
        return result;
    }

    result.certification = m_certifier.evaluate(input);
    if (result.certification.status
        != BackupCertificationStatusV4::Valid) {
        result.status = BackupInterpolationStatusV4::CertificationFailed;
        return result;
    }

    const double maximum_initial_rate =
        result.certification.effective_max_heading_rate_initial_radps;
    if (!finite(maximum_initial_rate)
        || std::abs(nominal_heading_rate_v4_radps)
            > maximum_initial_rate
                + m_params.certifier.model.heading_rate_tolerance_radps) {
        result.status = BackupInterpolationStatusV4::InvalidNominalRate;
        return result;
    }

    result.certified_branch_count =
        static_cast<std::size_t>(result.certification.left.certified)
        + static_cast<std::size_t>(result.certification.right.certified);
    if (result.certified_branch_count == 0U) {
        result.status = BackupInterpolationStatusV4::NoCertifiedBranch;
        return result;
    }

    const auto evaluateBranch = [this, &input, nominal_heading_rate_v4_radps](
        const BackupBranchResultV4 & certificate,
        BackupInterpolationBranchResultV4 & branch) noexcept -> bool {
            branch.direction = certificate.direction;
            branch.nominal_heading_rate_v4_radps =
                nominal_heading_rate_v4_radps;
            branch.backup_heading_rate_v4_radps =
                certificate.backup_heading_rate_initial_v4_radps;
            if (!certificate.certified) {
                branch.status =
                    BackupInterpolationBranchStatusV4::NotCertified;
                return true;
            }

            const auto & trajectory = certificate.trajectory;
            if (trajectory.status != BackupPropagationStatusV4::Valid
                || trajectory.point_count == 0U
                || trajectory.point_count > trajectory.points.size()) {
                branch.status =
                    BackupInterpolationBranchStatusV4::NumericalFailure;
                return false;
            }

            std::array<
                DirectionalVector,
                kMaximumBackupTrajectoryPointsV4> q_nominal{};
            std::array<
                DirectionalVector,
                kMaximumBackupTrajectoryPointsV4> q_backup{};
            const auto & initial = trajectory.points[0];
            const double acceleration_mps2 =
                input.ownship.longitudinal_acceleration_mps2;
            q_nominal[0] = {
                initial.true_airspeed_mps
                    * std::cos(initial.heading_ned_rad),
                initial.true_airspeed_mps
                    * std::sin(initial.heading_ned_rad),
                -nominal_heading_rate_v4_radps,
                acceleration_mps2,
            };
            q_backup[0] = {
                initial.true_airspeed_mps
                    * std::cos(initial.heading_ned_rad),
                initial.true_airspeed_mps
                    * std::sin(initial.heading_ned_rad),
                -certificate.backup_heading_rate_initial_v4_radps,
                acceleration_mps2,
            };

            const double sigma = directionSign(certificate.direction);
            for (std::size_t point_index = 1;
                 point_index < trajectory.point_count;
                 ++point_index) {
                const auto & previous = trajectory.points[point_index - 1];
                const auto & current = trajectory.points[point_index];
                const double step_s =
                    current.time_offset_s - previous.time_offset_s;
                const double midpoint_heading_rad = 0.5
                    * (previous.heading_ned_rad
                        + current.heading_ned_rad);
                const double midpoint_speed_mps = 0.5
                    * (previous.true_airspeed_mps
                        + current.true_airspeed_mps);
                if (!finite(step_s) || step_s <= 0.0) {
                    branch.status =
                        BackupInterpolationBranchStatusV4::NumericalFailure;
                    return false;
                }

                const double previous_rate_derivative = -sigma
                    * m_model.effectiveMaxHeadingRateSpeedDerivative(
                        previous.true_airspeed_mps);
                const double midpoint_rate_derivative = -sigma
                    * m_model.effectiveMaxHeadingRateSpeedDerivative(
                        midpoint_speed_mps);
                const double current_rate_derivative = -sigma
                    * m_model.effectiveMaxHeadingRateSpeedDerivative(
                        current.true_airspeed_mps);
                if (!finite(previous_rate_derivative)
                    || !finite(midpoint_rate_derivative)
                    || !finite(current_rate_derivative)) {
                    branch.status =
                        BackupInterpolationBranchStatusV4::NumericalFailure;
                    return false;
                }

                const auto integrateVector = [
                    step_s,
                    &previous,
                    &current,
                    midpoint_heading_rad,
                    midpoint_speed_mps,
                    previous_rate_derivative,
                    midpoint_rate_derivative,
                    current_rate_derivative](
                    const DirectionalVector & value) noexcept {
                        const auto k1 = directionalDerivative(
                            value,
                            previous.heading_ned_rad,
                            previous.true_airspeed_mps,
                            previous_rate_derivative);
                        const auto k2 = directionalDerivative(
                            addScaled(value, k1, 0.5 * step_s),
                            midpoint_heading_rad,
                            midpoint_speed_mps,
                            midpoint_rate_derivative);
                        const auto k3 = directionalDerivative(
                            addScaled(value, k2, 0.5 * step_s),
                            midpoint_heading_rad,
                            midpoint_speed_mps,
                            midpoint_rate_derivative);
                        const auto k4 = directionalDerivative(
                            addScaled(value, k3, step_s),
                            current.heading_ned_rad,
                            current.true_airspeed_mps,
                            current_rate_derivative);
                        return DirectionalVector{
                            value.north_mps + step_s
                                * (k1.north_mps + 2.0 * k2.north_mps
                                    + 2.0 * k3.north_mps + k4.north_mps)
                                / 6.0,
                            value.east_mps + step_s
                                * (k1.east_mps + 2.0 * k2.east_mps
                                    + 2.0 * k3.east_mps + k4.east_mps)
                                / 6.0,
                            value.heading_ned_radps + step_s
                                * (k1.heading_ned_radps
                                    + 2.0 * k2.heading_ned_radps
                                    + 2.0 * k3.heading_ned_radps
                                    + k4.heading_ned_radps)
                                / 6.0,
                            value.airspeed_mps2 + step_s
                                * (k1.airspeed_mps2
                                    + 2.0 * k2.airspeed_mps2
                                    + 2.0 * k3.airspeed_mps2
                                    + k4.airspeed_mps2)
                                / 6.0,
                        };
                    };

                q_nominal[point_index] = integrateVector(
                    q_nominal[point_index - 1]);
                q_backup[point_index] = integrateVector(
                    q_backup[point_index - 1]);
                if (!finiteVector(q_nominal[point_index])
                    || !finiteVector(q_backup[point_index])) {
                    branch.status =
                        BackupInterpolationBranchStatusV4::NumericalFailure;
                    return false;
                }
            }

            std::array<
                ConstraintRecord,
                kMaximumBackupInterpolationConstraintsV4> constraints{};
            std::size_t constraint_count = 0;
            const auto appendConstraint = [
                &constraints,
                &constraint_count](
                const BackupInterpolationConstraintDiagnosticV4 & diagnostic)
                noexcept -> bool {
                    if (constraint_count >= constraints.size()) {
                        return false;
                    }
                    constraints[constraint_count++].diagnostic = diagnostic;
                    return true;
                };

            // The position-separation function has relative degree greater
            // than one with respect to heading rate at the initial state.
            // Phase 1 still certifies the t=0 path margin; interpolation
            // constraints start at the first propagated sample, matching the
            // directional-sensitivity architecture used by the reference.
            for (std::size_t point_index = 1;
                 point_index < trajectory.point_count;
                 ++point_index) {
                const auto & ownship_point = trajectory.points[point_index];
                for (std::size_t threat_index = 0;
                     threat_index < input.threat_count;
                     ++threat_index) {
                    const auto & threat = input.threats[threat_index];
                    const auto & threat_point = threat.points[point_index];
                    const double relative_north_m =
                        ownship_point.north_m - threat_point.north_m;
                    const double relative_east_m =
                        ownship_point.east_m - threat_point.east_m;
                    const double distance_m = std::hypot(
                        relative_north_m, relative_east_m);
                    if (!finite(distance_m)
                        || distance_m <= m_params.tolerances.distance_m) {
                        branch.status =
                            BackupInterpolationBranchStatusV4::NumericalFailure;
                        return false;
                    }
                    const double gradient_north = relative_north_m / distance_m;
                    const double gradient_east = relative_east_m / distance_m;
                    BackupInterpolationConstraintDiagnosticV4 diagnostic{};
                    diagnostic.kind =
                        BackupInterpolationConstraintKindV4::PathSeparation;
                    diagnostic.vehicle_id = threat.vehicle_id;
                    diagnostic.time_offset_s = ownship_point.time_offset_s;
                    diagnostic.margin_m = distance_m
                        - threat.physical_clearance_m
                        - m_params.certifier.reference_margin_m;
                    diagnostic.hdot_nominal_mps =
                        gradient_north * q_nominal[point_index].north_mps
                        + gradient_east * q_nominal[point_index].east_mps;
                    diagnostic.hdot_backup_mps =
                        gradient_north * q_backup[point_index].north_mps
                        + gradient_east * q_backup[point_index].east_mps;
                    diagnostic.a_mps = diagnostic.hdot_nominal_mps
                        + m_params.path_alpha_gain_per_s
                            * diagnostic.margin_m;
                    diagnostic.b_mps = diagnostic.hdot_backup_mps
                        - diagnostic.hdot_nominal_mps;
                    if (!finite(diagnostic.margin_m)
                        || !finite(diagnostic.a_mps)
                        || !finite(diagnostic.b_mps)
                        || !appendConstraint(diagnostic)) {
                        branch.status =
                            BackupInterpolationBranchStatusV4::NumericalFailure;
                        return false;
                    }
                }
            }

            const std::size_t terminal_index = trajectory.point_count - 1;
            const auto & terminal = trajectory.points[terminal_index];
            const double maximum_rate = m_model.effectiveMaxHeadingRate(
                terminal.true_airspeed_mps);
            const double radius_speed_derivative =
                m_model.turningRadiusSpeedDerivative(
                    terminal.true_airspeed_mps);
            if (!finite(maximum_rate)
                || !finite(radius_speed_derivative)) {
                branch.status =
                    BackupInterpolationBranchStatusV4::NumericalFailure;
                return false;
            }
            const double radius_m = terminal.true_airspeed_mps
                / maximum_rate;
            const double left_normal_north =
                std::sin(terminal.heading_ned_rad);
            const double left_normal_east =
                -std::cos(terminal.heading_ned_rad);
            const double direction_north =
                std::cos(terminal.heading_ned_rad);
            const double direction_east =
                std::sin(terminal.heading_ned_rad);
            const double center_north_m = terminal.north_m
                + sigma * radius_m * left_normal_north;
            const double center_east_m = terminal.east_m
                + sigma * radius_m * left_normal_east;

            for (std::size_t threat_index = 0;
                 threat_index < input.threat_count;
                 ++threat_index) {
                const auto & threat = input.threats[threat_index];
                const auto & threat_point = threat.points[terminal_index];
                const double relative_north_m =
                    center_north_m - threat_point.north_m;
                const double relative_east_m =
                    center_east_m - threat_point.east_m;
                const double distance_m = std::hypot(
                    relative_north_m, relative_east_m);
                if (!finite(distance_m)
                    || distance_m <= m_params.tolerances.distance_m) {
                    branch.status =
                        BackupInterpolationBranchStatusV4::NumericalFailure;
                    return false;
                }
                const double qhat_north = relative_north_m / distance_m;
                const double qhat_east = relative_east_m / distance_m;
                const double gradient_heading = sigma * radius_m
                    * (qhat_north * direction_north
                        + qhat_east * direction_east);
                const double gradient_speed = radius_speed_derivative
                    * (sigma
                            * (qhat_north * left_normal_north
                                + qhat_east * left_normal_east)
                        - 1.0);
                const auto directionalRate = [
                    qhat_north,
                    qhat_east,
                    gradient_heading,
                    gradient_speed](const DirectionalVector & value) noexcept {
                        return qhat_north * value.north_mps
                            + qhat_east * value.east_mps
                            + gradient_heading * value.heading_ned_radps
                            + gradient_speed * value.airspeed_mps2;
                    };
                BackupInterpolationConstraintDiagnosticV4 diagnostic{};
                diagnostic.kind = BackupInterpolationConstraintKindV4::
                    TerminalTurnCertificate;
                diagnostic.vehicle_id = threat.vehicle_id;
                diagnostic.time_offset_s = terminal.time_offset_s;
                diagnostic.margin_m = distance_m
                    - (threat.physical_clearance_m + radius_m);
                diagnostic.hdot_nominal_mps = directionalRate(
                    q_nominal[terminal_index]);
                diagnostic.hdot_backup_mps = directionalRate(
                    q_backup[terminal_index]);
                diagnostic.a_mps = diagnostic.hdot_nominal_mps
                    + m_params.terminal_alpha_gain_per_s
                        * diagnostic.margin_m;
                diagnostic.b_mps = diagnostic.hdot_backup_mps
                    - diagnostic.hdot_nominal_mps;
                if (!finite(diagnostic.margin_m)
                    || !finite(diagnostic.a_mps)
                    || !finite(diagnostic.b_mps)
                    || !appendConstraint(diagnostic)) {
                    branch.status =
                        BackupInterpolationBranchStatusV4::NumericalFailure;
                    return false;
                }
            }

            branch.constraint_count = constraint_count;
            ScalarMuIntervalV4 interval{};
            for (std::size_t index = 0; index < constraint_count; ++index) {
                auto diagnostic = constraints[index].diagnostic;
                const ScalarMuIntervalV4 previous = interval;
                if (std::abs(diagnostic.b_mps)
                    > m_params.tolerances.coefficient_mps) {
                    diagnostic.imposed_mu_bound =
                        -diagnostic.a_mps / diagnostic.b_mps;
                }
                if (!applyScalarConstraint(
                        diagnostic.a_mps,
                        diagnostic.b_mps,
                        m_params.tolerances,
                        interval)) {
                    branch.mu_interval = interval;
                    branch.infeasible_diagnostic_valid = true;
                    branch.infeasible_diagnostic = diagnostic;
                    branch.status =
                        BackupInterpolationBranchStatusV4::Infeasible;
                    return true;
                }
                if (interval.lower > previous.lower
                    + m_params.tolerances.mu) {
                    branch.lower_bound_diagnostic_valid = true;
                    branch.lower_bound_diagnostic = diagnostic;
                }
                if (interval.upper < previous.upper
                    - m_params.tolerances.mu) {
                    branch.upper_bound_diagnostic_valid = true;
                    branch.upper_bound_diagnostic = diagnostic;
                }
            }

            branch.mu_interval = interval;
            branch.mu_star = interval.lower;
            branch.safe_heading_rate_v4_radps =
                (1.0 - branch.mu_star)
                    * nominal_heading_rate_v4_radps
                + branch.mu_star
                    * certificate.backup_heading_rate_initial_v4_radps;
            for (std::size_t index = 0; index < constraint_count; ++index) {
                const auto & diagnostic = constraints[index].diagnostic;
                const double residual = diagnostic.a_mps
                    + diagnostic.b_mps * branch.mu_star;
                branch.minimum_residual_at_mu_star_mps = std::min(
                    branch.minimum_residual_at_mu_star_mps, residual);
            }
            if (!finite(branch.mu_star)
                || branch.mu_star < -m_params.tolerances.mu
                || branch.mu_star > 1.0 + m_params.tolerances.mu
                || !finite(branch.safe_heading_rate_v4_radps)
                || branch.minimum_residual_at_mu_star_mps
                    < -m_params.tolerances.residual_mps) {
                branch.status =
                    BackupInterpolationBranchStatusV4::NumericalFailure;
                return false;
            }
            branch.status = BackupInterpolationBranchStatusV4::Feasible;
            return true;
        };

    if (!evaluateBranch(result.certification.left, result.left)
        || !evaluateBranch(result.certification.right, result.right)) {
        result.status = BackupInterpolationStatusV4::NumericalFailure;
        return result;
    }

    result.feasible_branch_count = static_cast<std::size_t>(
        result.left.status == BackupInterpolationBranchStatusV4::Feasible)
        + static_cast<std::size_t>(
            result.right.status == BackupInterpolationBranchStatusV4::Feasible);
    result.status = result.feasible_branch_count > 0U
        ? BackupInterpolationStatusV4::Valid
        : BackupInterpolationStatusV4::InterpolationInfeasible;
    return result;
}

const char * backupInterpolationStatusName(
    BackupInterpolationStatusV4 status) noexcept
{
    switch (status) {
    case BackupInterpolationStatusV4::Valid:
        return "valid";
    case BackupInterpolationStatusV4::InvalidConfiguration:
        return "invalid_configuration";
    case BackupInterpolationStatusV4::InvalidNominalRate:
        return "invalid_nominal_rate";
    case BackupInterpolationStatusV4::CertificationFailed:
        return "certification_failed";
    case BackupInterpolationStatusV4::NoCertifiedBranch:
        return "no_certified_branch";
    case BackupInterpolationStatusV4::InterpolationInfeasible:
        return "interpolation_infeasible";
    case BackupInterpolationStatusV4::NumericalFailure:
        return "numerical_failure";
    }
    return "unknown";
}

const char * backupInterpolationBranchStatusName(
    BackupInterpolationBranchStatusV4 status) noexcept
{
    switch (status) {
    case BackupInterpolationBranchStatusV4::NotCertified:
        return "not_certified";
    case BackupInterpolationBranchStatusV4::Feasible:
        return "feasible";
    case BackupInterpolationBranchStatusV4::Infeasible:
        return "infeasible";
    case BackupInterpolationBranchStatusV4::NumericalFailure:
        return "numerical_failure";
    }
    return "unknown";
}

}  // namespace collision_avoidance::selection
