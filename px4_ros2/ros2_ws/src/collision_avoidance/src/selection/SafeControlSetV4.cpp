#include <collision_avoidance/selection/SafeControlSetV4.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace collision_avoidance::selection
{
namespace
{

constexpr double kHalfPi = 1.57079632679489661923;

bool finite(double value) noexcept
{
    return std::isfinite(value);
}

bool validTolerances(
    const SafeControlNumericalTolerances & tolerances) noexcept
{
    return finite(tolerances.constraint_mps)
        && tolerances.constraint_mps > 0.0
        && finite(tolerances.interval_radps)
        && tolerances.interval_radps > 0.0
        && finite(tolerances.speed_mps)
        && tolerances.speed_mps > 0.0
        && finite(tolerances.direction_m)
        && tolerances.direction_m > 0.0;
}

bool validLongitudinalSource(
    LongitudinalDriftSource source) noexcept
{
    return source == LongitudinalDriftSource::ValidatedExternal
        || source == LongitudinalDriftSource::LocalOneStepFreeze;
}

bool validOwnship(
    const SafeControlOwnshipState & ownship) noexcept
{
    return finite(ownship.north_m)
        && finite(ownship.east_m)
        && finite(ownship.heading_ned_rad)
        && finite(ownship.true_airspeed_mps)
        && finite(ownship.longitudinal_acceleration_mps2)
        && validLongitudinalSource(ownship.longitudinal_source);
}

bool validThreat(
    const SafeControlThreatState & threat) noexcept
{
    return threat.vehicle_id >= 0
        && finite(threat.north_m)
        && finite(threat.east_m)
        && finite(threat.velocity_north_mps)
        && finite(threat.velocity_east_mps)
        && finite(threat.physical_clearance_m)
        && threat.physical_clearance_m >= 0.0;
}

HeadingRateInterval physicalInterval(
    SafeControlDirection direction,
    double maximum_rate_radps) noexcept
{
    return direction == SafeControlDirection::Left
        ? HeadingRateInterval{0.0, maximum_rate_radps, true}
        : HeadingRateInterval{-maximum_rate_radps, 0.0, true};
}

void markInfeasible(
    HeadingRateInterval & interval,
    const SafeControlThreatState & threat,
    SafeControlDirection direction,
    SafeControlSetV4Result & result) noexcept
{
    interval.feasible = false;
    if (result.first_infeasible_vehicle_id < 0) {
        result.first_infeasible_vehicle_id = threat.vehicle_id;
        result.first_infeasible_direction = direction;
    }
}

bool intersectAffineConstraint(
    double coefficient_m,
    double required_mps,
    const SafeControlThreatState & threat,
    SafeControlDirection direction,
    const SafeControlNumericalTolerances & tolerances,
    HeadingRateInterval & interval,
    SafeControlThreatDiagnostic & diagnostic,
    SafeControlSetV4Result & result) noexcept
{
    diagnostic.rate_coefficient_m = coefficient_m;
    diagnostic.required_control_term_mps = required_mps;

    if (std::abs(coefficient_m) <= tolerances.direction_m) {
        diagnostic.constraint_degenerate = true;
        diagnostic.constraint_feasible =
            required_mps <= tolerances.constraint_mps;
        if (!diagnostic.constraint_feasible) {
            diagnostic.constraint_shortfall_mps = std::max(
                0.0, required_mps);
            markInfeasible(interval, threat, direction, result);
        }
        return diagnostic.constraint_feasible;
    }

    const double bound_radps = required_mps / coefficient_m;
    diagnostic.imposed_bound_radps = bound_radps;
    if (!finite(bound_radps)) {
        diagnostic.constraint_feasible = false;
        markInfeasible(interval, threat, direction, result);
        return false;
    }

    const double maximum_available_control_mps = coefficient_m > 0.0
        ? coefficient_m * interval.upper_radps
        : coefficient_m * interval.lower_radps;
    if (coefficient_m > 0.0) {
        interval.lower_radps = std::max(
            interval.lower_radps, bound_radps);
    } else {
        interval.upper_radps = std::min(
            interval.upper_radps, bound_radps);
    }

    if (interval.lower_radps
        > interval.upper_radps + tolerances.interval_radps) {
        diagnostic.constraint_feasible = false;
        diagnostic.constraint_shortfall_mps = std::max(
            0.0, required_mps - maximum_available_control_mps);
        markInfeasible(interval, threat, direction, result);
        return false;
    }

    if (interval.lower_radps > interval.upper_radps) {
        // A tolerance-sized violation collapses to the pre-existing physical
        // boundary. Averaging could place the returned point just outside the
        // physical interval.
        if (coefficient_m > 0.0) {
            interval.lower_radps = interval.upper_radps;
        } else {
            interval.upper_radps = interval.lower_radps;
        }
    }
    diagnostic.constraint_feasible = true;
    return true;
}

}  // namespace

SafeControlSetV4::SafeControlSetV4(
    const SafeControlSetV4Params & params)
: m_params(params)
{
}

const SafeControlSetV4Params &
SafeControlSetV4::params() const noexcept
{
    return m_params;
}

bool SafeControlSetV4::validParams(
    const SafeControlSetV4Params & params) noexcept
{
    return finite(params.margin_reference_m)
        && params.margin_reference_m >= 0.0
        && finite(params.margin_time_constant_s)
        && params.margin_time_constant_s > 0.0
        && finite(params.control_period_s)
        && params.control_period_s > 0.0
        && finite(params.gravity_mps2)
        && params.gravity_mps2 > 0.0
        && finite(params.maximum_roll_rad)
        && params.maximum_roll_rad > 0.0
        && params.maximum_roll_rad < kHalfPi
        && finite(params.maximum_yaw_rate_radps)
        && params.maximum_yaw_rate_radps > 0.0
        && validTolerances(params.tolerances);
}

SafeControlSetV4Result SafeControlSetV4::evaluate(
    const SafeControlSetV4Input & input) const noexcept
{
    SafeControlSetV4Result result{};
    if (!validParams(m_params)) {
        result.status = SafeControlSetStatus::InvalidConfiguration;
        return result;
    }
    if (!validOwnship(input.ownship)) {
        result.status = SafeControlSetStatus::InvalidOwnshipState;
        return result;
    }
    if (input.ownship.longitudinal_source
            == LongitudinalDriftSource::LocalOneStepFreeze
        && input.ownship.longitudinal_acceleration_mps2 != 0.0) {
        result.status = SafeControlSetStatus::InvalidOwnshipState;
        return result;
    }
    if (input.ownship.true_airspeed_mps <= m_params.tolerances.speed_mps) {
        result.status = SafeControlSetStatus::InvalidAirspeed;
        return result;
    }
    if (input.threat_count > input.threats.size()) {
        result.status = SafeControlSetStatus::InvalidThreatCount;
        return result;
    }
    result.longitudinal_source = input.ownship.longitudinal_source;

    for (std::size_t index = 0; index < input.threat_count; ++index) {
        const auto & threat = input.threats[index];
        if (!validThreat(threat)) {
            result.status = SafeControlSetStatus::InvalidThreatState;
            return result;
        }
        if (threat.timestamp_us > input.ownship.timestamp_us) {
            result.status = SafeControlSetStatus::FutureThreatTimestamp;
            return result;
        }
        if (threat.timestamp_us < input.ownship.timestamp_us) {
            // The runtime adapter is responsible for interpolation to the
            // common evaluation timestamp. The core never extrapolates tracks.
            result.status = SafeControlSetStatus::StaleThreatTimestamp;
            return result;
        }
    }

    const double bank_limited_rate = m_params.gravity_mps2
        * std::tan(m_params.maximum_roll_rad)
        / input.ownship.true_airspeed_mps;
    result.effective_max_heading_rate_radps = std::min(
        m_params.maximum_yaw_rate_radps, bank_limited_rate);
    if (!finite(result.effective_max_heading_rate_radps)
        || result.effective_max_heading_rate_radps
            <= m_params.tolerances.interval_radps) {
        result.status = SafeControlSetStatus::InvalidConfiguration;
        return result;
    }

    result.kappa_per_s = 1.0 / m_params.margin_time_constant_s;
    result.gamma_diagnostic = 1.0 - std::exp(
        -m_params.control_period_s / m_params.margin_time_constant_s);
    result.left_safe = physicalInterval(
        SafeControlDirection::Left,
        result.effective_max_heading_rate_radps);
    result.right_safe = physicalInterval(
        SafeControlDirection::Right,
        result.effective_max_heading_rate_radps);

    // Convert external NED (North, East, course-positive-right) to an internal
    // right-handed planar frame (x=North, y=-East, heading-positive-left).
    // This preserves the V4 contract r>0=Left without changing external APIs.
    const double heading = -input.ownship.heading_ned_rad;
    const double cos_heading = std::cos(heading);
    const double sin_heading = std::sin(heading);
    const double direction_x = cos_heading;
    const double direction_y = sin_heading;
    const double normal_x = -sin_heading;
    const double normal_y = cos_heading;
    const double ownship_x = input.ownship.north_m;
    const double ownship_y = -input.ownship.east_m;
    const double speed = input.ownship.true_airspeed_mps;
    const double maximum_rate =
        result.effective_max_heading_rate_radps;
    const double radius = speed / maximum_rate;
    const double rate_difference =
        bank_limited_rate - m_params.maximum_yaw_rate_radps;
    const bool bank_limit_active =
        rate_difference < -m_params.tolerances.interval_radps
        || (std::abs(rate_difference)
                <= m_params.tolerances.interval_radps
            && input.ownship.longitudinal_acceleration_mps2 >= 0.0);
    // rho=V/r_yaw on the yaw-limited branch and
    // rho=V^2/(g*tan(phi_max)) on the bank-limited branch. The latter
    // therefore has d(rho)/dV=2/r_max, as required when r_max is
    // state-dependent.
    const double radius_speed_derivative =
        (bank_limit_active ? 2.0 : 1.0) / maximum_rate;

    for (std::size_t direction_index = 0;
         direction_index < kSafeControlDirectionCount;
         ++direction_index) {
        const SafeControlDirection direction = direction_index == 0
            ? SafeControlDirection::Left
            : SafeControlDirection::Right;
        const double sigma =
            direction == SafeControlDirection::Left ? 1.0 : -1.0;
        HeadingRateInterval & interval =
            direction == SafeControlDirection::Left
                ? result.left_safe
                : result.right_safe;
        const double center_x =
            ownship_x + sigma * radius * normal_x;
        const double center_y =
            ownship_y + sigma * radius * normal_y;

        for (std::size_t threat_index = 0;
             threat_index < input.threat_count;
             ++threat_index) {
            const SafeControlThreatState & threat =
                input.threats[threat_index];
            SafeControlThreatDiagnostic diagnostic{};
            diagnostic.vehicle_id = threat.vehicle_id;
            diagnostic.direction = direction;

            const double threat_x = threat.north_m;
            const double threat_y = -threat.east_m;
            const double threat_velocity_x =
                threat.velocity_north_mps;
            const double threat_velocity_y =
                -threat.velocity_east_mps;
            const double q_x = center_x - threat_x;
            const double q_y = center_y - threat_y;
            const double q_norm = std::hypot(q_x, q_y);
            if (!finite(q_norm)
                || q_norm <= m_params.tolerances.direction_m) {
                result.status = SafeControlSetStatus::DegenerateGeometry;
                result.first_infeasible_vehicle_id = threat.vehicle_id;
                result.first_infeasible_direction = direction;
                return result;
            }

            const double q_hat_x = q_x / q_norm;
            const double q_hat_y = q_y / q_norm;
            diagnostic.clearance_m = q_norm
                - (threat.physical_clearance_m + radius);

            const double relative_velocity_x =
                speed * direction_x - threat_velocity_x;
            const double relative_velocity_y =
                speed * direction_y - threat_velocity_y;
            const double base_drift =
                q_hat_x * relative_velocity_x
                + q_hat_y * relative_velocity_y;
            const double heading_rate_coefficient =
                -sigma * radius
                * (q_hat_x * direction_x
                    + q_hat_y * direction_y);
            const double acceleration_coefficient =
                (sigma
                    * (q_hat_x * normal_x + q_hat_y * normal_y)
                    - 1.0)
                * radius_speed_derivative;
            diagnostic.drift_mps = base_drift
                + acceleration_coefficient
                    * input.ownship.longitudinal_acceleration_mps2;

            const double required_control_term =
                result.kappa_per_s
                    * (m_params.margin_reference_m
                        - diagnostic.clearance_m)
                - diagnostic.drift_mps;

            intersectAffineConstraint(
                heading_rate_coefficient,
                required_control_term,
                threat,
                direction,
                m_params.tolerances,
                interval,
                diagnostic,
                result);
            if (result.diagnostic_count < result.diagnostics.size()) {
                result.diagnostics[result.diagnostic_count++] = diagnostic;
            }
        }
    }

    result.evaluated_threat_count = input.threat_count;
    result.status = !result.left_safe.feasible
            && !result.right_safe.feasible
        ? SafeControlSetStatus::SearchSetInfeasible
        : SafeControlSetStatus::Valid;
    return result;
}

const char * safeControlSetStatusName(
    SafeControlSetStatus status) noexcept
{
    switch (status) {
    case SafeControlSetStatus::Valid:
        return "valid";
    case SafeControlSetStatus::InvalidConfiguration:
        return "invalid_configuration";
    case SafeControlSetStatus::InvalidOwnshipState:
        return "invalid_ownship_state";
    case SafeControlSetStatus::InvalidAirspeed:
        return "invalid_airspeed";
    case SafeControlSetStatus::InvalidThreatCount:
        return "invalid_threat_count";
    case SafeControlSetStatus::InvalidThreatState:
        return "invalid_threat_state";
    case SafeControlSetStatus::FutureThreatTimestamp:
        return "future_threat_timestamp";
    case SafeControlSetStatus::StaleThreatTimestamp:
        return "stale_threat_timestamp";
    case SafeControlSetStatus::DegenerateGeometry:
        return "degenerate_geometry";
    case SafeControlSetStatus::SearchSetInfeasible:
        return "search_set_infeasible";
    }
    return "unknown";
}

}  // namespace collision_avoidance::selection
