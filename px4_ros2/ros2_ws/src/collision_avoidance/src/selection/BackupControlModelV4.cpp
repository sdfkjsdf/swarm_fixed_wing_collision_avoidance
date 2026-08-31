#include <collision_avoidance/selection/BackupControlModelV4.hpp>

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

bool validDirection(BackupDirectionV4 direction) noexcept
{
    return direction == BackupDirectionV4::Left
        || direction == BackupDirectionV4::Right;
}

bool validInitialState(const SafeControlOwnshipState & state) noexcept
{
    if (!finite(state.north_m)
        || !finite(state.east_m)
        || !finite(state.heading_ned_rad)
        || !finite(state.true_airspeed_mps)
        || !finite(state.longitudinal_acceleration_mps2)) {
        return false;
    }
    if (state.longitudinal_source
            != LongitudinalDriftSource::ValidatedExternal
        && state.longitudinal_source
            != LongitudinalDriftSource::LocalOneStepFreeze) {
        return false;
    }
    return state.longitudinal_source
            != LongitudinalDriftSource::LocalOneStepFreeze
        || state.longitudinal_acceleration_mps2 == 0.0;
}

struct InternalState
{
    // Right-handed planar frame: x=North, y=-East, psi-positive-LEFT.
    double x_m{0.0};
    double y_m{0.0};
    double psi_rad{0.0};
    double speed_mps{0.0};
};

InternalState addScaled(
    const InternalState & state,
    const InternalState & derivative,
    double scale) noexcept
{
    return {
        state.x_m + scale * derivative.x_m,
        state.y_m + scale * derivative.y_m,
        state.psi_rad + scale * derivative.psi_rad,
        state.speed_mps + scale * derivative.speed_mps,
    };
}

}  // namespace

BackupControlModelV4::BackupControlModelV4(
    const BackupControlModelV4Params & params)
: m_params(params)
{
}

const BackupControlModelV4Params &
BackupControlModelV4::params() const noexcept
{
    return m_params;
}

bool BackupControlModelV4::validParams(
    const BackupControlModelV4Params & params) noexcept
{
    return finite(params.gravity_mps2)
        && params.gravity_mps2 > 0.0
        && finite(params.maximum_roll_rad)
        && params.maximum_roll_rad > 0.0
        && params.maximum_roll_rad < kHalfPi
        && finite(params.maximum_yaw_rate_radps)
        && params.maximum_yaw_rate_radps > 0.0
        && finite(params.speed_tolerance_mps)
        && params.speed_tolerance_mps > 0.0
        && finite(params.heading_rate_tolerance_radps)
        && params.heading_rate_tolerance_radps > 0.0
        && finite(params.time_tolerance_s)
        && params.time_tolerance_s > 0.0;
}

double BackupControlModelV4::effectiveMaxHeadingRate(
    double true_airspeed_mps) const noexcept
{
    if (!validParams(m_params)
        || !finite(true_airspeed_mps)
        || true_airspeed_mps <= m_params.speed_tolerance_mps) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    const double bank_limited_rate = m_params.gravity_mps2
        * std::tan(m_params.maximum_roll_rad)
        / true_airspeed_mps;
    const double result = std::min(
        m_params.maximum_yaw_rate_radps,
        bank_limited_rate);
    return finite(result)
            && result > m_params.heading_rate_tolerance_radps
        ? result
        : std::numeric_limits<double>::quiet_NaN();
}

double BackupControlModelV4::backupHeadingRate(
    BackupDirectionV4 direction,
    double true_airspeed_mps) const noexcept
{
    if (!validDirection(direction)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double maximum_rate = effectiveMaxHeadingRate(
        true_airspeed_mps);
    if (!finite(maximum_rate)) {
        return maximum_rate;
    }
    return direction == BackupDirectionV4::Left
        ? maximum_rate
        : -maximum_rate;
}

BackupTrajectoryV4 BackupControlModelV4::propagate(
    const SafeControlOwnshipState & initial_state,
    BackupDirectionV4 direction,
    double horizon_s,
    double integration_step_s) const noexcept
{
    BackupTrajectoryV4 result{};
    result.direction = direction;
    if (!validParams(m_params)
        || !validDirection(direction)
        || !finite(horizon_s)
        || horizon_s <= m_params.time_tolerance_s
        || !finite(integration_step_s)
        || integration_step_s <= 0.0) {
        result.status = BackupPropagationStatusV4::InvalidConfiguration;
        return result;
    }
    if (!validInitialState(initial_state)) {
        result.status = BackupPropagationStatusV4::InvalidInitialState;
        return result;
    }
    if (initial_state.true_airspeed_mps
        <= m_params.speed_tolerance_mps) {
        result.status = BackupPropagationStatusV4::InvalidAirspeed;
        return result;
    }

    const double required_steps_real = std::ceil(
        (horizon_s - m_params.time_tolerance_s)
        / integration_step_s);
    if (!finite(required_steps_real)
        || required_steps_real < 1.0
        || required_steps_real + 1.0
            > static_cast<double>(result.points.size())) {
        result.status =
            BackupPropagationStatusV4::TrajectoryCapacityExceeded;
        return result;
    }

    InternalState state{
        initial_state.north_m,
        -initial_state.east_m,
        -initial_state.heading_ned_rad,
        initial_state.true_airspeed_mps,
    };
    const double acceleration_mps2 =
        initial_state.longitudinal_acceleration_mps2;

    const auto derivative = [this, direction, acceleration_mps2](
        const InternalState & value,
        InternalState & output) noexcept -> bool {
            const double rate = backupHeadingRate(
                direction, value.speed_mps);
            if (!finite(rate)) {
                return false;
            }
            output.x_m = value.speed_mps * std::cos(value.psi_rad);
            output.y_m = value.speed_mps * std::sin(value.psi_rad);
            output.psi_rad = rate;
            output.speed_mps = acceleration_mps2;
            return finite(output.x_m)
                && finite(output.y_m)
                && finite(output.psi_rad)
                && finite(output.speed_mps);
        };

    const auto appendPoint = [this, direction, &result](
        const InternalState & value,
        double time_offset_s) noexcept -> bool {
            if (result.point_count >= result.points.size()) {
                return false;
            }
            const double rate = backupHeadingRate(
                direction, value.speed_mps);
            if (!finite(rate)) {
                return false;
            }
            auto & point = result.points[result.point_count++];
            point.time_offset_s = time_offset_s;
            point.north_m = value.x_m;
            point.east_m = -value.y_m;
            point.heading_ned_rad = -value.psi_rad;
            point.true_airspeed_mps = value.speed_mps;
            point.backup_heading_rate_v4_radps = rate;
            return finite(point.north_m)
                && finite(point.east_m)
                && finite(point.heading_ned_rad)
                && finite(point.true_airspeed_mps);
        };

    double time_s = 0.0;
    if (!appendPoint(state, time_s)) {
        result.status = BackupPropagationStatusV4::NumericalFailure;
        return result;
    }

    while (time_s < horizon_s - m_params.time_tolerance_s) {
        const double step_s = std::min(
            integration_step_s, horizon_s - time_s);
        InternalState k1;
        InternalState k2;
        InternalState k3;
        InternalState k4;
        if (!derivative(state, k1)
            || !derivative(addScaled(state, k1, 0.5 * step_s), k2)
            || !derivative(addScaled(state, k2, 0.5 * step_s), k3)
            || !derivative(addScaled(state, k3, step_s), k4)) {
            result.status = BackupPropagationStatusV4::InvalidAirspeed;
            return result;
        }

        state.x_m += step_s
            * (k1.x_m + 2.0 * k2.x_m + 2.0 * k3.x_m + k4.x_m)
            / 6.0;
        state.y_m += step_s
            * (k1.y_m + 2.0 * k2.y_m + 2.0 * k3.y_m + k4.y_m)
            / 6.0;
        state.psi_rad += step_s
            * (k1.psi_rad + 2.0 * k2.psi_rad
                + 2.0 * k3.psi_rad + k4.psi_rad)
            / 6.0;
        state.speed_mps += step_s
            * (k1.speed_mps + 2.0 * k2.speed_mps
                + 2.0 * k3.speed_mps + k4.speed_mps)
            / 6.0;
        time_s = std::min(horizon_s, time_s + step_s);
        if (!appendPoint(state, time_s)) {
            result.status = result.point_count >= result.points.size()
                ? BackupPropagationStatusV4::TrajectoryCapacityExceeded
                : BackupPropagationStatusV4::NumericalFailure;
            return result;
        }
    }

    result.status = BackupPropagationStatusV4::Valid;
    return result;
}

const char * backupPropagationStatusName(
    BackupPropagationStatusV4 status) noexcept
{
    switch (status) {
    case BackupPropagationStatusV4::Valid:
        return "valid";
    case BackupPropagationStatusV4::InvalidConfiguration:
        return "invalid_configuration";
    case BackupPropagationStatusV4::InvalidInitialState:
        return "invalid_initial_state";
    case BackupPropagationStatusV4::InvalidAirspeed:
        return "invalid_airspeed";
    case BackupPropagationStatusV4::TrajectoryCapacityExceeded:
        return "trajectory_capacity_exceeded";
    case BackupPropagationStatusV4::NumericalFailure:
        return "numerical_failure";
    }
    return "unknown";
}

}  // namespace collision_avoidance::selection
