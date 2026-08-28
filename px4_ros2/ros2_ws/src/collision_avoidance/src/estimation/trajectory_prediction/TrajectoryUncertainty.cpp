#include <collision_avoidance/estimation/trajectory_prediction/TrajectoryUncertainty.hpp>

#include <algorithm>
#include <array>
#include <cmath>

namespace collision_avoidance::estimation
{
namespace
{

constexpr std::size_t kX = kPredictStateDimension;
constexpr std::size_t kZ = kEstimatorBeliefDimension;

double wrapAngle(double angle)
{
    constexpr double two_pi = 2.0 * M_PI;
    angle = std::fmod(angle + M_PI, two_pi);
    angle = std::fmod(angle + two_pi, two_pi);
    return angle - M_PI;
}

std::array<double, kX> toArray(const PredictState & state)
{
    return {state.p_n, state.p_e, state.h, state.V,
            state.psi, state.h_dot, state.phi};
}

PredictState fromArray(const std::array<double, kX> & value)
{
    return {value[0], value[1], value[2], value[3],
            value[4], value[5], value[6]};
}

std::array<double, 4> quaternionMultiply(
    const std::array<double, 4> & lhs,
    const std::array<double, 4> & rhs)
{
    return {
        lhs[0] * rhs[0] - lhs[1] * rhs[1] - lhs[2] * rhs[2] - lhs[3] * rhs[3],
        lhs[0] * rhs[1] + lhs[1] * rhs[0] + lhs[2] * rhs[3] - lhs[3] * rhs[2],
        lhs[0] * rhs[2] - lhs[1] * rhs[3] + lhs[2] * rhs[0] + lhs[3] * rhs[1],
        lhs[0] * rhs[3] + lhs[1] * rhs[2] - lhs[2] * rhs[1] + lhs[3] * rhs[0]};
}

std::array<double, 4> applyAttitudeError(
    const std::array<double, 4> & quaternion,
    std::size_t axis,
    double angle)
{
    std::array<double, 4> delta{std::cos(0.5 * angle), 0.0, 0.0, 0.0};
    delta[axis + 1] = std::sin(0.5 * angle);
    auto result = quaternionMultiply(delta, quaternion);
    const double norm = std::sqrt(result[0] * result[0] + result[1] * result[1]
                                + result[2] * result[2] + result[3] * result[3]);
    for (double & value : result) {
        value /= norm;
    }
    return result;
}

PredictState beliefMeanToPredictState(
    const std::array<double, 4> & q,
    const std::array<double, 3> & velocity,
    const std::array<double, 3> & position)
{
    const double speed = std::sqrt(
        velocity[0] * velocity[0] + velocity[1] * velocity[1]
        + velocity[2] * velocity[2]);
    const double roll = std::atan2(
        2.0 * (q[0] * q[1] + q[2] * q[3]),
        1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]));

    return {
        position[0], position[1], -position[2], speed,
        std::atan2(velocity[1], velocity[0]), -velocity[2], roll};
}

bool finiteState(const PredictState & state)
{
    const auto value = toArray(state);
    return std::all_of(value.begin(), value.end(), [](double v) { return std::isfinite(v); });
}

void symmetrizeAndFloor(PredictStateCovariance & covariance, double floor)
{
    for (std::size_t row = 0; row < kX; ++row) {
        for (std::size_t column = row + 1; column < kX; ++column) {
            const double average = 0.5 * (
                covariance[row * kX + column] + covariance[column * kX + row]);
            covariance[row * kX + column] = average;
            covariance[column * kX + row] = average;
        }
        covariance[row * kX + row] =
            std::max(covariance[row * kX + row], floor);
    }
}

PositionCovariance positionCovarianceNed(const PredictStateCovariance & covariance)
{
    PositionCovariance result{};
    constexpr std::array<double, 3> sign{1.0, 1.0, -1.0};
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            result[row * 3 + column] =
                sign[row] * covariance[row * kX + column] * sign[column];
        }
    }
    return result;
}

}  // namespace

TrajectoryUncertainty::TrajectoryUncertainty(const UncertaintyParams & params)
: m_params(params)
{
}

bool TrajectoryUncertainty::initializeFromEstimatorBelief(
    const EstimatorTrajectoryBelief & belief,
    PredictState & state,
    PredictStateCovariance & covariance) const
{
    state = beliefMeanToPredictState(
        belief.attitude_q, belief.velocity_ned, belief.position_ned);
    if (!finiteState(state)) {
        return false;
    }

    std::array<double, kX * kZ> jacobian{};
    constexpr std::array<double, kZ> step{
        1.0e-5, 1.0e-5, 1.0e-5,
        1.0e-3, 1.0e-3, 1.0e-3,
        1.0e-3, 1.0e-3, 1.0e-3};
    const auto base = toArray(state);

    for (std::size_t column = 0; column < kZ; ++column) {
        auto q = belief.attitude_q;
        auto velocity = belief.velocity_ned;
        auto position = belief.position_ned;
        if (column < 3) {
            q = applyAttitudeError(q, column, step[column]);
        } else if (column < 6) {
            velocity[column - 3] += step[column];
        } else {
            position[column - 6] += step[column];
        }

        const auto perturbed = toArray(beliefMeanToPredictState(q, velocity, position));
        for (std::size_t row = 0; row < kX; ++row) {
            double delta = perturbed[row] - base[row];
            if (row == 4 || row == 6) {
                delta = wrapAngle(delta);
            }
            jacobian[row * kZ + column] = delta / step[column];
        }
    }

    covariance.fill(0.0);
    for (std::size_t row = 0; row < kX; ++row) {
        for (std::size_t column = 0; column < kX; ++column) {
            double value = 0.0;
            for (std::size_t left = 0; left < kZ; ++left) {
                for (std::size_t right = 0; right < kZ; ++right) {
                    value += jacobian[row * kZ + left]
                           * belief.covariance[left * kZ + right]
                           * jacobian[column * kZ + right];
                }
            }
            covariance[row * kX + column] = value;
        }
    }

    symmetrizeAndFloor(covariance, m_params.covariance_diagonal_floor);
    return covarianceIsFiniteAndPsd(covariance);
}

bool TrajectoryUncertainty::propagateOneStep(
    const TrajectoryPredict & predictor,
    const PredictInput & input,
    double dt,
    PredictState & state,
    PredictStateCovariance & covariance) const
{
    if (!(dt > 0.0) || !finiteState(state)) {
        return false;
    }

    const PredictState next = predictor.stepRK4(state, input, dt);
    if (!propagateCovarianceOneStep(
            predictor, input, dt, state, next, covariance)) {
        return false;
    }
    state = next;
    return finiteState(state);
}

bool TrajectoryUncertainty::propagateCovarianceOneStep(
    const TrajectoryPredict & predictor,
    const PredictInput & input,
    double dt,
    const PredictState & linearization_state,
    const PredictState & predicted_next_state,
    PredictStateCovariance & covariance) const
{
    if (!(dt > 0.0) || !finiteState(linearization_state)
        || !finiteState(predicted_next_state)
        || !covarianceIsFiniteAndPsd(covariance)) {
        return false;
    }

    const auto next_array = toArray(predicted_next_state);
    const auto state_array = toArray(linearization_state);
    std::array<double, kX * kX> transition{};

    for (std::size_t column = 0; column < kX; ++column) {
        auto perturbed_array = state_array;
        const double step = m_params.finite_difference_step[column];
        perturbed_array[column] += step;
        const auto perturbed_next = toArray(
            predictor.stepRK4(fromArray(perturbed_array), input, dt));
        for (std::size_t row = 0; row < kX; ++row) {
            double delta = perturbed_next[row] - next_array[row];
            if (row == 4 || row == 6) {
                delta = wrapAngle(delta);
            }
            transition[row * kX + column] = delta / step;
        }
    }

    PredictStateCovariance propagated{};
    for (std::size_t row = 0; row < kX; ++row) {
        for (std::size_t column = 0; column < kX; ++column) {
            double value = 0.0;
            for (std::size_t left = 0; left < kX; ++left) {
                for (std::size_t right = 0; right < kX; ++right) {
                    value += transition[row * kX + left]
                           * covariance[left * kX + right]
                           * transition[column * kX + right];
                }
            }
            propagated[row * kX + column] = value;
        }
        propagated[row * kX + row] +=
            m_params.process_noise_diagonal[row] * dt;
    }

    symmetrizeAndFloor(propagated, m_params.covariance_diagonal_floor);
    covariance = propagated;
    return covarianceIsFiniteAndPsd(covariance);
}

bool TrajectoryUncertainty::compensateFusionHorizonDelay(
    const TrajectoryPredict & predictor,
    const PredictInput & input,
    double delay_s,
    PredictState & state,
    PredictStateCovariance & covariance) const
{
    if (!std::isfinite(delay_s) || delay_s < 0.0 || delay_s > 1.0) {
        return false;
    }
    if (delay_s < 1.0e-6) {
        return true;
    }
    return propagateOneStep(predictor, input, delay_s, state, covariance);
}

bool TrajectoryUncertainty::propagate(
    const TrajectoryPredict & predictor,
    const PredictState & initial_state,
    const PredictStateCovariance & initial_covariance,
    const PredictionInputTrajectory & inputs,
    double dt,
    TrajectoryCone & cone) const
{
    PredictState state = initial_state;
    PredictStateCovariance covariance = initial_covariance;
    if (!finiteState(state) || !covarianceIsFiniteAndPsd(covariance)) {
        return false;
    }

    cone[0] = {0.0, state, covariance, positionCovarianceNed(covariance)};
    for (std::size_t index = 0; index < kTrajectoryIntervalCount; ++index) {
        if (!propagateOneStep(predictor, inputs[index], dt, state, covariance)) {
            return false;
        }
        cone[index + 1] = {
            static_cast<double>(index + 1) * dt,
            state,
            covariance,
            positionCovarianceNed(covariance)};
    }
    return true;
}

bool TrajectoryUncertainty::propagateAlongMean(
    const TrajectoryPredict & predictor,
    const PredictionMeanTrajectory & mean_trajectory,
    const PredictStateCovariance & initial_covariance,
    const PredictionInputTrajectory & inputs,
    double dt,
    TrajectoryCone & cone) const
{
    PredictStateCovariance covariance = initial_covariance;
    if (!(dt > 0.0) || !finiteState(mean_trajectory.front())
        || !covarianceIsFiniteAndPsd(covariance)) {
        return false;
    }

    cone[0] = {
        0.0,
        mean_trajectory[0],
        covariance,
        positionCovarianceNed(covariance)};
    for (std::size_t index = 0; index < kTrajectoryIntervalCount; ++index) {
        const PredictState & current_mean = mean_trajectory[index];
        const PredictState & next_mean = mean_trajectory[index + 1];
        if (!finiteState(current_mean) || !finiteState(next_mean)) {
            return false;
        }
        const PredictState predicted_next =
            predictor.stepRK4(current_mean, inputs[index], dt);
        if (!propagateCovarianceOneStep(
                predictor,
                inputs[index],
                dt,
                current_mean,
                predicted_next,
                covariance)) {
            return false;
        }
        cone[index + 1] = {
            static_cast<double>(index + 1) * dt,
            next_mean,
            covariance,
            positionCovarianceNed(covariance)};
    }
    return true;
}

bool TrajectoryUncertainty::covarianceIsFiniteAndPsd(
    const PredictStateCovariance & covariance,
    double tolerance)
{
    std::array<double, kX * kX> lower{};
    for (std::size_t row = 0; row < kX; ++row) {
        for (std::size_t column = 0; column <= row; ++column) {
            const double symmetric_value = 0.5 * (
                covariance[row * kX + column] + covariance[column * kX + row]);
            if (!std::isfinite(symmetric_value)) {
                return false;
            }
            double sum = symmetric_value;
            for (std::size_t k = 0; k < column; ++k) {
                sum -= lower[row * kX + k] * lower[column * kX + k];
            }
            if (row == column) {
                if (sum < -tolerance) {
                    return false;
                }
                lower[row * kX + column] = std::sqrt(std::max(sum, tolerance));
            } else {
                lower[row * kX + column] = sum / lower[column * kX + column];
            }
        }
    }
    return true;
}

}  // namespace collision_avoidance::estimation
