#pragma once

#include <array>
#include <cstddef>

#include <collision_avoidance/estimation/trajectory_prediction/PredictTypes.hpp>

namespace collision_avoidance::estimation
{

inline constexpr std::size_t kPredictStateDimension = 7;
inline constexpr std::size_t kEstimatorBeliefDimension = 9;
inline constexpr std::size_t kTrajectoryPointCount = 46;
inline constexpr std::size_t kTrajectoryIntervalCount = kTrajectoryPointCount - 1;

using PredictStateCovariance =
    std::array<double, kPredictStateDimension * kPredictStateDimension>;
using EstimatorBeliefCovariance =
    std::array<double, kEstimatorBeliefDimension * kEstimatorBeliefDimension>;
using PositionCovariance = std::array<double, 9>;
using PredictionInputTrajectory =
    std::array<PredictInput, kTrajectoryIntervalCount>;

struct EstimatorTrajectoryBelief
{
    std::array<double, 4> attitude_q{};
    std::array<double, 3> velocity_ned{};
    std::array<double, 3> position_ned{};
    EstimatorBeliefCovariance covariance{};
};

struct TrajectoryConePoint
{
    double time_offset_s{0.0};
    PredictState mean{};
    PredictStateCovariance state_covariance{};
    PositionCovariance position_covariance_ned{};
};

using TrajectoryCone = std::array<TrajectoryConePoint, kTrajectoryPointCount>;

struct UncertaintyParams
{
    // Continuous-time diagonal process-noise spectral density in predictor-state order.
    std::array<double, kPredictStateDimension> process_noise_diagonal{
        0.25, 0.25, 0.25, 0.04, 0.001, 0.04, 0.001};

    std::array<double, kPredictStateDimension> finite_difference_step{
        1.0e-3, 1.0e-3, 1.0e-3, 1.0e-3, 1.0e-5, 1.0e-3, 1.0e-5};

    double covariance_diagonal_floor{1.0e-10};
};

}  // namespace collision_avoidance::estimation
