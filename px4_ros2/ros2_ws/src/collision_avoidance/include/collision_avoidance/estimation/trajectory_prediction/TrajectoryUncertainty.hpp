#pragma once

#include <collision_avoidance/estimation/trajectory_prediction/TrajectoryPredict.hpp>
#include <collision_avoidance/estimation/trajectory_prediction/UncertaintyTypes.hpp>

namespace collision_avoidance::estimation
{

class TrajectoryUncertainty
{
public:
    explicit TrajectoryUncertainty(const UncertaintyParams & params = {});

    bool initializeFromEstimatorBelief(
        const EstimatorTrajectoryBelief & belief,
        PredictState & state,
        PredictStateCovariance & covariance) const;

    bool compensateFusionHorizonDelay(
        const TrajectoryPredict & predictor,
        const PredictInput & input,
        double delay_s,
        PredictState & state,
        PredictStateCovariance & covariance) const;

    bool propagate(
        const TrajectoryPredict & predictor,
        const PredictState & initial_state,
        const PredictStateCovariance & initial_covariance,
        const PredictionInputTrajectory & inputs,
        double dt,
        TrajectoryCone & cone) const;

    static bool covarianceIsFiniteAndPsd(
        const PredictStateCovariance & covariance,
        double tolerance = 1.0e-8);

private:
    bool propagateOneStep(
        const TrajectoryPredict & predictor,
        const PredictInput & input,
        double dt,
        PredictState & state,
        PredictStateCovariance & covariance) const;

    UncertaintyParams m_params;
};

}  // namespace collision_avoidance::estimation
