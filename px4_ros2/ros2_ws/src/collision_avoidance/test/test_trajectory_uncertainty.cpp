#include <gtest/gtest.h>

#include <limits>

#include <collision_avoidance/estimation/trajectory_prediction/TrajectoryUncertainty.hpp>

namespace ce = collision_avoidance::estimation;

TEST(TrajectoryUncertainty, InitializesAndPropagatesFullCovariance)
{
    ce::EstimatorTrajectoryBelief belief;
    belief.attitude_q = {1.0, 0.0, 0.0, 0.0};
    belief.velocity_ned = {15.0, 0.0, 0.0};
    belief.position_ned = {10.0, -5.0, -100.0};
    for (std::size_t index = 0; index < ce::kEstimatorBeliefDimension; ++index) {
        belief.covariance[index * ce::kEstimatorBeliefDimension + index] = 0.04;
    }
    // Preserve a real position/velocity cross-covariance through the transform.
    belief.covariance[3 * ce::kEstimatorBeliefDimension + 6] = 0.01;
    belief.covariance[6 * ce::kEstimatorBeliefDimension + 3] = 0.01;

    ce::TrajectoryUncertainty uncertainty;
    ce::PredictState initial_state;
    ce::PredictStateCovariance initial_covariance;
    ASSERT_TRUE(uncertainty.initializeFromEstimatorBelief(
        belief, initial_state, initial_covariance));
    EXPECT_NEAR(initial_state.p_n, 10.0, 1.0e-9);
    EXPECT_NEAR(initial_state.p_e, -5.0, 1.0e-9);
    EXPECT_NEAR(initial_state.h, 100.0, 1.0e-9);
    EXPECT_NEAR(initial_state.V, 15.0, 1.0e-9);
    EXPECT_NEAR(initial_covariance[0 * ce::kPredictStateDimension + 3], 0.01, 1.0e-5);

    ce::PredictParams predictor_params;
    ce::TrajectoryPredict predictor(predictor_params);
    ce::PredictionInputTrajectory inputs;
    inputs.fill(ce::PredictInput{15.0, 100.0, 0.0, 2.0});

    ce::TrajectoryCone cone;
    ASSERT_TRUE(uncertainty.propagate(
        predictor, initial_state, initial_covariance, inputs, 0.1, cone));
    EXPECT_NEAR(cone.front().time_offset_s, 0.0, 1.0e-12);
    EXPECT_NEAR(cone.back().time_offset_s, 4.5, 1.0e-12);
    EXPECT_TRUE(ce::TrajectoryUncertainty::covarianceIsFiniteAndPsd(
        cone.back().state_covariance));
    EXPECT_GT(cone.back().position_covariance_ned[0],
              cone.front().position_covariance_ned[0]);
}

TEST(TrajectoryUncertainty, RejectsNonFiniteBelief)
{
    ce::EstimatorTrajectoryBelief belief;
    belief.attitude_q = {1.0, 0.0, 0.0, 0.0};
    belief.velocity_ned = {
        std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0};
    belief.position_ned = {0.0, 0.0, 0.0};

    ce::TrajectoryUncertainty uncertainty;
    ce::PredictState state;
    ce::PredictStateCovariance covariance;
    EXPECT_FALSE(uncertainty.initializeFromEstimatorBelief(belief, state, covariance));
}

TEST(TrajectoryUncertainty, HoldsCommandFilterFixedInFlightStateJacobian)
{
    ce::UncertaintyParams uncertainty_params;
    uncertainty_params.process_noise_diagonal.fill(0.0);
    ce::TrajectoryUncertainty uncertainty(uncertainty_params);
    ce::PredictParams params;
    ce::TrajectoryPredict predictor(params);
    ce::PredictState state{0,0,100,20,0,0,.1};
    state.phi_setpoint=.4;
    ce::PredictStateCovariance covariance{};
    covariance[48]=.0001;
    ce::PredictInput input{20,100,0,-5};
    ASSERT_TRUE(uncertainty.compensateFusionHorizonDelay(predictor,input,.1,state,covariance));
    EXPECT_NEAR(covariance[48],.0001*std::exp(-.2/params.tau_phi),2e-9);
    EXPECT_NEAR(state.phi_setpoint,.4-params.phi_setpoint_rate_max*.1,1e-12);
}

TEST(TrajectoryUncertainty, ImplicitSeedIsConditionedOnTheUnperturbedMean)
{
    ce::TrajectoryUncertainty uncertainty;
    ce::TrajectoryPredict predictor(ce::PredictParams{});
    ce::PredictState implicit{0,0,100,20,0,0,.1};
    auto explicit_seed=implicit;
    explicit_seed.phi_setpoint=implicit.phi;
    ce::PredictStateCovariance a{}, b{};
    a[48]=b[48]=.0001;
    ce::PredictInput input{20,100,0,5};
    ASSERT_TRUE(uncertainty.compensateFusionHorizonDelay(predictor,input,.1,implicit,a));
    ASSERT_TRUE(uncertainty.compensateFusionHorizonDelay(predictor,input,.1,explicit_seed,b));
    for (std::size_t k=0;k<a.size();++k) EXPECT_DOUBLE_EQ(a[k],b[k]);
}
