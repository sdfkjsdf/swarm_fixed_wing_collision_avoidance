#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
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

TEST(TrajectoryUncertainty, DenseCovarianceMatchesDirectQuadraticFormOverFullHorizon)
{
    // Independent scalar reference for the covariance multiplication. Keep
    // the slow four-loop expression here only, never on a runtime path.
    constexpr std::size_t n = ce::kPredictStateDimension;
    const auto as_array = [](const ce::PredictState & x) {
        return std::array<double, n>{x.p_n,x.p_e,x.h,x.V,x.psi,x.h_dot,x.phi};
    };
    const auto wrap = [](double angle) {
        angle = std::fmod(angle + M_PI, 2*M_PI);
        return std::fmod(angle + 2*M_PI, 2*M_PI) - M_PI;
    };
    ce::TrajectoryPredict predictor(ce::PredictParams{});
    for (double scale : {1e-4, 1.0, 100.0}) {
        for (double bank : {-50.0, 0.0, 50.0}) {
            for (bool process_noise : {false, true}) {
                SCOPED_TRACE(::testing::Message() << scale << ',' << bank << ',' << process_noise);
                ce::UncertaintyParams params;
                if (!process_noise) params.process_noise_diagonal.fill(0.0);
                ce::TrajectoryUncertainty uncertainty(params);
                ce::PredictState state{10,-5,100,20,3.13,1.2,-bank*M_PI/180.0};
                state.phi_setpoint = -.5*state.phi;
                ce::PredictionInputTrajectory inputs;
                inputs.fill({22,103,-.4,9.80665*std::tan(bank*M_PI/180.0)});
                ce::PredictionMeanTrajectory mean;
                predictor.predict(state,inputs[0],.1,mean);
                ce::PredictStateCovariance expected{};
                for (std::size_t r=0;r<n;++r) for (std::size_t c=0;c<n;++c)
                    expected[r*n+c] = scale*((r==c ? .04 : 0.0)
                        + .002*std::cos(double(r)-double(c)));
                ce::TrajectoryCone actual;
                ASSERT_TRUE(uncertainty.propagateAlongMean(
                    predictor,mean,expected,inputs,.1,actual));
                for (std::size_t k=0;k<ce::kTrajectoryIntervalCount;++k) {
                    const auto base = as_array(mean[k]);
                    const auto next = as_array(predictor.stepRK4(mean[k],inputs[k],.1));
                    std::array<double,n*n> a{};
                    for (std::size_t c=0;c<n;++c) {
                        auto x = base;
                        x[c] += params.finite_difference_step[c];
                        ce::PredictState perturbed{x[0],x[1],x[2],x[3],x[4],x[5],x[6]};
                        perturbed.phi_setpoint = std::isfinite(mean[k].phi_setpoint)
                            ? mean[k].phi_setpoint : mean[k].phi;
                        const auto step = as_array(predictor.stepRK4(perturbed,inputs[k],.1));
                        for (std::size_t r=0;r<n;++r) {
                            double delta = step[r]-next[r];
                            if (r==4 || r==6) delta = wrap(delta);
                            a[r*n+c] = delta/params.finite_difference_step[c];
                        }
                    }
                    ce::PredictStateCovariance next_p{};
                    for (std::size_t r=0;r<n;++r) for (std::size_t c=0;c<n;++c)
                        for (std::size_t i=0;i<n;++i) for (std::size_t j=0;j<n;++j)
                            next_p[r*n+c] += a[r*n+i]*expected[i*n+j]*a[c*n+j];
                    for (std::size_t r=0;r<n;++r) {
                        next_p[r*n+r] = std::max(params.covariance_diagonal_floor,
                            next_p[r*n+r]+params.process_noise_diagonal[r]*.1);
                        for (std::size_t c=r+1;c<n;++c)
                            next_p[r*n+c] = next_p[c*n+r] =
                                .5*(next_p[r*n+c]+next_p[c*n+r]);
                    }
                    expected = next_p;
                    ASSERT_TRUE(ce::TrajectoryUncertainty::covarianceIsFiniteAndPsd(
                        actual[k+1].state_covariance));
                    for (std::size_t i=0;i<n*n;++i)
                        EXPECT_NEAR(actual[k+1].state_covariance[i],expected[i],
                            1e-11*std::max(1.0,std::abs(expected[i])));
                }
            }
        }
    }
}

TEST(TrajectoryUncertainty, DenseEstimatorCovarianceMatchesRectangularTransform)
{
    constexpr std::size_t n = ce::kPredictStateDimension;
    constexpr std::size_t m = ce::kEstimatorBeliefDimension;
    ce::EstimatorTrajectoryBelief belief;
    belief.attitude_q = {1,0,0,0};
    belief.velocity_ned = {15,0,0};
    belief.position_ned = {0,0,0};
    for (std::size_t r=0;r<m;++r) for (std::size_t c=0;c<m;++c)
        belief.covariance[r*m+c] = (r==c ? .04 : 0.0)
            + .002*std::cos(double(r)-double(c));
    ce::PredictState state;
    ce::PredictStateCovariance covariance;
    ASSERT_TRUE(ce::TrajectoryUncertainty{}.initializeFromEstimatorBelief(
        belief,state,covariance));
    // At this axis-aligned mean the finite-difference transform is explicit.
    constexpr double dv = 1e-3;
    std::array<double,n*m> j{};
    j[0*m+6] = j[1*m+7] = 1;
    j[2*m+8] = -1;
    j[3*m+3] = ((15+dv)-15)/dv;
    j[3*m+4] = j[3*m+5] = (std::sqrt(225+dv*dv)-15)/dv;
    j[4*m+4] = std::atan2(dv,15)/dv;
    j[5*m+5] = -1;
    j[6*m+0] = 1;
    for (std::size_t r=0;r<n;++r) for (std::size_t c=0;c<n;++c) {
        double expected = 0;
        for (std::size_t a=0;a<m;++a) for (std::size_t b=0;b<m;++b)
            expected += j[r*m+a]*belief.covariance[a*m+b]*j[c*m+b];
        EXPECT_NEAR(covariance[r*n+c],expected,1e-10);
    }
}
