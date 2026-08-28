#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>

#include <collision_avoidance/estimation/trajectory_prediction/TrajectoryIntent.hpp>

namespace ce = collision_avoidance::estimation;

namespace
{

ce::PredictStateCovariance diagonalCovariance(double value)
{
    ce::PredictStateCovariance covariance{};
    for (std::size_t index = 0; index < ce::kPredictStateDimension; ++index) {
        covariance[index * ce::kPredictStateDimension + index] = value;
    }
    return covariance;
}

ce::TrajectorySample straightSample(float east_offset)
{
    ce::TrajectorySample sample{};
    sample.pos_t0 = {0.0F, east_offset, -100.0F};
    sample.vel_t0 = {10.0F, 0.0F, 0.0F};
    sample.pos_t15 = {15.0F, east_offset, -100.0F};
    sample.pos_t30 = {30.0F, east_offset, -100.0F};
    sample.pos_t45 = {45.0F, east_offset, -100.0F};
    sample.vel_t45 = {10.0F, 0.0F, 0.0F};
    return sample;
}

}  // namespace

TEST(TrajectoryIntent, BuildsRequiredRollCandidateLookup)
{
    constexpr double gravity = 9.80665;
    const auto candidates = ce::makeLevelTurnCandidateTable(
        20.0, 100.0, gravity);

    ASSERT_NE(candidates.find(0), nullptr);
    ASSERT_NE(candidates.find(3), nullptr);
    ASSERT_NE(candidates.find(6), nullptr);
    EXPECT_EQ(candidates.find(7), nullptr);
    EXPECT_NEAR(candidates.find(0)->a_lat_cmd, -gravity, 1.0e-12);
    EXPECT_NEAR(candidates.find(3)->a_lat_cmd, 0.0, 1.0e-12);
    EXPECT_NEAR(candidates.find(6)->a_lat_cmd, gravity, 1.0e-12);
    EXPECT_NEAR(
        candidates.find(4)->a_lat_cmd,
        gravity * std::tan(15.0 * M_PI / 180.0),
        1.0e-12);
}

TEST(TrajectoryIntent, ReconstructorsKeepIndependentState)
{
    ce::ReconstructTrajectory first;
    ce::ReconstructTrajectory second;
    first.calculate_clamp_cubic_spline(straightSample(0.0F));
    const auto before = first.reconstruct(2.25F);

    second.calculate_clamp_cubic_spline(straightSample(50.0F));
    const auto after = first.reconstruct(2.25F);
    const auto other = second.reconstruct(2.25F);

    EXPECT_NEAR(before.pos.x(), 22.5F, 1.0e-4F);
    EXPECT_NEAR(after.pos.x(), before.pos.x(), 1.0e-6F);
    EXPECT_NEAR(after.pos.y(), before.pos.y(), 1.0e-6F);
    EXPECT_NEAR(other.pos.y(), 50.0F, 1.0e-4F);
}

TEST(TrajectoryIntent, TransfersReconstructedMeanAndPropagatesCone)
{
    ce::PredictParams params;
    params.V_min = 10.0;
    params.V_max = 30.0;
    ce::TrajectoryPredict predictor(params);
    const auto candidates = ce::makeLevelTurnCandidateTable(20.0, 120.0);
    ce::TrajectoryIntentSender sender(predictor, candidates);
    ce::TrajectoryIntentReceiver receiver(predictor, candidates);

    const ce::PredictState initial_state{
        10.0, -5.0, 120.0, 20.0, 0.25, 0.0, 0.0};
    const auto initial_covariance = diagonalCovariance(0.04);
    constexpr std::uint64_t source_timestamp_us = 123456789ULL;
    const auto candidate_id = static_cast<std::uint8_t>(
        ce::ManeuverCandidateId::RollPlus15);

    ce::TrajectoryIntentPacket packet;
    ASSERT_TRUE(sender.buildForSelectedCandidate(
        source_timestamp_us,
        candidate_id,
        initial_state,
        initial_covariance,
        packet));

    ce::ReceivedTrajectoryIntent received;
    ASSERT_TRUE(receiver.receive(packet, received));
    EXPECT_EQ(received.source_timestamp_us, source_timestamp_us);
    EXPECT_EQ(received.candidate_id, candidate_id);
    EXPECT_NEAR(received.cone.front().time_offset_s, 0.0, 1.0e-12);
    EXPECT_NEAR(received.cone.back().time_offset_s, 4.5, 1.0e-12);

    ce::PredictionMeanTrajectory source_mean{};
    predictor.predict(
        initial_state,
        *candidates.find(candidate_id),
        ce::kTrajectoryIntentStepSeconds,
        source_mean);
    for (const std::size_t index : {std::size_t{0}, std::size_t{15},
                                    std::size_t{30}, std::size_t{45}}) {
        EXPECT_NEAR(
            received.reconstructed_mean[index].p_n,
            source_mean[index].p_n,
            1.0e-3);
        EXPECT_NEAR(
            received.reconstructed_mean[index].p_e,
            source_mean[index].p_e,
            1.0e-3);
        EXPECT_NEAR(
            received.reconstructed_mean[index].h,
            source_mean[index].h,
            1.0e-3);
    }

    for (std::size_t index = 0; index < ce::kTrajectoryPointCount; ++index) {
        EXPECT_DOUBLE_EQ(
            received.cone[index].mean.p_n,
            received.reconstructed_mean[index].p_n);
        EXPECT_TRUE(ce::TrajectoryUncertainty::covarianceIsFiniteAndPsd(
            received.cone[index].state_covariance));
    }
    EXPECT_GT(
        received.cone.back().position_covariance_ned[0],
        received.cone.front().position_covariance_ned[0]);
}

TEST(TrajectoryIntent, RejectsUnknownCandidateAndInvalidCovariance)
{
    ce::TrajectoryPredict predictor(ce::PredictParams{});
    const auto candidates = ce::makeLevelTurnCandidateTable(20.0, 100.0);
    ce::TrajectoryIntentSender sender(predictor, candidates);
    const ce::PredictState state{0.0, 0.0, 100.0, 20.0, 0.0, 0.0, 0.0};
    auto covariance = diagonalCovariance(0.04);
    ce::TrajectoryIntentPacket packet;

    EXPECT_FALSE(sender.buildForSelectedCandidate(
        1, 7, state, covariance, packet));
    covariance[0] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(sender.buildForSelectedCandidate(
        1,
        static_cast<std::uint8_t>(ce::ManeuverCandidateId::RollZero),
        state,
        covariance,
        packet));
}
