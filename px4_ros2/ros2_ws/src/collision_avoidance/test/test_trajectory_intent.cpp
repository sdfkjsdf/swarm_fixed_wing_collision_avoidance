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
    EXPECT_NEAR(
        candidates.find(0)->a_lat_cmd,
        -gravity * std::tan(50.0 * M_PI / 180.0),
        1.0e-12);
    EXPECT_NEAR(candidates.find(3)->a_lat_cmd, 0.0, 1.0e-12);
    EXPECT_NEAR(
        candidates.find(6)->a_lat_cmd,
        gravity * std::tan(50.0 * M_PI / 180.0),
        1.0e-12);
    EXPECT_NEAR(
        ce::PredictParams{}.a_lat_max,
        candidates.find(6)->a_lat_cmd,
        1.0e-12);
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
    ce::TrajectoryIntentReceiver receiver(predictor);

    const ce::PredictState initial_state{
        10.0, -5.0, 120.0, 20.0, 0.25, 0.0, 0.0};
    const auto initial_covariance = diagonalCovariance(0.04);
    constexpr std::uint64_t source_timestamp_us = 123456789ULL;
    constexpr std::uint64_t selection_epoch = 17ULL;
    const auto candidate_id = static_cast<std::uint8_t>(
        ce::ManeuverCandidateId::RollPlus15);

    ce::TrajectoryIntentPacket packet;
    ASSERT_TRUE(sender.buildForSelectedCandidate(
        source_timestamp_us,
        candidate_id,
        initial_state,
        initial_covariance,
        packet,
        selection_epoch));

    ce::ReceivedTrajectoryIntent received;
    ASSERT_TRUE(receiver.receive(packet, received));
    EXPECT_EQ(received.source_timestamp_us, source_timestamp_us);
    EXPECT_EQ(received.selection_epoch, selection_epoch);
    EXPECT_EQ(received.candidate_id, candidate_id);
    EXPECT_EQ(
        received.candidate_input_revision,
        packet.candidate_input_revision);
    EXPECT_NE(received.candidate_input_revision, 0U);
    EXPECT_NEAR(received.cone.front().time_offset_s, 0.0, 1.0e-12);
    EXPECT_NEAR(received.cone.back().time_offset_s, 4.5, 1.0e-12);

    ce::PredictionMeanTrajectory source_mean{};
    predictor.predict(
        initial_state,
        received.candidate_input,
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

TEST(TrajectoryIntent, DynamicInputIsTransportedInsteadOfReconstructedFromId)
{
    ce::TrajectoryPredict predictor(ce::PredictParams{});
    const auto candidates = ce::makeLevelTurnCandidateTable(20.0, 100.0);
    ce::TrajectoryIntentSender sender(predictor, candidates);
    ce::TrajectoryIntentReceiver receiver(predictor);
    const ce::PredictState state{0.0, 0.0, 100.0, 20.0, 0.0, 0.0, 0.0};
    const auto covariance = diagonalCovariance(0.04);
    const auto candidate_id = static_cast<std::uint8_t>(
        ce::ManeuverCandidateId::RollZero);
    const ce::PredictInput dynamic_input{
        19.25,
        std::numeric_limits<double>::quiet_NaN(),
        0.0,
        4.2};

    ce::TrajectoryIntentPacket packet;
    ASSERT_TRUE(sender.buildForCandidateInput(
        1'000'000ULL,
        candidate_id,
        dynamic_input,
        state,
        covariance,
        packet,
        4ULL));
    ce::ReceivedTrajectoryIntent received;
    ASSERT_TRUE(receiver.receive(packet, received));

    EXPECT_EQ(received.reconstructed_mean.size(), ce::kTrajectoryPointCount);
    EXPECT_NE(received.candidate_input_revision, 0U);
    EXPECT_DOUBLE_EQ(received.candidate_input.V_cmd, 19.25);
    EXPECT_TRUE(std::isnan(received.candidate_input.h_cmd));
    EXPECT_DOUBLE_EQ(received.candidate_input.h_dot_cmd, 0.0);
    EXPECT_NEAR(received.candidate_input.a_lat_cmd, 4.2, 1.0e-6);
    EXPECT_GT(std::abs(received.reconstructed_mean.back().p_e), 1.0);

    ce::TrajectoryIntentPacket different_packet;
    auto different_input = dynamic_input;
    different_input.a_lat_cmd = -4.2;
    ASSERT_TRUE(sender.buildForCandidateInput(
        1'000'000ULL,
        candidate_id,
        different_input,
        state,
        covariance,
        different_packet,
        4ULL));
    EXPECT_NE(
        different_packet.candidate_input_revision,
        packet.candidate_input_revision);

    auto tampered_packet = packet;
    tampered_packet.candidate_input[3] += 1.0F;
    EXPECT_FALSE(receiver.receive(tampered_packet, received));
}

TEST(TrajectoryIntent, DelayedRolloutUsesCurrentThenCandidateInputEverywhere)
{
    ce::PredictParams params;
    params.phi_rate_max = 70.0 * M_PI / 180.0;
    ce::TrajectoryPredict predictor(params);
    const auto candidates = ce::makeLevelTurnCandidateTable(20.0, 100.0);
    ce::TrajectoryIntentSender sender(predictor, candidates);
    ce::TrajectoryIntentReceiver receiver(predictor);
    const ce::PredictState state{0.0, 0.0, 100.0, 20.0, 0.0, 0.0, 0.0};
    const auto covariance = diagonalCovariance(0.04);
    const ce::PredictInput current_input = *candidates.find(
        static_cast<std::uint8_t>(ce::ManeuverCandidateId::RollPlus50));
    const auto candidate_id = static_cast<std::uint8_t>(
        ce::ManeuverCandidateId::RollMinus50);
    constexpr double command_delay_s = 0.25;

    ce::TrajectoryIntentPacket packet;
    ASSERT_TRUE(sender.buildForSelectedCandidateWithCommandDelay(
        2'000'000ULL,
        candidate_id,
        current_input,
        command_delay_s,
        state,
        covariance,
        packet,
        9ULL));
    ce::ReceivedTrajectoryIntent received;
    ASSERT_TRUE(receiver.receive(packet, received));

    EXPECT_NEAR(received.command_delay_s, command_delay_s, 1.0e-7);
    EXPECT_NEAR(
        received.current_input.a_lat_cmd,
        current_input.a_lat_cmd,
        1.0e-6);

    ce::PredictionMeanTrajectory expected{};
    predictor.predictWithCommandDelay(
        state,
        received.current_input,
        received.candidate_input,
        received.command_delay_s,
        ce::kTrajectoryIntentStepSeconds,
        expected);
    ce::PredictionMeanTrajectory immediate{};
    predictor.predict(
        state,
        received.candidate_input,
        ce::kTrajectoryIntentStepSeconds,
        immediate);

    EXPECT_GT(expected[2].phi, 0.0);
    EXPECT_LT(immediate[2].phi, 0.0);
    const auto at_020 = predictor.stepRK4(
        predictor.stepRK4(state, received.current_input, 0.1),
        received.current_input,
        0.1);
    const auto at_025 = predictor.stepRK4(
        at_020, received.current_input, 0.05);
    const auto at_030 = predictor.stepRK4(
        at_025, received.candidate_input, 0.05);
    EXPECT_NEAR(expected[3].phi, at_030.phi, 1.0e-12);

    for (const std::size_t index : {std::size_t{0}, std::size_t{15},
                                    std::size_t{30}, std::size_t{45}}) {
        EXPECT_NEAR(received.reconstructed_mean[index].p_n, expected[index].p_n, 1.0e-3);
        EXPECT_NEAR(received.reconstructed_mean[index].p_e, expected[index].p_e, 1.0e-3);
        EXPECT_NEAR(received.reconstructed_mean[index].phi, expected[index].phi, 1.0e-9);
        EXPECT_TRUE(ce::TrajectoryUncertainty::covarianceIsFiniteAndPsd(
            received.cone[index].state_covariance));
    }
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
