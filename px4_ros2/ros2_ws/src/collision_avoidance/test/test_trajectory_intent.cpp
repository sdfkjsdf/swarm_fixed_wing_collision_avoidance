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

TEST(TrajectoryIntent, AlignsStateAndCovarianceUsingPublishedInputNotCandidate)
{
    ce::TrajectoryPredict predictor(ce::PredictParams{});
    ce::TrajectoryUncertainty uncertainty;
    const auto table = ce::makeLevelTurnCandidateTable(20,100);
    ce::TrajectoryIntentSender sender(predictor,table);
    ce::TrajectoryIntentReceiver receiver(predictor);
    ce::PredictState initial{0,0,100,20,0,0,.25};
    initial.phi_setpoint = .4;
    ce::TrajectoryIntentPacket packet;
    ASSERT_TRUE(sender.buildForSelectedCandidate(1'000'000,0,initial,
        diagonalCovariance(.04),packet));
    packet.source_execution_input = {19,100,0,8};
    packet.source_execution_input_available = true;
    ce::ReceivedTrajectoryIntent received;
    ASSERT_TRUE(receiver.receive(packet,received));
    auto expected_state = received.cone.front().mean;
    auto expected_covariance = received.cone.front().state_covariance;
    ASSERT_TRUE(uncertainty.compensateFusionHorizonDelay(predictor,
        received.source_execution_input,.2,expected_state,expected_covariance));
    ce::PredictState aligned{};
    ce::PredictStateCovariance covariance{};
    ASSERT_TRUE(receiver.executionStateAt(received,1'200'000,aligned,covariance));
    EXPECT_DOUBLE_EQ(aligned.p_n,expected_state.p_n);
    EXPECT_DOUBLE_EQ(aligned.p_e,expected_state.p_e);
    EXPECT_DOUBLE_EQ(aligned.phi,expected_state.phi);
    EXPECT_DOUBLE_EQ(aligned.phi_setpoint,expected_state.phi_setpoint);
    for (std::size_t i=0;i<covariance.size();++i)
        EXPECT_NEAR(covariance[i],expected_covariance[i],1e-10);
    EXPECT_GT(aligned.phi,received.cone[2].mean.phi);

    // A new future command begins at the aligned state, never before it.
    ce::TrajectoryIntentPacket rejoin;
    ASSERT_TRUE(sender.buildForCandidateInput(1'200'000,3,{20,100,0,-4},
        aligned,covariance,rejoin));
    EXPECT_FLOAT_EQ(rejoin.initial_state[0],static_cast<float>(aligned.p_n));
    EXPECT_FLOAT_EQ(rejoin.initial_state[1],static_cast<float>(aligned.p_e));
    EXPECT_FALSE(receiver.executionStateAt(received,999'999,aligned,covariance));
    EXPECT_FALSE(receiver.executionStateAt(received,2'000'001,aligned,covariance));

    // Missing/invalid auxiliary input blocks extrapolation, not the candidates.
    packet.source_execution_input_available = false;
    ASSERT_TRUE(receiver.receive(packet,received));
    EXPECT_FALSE(receiver.executionStateAt(received,1'200'000,aligned,covariance));
    EXPECT_TRUE(receiver.executionStateAt(received,1'000'000,aligned,covariance));
    packet.source_execution_input_available = true;
    packet.source_execution_input[3] = std::numeric_limits<float>::quiet_NaN();
    ASSERT_TRUE(receiver.receive(packet,received));
    EXPECT_FALSE(receiver.executionStateAt(received,1'200'000,aligned,covariance));
}

TEST(TrajectoryIntent, SharesTheCommandFilterSeedNotTheActualRoll)
{
    ce::TrajectoryPredict predictor(ce::PredictParams{});
    const auto table = ce::makeLevelTurnCandidateTable(20.0,100.0);
    ce::TrajectoryIntentSender sender(predictor,table);
    ce::TrajectoryIntentReceiver receiver(predictor);
    ce::PredictState x{0,0,100,20,0,0,.1};
    x.phi_setpoint = .4;
    ce::TrajectoryIntentPacket packet;
    ASSERT_TRUE(sender.buildForSelectedCandidate(1000000,0,x,diagonalCovariance(.04),packet,4));
    EXPECT_FLOAT_EQ(packet.initial_roll_setpoint_rad,.4F);
    ce::ReceivedTrajectoryIntent received;
    ASSERT_TRUE(receiver.receive(packet,received));
    x.phi_setpoint = packet.initial_roll_setpoint_rad;
    ce::PredictionMeanTrajectory mean;
    predictor.predict(x,*table.find(0),.1,mean);
    for (std::size_t k=0;k<mean.size();++k) {
        EXPECT_NEAR(received.reconstructed_mean[k].phi,mean[k].phi,2e-7);
        EXPECT_NEAR(received.cone[k].mean.phi_setpoint,mean[k].phi_setpoint,2e-7);
        EXPECT_TRUE(ce::TrajectoryUncertainty::covarianceIsFiniteAndPsd(received.cone[k].state_covariance));
        EXPECT_LT(received.cone[k].state_covariance[48],1.0);
    }
    packet.initial_roll_setpoint_rad = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(receiver.receive(packet,received));
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
    packet.nominal_lateral_acceleration_mps2 = -1.25F;
    packet.safe_rejoin_requested = true;
    ce::ReceivedTrajectoryIntent received;
    ASSERT_TRUE(receiver.receive(packet, received));

    EXPECT_EQ(received.reconstructed_mean.size(), ce::kTrajectoryPointCount);
    EXPECT_NE(received.candidate_input_revision, 0U);
    EXPECT_DOUBLE_EQ(received.candidate_input.V_cmd, 19.25);
    EXPECT_TRUE(std::isnan(received.candidate_input.h_cmd));
    EXPECT_DOUBLE_EQ(received.candidate_input.h_dot_cmd, 0.0);
    EXPECT_NEAR(received.candidate_input.a_lat_cmd, 4.2, 1.0e-6);
    EXPECT_DOUBLE_EQ(received.nominal_lateral_acceleration_mps2, -1.25);
    EXPECT_TRUE(received.safe_rejoin_requested);
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
