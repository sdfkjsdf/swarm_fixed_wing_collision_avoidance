#include <gtest/gtest.h>

#include <collision_avoidance/communication/TrajectoryIntentTransport.hpp>

namespace ce = collision_avoidance::estimation;
namespace cc = collision_avoidance::communication;

TEST(TrajectoryIntentTransport, UsesReliableVolatileBatchQos)
{
    const auto qos = cc::trajectoryIntentQos(42).get_rmw_qos_profile();
    EXPECT_EQ(qos.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    EXPECT_EQ(qos.depth, 42U);
    EXPECT_EQ(qos.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(qos.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
}

TEST(TrajectoryIntentTransport, SizesHistoryForEveryRefreshInCoordinationWindow)
{
    EXPECT_EQ(cc::requiredTrajectoryIntentHistoryDepth(
        7, 250'000, 50'000), 42U);
    EXPECT_EQ(cc::requiredTrajectoryIntentHistoryDepth(
        3, 250'000, 50'000), 18U);
    EXPECT_EQ(cc::requiredTrajectoryIntentHistoryDepth(
        0, 250'000, 50'000), 1U);
}

TEST(TrajectoryIntentTransport, PreservesFixedPacketFields)
{
    ce::TrajectoryIntentPacket source{};
    source.source_timestamp_us = 987654321ULL;
    source.selection_epoch = 42ULL;
    source.candidate_id = static_cast<std::uint8_t>(
        ce::ManeuverCandidateId::RollMinus30);
    source.candidate_set_size = 3;
    source.candidate_set_kind = ce::CandidateSetKind::V4SafeControl;
    source.candidate_input = {19.5F, 120.0F, 0.0F, -4.25F};
    source.candidate_input_revision = 123456789012345ULL;
    source.nominal_lateral_acceleration_mps2 = 1.75F;
    source.safe_rejoin_requested = true;
    for (std::size_t index = 0; index < source.initial_state.size(); ++index) {
        source.initial_state[index] = static_cast<float>(index) + 0.25F;
    }
    for (std::size_t index = 0; index < source.initial_covariance.size(); ++index) {
        source.initial_covariance[index] = static_cast<float>(index) * 0.01F;
    }
    source.compressed_mean = {
        {1.0F, 2.0F, 3.0F},
        {4.0F, 5.0F, 6.0F},
        {7.0F, 8.0F, 9.0F},
        {10.0F, 11.0F, 12.0F},
        {13.0F, 14.0F, 15.0F},
        {16.0F, 17.0F, 18.0F}};

    const auto message = cc::toRosMessage(source);
    const auto received = cc::fromRosMessage(message);

    EXPECT_EQ(received.source_timestamp_us, source.source_timestamp_us);
    EXPECT_EQ(received.selection_epoch, source.selection_epoch);
    EXPECT_EQ(received.candidate_id, source.candidate_id);
    EXPECT_EQ(received.candidate_set_size, source.candidate_set_size);
    EXPECT_EQ(received.candidate_set_kind, source.candidate_set_kind);
    EXPECT_EQ(received.candidate_input, source.candidate_input);
    EXPECT_EQ(
        received.candidate_input_revision,
        source.candidate_input_revision);
    EXPECT_FLOAT_EQ(
        received.nominal_lateral_acceleration_mps2,
        source.nominal_lateral_acceleration_mps2);
    EXPECT_EQ(received.safe_rejoin_requested, source.safe_rejoin_requested);
    EXPECT_EQ(received.initial_state, source.initial_state);
    EXPECT_EQ(received.initial_covariance, source.initial_covariance);
    EXPECT_FLOAT_EQ(
        received.compressed_mean.pos_t0.x,
        source.compressed_mean.pos_t0.x);
    EXPECT_FLOAT_EQ(
        received.compressed_mean.vel_t45.z,
        source.compressed_mean.vel_t45.z);
}
