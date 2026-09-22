#include <gtest/gtest.h>
#include <chrono>
#include <sstream>
#include <thread>

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
    source.source_execution_input = {17.0F, 100.0F, 0.0F, 8.0F};
    source.source_execution_input_available = true;
    source.nominal_lateral_acceleration_mps2 = 1.75F;
    source.safe_rejoin_requested = true;
    source.initial_roll_setpoint_rad = -0.42F;
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
    EXPECT_EQ(received.source_execution_input, source.source_execution_input);
    EXPECT_EQ(received.source_execution_input_available, source.source_execution_input_available);
    EXPECT_EQ(
        received.candidate_input_revision,
        source.candidate_input_revision);
    EXPECT_FLOAT_EQ(
        received.nominal_lateral_acceleration_mps2,
        source.nominal_lateral_acceleration_mps2);
    EXPECT_EQ(received.safe_rejoin_requested, source.safe_rejoin_requested);
    EXPECT_EQ(received.initial_state, source.initial_state);
    EXPECT_FLOAT_EQ(received.initial_roll_setpoint_rad, source.initial_roll_setpoint_rad);
    EXPECT_EQ(received.initial_covariance, source.initial_covariance);
    EXPECT_FLOAT_EQ(
        received.compressed_mean.pos_t0.x,
        source.compressed_mean.pos_t0.x);
    EXPECT_FLOAT_EQ(
        received.compressed_mean.vel_t45.z,
        source.compressed_mean.vel_t45.z);
}

TEST(TrajectoryIntentTransport, StoppedTimingDoesNotChangePacketOrDelayInputHandoff)
{
    if (!rclcpp::ok()) { int argc = 0; rclcpp::init(argc, nullptr); }
    auto node = std::make_shared<rclcpp::Node>("stopped_transport_timing_test");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    ce::TrajectoryIntentPacket packet{};
    packet.source_timestamp_us = 123456;
    packet.selection_epoch = 17;
    packet.candidate_id = 6;
    packet.candidate_input_revision = 91;
    packet.candidate_input = {20.F, 100.F, 0.F, 8.F};
    std::uint64_t delivered = 0;
    std::int64_t callback_wall = 0;
    std::ostringstream during_callback;
    cc::TrajectoryIntentSubscription * subscription = nullptr;
    cc::TrajectoryIntentSubscription receiver(*node, "/test/stopped_transport",
        [&](const ce::TrajectoryIntentPacket & received) {
            ++delivered;
            callback_wall = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            EXPECT_EQ(received.source_timestamp_us, packet.source_timestamp_us);
            EXPECT_EQ(received.candidate_input, packet.candidate_input);
            EXPECT_EQ(received.selection_epoch, packet.selection_epoch);
            // Same-thread test only: prove append occurs after handing off input.
            subscription->writeStoppedTiming(during_callback, 1, 0);
        }, 42, true);
    subscription = &receiver;
    cc::TrajectoryIntentPublisher sender(*node, "/test/stopped_transport", 42, true);
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    sender.publish(packet);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!delivered && std::chrono::steady_clock::now() < deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ASSERT_EQ(delivered, 1U);
    EXPECT_NE(during_callback.str().find("[stop-transport-begin],1,1,0,rx,0,0"), std::string::npos);
    std::ostringstream tx, rx;
    sender.writeStoppedTiming(tx, 0);
    receiver.writeStoppedTiming(rx, 1, 0);
    EXPECT_NE(tx.str().find("[stop-transport-begin],1,0,-1,tx,1,0"), std::string::npos);
    EXPECT_NE(rx.str().find("[stop-transport-begin],1,1,0,rx,1,0"), std::string::npos);
    auto event_time = [](const std::string & text) {
        auto begin = text.find("[stop-transport],");
        for (int i = 0; i < 5; ++i) begin = text.find(',', begin) + 1;
        return std::stoll(text.substr(begin));
    };
    EXPECT_LE(event_time(tx.str()), event_time(rx.str()));
    EXPECT_LE(event_time(rx.str()), callback_wall);

    cc::TrajectoryIntentPublisher disabled(*node, "/test/disabled_transport", 42);
    disabled.publish(packet);
    std::ostringstream disabled_log;
    disabled.writeStoppedTiming(disabled_log, 0);
    EXPECT_TRUE(disabled_log.str().empty());
}
