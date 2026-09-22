#include <gtest/gtest.h>
#include <chrono>
#include <cstring>
#include <limits>
#include <sstream>
#include <thread>
#include <rclcpp/serialization.hpp>

#include <collision_avoidance/communication/TrajectoryIntentTransport.hpp>

namespace ce = collision_avoidance::estimation;
namespace cc = collision_avoidance::communication;

TEST(TrajectoryIntentTransport, UsesReliableVolatileBatchQos)
{
    const auto qos = cc::trajectoryIntentQos(6).get_rmw_qos_profile();
    EXPECT_EQ(qos.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    EXPECT_EQ(qos.depth, 6U);
    EXPECT_EQ(qos.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(qos.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
}

TEST(TrajectoryIntentTransport, SizesHistoryForEveryRefreshInCoordinationWindow)
{
    EXPECT_EQ(cc::requiredTrajectoryIntentHistoryDepth(
        250'000, 50'000), 6U);
    EXPECT_EQ(cc::requiredTrajectoryIntentHistoryDepth(
        0, 50'000), 1U);
    EXPECT_EQ(cc::requiredTrajectoryIntentHistoryDepth(
        250'000, 0), 1U);
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

    cc::TrajectoryIntentPackets packets{};
    for (std::size_t i = 0; i < 3; ++i) {
        packets[i] = source;
        packets[i].candidate_id = static_cast<std::uint8_t>(i);
    }
    collision_avoidance::msg::TrajectoryIntentBatch message;
    ASSERT_TRUE(cc::toRosMessage(packets, 3, message));
    cc::TrajectoryIntentPackets decoded{};
    ASSERT_TRUE(cc::fromRosMessage(message, decoded));
    const auto & received = decoded[source.candidate_id];

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
    packet.candidate_set_size = 7;
    packet.candidate_input_revision = 91;
    packet.candidate_input = {20.F, 100.F, 0.F, 8.F};
    std::uint64_t delivered = 0;
    std::uint64_t wire_messages = 0;
    auto wire_receiver = node->create_subscription<collision_avoidance::msg::TrajectoryIntentBatch>(
        "/test/stopped_transport", cc::trajectoryIntentQos(6),
        [&](collision_avoidance::msg::TrajectoryIntentBatch::ConstSharedPtr message) {
            ++wire_messages;
            EXPECT_EQ(message->candidate_set_size, 7);
        });
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
        }, 6, true);
    subscription = &receiver;
    cc::TrajectoryIntentPublisher sender(*node, "/test/stopped_transport", 6, true);
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    cc::TrajectoryIntentPackets packets{};
    for (std::size_t i = 0; i < packets.size(); ++i) {
        packets[i] = packet;
        packets[i].candidate_id = static_cast<std::uint8_t>(i);
    }
    ASSERT_TRUE(sender.publish(packets, 7));
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while ((delivered < 7 || wire_messages == 0) && std::chrono::steady_clock::now() < deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ASSERT_EQ(delivered, 7U);
    EXPECT_EQ(wire_messages, 1U);
    EXPECT_NE(during_callback.str().find("[stop-transport-begin],1,1,0,rx,0,0"), std::string::npos);
    std::ostringstream tx, rx;
    sender.writeStoppedTiming(tx, 0);
    receiver.writeStoppedTiming(rx, 1, 0);
    EXPECT_NE(tx.str().find("[stop-transport-begin],1,0,-1,tx,7,0"), std::string::npos);
    EXPECT_NE(rx.str().find("[stop-transport-begin],1,1,0,rx,7,0"), std::string::npos);
    auto event_time = [](const std::string & text) {
        auto begin = text.find("[stop-transport],");
        for (int i = 0; i < 5; ++i) begin = text.find(',', begin) + 1;
        return std::stoll(text.substr(begin));
    };
    EXPECT_LE(event_time(tx.str()), event_time(rx.str()));
    EXPECT_LE(event_time(rx.str()), callback_wall);

    cc::TrajectoryIntentPublisher disabled(*node, "/test/disabled_transport", 6);
    EXPECT_TRUE(disabled.publish(packets, 7));
    std::ostringstream disabled_log;
    disabled.writeStoppedTiming(disabled_log, 0);
    EXPECT_TRUE(disabled_log.str().empty());
}

TEST(TrajectoryIntentTransport, RejectsMixedOrIncompleteBatchesWithoutChangingOutput)
{
    cc::TrajectoryIntentPackets packets{};
    for (std::size_t i = 0; i < packets.size(); ++i) {
        packets[i].source_timestamp_us = 1'000'000;
        packets[i].selection_epoch = 4;
        packets[i].candidate_id = static_cast<std::uint8_t>(i);
        packets[i].candidate_set_size = 7;
    }
    collision_avoidance::msg::TrajectoryIntentBatch message;
    ASSERT_TRUE(cc::toRosMessage(packets, 7, message));
    EXPECT_FALSE(cc::toRosMessage(packets, 0, message));
    EXPECT_FALSE(cc::toRosMessage(packets, 6, message));
    EXPECT_FALSE(cc::toRosMessage(packets, 8, message));
    const auto original = packets;
    packets[6].initial_covariance[0] = 1.F;
    EXPECT_FALSE(cc::toRosMessage(packets, 7, message));
    packets = original; ++packets[6].source_timestamp_us;
    EXPECT_FALSE(cc::toRosMessage(packets, 7, message));
    packets = original; ++packets[6].selection_epoch;
    EXPECT_FALSE(cc::toRosMessage(packets, 7, message));
    packets = original; packets[6].candidate_id = 0;
    EXPECT_FALSE(cc::toRosMessage(packets, 7, message));
    packets = original; packets[6].candidate_set_kind = ce::CandidateSetKind::V4SafeControl;
    EXPECT_FALSE(cc::toRosMessage(packets, 7, message));
    packets = original; packets[6].initial_state[0] = 1.F;
    EXPECT_FALSE(cc::toRosMessage(packets, 7, message));
    packets = original; packets[6].initial_roll_setpoint_rad = .1F;
    EXPECT_FALSE(cc::toRosMessage(packets, 7, message));
    packets = original; packets[6].source_execution_input[3] = 1.F;
    EXPECT_FALSE(cc::toRosMessage(packets, 7, message));
    packets = original; packets[6].source_execution_input_available = true;
    EXPECT_FALSE(cc::toRosMessage(packets, 7, message));
    EXPECT_EQ(message.source_timestamp_us, original[0].source_timestamp_us);
    EXPECT_EQ(message.initial_covariance, original[0].initial_covariance);

    cc::TrajectoryIntentPackets decoded{};
    decoded[0].source_timestamp_us = 99;
    message.candidates[6].candidate_id = 0;
    EXPECT_FALSE(cc::fromRosMessage(message, decoded));
    EXPECT_EQ(decoded[0].source_timestamp_us, 99U);
    message.candidates[6].candidate_id = 7;
    EXPECT_FALSE(cc::fromRosMessage(message, decoded));
    message.candidates[6].candidate_id = 6;
    message.candidate_set_size = 8;
    EXPECT_FALSE(cc::fromRosMessage(message, decoded));
    message.candidate_set_size = 0;
    EXPECT_FALSE(cc::fromRosMessage(message, decoded));
    message.candidate_set_size = 7; message.candidate_set_kind = 2;
    EXPECT_FALSE(cc::fromRosMessage(message, decoded));
}

TEST(TrajectoryIntentTransport, SevenCandidatesShareSourceSnapshotAndPreserveEveryCone)
{
    ce::TrajectoryPredict predictor(ce::PredictParams{});
    auto table = ce::makeLevelTurnCandidateTable(20., 100.);
    ce::TrajectoryIntentSender sender(predictor, table);
    ce::TrajectoryIntentReceiver receiver(predictor);
    ce::PredictState state{};
    state.V = 20.; state.h = 100.;
    ce::PredictStateCovariance covariance{};
    for (std::size_t i = 0; i < 7; ++i) covariance[i * 7 + i] = .04;
    covariance[1] = covariance[7] = .01; // Preserve off-diagonal information.
    cc::TrajectoryIntentPackets packets{};
    for (std::size_t i = 0; i < packets.size(); ++i) {
        ASSERT_TRUE(sender.buildForSelectedCandidate(1'000'000, i, state, covariance, packets[i], 4));
        packets[i].candidate_set_size = 7;
        packets[i].source_execution_input = {
            20.F, std::numeric_limits<float>::quiet_NaN(), 0.F, 4.F};
        packets[i].source_execution_input_available = true;
    }
    collision_avoidance::msg::TrajectoryIntentBatch message;
    ASSERT_TRUE(cc::toRosMessage(packets, 7, message));
    static_assert(rosidl_generator_traits::has_fixed_size<decltype(message)>::value);
    rclcpp::Serialization<decltype(message)> serializer;
    rclcpp::SerializedMessage bytes;
    serializer.serialize_message(&message, &bytes);
    EXPECT_LE(bytes.size(), 1024U); // Previously 1344 B with repeated source fields.
    std::cout << "Seven-candidate CDR batch bytes: " << bytes.size() << '\n';
    decltype(message) wire;
    serializer.deserialize_message(&bytes, &wire);
    cc::TrajectoryIntentPackets decoded{};
    ASSERT_TRUE(cc::fromRosMessage(wire, decoded));
    for (std::size_t i = 0; i < packets.size(); ++i) {
        EXPECT_EQ(decoded[i].candidate_input_revision, packets[i].candidate_input_revision);
        EXPECT_EQ(decoded[i].initial_state, packets[i].initial_state);
        EXPECT_EQ(decoded[i].initial_roll_setpoint_rad, packets[i].initial_roll_setpoint_rad);
        EXPECT_TRUE(decoded[i].source_execution_input_available);
        EXPECT_EQ(std::memcmp(decoded[i].source_execution_input.data(),
            packets[i].source_execution_input.data(), 4 * sizeof(float)), 0);
        EXPECT_EQ(std::memcmp(decoded[i].initial_covariance.data(), packets[i].initial_covariance.data(),
            49 * sizeof(float)), 0);
        EXPECT_EQ(std::memcmp(&decoded[i].compressed_mean, &packets[i].compressed_mean,
            sizeof(ce::TrajectorySample)), 0);
        ce::ReceivedTrajectoryIntent before{}, after{};
        ASSERT_TRUE(receiver.receive(packets[i], before));
        ASSERT_TRUE(receiver.receive(decoded[i], after));
        const auto expect_same_state = [](const auto & a, const auto & b) {
            EXPECT_DOUBLE_EQ(a.p_n, b.p_n); EXPECT_DOUBLE_EQ(a.p_e, b.p_e);
            EXPECT_DOUBLE_EQ(a.h, b.h); EXPECT_DOUBLE_EQ(a.V, b.V);
            EXPECT_DOUBLE_EQ(a.psi, b.psi); EXPECT_DOUBLE_EQ(a.h_dot, b.h_dot);
            EXPECT_DOUBLE_EQ(a.phi, b.phi); EXPECT_DOUBLE_EQ(a.phi_setpoint, b.phi_setpoint);
        };
        for (std::size_t k = 0; k < before.cone.size(); ++k) {
            expect_same_state(before.cone[k].mean, after.cone[k].mean);
            EXPECT_EQ(before.cone[k].state_covariance, after.cone[k].state_covariance);
            EXPECT_EQ(before.cone[k].position_covariance_ned, after.cone[k].position_covariance_ned);
        }
        ce::PredictState before_state, after_state;
        ce::PredictStateCovariance before_covariance, after_covariance;
        ASSERT_TRUE(receiver.executionStateAt(before, 1'150'000, before_state, before_covariance));
        ASSERT_TRUE(receiver.executionStateAt(after, 1'150'000, after_state, after_covariance));
        expect_same_state(before_state, after_state);
        EXPECT_EQ(before_covariance, after_covariance);
    }
}
