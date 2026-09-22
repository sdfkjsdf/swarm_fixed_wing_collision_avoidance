#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <ostream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <collision_avoidance/estimation/trajectory_prediction/TrajectoryIntent.hpp>
#include <collision_avoidance/common/StoppedRecordBuffer.hpp>
#include <collision_avoidance/msg/trajectory_intent.hpp>

namespace collision_avoidance::communication
{

// Diagnostics only: never part of the trajectory packet or a control clock.
// One callback-group writer; export only after the executor has stopped.
struct TrajectoryTransportTimingRecord
{
    std::uint64_t source_us{0}, epoch{0}, input_revision{0};
    std::int64_t wall_ns{0}, dds_source_ns{0}, dds_received_ns{0};
    std::uint8_t candidate_id{0};
};
using TrajectoryTransportTimingBuffer = common::StoppedRecordBuffer<
    TrajectoryTransportTimingRecord, 65536>;

collision_avoidance::msg::TrajectoryIntent toRosMessage(
    const estimation::TrajectoryIntentPacket & packet);

estimation::TrajectoryIntentPacket fromRosMessage(
    const collision_avoidance::msg::TrajectoryIntent & message);

std::size_t requiredTrajectoryIntentHistoryDepth(
    std::size_t candidate_count,
    std::uint64_t coordination_delay_us,
    std::uint64_t trajectory_refresh_period_us) noexcept;

rclcpp::QoS trajectoryIntentQos(std::size_t history_depth);

class TrajectoryIntentPublisher
{
public:
    TrajectoryIntentPublisher(
        rclcpp::Node & node,
        const std::string & topic_name,
        std::size_t history_depth,
        bool measure_transport = false);

    void publish(const estimation::TrajectoryIntentPacket & packet);
    void writeStoppedTiming(std::ostream & out, int vehicle) const;

private:
    std::unique_ptr<TrajectoryTransportTimingBuffer> m_timing;
    rclcpp::Publisher<collision_avoidance::msg::TrajectoryIntent>::SharedPtr
        m_publisher;
};

class TrajectoryIntentSubscription
{
public:
    using PacketCallback =
        std::function<void(const estimation::TrajectoryIntentPacket &)>;

    TrajectoryIntentSubscription(
        rclcpp::Node & node,
        const std::string & topic_name,
        PacketCallback callback,
        std::size_t history_depth,
        bool measure_transport = false);
    void writeStoppedTiming(std::ostream & out, int vehicle, int peer) const;

private:
    std::unique_ptr<TrajectoryTransportTimingBuffer> m_timing;
    rclcpp::Subscription<collision_avoidance::msg::TrajectoryIntent>::SharedPtr
        m_subscription;
};

}  // namespace collision_avoidance::communication
