#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <collision_avoidance/estimation/trajectory_prediction/TrajectoryIntent.hpp>
#include <collision_avoidance/msg/trajectory_intent.hpp>

namespace collision_avoidance::communication
{

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
        std::size_t history_depth);

    void publish(const estimation::TrajectoryIntentPacket & packet);

private:
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
        std::size_t history_depth);

private:
    rclcpp::Subscription<collision_avoidance::msg::TrajectoryIntent>::SharedPtr
        m_subscription;
};

}  // namespace collision_avoidance::communication
