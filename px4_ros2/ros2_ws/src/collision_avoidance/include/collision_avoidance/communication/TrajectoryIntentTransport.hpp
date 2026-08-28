#pragma once

#include <functional>
#include <cstddef>
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

class TrajectoryIntentPublisher
{
public:
    TrajectoryIntentPublisher(
        rclcpp::Node & node,
        const std::string & topic_name,
        std::size_t history_depth = 5);

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
        std::size_t history_depth = 5);

private:
    rclcpp::Subscription<collision_avoidance::msg::TrajectoryIntent>::SharedPtr
        m_subscription;
};

}  // namespace collision_avoidance::communication
