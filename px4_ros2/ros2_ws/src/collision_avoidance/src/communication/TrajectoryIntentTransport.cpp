#include <collision_avoidance/communication/TrajectoryIntentTransport.hpp>

#include <algorithm>
#include <array>
#include <utility>

namespace collision_avoidance::communication
{
namespace
{

using estimation::TrajectorySample;
using estimation::Vec3;

rclcpp::SensorDataQoS intentQos(std::size_t history_depth)
{
    rclcpp::SensorDataQoS qos;
    qos.keep_last(history_depth);
    return qos;
}

std::array<float, 18> encodeCompressedMean(
    const TrajectorySample & sample) noexcept
{
    return {
        sample.pos_t0.x, sample.pos_t0.y, sample.pos_t0.z,
        sample.vel_t0.x, sample.vel_t0.y, sample.vel_t0.z,
        sample.pos_t15.x, sample.pos_t15.y, sample.pos_t15.z,
        sample.pos_t30.x, sample.pos_t30.y, sample.pos_t30.z,
        sample.pos_t45.x, sample.pos_t45.y, sample.pos_t45.z,
        sample.vel_t45.x, sample.vel_t45.y, sample.vel_t45.z};
}

TrajectorySample decodeCompressedMean(
    const std::array<float, 18> & values) noexcept
{
    return TrajectorySample{
        Vec3{values[0], values[1], values[2]},
        Vec3{values[3], values[4], values[5]},
        Vec3{values[6], values[7], values[8]},
        Vec3{values[9], values[10], values[11]},
        Vec3{values[12], values[13], values[14]},
        Vec3{values[15], values[16], values[17]}};
}

}  // namespace

collision_avoidance::msg::TrajectoryIntent toRosMessage(
    const estimation::TrajectoryIntentPacket & packet)
{
    collision_avoidance::msg::TrajectoryIntent message;
    message.source_timestamp_us = packet.source_timestamp_us;
    message.selection_epoch = packet.selection_epoch;
    message.candidate_id = packet.candidate_id;
    message.candidate_set_size = packet.candidate_set_size;
    message.candidate_set_kind = static_cast<std::uint8_t>(
        packet.candidate_set_kind);
    std::copy(
        packet.candidate_input.begin(),
        packet.candidate_input.end(),
        message.candidate_input.begin());
    message.candidate_input_revision = packet.candidate_input_revision;
    std::copy(
        packet.initial_state.begin(),
        packet.initial_state.end(),
        message.initial_state.begin());
    std::copy(
        packet.initial_covariance.begin(),
        packet.initial_covariance.end(),
        message.initial_covariance.begin());
    const auto compressed_mean = encodeCompressedMean(packet.compressed_mean);
    std::copy(
        compressed_mean.begin(),
        compressed_mean.end(),
        message.compressed_mean.begin());
    return message;
}

estimation::TrajectoryIntentPacket fromRosMessage(
    const collision_avoidance::msg::TrajectoryIntent & message)
{
    estimation::TrajectoryIntentPacket packet{};
    packet.source_timestamp_us = message.source_timestamp_us;
    packet.selection_epoch = message.selection_epoch;
    packet.candidate_id = message.candidate_id;
    packet.candidate_set_size = message.candidate_set_size;
    packet.candidate_set_kind = static_cast<estimation::CandidateSetKind>(
        message.candidate_set_kind);
    std::copy(
        message.candidate_input.begin(),
        message.candidate_input.end(),
        packet.candidate_input.begin());
    packet.candidate_input_revision = message.candidate_input_revision;
    std::copy(
        message.initial_state.begin(),
        message.initial_state.end(),
        packet.initial_state.begin());
    std::copy(
        message.initial_covariance.begin(),
        message.initial_covariance.end(),
        packet.initial_covariance.begin());
    std::array<float, 18> compressed_mean{};
    std::copy(
        message.compressed_mean.begin(),
        message.compressed_mean.end(),
        compressed_mean.begin());
    packet.compressed_mean = decodeCompressedMean(compressed_mean);
    return packet;
}

TrajectoryIntentPublisher::TrajectoryIntentPublisher(
    rclcpp::Node & node,
    const std::string & topic_name,
    std::size_t history_depth)
: m_publisher(node.create_publisher<collision_avoidance::msg::TrajectoryIntent>(
      topic_name, intentQos(history_depth)))
{
}

void TrajectoryIntentPublisher::publish(
    const estimation::TrajectoryIntentPacket & packet)
{
    m_publisher->publish(toRosMessage(packet));
}

TrajectoryIntentSubscription::TrajectoryIntentSubscription(
    rclcpp::Node & node,
    const std::string & topic_name,
    PacketCallback callback,
    std::size_t history_depth)
{
    m_subscription =
        node.create_subscription<collision_avoidance::msg::TrajectoryIntent>(
            topic_name,
            intentQos(history_depth),
            [callback = std::move(callback)](
                collision_avoidance::msg::TrajectoryIntent::ConstSharedPtr message) {
                if (callback) {
                    callback(fromRosMessage(*message));
                }
            });
}

}  // namespace collision_avoidance::communication
