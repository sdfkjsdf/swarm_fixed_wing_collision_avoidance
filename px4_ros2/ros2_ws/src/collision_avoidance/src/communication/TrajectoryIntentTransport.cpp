#include <collision_avoidance/communication/TrajectoryIntentTransport.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <utility>

namespace collision_avoidance::communication
{
namespace
{

using estimation::TrajectorySample;
using estimation::Vec3;

std::int64_t wallNowNs() noexcept
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

void writeTransportTiming(std::ostream & out, int vehicle, int peer,
    const char * direction, const TrajectoryTransportTimingBuffer * timing)
{
    if (!timing) return;
    out << "[stop-transport-begin],1," << vehicle << ',' << peer << ','
        << direction << ',' << timing->size << ',' << timing->dropped << '\n';
    for (std::size_t i = 0; i < timing->size; ++i) {
        const auto & r = timing->records[i];
        out << "[stop-transport]," << r.source_us << ',' << r.epoch << ','
            << unsigned(r.candidate_id) << ',' << r.input_revision << ','
            << r.wall_ns << ',' << r.dds_source_ns << ',' << r.dds_received_ns << '\n';
    }
    out << "[stop-transport-end]," << vehicle << ',' << peer << ','
        << direction << ',' << timing->size << '\n';
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

std::size_t requiredTrajectoryIntentHistoryDepth(
    std::size_t candidate_count,
    std::uint64_t coordination_delay_us,
    std::uint64_t trajectory_refresh_period_us) noexcept
{
    if (candidate_count == 0) {
        return 1;
    }
    if (trajectory_refresh_period_us == 0) {
        return candidate_count;
    }
    const std::uint64_t refresh_count =
        coordination_delay_us / trajectory_refresh_period_us + 1;
    return candidate_count * static_cast<std::size_t>(refresh_count);
}

rclcpp::QoS trajectoryIntentQos(std::size_t history_depth)
{
    // All candidate messages from one refresh form one logical library.  A
    // missing member must be recovered by DDS rather than silently turning a
    // complete published library into CandidateSetsIncomplete downstream.
    rclcpp::QoS qos(rclcpp::KeepLast(std::max<std::size_t>(history_depth, 1)));
    qos.reliable();
    qos.durability_volatile();
    return qos;
}

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
    message.source_execution_input = packet.source_execution_input;
    message.source_execution_input_available = packet.source_execution_input_available;
    message.nominal_lateral_acceleration_mps2 =
        packet.nominal_lateral_acceleration_mps2;
    message.safe_rejoin_requested = packet.safe_rejoin_requested;
    message.initial_roll_setpoint_rad = packet.initial_roll_setpoint_rad;
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
    packet.source_execution_input = message.source_execution_input;
    packet.source_execution_input_available = message.source_execution_input_available;
    packet.nominal_lateral_acceleration_mps2 =
        message.nominal_lateral_acceleration_mps2;
    packet.safe_rejoin_requested = message.safe_rejoin_requested;
    packet.initial_roll_setpoint_rad = message.initial_roll_setpoint_rad;
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
    std::size_t history_depth,
    bool measure_transport)
: m_timing(measure_transport ? std::make_unique<TrajectoryTransportTimingBuffer>() : nullptr),
  m_publisher(node.create_publisher<collision_avoidance::msg::TrajectoryIntent>(
      topic_name, trajectoryIntentQos(history_depth)))
{
}

void TrajectoryIntentPublisher::publish(
    const estimation::TrajectoryIntentPacket & packet)
{
    const auto message = toRosMessage(packet);
    const auto sent_ns = m_timing ? wallNowNs() : 0;
    m_publisher->publish(message);
    // Control transport first; no allocation, formatting or I/O while running.
    if (m_timing) {
        m_timing->append({packet.source_timestamp_us, packet.selection_epoch,
            packet.candidate_input_revision, sent_ns, 0, 0, packet.candidate_id});
    }
}

void TrajectoryIntentPublisher::writeStoppedTiming(std::ostream & out, int vehicle) const
{
    writeTransportTiming(out, vehicle, -1, "tx", m_timing.get());
}

TrajectoryIntentSubscription::TrajectoryIntentSubscription(
    rclcpp::Node & node,
    const std::string & topic_name,
    PacketCallback callback,
    std::size_t history_depth,
    bool measure_transport)
: m_timing(measure_transport ? std::make_unique<TrajectoryTransportTimingBuffer>() : nullptr)
{
    m_subscription =
        node.create_subscription<collision_avoidance::msg::TrajectoryIntent>(
            topic_name,
            trajectoryIntentQos(history_depth),
            [this, callback = std::move(callback)](
                collision_avoidance::msg::TrajectoryIntent::ConstSharedPtr message,
                const rclcpp::MessageInfo & info) {
                const auto received_ns = m_timing ? wallNowNs() : 0;
                if (callback) {
                    callback(fromRosMessage(*message));
                }
                // The production input queue is serviced before diagnostic storage.
                if (m_timing) {
                    const auto & metadata = info.get_rmw_message_info();
                    m_timing->append({message->source_timestamp_us, message->selection_epoch,
                        message->candidate_input_revision, received_ns,
                        metadata.source_timestamp, metadata.received_timestamp,
                        message->candidate_id});
                }
            });
}

void TrajectoryIntentSubscription::writeStoppedTiming(
    std::ostream & out, int vehicle, int peer) const
{
    writeTransportTiming(out, vehicle, peer, "rx", m_timing.get());
}

}  // namespace collision_avoidance::communication
