#include <collision_avoidance/communication/TrajectoryIntentTransport.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
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

// NaN is allowed for an unused altitude command. Compare the transmitted
// float bits, not numeric equality, when validating a shared source snapshot.
template<typename T>
bool sameBits(const T & first, const T & second) noexcept
{
    return std::memcmp(&first, &second, sizeof(T)) == 0;
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
    std::uint64_t coordination_delay_us,
    std::uint64_t trajectory_refresh_period_us) noexcept
{
    if (trajectory_refresh_period_us == 0) {
        return 1;
    }
    const std::uint64_t refresh_count =
        coordination_delay_us / trajectory_refresh_period_us + 1;
    return static_cast<std::size_t>(refresh_count);
}

rclcpp::QoS trajectoryIntentQos(std::size_t history_depth)
{
    // History counts complete refreshes, not individual candidates. Keep the
    // same coordination-window coverage and reliable/volatile delivery.
    rclcpp::QoS qos(rclcpp::KeepLast(std::max<std::size_t>(history_depth, 1)));
    qos.reliable();
    qos.durability_volatile();
    return qos;
}

bool toRosMessage(const TrajectoryIntentPackets & packets, std::size_t count,
    collision_avoidance::msg::TrajectoryIntentBatch & message)
{
    if (count == 0 || count > packets.size()) return false;
    const auto & common = packets[0];
    if (common.candidate_set_kind != estimation::CandidateSetKind::LegacyRoll
        && common.candidate_set_kind != estimation::CandidateSetKind::V4SafeControl) return false;
    std::array<bool, estimation::kManeuverCandidateCount> seen{};
    for (std::size_t i = 0; i < count; ++i) {
        const auto & packet = packets[i];
        if (packet.candidate_id >= seen.size() || seen[packet.candidate_id]
            || packet.candidate_set_size != count
            || packet.source_timestamp_us != common.source_timestamp_us
            || packet.selection_epoch != common.selection_epoch
            || packet.candidate_set_kind != common.candidate_set_kind
            || !sameBits(packet.initial_covariance, common.initial_covariance)
            || !sameBits(packet.initial_state, common.initial_state)
            || !sameBits(packet.initial_roll_setpoint_rad, common.initial_roll_setpoint_rad)
            || !sameBits(packet.source_execution_input, common.source_execution_input)
            || packet.source_execution_input_available
                != common.source_execution_input_available) return false;
        seen[packet.candidate_id] = true;
    }
    message = collision_avoidance::msg::TrajectoryIntentBatch{};
    message.source_timestamp_us = common.source_timestamp_us;
    message.selection_epoch = common.selection_epoch;
    message.candidate_set_size = static_cast<std::uint8_t>(count);
    message.candidate_set_kind = static_cast<std::uint8_t>(
        common.candidate_set_kind);
    message.initial_covariance = common.initial_covariance;
    message.initial_state = common.initial_state;
    message.initial_roll_setpoint_rad = common.initial_roll_setpoint_rad;
    message.source_execution_input = common.source_execution_input;
    message.source_execution_input_available = common.source_execution_input_available;
    for (std::size_t i = 0; i < count; ++i) {
        const auto & packet = packets[i];
        auto & candidate = message.candidates[i];
        candidate.candidate_id = packet.candidate_id;
        candidate.candidate_input = packet.candidate_input;
        candidate.candidate_input_revision = packet.candidate_input_revision;
        candidate.compressed_mean = encodeCompressedMean(packet.compressed_mean);
    }
    return true;
}

bool fromRosMessage(const collision_avoidance::msg::TrajectoryIntentBatch & message,
    TrajectoryIntentPackets & packets)
{
    const auto count = message.candidate_set_size;
    if (count == 0 || count > packets.size()
        || message.candidate_set_kind > static_cast<std::uint8_t>(
            estimation::CandidateSetKind::V4SafeControl)) return false;
    std::array<bool, estimation::kManeuverCandidateCount> seen{};
    for (std::size_t i = 0; i < count; ++i) {
        const auto id = message.candidates[i].candidate_id;
        if (id >= seen.size() || seen[id]) return false;
        seen[id] = true;
    }
    for (std::size_t i = 0; i < count; ++i) {
        auto & packet = packets[i];
        const auto & candidate = message.candidates[i];
        packet.source_timestamp_us = message.source_timestamp_us;
        packet.selection_epoch = message.selection_epoch;
        packet.candidate_set_size = count;
        packet.candidate_set_kind = static_cast<estimation::CandidateSetKind>(message.candidate_set_kind);
        packet.initial_covariance = message.initial_covariance;
        packet.candidate_id = candidate.candidate_id;
        packet.candidate_input = candidate.candidate_input;
        packet.candidate_input_revision = candidate.candidate_input_revision;
        packet.source_execution_input = message.source_execution_input;
        packet.source_execution_input_available = message.source_execution_input_available;
        packet.initial_roll_setpoint_rad = message.initial_roll_setpoint_rad;
        packet.initial_state = message.initial_state;
        packet.compressed_mean = decodeCompressedMean(candidate.compressed_mean);
    }
    return true;
}

TrajectoryIntentPublisher::TrajectoryIntentPublisher(
    rclcpp::Node & node,
    const std::string & topic_name,
    std::size_t history_depth,
    bool measure_transport)
: m_timing(measure_transport ? std::make_unique<TrajectoryTransportTimingBuffer>() : nullptr),
  m_publisher(node.create_publisher<collision_avoidance::msg::TrajectoryIntentBatch>(
      topic_name, trajectoryIntentQos(history_depth)))
{
}

bool TrajectoryIntentPublisher::publish(const TrajectoryIntentPackets & packets, std::size_t count)
{
    collision_avoidance::msg::TrajectoryIntentBatch message;
    if (!toRosMessage(packets, count, message)) return false;
    const auto sent_ns = m_timing ? wallNowNs() : 0;
    m_publisher->publish(message);
    // Control transport first; no allocation, formatting or I/O while running.
    if (m_timing) {
        // Preserve logical candidate identities for existing latency analysis;
        // all candidates now share ONE publish/callback timestamp and DDS sample.
        for (std::size_t i = 0; i < count; ++i) {
            const auto & packet = packets[i];
            m_timing->append({packet.source_timestamp_us, packet.selection_epoch,
                packet.candidate_input_revision, sent_ns, 0, 0, packet.candidate_id});
        }
    }
    return true;
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
        node.create_subscription<collision_avoidance::msg::TrajectoryIntentBatch>(
            topic_name,
            trajectoryIntentQos(history_depth),
            [this, callback = std::move(callback)](
                collision_avoidance::msg::TrajectoryIntentBatch::ConstSharedPtr message,
                const rclcpp::MessageInfo & info) {
                const auto received_ns = m_timing ? wallNowNs() : 0;
                TrajectoryIntentPackets packets{};
                if (!fromRosMessage(*message, packets)) return;
                if (callback) {
                    for (std::size_t i = 0; i < message->candidate_set_size; ++i) {
                        callback(packets[i]);
                    }
                }
                // The production input queue is serviced before diagnostic storage.
                if (m_timing) {
                    const auto & metadata = info.get_rmw_message_info();
                    for (std::size_t i = 0; i < message->candidate_set_size; ++i) {
                        const auto & candidate = message->candidates[i];
                        m_timing->append({message->source_timestamp_us, message->selection_epoch,
                            candidate.candidate_input_revision, received_ns,
                            metadata.source_timestamp, metadata.received_timestamp,
                            candidate.candidate_id});
                    }
                }
            });
}

void TrajectoryIntentSubscription::writeStoppedTiming(
    std::ostream & out, int vehicle, int peer) const
{
    writeTransportTiming(out, vehicle, peer, "rx", m_timing.get());
}

}  // namespace collision_avoidance::communication
