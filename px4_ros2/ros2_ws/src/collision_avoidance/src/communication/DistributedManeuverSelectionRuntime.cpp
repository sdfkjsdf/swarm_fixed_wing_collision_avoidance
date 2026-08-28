#include <collision_avoidance/communication/DistributedManeuverSelectionRuntime.hpp>

#include <algorithm>
#include <chrono>
#include <utility>

namespace collision_avoidance::communication
{

DistributedManeuverSelectionRuntime::DistributedManeuverSelectionRuntime(
    rclcpp::Node & node,
    int vehicle_id,
    int total_agent_count,
    const selection::ManeuverSelectionWorkerParams & worker_params,
    DecisionCallback decision_callback)
: m_node(node),
  m_vehicle_id(vehicle_id),
  m_decision_callback(std::move(decision_callback)),
  m_worker(worker_params)
{
    if (total_agent_count != 2 || vehicle_id < 0 || vehicle_id >= total_agent_count) {
        RCLCPP_WARN(
            m_node.get_logger(),
            "[maneuver-selection] runtime disabled: first milestone requires "
            "exactly two agents (vehicle=%d agents=%d)",
            vehicle_id,
            total_agent_count);
        return;
    }

    m_remote_vehicle_id = vehicle_id == 0 ? 1 : 0;
    const std::string own_intent_topic =
        "/common/px4_" + std::to_string(m_vehicle_id) + "/trajectory_intent";
    const std::string remote_intent_topic =
        "/common/px4_" + std::to_string(m_remote_vehicle_id)
        + "/trajectory_intent";
    const std::string belief_topic =
        "/common/px4_" + std::to_string(m_vehicle_id)
        + "/trans_estimator_trajectory_belief";

    m_intent_publisher = std::make_unique<TrajectoryIntentPublisher>(
        m_node, own_intent_topic);
    m_intent_subscription = std::make_unique<TrajectoryIntentSubscription>(
        m_node,
        remote_intent_topic,
        [this](const estimation::TrajectoryIntentPacket & packet) {
            if (!m_worker.pushRemoteIntent(packet)) {
                RCLCPP_WARN_THROTTLE(
                    m_node.get_logger(), *m_node.get_clock(), 1000,
                    "[maneuver-selection] remote intent input queue full");
            }
        });
    m_belief_subscription =
        m_node.create_subscription<px4_msgs::msg::EstimatorTrajectoryBelief>(
            belief_topic,
            rclcpp::SensorDataQoS(),
            [this](
                px4_msgs::msg::EstimatorTrajectoryBelief::ConstSharedPtr message) {
                onBelief(*message);
            });
    m_output_timer = m_node.create_wall_timer(
        std::chrono::milliseconds(10),
        [this]() { drainWorkerOutput(); });

    if (!m_worker.start()) {
        RCLCPP_ERROR(
            m_node.get_logger(),
            "[maneuver-selection] failed to start worker for vehicle %d",
            m_vehicle_id);
        m_output_timer.reset();
        m_belief_subscription.reset();
        m_intent_subscription.reset();
        m_intent_publisher.reset();
        return;
    }

    m_enabled = true;
    RCLCPP_INFO(
        m_node.get_logger(),
        "[maneuver-selection] vehicle=%d remote=%d belief='%s' tx='%s' rx='%s'",
        m_vehicle_id,
        m_remote_vehicle_id,
        belief_topic.c_str(),
        own_intent_topic.c_str(),
        remote_intent_topic.c_str());
}

DistributedManeuverSelectionRuntime::~DistributedManeuverSelectionRuntime()
{
    m_output_timer.reset();
    m_belief_subscription.reset();
    m_intent_subscription.reset();
    m_worker.stop();
    m_intent_publisher.reset();
}

bool DistributedManeuverSelectionRuntime::enabled() const noexcept
{
    return m_enabled;
}

void DistributedManeuverSelectionRuntime::onBelief(
    const px4_msgs::msg::EstimatorTrajectoryBelief & message)
{
    selection::ManeuverSelectionBeliefSnapshot snapshot;
    snapshot.timestamp_us = message.timestamp;
    snapshot.timestamp_sample_us = message.timestamp_sample;
    snapshot.valid = message.valid;
    std::copy(
        message.attitude_q.begin(), message.attitude_q.end(),
        snapshot.belief.attitude_q.begin());
    std::copy(
        message.velocity.begin(), message.velocity.end(),
        snapshot.belief.velocity_ned.begin());
    std::copy(
        message.position.begin(), message.position.end(),
        snapshot.belief.position_ned.begin());

    std::size_t packed_index = 0;
    for (std::size_t row = 0;
         row < estimation::kEstimatorBeliefDimension; ++row) {
        for (std::size_t column = row;
             column < estimation::kEstimatorBeliefDimension; ++column) {
            const double value = message.covariance_upper_triangle[packed_index++];
            snapshot.belief.covariance[
                row * estimation::kEstimatorBeliefDimension + column] = value;
            snapshot.belief.covariance[
                column * estimation::kEstimatorBeliefDimension + row] = value;
        }
    }

    if (!m_worker.pushOwnshipBelief(snapshot)) {
        RCLCPP_WARN_THROTTLE(
            m_node.get_logger(), *m_node.get_clock(), 1000,
            "[maneuver-selection] belief input queue full");
    }
}

void DistributedManeuverSelectionRuntime::drainWorkerOutput()
{
    while (const auto output = m_worker.tryPopOutput()) {
        if (m_intent_publisher) {
            for (std::size_t index = 0;
                 index < output->intent_packet_count; ++index) {
                m_intent_publisher->publish(output->intent_packets[index]);
            }
        }
        if (output->has_decision) {
            if (m_decision_callback) {
                m_decision_callback(output->decision);
            }
            RCLCPP_INFO(
                m_node.get_logger(),
                "[maneuver-selection] vehicle=%d epoch=%llu remote_epoch=%llu "
                "qualified=%d own=%u threat=%u AD=%.3f activate=%d",
                m_vehicle_id,
                static_cast<unsigned long long>(
                    output->decision.local_selection_epoch),
                static_cast<unsigned long long>(
                    output->decision.remote_selection_epoch),
                output->decision.coordination_qualified ? 1 : 0,
                static_cast<unsigned>(output->decision.ownship_candidate_id),
                static_cast<unsigned>(output->decision.threat_candidate_id),
                output->decision.ad_m,
                output->decision.activation_requested ? 1 : 0);
        }
    }

    if (m_worker.droppedInputCount() > 0 || m_worker.droppedOutputCount() > 0) {
        RCLCPP_WARN_THROTTLE(
            m_node.get_logger(), *m_node.get_clock(), 1000,
            "[maneuver-selection] queue drops: input=%llu output=%llu",
            static_cast<unsigned long long>(m_worker.droppedInputCount()),
            static_cast<unsigned long long>(m_worker.droppedOutputCount()));
    }
}

}  // namespace collision_avoidance::communication
