#include <collision_avoidance/communication/DistributedManeuverSelectionRuntime.hpp>

#include <algorithm>
#include <chrono>
#include <utility>

namespace collision_avoidance::communication
{
namespace
{

selection::ManeuverSelectionWorkerParams withAircraftIdentity(
    int vehicle_id,
    int total_agent_count,
    const selection::ManeuverSelectionWorkerParams & params)
{
    auto result = params;
    result.vehicle_id = vehicle_id;
    result.total_agent_count = total_agent_count;
    return result;
}

}  // namespace

DistributedManeuverSelectionRuntime::DistributedManeuverSelectionRuntime(
    rclcpp::Node & node,
    int vehicle_id,
    int total_agent_count,
    const selection::ManeuverSelectionWorkerParams & worker_params,
    DecisionCallback decision_callback)
: m_node(node),
  m_vehicle_id(vehicle_id),
  m_total_agent_count(total_agent_count),
  m_decision_callback(std::move(decision_callback)),
  m_worker(withAircraftIdentity(vehicle_id, total_agent_count, worker_params))
{
    // Candidate generation and exchange may warm up before Formation, but the
    // AMAC execution state is armed only by the Formation lifecycle.
    m_worker.setActivationEnabled(false);

    if (total_agent_count < 2
        || total_agent_count
            > static_cast<int>(selection::kMaximumSelectionAircraft)
        || vehicle_id < 0 || vehicle_id >= total_agent_count) {
        RCLCPP_WARN(
            m_node.get_logger(),
            "[maneuver-selection] runtime disabled: expected 2..%zu agents "
            "and a valid vehicle ID (vehicle=%d agents=%d)",
            selection::kMaximumSelectionAircraft,
            vehicle_id,
            total_agent_count);
        return;
    }

    const std::string own_intent_topic =
        "/common/px4_" + std::to_string(m_vehicle_id) + "/trajectory_intent";
    const std::string belief_topic =
        "/common/px4_" + std::to_string(m_vehicle_id)
        + "/trans_estimator_trajectory_belief";
    const std::size_t intent_history_depth =
        worker_params.exhaustive_test_mode ? 16U : 5U;

    m_intent_publisher = std::make_unique<TrajectoryIntentPublisher>(
        m_node, own_intent_topic, intent_history_depth);
    m_decision_publisher = m_node.create_publisher<
        collision_avoidance::msg::ManeuverSelectionDecision>(
        "/common/px4_" + std::to_string(m_vehicle_id)
            + "/maneuver_selection_decision",
        rclcpp::SensorDataQoS());
    m_intent_subscriptions.reserve(
        static_cast<std::size_t>(total_agent_count - 1));
    m_decision_subscriptions.reserve(
        static_cast<std::size_t>(total_agent_count - 1));
    for (int remote_vehicle_id = 0;
         remote_vehicle_id < total_agent_count; ++remote_vehicle_id) {
        if (remote_vehicle_id == m_vehicle_id) {
            continue;
        }
        const std::string remote_intent_topic =
            "/common/px4_" + std::to_string(remote_vehicle_id)
            + "/trajectory_intent";
        m_intent_subscriptions.push_back(
            std::make_unique<TrajectoryIntentSubscription>(
                m_node,
                remote_intent_topic,
                [this, remote_vehicle_id](
                    const estimation::TrajectoryIntentPacket & packet) {
                    if (!m_worker.pushRemoteIntent(
                            remote_vehicle_id, packet)) {
                        RCLCPP_WARN_THROTTLE(
                            m_node.get_logger(), *m_node.get_clock(), 1000,
                            "[maneuver-selection] remote intent input queue full");
                    }
                },
                intent_history_depth));
        const std::string remote_decision_topic =
            "/common/px4_" + std::to_string(remote_vehicle_id)
            + "/maneuver_selection_decision";
        m_decision_subscriptions.push_back(
            m_node.create_subscription<
                collision_avoidance::msg::ManeuverSelectionDecision>(
                remote_decision_topic,
                rclcpp::SensorDataQoS(),
                [this, remote_vehicle_id](
                    collision_avoidance::msg::ManeuverSelectionDecision::
                        ConstSharedPtr message) {
                    selection::ManeuverSelectionPeerDecision decision;
                    decision.vehicle_id = message->vehicle_id;
                    decision.selection_timestamp_us =
                        message->selection_timestamp_us;
                    decision.local_selection_epoch =
                        message->local_selection_epoch;
                    std::copy(
                        message->selected_candidate_ids.begin(),
                        message->selected_candidate_ids.end(),
                        decision.selected_candidate_ids.begin());
                    decision.ownship_candidate_id =
                        message->ownship_candidate_id;
                    decision.proposal_timestamp_us =
                        message->proposal_timestamp_us;
                    decision.proposal_epoch = message->proposal_epoch;
                    std::copy(
                        message->proposed_candidate_ids.begin(),
                        message->proposed_candidate_ids.end(),
                        decision.proposed_candidate_ids.begin());
                    decision.proposal_valid = message->proposal_valid;
                    decision.proposal_consensus_confirmed =
                        message->proposal_consensus_confirmed;
                    decision.coordination_qualified =
                        message->coordination_qualified;
                    decision.activation_requested =
                        message->activation_requested;
                    if (!m_worker.pushRemoteDecision(
                            remote_vehicle_id, decision)) {
                        RCLCPP_WARN_THROTTLE(
                            m_node.get_logger(), *m_node.get_clock(), 1000,
                            "[maneuver-selection] remote decision input queue full");
                    }
                }));
    }
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
        m_intent_subscriptions.clear();
        m_decision_subscriptions.clear();
        m_intent_publisher.reset();
        m_decision_publisher.reset();
        return;
    }

    m_enabled = true;
    RCLCPP_INFO(
        m_node.get_logger(),
        "[maneuver-selection] vehicle=%d peers=%d belief='%s' tx='%s'",
        m_vehicle_id,
        m_total_agent_count - 1,
        belief_topic.c_str(),
        own_intent_topic.c_str());
}

DistributedManeuverSelectionRuntime::~DistributedManeuverSelectionRuntime()
{
    m_output_timer.reset();
    m_belief_subscription.reset();
    m_intent_subscriptions.clear();
    m_decision_subscriptions.clear();
    m_worker.stop();
    m_decision_publisher.reset();
    m_intent_publisher.reset();
}

bool DistributedManeuverSelectionRuntime::enabled() const noexcept
{
    return m_enabled;
}

void DistributedManeuverSelectionRuntime::setActivationEnabled(
    bool enabled) noexcept
{
    m_worker.setActivationEnabled(enabled);
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
            if (m_decision_publisher) {
                collision_avoidance::msg::ManeuverSelectionDecision message;
                const auto & decision = output->decision;
                message.selection_timestamp_us =
                    decision.selection_timestamp_us;
                message.vehicle_id = decision.vehicle_id;
                message.aircraft_count = static_cast<std::uint8_t>(
                    decision.aircraft_count);
                message.local_selection_epoch =
                    decision.local_selection_epoch;
                std::copy(
                    decision.selection_epochs_by_aircraft.begin(),
                    decision.selection_epochs_by_aircraft.end(),
                    message.selection_epochs_by_aircraft.begin());
                message.selected_combination_index =
                    static_cast<std::uint16_t>(
                        decision.selected_combination_index);
                message.evaluated_combination_count =
                    static_cast<std::uint16_t>(
                        decision.evaluated_combination_count);
                std::copy(
                    decision.selected_candidate_ids.begin(),
                    decision.selected_candidate_ids.end(),
                    message.selected_candidate_ids.begin());
                message.ownship_candidate_id = decision.ownship_candidate_id;
                message.proposal_timestamp_us =
                    decision.proposal_timestamp_us;
                message.proposal_epoch = decision.proposal_epoch;
                std::copy(
                    decision.proposed_candidate_ids.begin(),
                    decision.proposed_candidate_ids.end(),
                    message.proposed_candidate_ids.begin());
                message.proposal_valid = decision.proposal_valid;
                message.proposal_consensus_confirmed =
                    decision.proposal_consensus_confirmed;
                message.pmr_m = static_cast<float>(decision.pmr_m);
                message.masd_m = static_cast<float>(decision.masd_m);
                message.ad_m = static_cast<float>(decision.ad_m);
                message.reciprocal_cost_sum = static_cast<float>(
                    decision.reciprocal_cost_sum);
                message.activation_timestamp_us =
                    decision.activation_timestamp_us;
                message.deactivation_reason = static_cast<std::uint8_t>(
                    decision.deactivation_reason);
                message.coordination_qualified =
                    decision.coordination_qualified;
                message.new_best_accepted = decision.new_best_accepted;
                message.previous_best_retained =
                    decision.previous_best_retained;
                message.activation_requested = decision.activation_requested;
                message.activation_just_started =
                    decision.activation_just_started;
                message.activation_just_ended =
                    decision.activation_just_ended;
                m_decision_publisher->publish(message);
            }
            if (m_decision_callback) {
                m_decision_callback(output->decision);
            }
            RCLCPP_INFO(
                m_node.get_logger(),
                "[maneuver-selection] vehicle=%d selected_epoch=%llu "
                "proposal_epoch=%llu remote_epoch=%llu qualified=%d "
                "proposal_confirmed=%d own=%u threat=%u AD=%.3f active=%d "
                "start=%d end=%d reason=%u",
                m_vehicle_id,
                static_cast<unsigned long long>(
                    output->decision.local_selection_epoch),
                static_cast<unsigned long long>(
                    output->decision.proposal_epoch),
                static_cast<unsigned long long>(
                    output->decision.remote_selection_epoch),
                output->decision.coordination_qualified ? 1 : 0,
                output->decision.proposal_consensus_confirmed ? 1 : 0,
                static_cast<unsigned>(output->decision.ownship_candidate_id),
                static_cast<unsigned>(output->decision.threat_candidate_id),
                output->decision.ad_m,
                output->decision.activation_requested ? 1 : 0,
                output->decision.activation_just_started ? 1 : 0,
                output->decision.activation_just_ended ? 1 : 0,
                static_cast<unsigned>(output->decision.deactivation_reason));
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
