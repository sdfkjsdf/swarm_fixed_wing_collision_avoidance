#include <collision_avoidance/communication/DistributedManeuverSelectionRuntime.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
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
    const std::string airspeed_topic =
        "/px4_" + std::to_string(m_vehicle_id)
        + "/fmu/out/airspeed_validated_v1";
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
                    std::copy(
                        message->selected_candidate_input_revisions.begin(),
                        message->selected_candidate_input_revisions.end(),
                        decision.selected_candidate_input_revisions.begin());
                    std::copy(
                        message->selected_candidate_source_timestamps_us.begin(),
                        message->selected_candidate_source_timestamps_us.end(),
                        decision.selected_candidate_source_timestamps_us.begin());
                    decision.selected_v4_cutover =
                        message->selected_v4_cutover;
                    decision.ownship_candidate_id =
                        message->ownship_candidate_id;
                    decision.proposal_timestamp_us =
                        message->proposal_timestamp_us;
                    decision.proposal_epoch = message->proposal_epoch;
                    std::copy(
                        message->proposed_candidate_ids.begin(),
                        message->proposed_candidate_ids.end(),
                        decision.proposed_candidate_ids.begin());
                    std::copy(
                        message->proposed_candidate_input_revisions.begin(),
                        message->proposed_candidate_input_revisions.end(),
                        decision.proposed_candidate_input_revisions.begin());
                    std::copy(
                        message->proposed_candidate_source_timestamps_us.begin(),
                        message->proposed_candidate_source_timestamps_us.end(),
                        decision.proposed_candidate_source_timestamps_us.begin());
                    decision.proposed_v4_cutover =
                        message->proposed_v4_cutover;
                    decision.proposal_valid = message->proposal_valid;
                    decision.proposal_consensus_confirmed =
                        message->proposal_consensus_confirmed;
                    decision.coordination_qualified =
                        message->coordination_qualified;
                    decision.activation_requested =
                        message->activation_requested;
                    decision.command_execution_requested =
                        message->command_execution_requested;
                    decision.v4_horizon_local_gate_active =
                        message->v4_horizon_local_gate_active;
                    decision.v4_control_architecture =
                        static_cast<selection::V4ControlArchitecture>(
                            message->v4_control_architecture);
                    decision.v4_cutover_candidate_ready =
                        message->v4_enabled
                        && !message->v4_shadow_only
                        && message->v4_shadow_evaluated
                        && message->v4_shadow_status
                            == static_cast<std::uint8_t>(
                                selection::V4ShadowEvaluationStatus::
                                    CoreEvaluated)
                        && message->v4_candidate_status
                            == static_cast<std::uint8_t>(
                                selection::
                                    SafeControlCandidateAdapterStatus::Valid)
                        && message->v4_candidate_count > 0
                        && message->v4_candidate_count
                            <= selection::kMaximumSafeControlCandidates;
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
    if (worker_params.v4_safe_control_enabled) {
        m_airspeed_subscription =
            m_node.create_subscription<px4_msgs::msg::AirspeedValidated>(
                airspeed_topic,
                rclcpp::SensorDataQoS(),
                [this](
                    px4_msgs::msg::AirspeedValidated::ConstSharedPtr message) {
                    onAirspeed(*message);
                });
    }
    m_output_timer = m_node.create_wall_timer(
        std::chrono::milliseconds(10),
        [this]() { drainWorkerOutput(); });

    if (!m_worker.start()) {
        RCLCPP_ERROR(
            m_node.get_logger(),
            "[maneuver-selection] failed to start worker for vehicle %d",
            m_vehicle_id);
        m_output_timer.reset();
        m_airspeed_subscription.reset();
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
        "[maneuver-selection] vehicle=%d peers=%d belief='%s' tas='%s' "
        "tx='%s'",
        m_vehicle_id,
        m_total_agent_count - 1,
        belief_topic.c_str(),
        airspeed_topic.c_str(),
        own_intent_topic.c_str());
}

DistributedManeuverSelectionRuntime::~DistributedManeuverSelectionRuntime()
{
    m_output_timer.reset();
    m_airspeed_subscription.reset();
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

bool DistributedManeuverSelectionRuntime::pushNominalSetpoint(
    const selection::ManeuverSelectionNominalSetpointSnapshot & snapshot)
    noexcept
{
    const bool accepted = m_worker.pushNominalSetpoint(snapshot);
    if (!accepted) {
        RCLCPP_WARN_THROTTLE(
            m_node.get_logger(), *m_node.get_clock(), 1000,
            "[maneuver-selection] nominal setpoint input queue full");
    }
    return accepted;
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

void DistributedManeuverSelectionRuntime::onAirspeed(
    const px4_msgs::msg::AirspeedValidated & message)
{
    selection::ManeuverSelectionAirspeedSnapshot snapshot;
    snapshot.timestamp_us = message.timestamp;
    snapshot.true_airspeed_mps = message.true_airspeed_m_s;
    snapshot.px4_airspeed_source = message.airspeed_source;
    snapshot.valid = message.airspeed_source
            != px4_msgs::msg::AirspeedValidated::SOURCE_DISABLED
        && std::isfinite(message.true_airspeed_m_s)
        && message.true_airspeed_m_s > 0.0F;
    if (!m_worker.pushAirspeed(snapshot)) {
        RCLCPP_WARN_THROTTLE(
            m_node.get_logger(), *m_node.get_clock(), 1000,
            "[maneuver-selection] airspeed input queue full");
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
                std::copy(
                    decision.selected_candidate_input_revisions.begin(),
                    decision.selected_candidate_input_revisions.end(),
                    message.selected_candidate_input_revisions.begin());
                std::copy(
                    decision.selected_candidate_source_timestamps_us.begin(),
                    decision.selected_candidate_source_timestamps_us.end(),
                    message.selected_candidate_source_timestamps_us.begin());
                message.selected_v4_cutover =
                    decision.selected_v4_cutover;
                message.ownship_candidate_id = decision.ownship_candidate_id;
                message.proposal_timestamp_us =
                    decision.proposal_timestamp_us;
                message.proposal_epoch = decision.proposal_epoch;
                std::copy(
                    decision.proposed_candidate_ids.begin(),
                    decision.proposed_candidate_ids.end(),
                    message.proposed_candidate_ids.begin());
                std::copy(
                    decision.proposed_candidate_input_revisions.begin(),
                    decision.proposed_candidate_input_revisions.end(),
                    message.proposed_candidate_input_revisions.begin());
                std::copy(
                    decision.proposed_candidate_source_timestamps_us.begin(),
                    decision.proposed_candidate_source_timestamps_us.end(),
                    message.proposed_candidate_source_timestamps_us.begin());
                message.proposed_v4_cutover =
                    decision.proposed_v4_cutover;
                message.proposal_valid = decision.proposal_valid;
                message.proposal_consensus_confirmed =
                    decision.proposal_consensus_confirmed;
                message.switch_superiority_evaluated =
                    decision.switch_superiority_evaluated;
                message.switch_clearly_superior =
                    decision.switch_clearly_superior;
                message.switch_current_cost = static_cast<float>(
                    decision.switch_current_cost);
                message.switch_proposed_cost = static_cast<float>(
                    decision.switch_proposed_cost);
                message.switch_current_minimum_ad_m = static_cast<float>(
                    decision.switch_current_minimum_ad_m);
                message.switch_proposed_minimum_ad_m = static_cast<float>(
                    decision.switch_proposed_minimum_ad_m);
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
                message.command_execution_requested =
                    decision.command_execution_requested;
                message.v4_horizon_gate_evaluated =
                    decision.v4_horizon_gate_evaluated;
                message.v4_horizon_gate_valid =
                    decision.v4_horizon_gate_valid;
                message.v4_horizon_local_gate_active =
                    decision.v4_horizon_local_gate_active;
                message.v4_horizon_gate_active =
                    decision.v4_horizon_gate_active;
                message.v4_horizon_h_worst_m = static_cast<float>(
                    decision.v4_horizon_h_worst_m);
                message.v4_horizon_trigger_m = static_cast<float>(
                    decision.v4_horizon_trigger_m);
                message.v4_horizon_worst_time_offset_s = static_cast<float>(
                    decision.v4_horizon_worst_time_offset_s);
                message.v4_horizon_worst_first_vehicle_id =
                    decision.v4_horizon_worst_first_vehicle_id;
                message.v4_horizon_worst_second_vehicle_id =
                    decision.v4_horizon_worst_second_vehicle_id;
                message.activation_just_started =
                    decision.activation_just_started;
                message.activation_just_ended =
                    decision.activation_just_ended;
                message.formation_evaluated = decision.formation_evaluated;
                message.formation_inhibit = decision.formation_inhibit;
                message.formation_allow_new_activation =
                    decision.formation_allow_new_activation;
                message.formation_inhibited_threat_mask =
                    decision.formation_inhibited_threat_mask;
                message.v4_enabled = decision.v4_enabled;
                message.v4_shadow_only = decision.v4_shadow_only;
                message.v4_control_architecture = static_cast<std::uint8_t>(
                    decision.v4_control_architecture);
                message.v4_shadow_evaluated =
                    decision.v4_shadow_evaluated;
                message.v4_shadow_status = static_cast<std::uint8_t>(
                    decision.v4_shadow_status);
                message.v4_airspeed_snapshot_status =
                    static_cast<std::uint8_t>(
                        decision.v4_airspeed_snapshot_status);
                message.v4_airspeed_source = static_cast<std::uint8_t>(
                    decision.v4_airspeed_source);
                message.v4_px4_airspeed_source =
                    decision.v4_px4_airspeed_source;
                message.v4_airspeed_timestamp_us =
                    decision.v4_airspeed_timestamp_us;
                message.v4_airspeed_age_us = decision.v4_airspeed_age_us;
                message.v4_nominal_snapshot_status =
                    static_cast<std::uint8_t>(
                        decision.v4_nominal_snapshot_status);
                message.v4_nominal_available =
                    decision.v4_nominal_available;
                message.v4_nominal_timestamp_us =
                    decision.v4_nominal_timestamp_us;
                message.v4_nominal_age_us = decision.v4_nominal_age_us;
                message.v4_core_status = static_cast<std::uint8_t>(
                    decision.v4_safe_control.status);
                message.v4_longitudinal_source = static_cast<std::uint8_t>(
                    decision.v4_safe_control.longitudinal_source);
                message.v4_effective_max_heading_rate_radps =
                    static_cast<float>(decision.v4_safe_control
                        .effective_max_heading_rate_radps);
                message.v4_left_feasible =
                    decision.v4_safe_control.left_safe.feasible;
                message.v4_left_lower_radps = static_cast<float>(
                    decision.v4_safe_control.left_safe.lower_radps);
                message.v4_left_upper_radps = static_cast<float>(
                    decision.v4_safe_control.left_safe.upper_radps);
                message.v4_right_feasible =
                    decision.v4_safe_control.right_safe.feasible;
                message.v4_right_lower_radps = static_cast<float>(
                    decision.v4_safe_control.right_safe.lower_radps);
                message.v4_right_upper_radps = static_cast<float>(
                    decision.v4_safe_control.right_safe.upper_radps);
                message.v4_first_infeasible_vehicle_id =
                    decision.v4_safe_control.first_infeasible_vehicle_id;
                message.v4_first_infeasible_direction =
                    static_cast<std::uint8_t>(decision.v4_safe_control
                        .first_infeasible_direction);
                message.v4_first_infeasible_residual_mps = 0.0F;
                for (std::size_t index = 0;
                     index < decision.v4_safe_control.diagnostic_count;
                     ++index) {
                    const auto & diagnostic =
                        decision.v4_safe_control.diagnostics[index];
                    if (!diagnostic.constraint_feasible
                        && diagnostic.vehicle_id
                            == decision.v4_safe_control
                                .first_infeasible_vehicle_id
                        && diagnostic.direction
                            == decision.v4_safe_control
                                .first_infeasible_direction) {
                        message.v4_first_infeasible_residual_mps =
                            static_cast<float>(
                                diagnostic.constraint_shortfall_mps);
                        break;
                    }
                }
                message.mode_b_threat_status = static_cast<std::uint8_t>(
                    decision.mode_b_threat_status);
                message.mode_b_invalid_threat_vehicle_id =
                    decision.mode_b_invalid_threat_vehicle_id;
                message.mode_b_interpolation_status =
                    static_cast<std::uint8_t>(
                        decision.mode_b_interpolation_status);
                message.mode_b_branch_classification =
                    static_cast<std::uint8_t>(
                        decision.mode_b_branch_classification);
                message.mode_b_left_certified =
                    decision.mode_b_left_certified;
                message.mode_b_right_certified =
                    decision.mode_b_right_certified;
                message.mode_b_left_minimum_path_margin_m =
                    static_cast<float>(
                        decision.mode_b_left_minimum_path_margin_m);
                message.mode_b_right_minimum_path_margin_m =
                    static_cast<float>(
                        decision.mode_b_right_minimum_path_margin_m);
                message.mode_b_left_terminal_turn_margin_m =
                    static_cast<float>(
                        decision.mode_b_left_terminal_turn_margin_m);
                message.mode_b_right_terminal_turn_margin_m =
                    static_cast<float>(
                        decision.mode_b_right_terminal_turn_margin_m);
                message.mode_b_left_interpolation_status =
                    static_cast<std::uint8_t>(
                        decision.mode_b_left_interpolation_status);
                message.mode_b_right_interpolation_status =
                    static_cast<std::uint8_t>(
                        decision.mode_b_right_interpolation_status);
                message.mode_b_left_mu_star = static_cast<float>(
                    decision.mode_b_left_mu_star);
                message.mode_b_right_mu_star = static_cast<float>(
                    decision.mode_b_right_mu_star);
                message.mode_b_left_safe_rate_radps = static_cast<float>(
                    decision.mode_b_left_safe_rate_radps);
                message.mode_b_right_safe_rate_radps = static_cast<float>(
                    decision.mode_b_right_safe_rate_radps);
                message.v4_candidate_status = static_cast<std::uint8_t>(
                    decision.v4_candidates.status);
                message.v4_candidate_count = static_cast<std::uint8_t>(
                    decision.v4_candidates.candidate_count);
                for (std::size_t index = 0;
                     index < decision.v4_candidates.candidate_count
                        && index < decision.v4_candidates.candidates.size();
                     ++index) {
                    message.v4_candidate_roles[index] =
                        static_cast<std::uint8_t>(
                            decision.v4_candidates.candidates[index].role);
                    message.v4_candidate_rates_radps[index] =
                        static_cast<float>(decision.v4_candidates
                            .candidates[index].heading_rate_v4_radps);
                }
                m_decision_publisher->publish(message);
            }
            if (m_decision_callback) {
                m_decision_callback(output->decision);
            }
            RCLCPP_INFO(
                m_node.get_logger(),
                "[maneuver-selection] vehicle=%d selected_epoch=%llu "
                "proposal_epoch=%llu remote_epoch=%llu qualified=%d "
                "proposal_confirmed=%d switch_eval=%d superior=%d "
                "accepted=%d own=%u threat=%u AD=%.3f active=%d "
                "execute=%d h_local=%d h_gate=%d h_worst=%.3f "
                "formation_eval=%d formation_inhibit=%d "
                "formation_allow=%d formation_mask=0x%08x "
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
                output->decision.switch_superiority_evaluated ? 1 : 0,
                output->decision.switch_clearly_superior ? 1 : 0,
                output->decision.new_best_accepted ? 1 : 0,
                static_cast<unsigned>(output->decision.ownship_candidate_id),
                static_cast<unsigned>(output->decision.threat_candidate_id),
                output->decision.ad_m,
                output->decision.activation_requested ? 1 : 0,
                output->decision.command_execution_requested ? 1 : 0,
                output->decision.v4_horizon_local_gate_active ? 1 : 0,
                output->decision.v4_horizon_gate_active ? 1 : 0,
                output->decision.v4_horizon_h_worst_m,
                output->decision.formation_evaluated ? 1 : 0,
                output->decision.formation_inhibit ? 1 : 0,
                output->decision.formation_allow_new_activation ? 1 : 0,
                output->decision.formation_inhibited_threat_mask,
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
