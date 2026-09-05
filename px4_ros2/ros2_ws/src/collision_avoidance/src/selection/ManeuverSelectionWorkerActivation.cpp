#include "ManeuverSelectionWorkerInternal.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace collision_avoidance::selection
{
using namespace worker_detail;

std::uint32_t ManeuverSelectionWorker::selectedComponentMemberMask(
    const std::size_t aircraft_index) const noexcept
{
    if (!m_selected_component_graph
        || aircraft_index >= static_cast<std::size_t>(m_params.total_agent_count)
        || m_selected_component_count == 0) {
        return 0U;
    }

    const std::uint8_t component_id =
        m_selected_component_ids[aircraft_index];
    if (component_id >= m_selected_component_count) {
        return 0U;
    }

    std::uint32_t member_mask = 0U;
    for (int aircraft = 0; aircraft < m_params.total_agent_count; ++aircraft) {
        const std::size_t index = static_cast<std::size_t>(aircraft);
        if (m_selected_component_ids[index] == component_id) {
            member_mask |= std::uint32_t{1} << index;
        }
    }
    return member_mask;
}

bool ManeuverSelectionWorker::selectedComponentActivationRequested(
    const std::uint32_t ownship_component_mask) const noexcept
{
    if (!m_selected_component_graph || ownship_component_mask == 0U) {
        return false;
    }

    for (int aircraft = 0; aircraft < m_params.total_agent_count; ++aircraft) {
        if (aircraft == m_params.vehicle_id) {
            continue;
        }
        const std::size_t aircraft_index = static_cast<std::size_t>(aircraft);
        const std::uint32_t aircraft_bit = std::uint32_t{1} << aircraft_index;
        if ((ownship_component_mask & aircraft_bit) == 0U) {
            continue;
        }
        const RemoteDecisionCache & peer =
            m_remote_decision_caches[aircraft_index];
        if (peer.valid && peer.decision.coordination_qualified
            && (ownship_component_mask & aircraft_bit) != 0U
            && peer.decision.local_selection_epoch
                == m_latest_selection_decision.local_selection_epoch
            && peer.decision.selected_candidate_valid_mask
                == m_selected_candidate_valid_mask
            && peer.decision.selected_candidate_ids
                == m_selected_candidate_ids
            && peer.activation_start_pending) {
            return true;
        }
    }
    return false;
}

bool ManeuverSelectionWorker::buildActivationSample(
    std::uint64_t now_us,
    ManeuverActivationSample & sample,
    ManeuverSelectionDecision & decision)
{
    sample = ManeuverActivationSample{};
    sample.timestamp_us = now_us;
    sample.activation_criteria_m.fill(
        std::numeric_limits<double>::quiet_NaN());
    for (auto & relative_position : sample.relative_positions_ned_m) {
        relative_position.fill(std::numeric_limits<double>::quiet_NaN());
    }
    for (auto & relative_velocity : sample.relative_velocities_ned_mps) {
        relative_velocity.fill(std::numeric_limits<double>::quiet_NaN());
    }
    if (!m_has_selected_combination || !m_ownship_candidates_complete
        || m_ownship_candidate_count == 0
        || m_selected_candidate_valid_mask
            != candidateMaskForAircraftCount(static_cast<std::size_t>(
                m_params.total_agent_count))
        || (m_selected_v4_cutover
            != (m_ownship_candidate_set_kind
                == estimation::CandidateSetKind::V4SafeControl))) {
        return false;
    }

    const auto find_candidate = [](
                                    const ExhaustiveCandidateIntentSet & candidates,
                                    std::size_t candidate_count,
                                    std::uint8_t candidate_id,
                                    std::uint64_t candidate_input_revision)
        -> const estimation::ReceivedTrajectoryIntent * {
        for (std::size_t index = 0;
             index < candidate_count; ++index) {
            if (candidates[index].candidate_id == candidate_id
                && candidates[index].candidate_input_revision
                    == candidate_input_revision) {
                return &candidates[index];
            }
        }
        return nullptr;
    };
    const std::size_t ownship_index = static_cast<std::size_t>(
        m_params.vehicle_id);
    if (!m_has_latest_state || m_latest_state_timestamp_us != now_us) {
        return false;
    }
    const ManeuverActivationStatus activation =
        m_activation_controller.status();
    const bool monitor_actual_execution = activation.active;
    const std::uint8_t ownship_candidate_id = monitor_actual_execution
        ? activation.latched_candidate_id
        : m_selected_candidate_ids[ownship_index];
    const std::uint64_t ownship_candidate_input_revision =
        monitor_actual_execution
        ? activation.latched_candidate_input_revision
        : m_selected_candidate_input_revisions[ownship_index];
    estimation::ReceivedTrajectoryIntent active_ownship_intent;
    const estimation::ReceivedTrajectoryIntent * ownship_intent = nullptr;
    if (monitor_actual_execution) {
        estimation::TrajectoryIntentPacket packet;
        if (!m_sender.buildForCandidateInput(
                now_us,
                ownship_candidate_id,
                activation.latched_input,
                m_latest_state,
                m_latest_covariance,
                packet,
                m_selection_epoch)
            || !m_receiver.receive(packet, active_ownship_intent)) {
            return false;
        }
        ownship_intent = &active_ownship_intent;
    } else {
        ownship_intent = find_candidate(
            m_ownship_candidates,
            m_ownship_candidate_count,
            ownship_candidate_id,
            ownship_candidate_input_revision);
    }
    if (ownship_intent == nullptr) {
        return false;
    }

    // Before activation, AD is evaluated on the coordinated best tuple. Once
    // active, termination is evaluated on the commands actually being flown:
    // this aircraft's latch, each active peer's advertised latch, and the
    // Formation intent of every inactive peer. CPA always starts from the
    // measured ownship flight vector.
    const double horizontal_speed_squared_m2ps2 =
        m_latest_state.V * m_latest_state.V
        - m_latest_state.h_dot * m_latest_state.h_dot;
    if (!std::isfinite(horizontal_speed_squared_m2ps2)) {
        return false;
    }
    IntentKinematics ownship_kinematics;
    const double horizontal_speed_mps = std::sqrt(std::max(
        0.0, horizontal_speed_squared_m2ps2));
    ownship_kinematics.position_ned = {
        m_latest_state.p_n, m_latest_state.p_e, -m_latest_state.h};
    ownship_kinematics.velocity_ned = {
        horizontal_speed_mps * std::cos(m_latest_state.psi),
        horizontal_speed_mps * std::sin(m_latest_state.psi),
        -m_latest_state.h_dot};

    sample.selected_candidate_id = ownship_candidate_id;
    sample.selected_candidate_input_revision =
        ownship_candidate_input_revision;
    sample.selected_input = ownship_intent->candidate_input;
    double minimum_ad = std::numeric_limits<double>::infinity();
    double reciprocal_cost_sum = 0.0;
    bool reciprocal_cost_defined = true;
    std::size_t evaluated_threat_count = 0;
    MultiAircraftCandidateIntentSets nominal_sets{};
    std::array<std::size_t, kMaximumSelectionAircraft> nominal_counts{};
    bool nominal_intents_built = false;
    for (int remote_id = 0;
         remote_id < m_params.total_agent_count; ++remote_id) {
        if (remote_id == m_params.vehicle_id) {
            continue;
        }
        const std::size_t remote_index = static_cast<std::size_t>(remote_id);
        std::uint8_t remote_candidate_id =
            m_selected_candidate_ids[remote_index];
        std::uint64_t remote_candidate_input_revision =
            m_selected_candidate_input_revisions[remote_index];
        const RemoteCandidateCache * cache = &m_remote_caches[remote_index];
        const estimation::ReceivedTrajectoryIntent * remote_intent = nullptr;
        bool remote_avoidance_active = false;
        if (monitor_actual_execution) {
            const RemoteDecisionCache & peer =
                m_remote_decision_caches[remote_index];
            remote_avoidance_active = peer.valid
                && peer.decision.coordination_qualified
                && peer.decision.activation_requested
                && peer.decision.ownship_candidate_valid;
            if (remote_avoidance_active) {
                remote_candidate_id = peer.decision.ownship_candidate_id;
                remote_candidate_input_revision =
                    peer.decision.selected_candidate_input_revisions[
                        remote_index];
            } else {
                if (!nominal_intents_built) {
                    if (!buildNominalIntentSet(
                            now_us, nominal_sets, nominal_counts)) {
                        return false;
                    }
                    nominal_intents_built = true;
                }
                if (nominal_counts[remote_index] != 1U) {
                    return false;
                }
                remote_candidate_id = kRollZeroId;
                remote_candidate_input_revision =
                    nominal_sets[remote_index][0].candidate_input_revision;
                remote_intent = &nominal_sets[remote_index][0];
            }
        }
        if (remote_intent == nullptr
            && cache->count > 0 && cache->count == cache->expected_count
            && ((m_selected_v4_cutover
                    && cache->candidate_set_kind
                        == estimation::CandidateSetKind::V4SafeControl)
                || (!m_selected_v4_cutover
                    && cache->candidate_set_kind
                        == estimation::CandidateSetKind::LegacyRoll))) {
            remote_intent = find_candidate(
                cache->candidates,
                cache->count,
                remote_candidate_id,
                remote_candidate_input_revision);
        }
        if (remote_intent == nullptr) {
            cache = &m_remote_previous_caches[remote_index];
            if (cache->count > 0 && cache->count == cache->expected_count
                && ((m_selected_v4_cutover
                        && cache->candidate_set_kind
                            == estimation::CandidateSetKind::V4SafeControl)
                    || (!m_selected_v4_cutover
                        && cache->candidate_set_kind
                            == estimation::CandidateSetKind::LegacyRoll))) {
                remote_intent = find_candidate(
                    cache->candidates,
                    cache->count,
                    remote_candidate_id,
                    remote_candidate_input_revision);
            }
        }
        if (remote_intent == nullptr) {
            const RemoteSelectedIntentCache & retained =
                m_remote_selected_caches[remote_index];
            if (retained.valid
                && retained.intent.candidate_id == remote_candidate_id
                && retained.intent.candidate_input_revision
                    == remote_candidate_input_revision
                && ((m_selected_v4_cutover
                        && retained.intent.candidate_set_kind
                            == estimation::CandidateSetKind::V4SafeControl)
                    || (!m_selected_v4_cutover
                        && retained.intent.candidate_set_kind
                            == estimation::CandidateSetKind::LegacyRoll))) {
                remote_intent = &retained.intent;
            }
        }
        if (remote_intent == nullptr) {
            return false;
        }

        CombinationEvaluation pair;
        if (!m_pair_evaluator.evaluatePair(
                now_us, *ownship_intent, *remote_intent, pair)) {
            return false;
        }
        ++evaluated_threat_count;
        if (m_params.masd_diagnostics_enabled) {
            ManeuverBudgetTrace trace;
            trace.event = 3;
            trace.epoch = m_latest_selection_decision.local_selection_epoch;
            trace.evaluation_timestamp_us = now_us;
            trace.peer_id = remote_id;
            trace.candidate_id = ownship_candidate_id;
            trace.peer_candidate_id = remote_candidate_id;
            trace.input_revision = ownship_intent->candidate_input_revision;
            trace.peer_input_revision = remote_intent->candidate_input_revision;
            trace.source_timestamp_us = ownship_intent->source_timestamp_us;
            trace.peer_source_timestamp_us = remote_intent->source_timestamp_us;
            trace.active = monitor_actual_execution;
            trace.pmr_m = pair.pmr_m;
            trace.masd_m = pair.masd_m;
            trace.u95_m = pair.uncertainty_margin_95_m;
            trace.ad_m = pair.ad_m;
            trace.pmr_horizon_s = pair.pmr_time_offset_s;
            recordBudgetTrace(trace);
        }
        if (pair.ad_m < minimum_ad) {
            minimum_ad = pair.ad_m;
            decision.pmr_m = pair.pmr_m;
            decision.masd_m = pair.masd_m;
            decision.communication_delay_margin_m =
                pair.communication_delay_margin_m;
            decision.threat_candidate_id = remote_candidate_id;
        }
        if (pair.ad_m < 0.0) {
            sample.unsafe_threat_mask |= std::uint32_t{1} << remote_index;
        }
        // The public alternate-termination study does not publish the exact
        // threshold implementation. This project maps its "original
        // activation criteria" to the pair MASD used by AD = PMR - MASD and
        // freezes that value when the pair first becomes unsafe.
        sample.activation_criteria_m[remote_index] = pair.masd_m;
        if (pair.reciprocal_cost_defined) {
            reciprocal_cost_sum += pair.reciprocal_cost;
        } else {
            reciprocal_cost_defined = false;
        }

        IntentKinematics remote_kinematics;
        // A cooperating peer has no local estimator state in this worker. Its
        // timestamped intent is therefore propagated to the current ownship
        // time before forming the peer's current flight vector.
        if (interpolateIntentKinematics(
                *remote_intent, now_us, remote_kinematics)
                != IntentKinematicsStatus::Valid) {
            return false;
        }
        for (std::size_t axis = 0;
             axis < ownship_kinematics.position_ned.size(); ++axis) {
            sample.relative_positions_ned_m[remote_index][axis] =
                remote_kinematics.position_ned[axis]
                - ownship_kinematics.position_ned[axis];
            sample.relative_velocities_ned_mps[remote_index][axis] =
                remote_kinematics.velocity_ned[axis]
                - ownship_kinematics.velocity_ned[axis];
        }
    }

    if (evaluated_threat_count
        != static_cast<std::size_t>(m_params.total_agent_count - 1)
        || !std::isfinite(minimum_ad)) {
        return false;
    }
    sample.minimum_ad_m = minimum_ad;
    sample.valid = true;
    decision.ad_m = minimum_ad;
    decision.reciprocal_cost_sum = reciprocal_cost_defined
        ? reciprocal_cost_sum
        : std::numeric_limits<double>::quiet_NaN();
    return true;
}

bool ManeuverSelectionWorker::buildNominalIntentSet(
    std::uint64_t now_us,
    MultiAircraftCandidateIntentSets & candidate_sets,
    std::array<std::size_t, kMaximumSelectionAircraft> & candidate_counts)
{
    if (!m_has_latest_state || !m_has_latest_nominal) {
        return false;
    }

    candidate_sets = MultiAircraftCandidateIntentSets{};
    candidate_counts.fill(0U);
    estimation::PredictInput ownship_input{};
    if (!nominalPredictInput(
            m_latest_nominal,
            now_us,
            m_params.v4_maximum_nominal_age_us,
            ownship_input)) {
        return false;
    }

    const auto build_intent = [this](
                                  std::uint64_t source_timestamp_us,
                                  const estimation::PredictInput & input,
                                  const estimation::PredictState & state,
                                  const estimation::PredictStateCovariance
                                      & covariance,
                                  estimation::ReceivedTrajectoryIntent
                                      & received) {
        estimation::TrajectoryIntentPacket packet;
        if (!m_sender.buildForCandidateInput(
                source_timestamp_us,
                kRollZeroId,
                input,
                state,
                covariance,
                packet,
                m_selection_epoch)) {
            return false;
        }
        packet.candidate_set_size = 1;
        packet.candidate_set_kind =
            estimation::CandidateSetKind::LegacyRoll;
        return m_receiver.receive(packet, received);
    };

    const std::size_t ownship_index = static_cast<std::size_t>(
        m_params.vehicle_id);
    if (!build_intent(
            m_latest_state_timestamp_us,
            ownship_input,
            m_latest_state,
            m_latest_covariance,
            candidate_sets[ownship_index][0])) {
        return false;
    }
    candidate_counts[ownship_index] = 1;

    for (int aircraft = 0;
         aircraft < m_params.total_agent_count; ++aircraft) {
        if (aircraft == m_params.vehicle_id) {
            continue;
        }
        const std::size_t aircraft_index = static_cast<std::size_t>(aircraft);
        const RemoteDecisionCache & decision_cache =
            m_remote_decision_caches[aircraft_index];
        if (!decision_cache.valid) {
            return false;
        }
        const ManeuverSelectionPeerDecision & peer = decision_cache.decision;
        ManeuverSelectionNominalSetpointSnapshot peer_nominal;
        peer_nominal.timestamp_us = peer.nominal_setpoint_timestamp_us;
        peer_nominal.ground_speed_command_mps =
            peer.nominal_ground_speed_command_mps;
        peer_nominal.altitude_command_m = peer.nominal_altitude_command_m;
        peer_nominal.lateral_acceleration_px4_mps2 =
            peer.nominal_lateral_acceleration_mps2;
        peer_nominal.valid = peer.nominal_setpoint_available;
        estimation::PredictInput peer_input{};
        if (!nominalPredictInput(
                peer_nominal,
                now_us,
                m_params.v4_maximum_nominal_age_us,
                peer_input)) {
            return false;
        }

        const estimation::ReceivedTrajectoryIntent * anchor = nullptr;
        const auto consider_cache = [&anchor](const RemoteCandidateCache & cache) {
            if (cache.count == 0 || cache.count != cache.expected_count) {
                return;
            }
            for (std::size_t index = 0; index < cache.count; ++index) {
                const auto & candidate = cache.candidates[index];
                if (anchor == nullptr
                    || candidate.source_timestamp_us
                        > anchor->source_timestamp_us) {
                    anchor = &candidate;
                }
            }
        };
        consider_cache(m_remote_caches[aircraft_index]);
        consider_cache(m_remote_previous_caches[aircraft_index]);
        if (anchor == nullptr
            || !build_intent(
                anchor->source_timestamp_us,
                peer_input,
                anchor->cone[0].mean,
                anchor->cone[0].state_covariance,
                candidate_sets[aircraft_index][0])) {
            return false;
        }
        candidate_counts[aircraft_index] = 1;
    }

    return true;
}

bool ManeuverSelectionWorker::evaluateNominalPostRelease(
    std::uint64_t now_us,
    JointCombinationEvaluation & evaluation)
{
    MultiAircraftCandidateIntentSets candidate_sets{};
    std::array<std::size_t, kMaximumSelectionAircraft> candidate_counts{};
    if (!buildNominalIntentSet(now_us, candidate_sets, candidate_counts)) {
        return false;
    }

    JointManeuverEvaluation nominal_evaluation;
    if (!m_joint_evaluator.evaluate(
            now_us,
            candidate_sets,
            candidate_counts,
            static_cast<std::size_t>(m_params.total_agent_count),
            nominal_evaluation)
        || !nominal_evaluation.has_best) {
        return false;
    }
    evaluation = nominal_evaluation.combinations[
        nominal_evaluation.best_combination_index];
    return evaluation.valid;
}

bool ManeuverSelectionWorker::buildManeuverRejoinObjective(
    std::uint64_t now_us,
    ManeuverRejoinObjective & objective) const noexcept
{
    ManeuverRejoinObjective candidate{};
    estimation::PredictInput input{};
    if (!m_has_latest_nominal
        || !nominalPredictInput(
            m_latest_nominal,
            now_us,
            m_params.v4_maximum_nominal_age_us,
            input)) {
        return false;
    }
    candidate.nominal_lateral_acceleration_mps2[
        static_cast<std::size_t>(m_params.vehicle_id)] = input.a_lat_cmd;

    for (int aircraft = 0;
         aircraft < m_params.total_agent_count; ++aircraft) {
        if (aircraft == m_params.vehicle_id) {
            continue;
        }
        const RemoteDecisionCache & cache = m_remote_decision_caches[
            static_cast<std::size_t>(aircraft)];
        if (!cache.valid) {
            return false;
        }
        ManeuverSelectionNominalSetpointSnapshot peer_nominal;
        peer_nominal.timestamp_us =
            cache.decision.nominal_setpoint_timestamp_us;
        peer_nominal.ground_speed_command_mps =
            cache.decision.nominal_ground_speed_command_mps;
        peer_nominal.altitude_command_m =
            cache.decision.nominal_altitude_command_m;
        peer_nominal.lateral_acceleration_px4_mps2 =
            cache.decision.nominal_lateral_acceleration_mps2;
        peer_nominal.valid = cache.decision.nominal_setpoint_available;
        if (!nominalPredictInput(
                peer_nominal,
                now_us,
                m_params.v4_maximum_nominal_age_us,
                input)) {
            return false;
        }
        candidate.nominal_lateral_acceleration_mps2[
            static_cast<std::size_t>(aircraft)] = input.a_lat_cmd;
    }
    candidate.enabled = true;
    objective = candidate;
    return true;
}

bool ManeuverSelectionWorker::allPeersConfirmPostRelease(
    std::uint64_t now_us) const noexcept
{
    // Post-release safety is a fresh nominal-trajectory result, not a selected
    // avoidance-tuple result.  Component membership may change selection
    // epochs without changing that nominal safety fact, so freshness and the
    // peer's explicit safe result are the complete release contract here.
    for (int aircraft = 0;
         aircraft < m_params.total_agent_count; ++aircraft) {
        if (aircraft == m_params.vehicle_id) {
            continue;
        }
        const RemoteDecisionCache & cache = m_remote_decision_caches[
            static_cast<std::size_t>(aircraft)];
        if (!cache.valid || !cache.decision.post_release_evaluated
            || !cache.decision.post_release_safe
            || cache.decision.post_release_evaluation_timestamp_us == 0
            || cache.decision.post_release_evaluation_timestamp_us > now_us
            || now_us - cache.decision.post_release_evaluation_timestamp_us
                > m_params.maximum_belief_delay_us) {
            return false;
        }
    }
    return true;
}

void ManeuverSelectionWorker::applyFormationActivationGate(
    const std::uint64_t now_us,
    ManeuverActivationSample & sample,
    ManeuverSelectionDecision & decision)
{
    sample.allow_new_activation = true;
    decision.formation_evaluated = false;
    decision.formation_inhibit = false;
    decision.formation_allow_new_activation = true;
    decision.formation_inhibited_threat_mask = 0U;
    if (!m_formation_discriminator.has_value() || !sample.valid) {
        return;
    }

    const double horizontal_speed_mps = std::sqrt(std::max(
        0.0,
        m_latest_state.V * m_latest_state.V
            - m_latest_state.h_dot * m_latest_state.h_dot));
    formation::FormationKinematicState ownship;
    ownship.position_ned_m = {
        m_latest_state.p_n, m_latest_state.p_e, -m_latest_state.h};
    ownship.velocity_ned_mps = {
        horizontal_speed_mps * std::cos(m_latest_state.psi),
        horizontal_speed_mps * std::sin(m_latest_state.psi),
        -m_latest_state.h_dot};
    ownship.timestamp_s = static_cast<double>(now_us) * 1.0e-6;
    ownship.valid = true;

    std::vector<formation::FormationResult> results;
    std::vector<std::uint32_t> collision_relevant_threat_ids;
    results.reserve(static_cast<std::size_t>(m_params.total_agent_count - 1));
    collision_relevant_threat_ids.reserve(results.capacity());
    const std::uint32_t original_unsafe_threat_mask =
        sample.unsafe_threat_mask;
    for (int remote_id = 0;
         remote_id < m_params.total_agent_count; ++remote_id) {
        if (remote_id == m_params.vehicle_id) {
            continue;
        }
        const std::size_t remote_index = static_cast<std::size_t>(remote_id);
        formation::FormationKinematicState threat;
        for (std::size_t axis = 0; axis < 3; ++axis) {
            threat.position_ned_m[axis] = ownship.position_ned_m[axis]
                + sample.relative_positions_ned_m[remote_index][axis];
            threat.velocity_ned_mps[axis] = ownship.velocity_ned_mps[axis]
                + sample.relative_velocities_ned_mps[remote_index][axis];
        }
        threat.timestamp_s = ownship.timestamp_s;
        threat.valid = true;

        formation::FormationUpdateInput input;
        input.threat_id = static_cast<std::uint32_t>(remote_id);
        input.evaluation_timestamp_s = ownship.timestamp_s;
        input.ownship = ownship;
        input.threat = threat;
        const auto result = m_formation_discriminator->update(input);
        results.push_back(result);

        const std::uint32_t threat_bit = std::uint32_t{1} << remote_index;
        if ((original_unsafe_threat_mask & threat_bit) == 0U) {
            continue;
        }
        collision_relevant_threat_ids.push_back(
            static_cast<std::uint32_t>(remote_id));
        const double hard_safety_budget_m =
            sample.activation_criteria_m[remote_index];
        double current_separation_squared_m2 = 0.0;
        for (const double component :
             sample.relative_positions_ned_m[remote_index]) {
            current_separation_squared_m2 += component * component;
        }
        const double current_separation_m = std::sqrt(
            current_separation_squared_m2);
        if (result.formation_inhibit && result.timestamp_valid
            && result.geometry_valid
            && formationSpacingCompatible(
                m_params.formation_target_separation_m,
                current_separation_m,
                hard_safety_budget_m)) {
            decision.formation_inhibited_threat_mask |= threat_bit;
        }
    }
    decision.formation_evaluated = true;

    // Formation is strictly a new-activation exemption. Once avoidance is
    // active, keep the complete AD-derived unsafe mask so newly unsafe threats
    // are added to the CPA termination monitor even if they resemble a
    // formation encounter.
    if (m_activation_controller.status().active) {
        return;
    }

    if (m_params.formation_aggregation_policy
            == formation::FormationAggregationPolicy::
                PerThreatExemptionOnly) {
        sample.unsafe_threat_mask &=
            ~decision.formation_inhibited_threat_mask;
        sample.allow_new_activation = sample.unsafe_threat_mask != 0U
            || original_unsafe_threat_mask == 0U;
    } else {
        const auto aggregation = formation::FormationInhibitAggregator::
            aggregate(
                results,
                collision_relevant_threat_ids,
                m_params.formation_aggregation_policy);
        sample.allow_new_activation = aggregation.allow_new_activation;
    }
    decision.formation_allow_new_activation = sample.allow_new_activation;
    decision.formation_inhibit = !sample.allow_new_activation;
}

void ManeuverSelectionWorker::updateActivationState(
    std::uint64_t now_us,
    bool force_decision_output,
    ManeuverSelectionWorkerOutput & output)
{
    if (!m_has_selected_combination) {
        return;
    }
    if (!force_decision_output
        && now_us <= m_last_activation_monitor_timestamp_us) {
        return;
    }
    m_last_activation_monitor_timestamp_us = now_us;

    if (m_params.execution_policy == ManeuverExecutionPolicy::ContinuousV4
        || m_params.execution_policy
            == ManeuverExecutionPolicy::HorizonGatedV4) {
        // Continuous V4 has no AMAC activation/latch state. Keeping these
        // semantics separate allows the coordinated V4 tuple to be refreshed
        // instead of being frozen for the AMAC active duration.
        m_activation_controller.reset();
        m_latest_selection_decision.activation_requested = false;
        m_latest_selection_decision.activation_just_started = false;
        m_latest_selection_decision.activation_just_ended = false;
        m_latest_selection_decision.activation_timestamp_us = 0;
        m_latest_selection_decision.deactivation_reason =
            ManeuverDeactivationReason::None;
        if (m_params.execution_policy
                == ManeuverExecutionPolicy::HorizonGatedV4
            && m_v4_horizon_gate_active && m_v4_horizon_latch_valid) {
            m_latest_selection_decision.ownship_candidate_id =
                m_v4_horizon_latched_candidate_id;
            m_latest_selection_decision.ownship_input =
                m_v4_horizon_latched_input;
            m_current_best_id = m_v4_horizon_latched_candidate_id;
        }
        if (force_decision_output) {
            output.decision = m_latest_selection_decision;
            output.has_decision = true;
        }
        return;
    }

    if (v4CutoverMode() && !m_selected_v4_cutover) {
        m_activation_controller.reset();
        m_latest_selection_decision.activation_requested = false;
        m_latest_selection_decision.activation_just_started = false;
        m_latest_selection_decision.activation_just_ended = false;
        m_latest_selection_decision.activation_timestamp_us = 0;
        m_latest_selection_decision.deactivation_reason =
            ManeuverDeactivationReason::None;
        if (force_decision_output) {
            output.decision = m_latest_selection_decision;
            output.has_decision = true;
        }
        return;
    }

    if (!m_activation_enabled.load(std::memory_order_acquire)) {
        m_activation_controller.reset();
        m_latest_selection_decision.activation_requested = false;
        m_latest_selection_decision.activation_just_started = false;
        m_latest_selection_decision.activation_just_ended = false;
        m_latest_selection_decision.activation_timestamp_us = 0;
        m_latest_selection_decision.deactivation_reason =
            ManeuverDeactivationReason::None;
        if (force_decision_output) {
            output.decision = m_latest_selection_decision;
            output.has_decision = true;
        }
        return;
    }

    ManeuverSelectionDecision decision = m_latest_selection_decision;
    decision.new_best_accepted = force_decision_output
        && decision.new_best_accepted;
    decision.previous_best_retained = !decision.new_best_accepted;
    decision.activation_just_started = false;
    decision.activation_just_ended = false;
    decision.deactivation_reason = ManeuverDeactivationReason::None;
    estimation::PredictInput nominal_input{};
    decision.nominal_setpoint_available = m_has_latest_nominal
        && nominalPredictInput(
            m_latest_nominal,
            now_us,
            m_params.v4_maximum_nominal_age_us,
            nominal_input);
    decision.nominal_setpoint_timestamp_us = m_has_latest_nominal
        ? m_latest_nominal.timestamp_us : 0;
    decision.nominal_ground_speed_command_mps =
        decision.nominal_setpoint_available
        ? nominal_input.V_cmd : std::numeric_limits<double>::quiet_NaN();
    decision.nominal_altitude_command_m =
        decision.nominal_setpoint_available
        ? nominal_input.h_cmd : std::numeric_limits<double>::quiet_NaN();
    decision.nominal_lateral_acceleration_mps2 =
        decision.nominal_setpoint_available
        ? nominal_input.a_lat_cmd : std::numeric_limits<double>::quiet_NaN();
    decision.cpa_clear = false;
    const bool retained_post_release_evaluation =
        m_has_last_post_release_evaluation
        && m_last_post_release_evaluation_timestamp_us <= now_us
        && now_us - m_last_post_release_evaluation_timestamp_us
            <= m_params.maximum_belief_delay_us;
    decision.post_release_evaluated = retained_post_release_evaluation;
    decision.post_release_safe = retained_post_release_evaluation
        && m_last_post_release_evaluation.all_pairs_feasible;
    decision.post_release_minimum_ad_m = retained_post_release_evaluation
        ? m_last_post_release_evaluation.minimum_ad_m
        : std::numeric_limits<double>::quiet_NaN();
    decision.post_release_evaluation_timestamp_us =
        retained_post_release_evaluation
        ? m_last_post_release_evaluation_timestamp_us : 0;
    decision.post_release_peer_confirmed = false;

    ManeuverActivationSample sample;
    const bool sample_valid = buildActivationSample(now_us, sample, decision);
    sample.valid = sample_valid;
    applyFormationActivationGate(now_us, sample, decision);
    const std::size_t activation_ownship_index = static_cast<std::size_t>(
        m_params.vehicle_id);
    const bool local_activation_trigger = sample.valid
        && sample.allow_new_activation && std::isfinite(sample.minimum_ad_m)
        && sample.minimum_ad_m < 0.0 && sample.unsafe_threat_mask != 0U;

    const ManeuverActivationStatus activation_before_rollup =
        m_activation_controller.status();
    if (!activation_before_rollup.active && m_selected_component_graph) {
        const std::size_t ownship_index = activation_ownship_index;
        const std::uint32_t ownship_bit = std::uint32_t{1} << ownship_index;
        const std::uint32_t component_mask =
            selectedComponentMemberMask(ownship_index);
        const bool peer_component_trigger =
            selectedComponentActivationRequested(component_mask);
        if (local_activation_trigger || peer_component_trigger) {
            sample.coordinated_activation_requested = true;
            sample.unsafe_threat_mask |= component_mask & ~ownship_bit;
            // A legitimate trigger from another member of the committed
            // component must not be vetoed by this node having no local unsafe
            // pair. Formation filtering has already been applied at the source.
            if (peer_component_trigger) {
                sample.allow_new_activation = true;
            }
        }
    }
    // activation_just_started is an edge, not a level. Consume every received
    // edge exactly once at the next local state update; an edge that does not
    // match the currently committed component tuple must not be reused later.
    for (RemoteDecisionCache & peer : m_remote_decision_caches) {
        peer.activation_start_pending = false;
    }
    JointCombinationEvaluation post_release_evaluation;
    if (evaluateNominalPostRelease(now_us, post_release_evaluation)) {
        m_last_post_release_evaluation = post_release_evaluation;
        m_last_post_release_evaluation_timestamp_us = now_us;
        m_has_last_post_release_evaluation = true;
        decision.post_release_evaluated = true;
        decision.post_release_safe =
            post_release_evaluation.all_pairs_feasible;
        decision.post_release_minimum_ad_m =
            post_release_evaluation.minimum_ad_m;
        decision.post_release_evaluation_timestamp_us = now_us;
    }
    const ManeuverActivationStatus previous_status =
        m_activation_controller.status();
    if (previous_status.active && sample.valid) {
        decision.cpa_clear = m_activation_controller.futureCpaClear(sample);
    }
    if (previous_status.active && decision.cpa_clear) {
        decision.post_release_peer_confirmed =
            decision.post_release_safe
            && std::isfinite(sample.minimum_ad_m)
            && sample.minimum_ad_m >= 0.0
            && allPeersConfirmPostRelease(now_us);
        sample.allow_deactivation =
            decision.post_release_peer_confirmed;
        m_safe_rejoin_active = !sample.allow_deactivation;
    } else {
        sample.allow_deactivation = true;
        m_safe_rejoin_active = false;
    }
    const ManeuverActivationStatus status =
        m_activation_controller.update(sample);
    decision.activation_requested = status.active;
    decision.activation_just_started = status.just_activated;
    decision.activation_just_ended = status.just_deactivated;
    decision.activation_timestamp_us = status.activation_timestamp_us;
    decision.deactivation_reason = status.deactivation_reason;
    if (!status.active) {
        m_safe_rejoin_active = false;
    }
    decision.safe_rejoin_active = m_safe_rejoin_active;
    if (status.active) {
        decision.ownship_candidate_id = status.latched_candidate_id;
        decision.ownship_candidate_valid = true;
        decision.ownship_input = status.latched_input;
        m_current_best_id = status.latched_candidate_id;
    } else {
        decision.ownship_candidate_valid = candidateIsValid(
            m_selected_candidate_valid_mask,
            static_cast<std::size_t>(m_params.vehicle_id));
    }

    m_latest_selection_decision = decision;
    if (force_decision_output || status.just_activated
        || status.just_deactivated) {
        output.decision = decision;
        output.has_decision = true;
    }
}


}  // namespace collision_avoidance::selection
