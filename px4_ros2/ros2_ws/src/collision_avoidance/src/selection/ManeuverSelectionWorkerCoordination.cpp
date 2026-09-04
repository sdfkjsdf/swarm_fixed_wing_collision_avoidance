#include "ManeuverSelectionWorkerInternal.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace collision_avoidance::selection
{
using namespace worker_detail;

bool ManeuverSelectionWorker::evaluateSelectedTuple(
    std::uint64_t evaluation_timestamp_us,
    const MultiAircraftExhaustiveCandidateIntentSets & candidate_sets,
    const std::array<std::size_t, kMaximumSelectionAircraft>
        & candidate_counts,
    JointCombinationEvaluation & evaluation) const
{
    if (!m_has_selected_combination) {
        return false;
    }

    return evaluateCandidateIdTuple(
        evaluation_timestamp_us,
        candidate_sets,
        candidate_counts,
        m_selected_candidate_ids,
        evaluation);
}

bool ManeuverSelectionWorker::buildCommonIncumbentCandidateIds(
    std::array<std::uint8_t, kMaximumSelectionAircraft>
        & candidate_ids) const noexcept
{
    if (!m_has_selected_combination) {
        return false;
    }

    candidate_ids = m_selected_candidate_ids;
    const std::size_t ownship_index = static_cast<std::size_t>(
        m_params.vehicle_id);
    const ManeuverActivationStatus ownship_status =
        m_activation_controller.status();
    candidate_ids[ownship_index] = ownship_status.active
        ? ownship_status.latched_candidate_id
        : m_current_best_id;

    for (int aircraft = 0; aircraft < m_params.total_agent_count; ++aircraft) {
        if (aircraft == m_params.vehicle_id) {
            continue;
        }
        const std::size_t aircraft_index = static_cast<std::size_t>(aircraft);
        const RemoteDecisionCache & peer =
            m_remote_decision_caches[aircraft_index];
        if (!peer.valid || !peer.decision.coordination_qualified) {
            return false;
        }
        candidate_ids[aircraft_index] = peer.decision.ownship_candidate_id;
    }
    return true;
}

bool ManeuverSelectionWorker::evaluateCandidateIdTuple(
    std::uint64_t evaluation_timestamp_us,
    const MultiAircraftExhaustiveCandidateIntentSets & candidate_sets,
    const std::array<std::size_t, kMaximumSelectionAircraft>
        & candidate_counts,
    const std::array<std::uint8_t, kMaximumSelectionAircraft>
        & candidate_ids,
    JointCombinationEvaluation & evaluation) const
{

    // ReceivedTrajectoryIntent contains the full reconstructed cone. Keep the
    // temporary one-candidate-per-aircraft set off the worker thread stack.
    auto selected_sets =
        std::make_unique<MultiAircraftCandidateIntentSets>();
    std::array<std::size_t, kMaximumSelectionAircraft> selected_counts{};
    for (int aircraft = 0; aircraft < m_params.total_agent_count; ++aircraft) {
        const std::size_t aircraft_index = static_cast<std::size_t>(aircraft);
        const auto begin = candidate_sets[aircraft_index].begin();
        const auto end = begin + static_cast<std::ptrdiff_t>(
            candidate_counts[aircraft_index]);
        const std::uint8_t selected_id = candidate_ids[aircraft_index];
        const auto found = std::find_if(
            begin, end, [selected_id](const auto & candidate) {
                return candidate.candidate_id == selected_id;
            });
        if (found == end) {
            return false;
        }
        (*selected_sets)[aircraft_index][0] = *found;
        selected_counts[aircraft_index] = 1;
    }

    JointManeuverEvaluation selected_evaluation;
    static_cast<void>(m_joint_evaluator.evaluate(
            evaluation_timestamp_us,
            *selected_sets,
            selected_counts,
            static_cast<std::size_t>(m_params.total_agent_count),
            selected_evaluation));
    if (selected_evaluation.combination_count != 1) {
        return false;
    }
    evaluation = selected_evaluation.combinations[
        selected_evaluation.has_best
            ? selected_evaluation.best_combination_index : 0];
    return true;
}

bool ManeuverSelectionWorker::proposalChangesActiveCommand(
    const std::array<std::uint8_t, kMaximumSelectionAircraft>
        & candidate_ids,
    const std::array<std::uint64_t, kMaximumSelectionAircraft>
        & candidate_input_revisions,
    std::uint32_t candidate_valid_mask) const noexcept
{
    if (continuousV4RoleChanged(
            m_params.execution_policy,
            m_has_selected_combination,
            m_selected_v4_cutover,
            m_selected_candidate_ids,
            candidate_ids,
            static_cast<std::size_t>(m_params.total_agent_count))) {
        // Mode B continuously recomputes the safe rate for the incumbent role.
        // A new revision of the same role is a receding-control refresh, while
        // changing NearNominal/LEFT/RIGHT is a maneuver switch that must pass
        // superiority and distributed proposal coordination.
        return true;
    }
    if (m_params.execution_policy == ManeuverExecutionPolicy::ContinuousV4
        && m_has_selected_combination && m_selected_v4_cutover) {
        return false;
    }

    for (int aircraft = 0; aircraft < m_params.total_agent_count; ++aircraft) {
        const std::size_t aircraft_index = static_cast<std::size_t>(aircraft);
        bool active = false;
        std::uint8_t actual_candidate_id = kRollZeroId;
        std::uint64_t actual_input_revision = 0;
        if (aircraft == m_params.vehicle_id) {
            const ManeuverActivationStatus status =
                m_activation_controller.status();
            active = status.active;
            actual_candidate_id = status.latched_candidate_id;
            actual_input_revision = status.latched_candidate_input_revision;
        } else {
            const RemoteDecisionCache & peer =
                m_remote_decision_caches[aircraft_index];
            active = peer.valid && peer.decision.coordination_qualified
                && peer.decision.ownship_candidate_valid
                && peer.decision.activation_requested;
            actual_candidate_id = peer.decision.ownship_candidate_id;
            actual_input_revision =
                peer.decision.selected_candidate_input_revisions[
                    aircraft_index];
        }
        if (active
            && candidateIsValid(candidate_valid_mask, aircraft_index)
            && (candidate_ids[aircraft_index] != actual_candidate_id
                || (m_params.execution_policy
                        != ManeuverExecutionPolicy::AmacAdThreshold
                    && candidate_input_revisions[aircraft_index]
                        != actual_input_revision))) {
            return true;
        }
    }
    return false;
}

bool ManeuverSelectionWorker::clearlySuperior(
    const JointCombinationEvaluation & current,
    const JointCombinationEvaluation & proposed) const noexcept
{
    if (!current.valid || !proposed.valid) {
        return false;
    }
    if (proposed.all_pairs_feasible && !current.all_pairs_feasible) {
        return true;
    }
    if (proposed.all_pairs_feasible != current.all_pairs_feasible) {
        return false;
    }
    if (proposed.all_pairs_feasible) {
        return std::isfinite(current.reciprocal_cost_sum)
            && std::isfinite(proposed.reciprocal_cost_sum)
            && proposed.reciprocal_cost_sum
                < current.reciprocal_cost_sum
                    - m_params.active_switch_cost_margin;
    }
    return std::isfinite(current.minimum_ad_m)
        && std::isfinite(proposed.minimum_ad_m)
        && proposed.minimum_ad_m
            > current.minimum_ad_m
                + m_params.active_switch_minimum_ad_margin_m;
}

bool ManeuverSelectionWorker::allRevisionSensitiveParticipantsReady()
    const noexcept
{
    if (!m_pending_proposal.valid) {
        return false;
    }
    for (int aircraft = 0; aircraft < m_params.total_agent_count; ++aircraft) {
        if (aircraft == m_params.vehicle_id) {
            continue;
        }
        const RemoteDecisionCache & peer =
            m_remote_decision_caches[static_cast<std::size_t>(aircraft)];
        const bool proposal_match = !peer.decision.proposed_component_graph
            && peer.decision.proposed_candidate_ids
                == m_pending_proposal.candidate_ids
            && peer.decision.proposed_candidate_valid_mask
                == m_pending_proposal.candidate_valid_mask
            && peer.decision.proposed_candidate_input_revisions
                == m_pending_proposal.candidate_input_revisions;
        if (!peer.valid || !peer.decision.proposal_valid
            || !peer.decision.proposal_consensus_confirmed
            || peer.decision.proposal_epoch != m_pending_proposal.epoch
            || !proposal_match
            || peer.decision.proposed_v4_cutover
                != m_pending_proposal.v4_cutover) {
            return false;
        }
    }
    return true;
}

bool ManeuverSelectionWorker::finalizePendingCoordination(
    ManeuverSelectionWorkerOutput & output)
{
    if (!m_pending_proposal.valid || m_pending_proposal.resolved) {
        return false;
    }

    bool all_proposals_available = true;
    bool all_proposals_match = true;
    auto authoritative_source_timestamps_us =
        m_pending_proposal.candidate_source_timestamps_us;
    auto authoritative_input_revisions =
        m_pending_proposal.candidate_input_revisions;
    const bool require_revision_consensus =
        m_params.execution_policy != ManeuverExecutionPolicy::AmacAdThreshold;
    for (int aircraft = 0;
         aircraft < m_params.total_agent_count; ++aircraft) {
        if (aircraft == m_params.vehicle_id) {
            continue;
        }
        const RemoteDecisionCache & cache = m_remote_decision_caches[
            static_cast<std::size_t>(aircraft)];
        if (!cache.valid || !cache.decision.proposal_valid
            || cache.decision.proposal_epoch != m_pending_proposal.epoch) {
            all_proposals_available = false;
            continue;
        }
        const std::size_t aircraft_index =
            static_cast<std::size_t>(aircraft);
        authoritative_source_timestamps_us[aircraft_index] =
            cache.decision.proposed_candidate_source_timestamps_us[
                aircraft_index];
        authoritative_input_revisions[aircraft_index] =
            cache.decision.proposed_candidate_input_revisions[
                aircraft_index];
        const bool component_graph_match = m_pending_proposal.component_graph
            && cache.decision.proposed_component_graph
            && cache.decision.proposed_candidate_library_hash
                == m_pending_proposal.candidate_library_hash
            && cache.decision.proposed_graph_hash
                == m_pending_proposal.graph_hash
            && cache.decision.proposed_component_hash
                == m_pending_proposal.component_hash
            && cache.decision.proposed_component_solution_hash
                == m_pending_proposal.component_solution_hash
            && cache.decision.proposed_candidate_valid_mask
                == m_pending_proposal.candidate_valid_mask;
        const bool legacy_proposal_match = !m_pending_proposal.component_graph
            && !cache.decision.proposed_component_graph
            && cache.decision.proposed_candidate_ids
                == m_pending_proposal.candidate_ids
            && cache.decision.proposed_candidate_valid_mask
                == m_pending_proposal.candidate_valid_mask
            && (!require_revision_consensus
                || cache.decision.proposed_candidate_input_revisions
                    == m_pending_proposal.candidate_input_revisions);
        if ((!component_graph_match && !legacy_proposal_match)
            || cache.decision.proposed_v4_cutover
                != m_pending_proposal.v4_cutover
            || (!m_pending_proposal.component_graph
                && !m_pending_proposal.active_command_change
                && cache.decision.activation_requested
                && (!candidateIsValid(
                        cache.decision.proposed_candidate_valid_mask,
                        static_cast<std::size_t>(aircraft))
                    || cache.decision.proposed_candidate_ids[
                        static_cast<std::size_t>(aircraft)]
                        != cache.decision.ownship_candidate_id
                    || (require_revision_consensus
                        && cache.decision.proposed_candidate_input_revisions[
                            static_cast<std::size_t>(aircraft)]
                            != cache.decision
                                .selected_candidate_input_revisions[
                                    static_cast<std::size_t>(aircraft)])
                    || cache.decision.proposed_v4_cutover
                        != cache.decision.selected_v4_cutover))) {
            all_proposals_match = false;
        }
    }
    if (!all_proposals_available) {
        return false;
    }

    ManeuverSelectionDecision decision = m_latest_selection_decision;
    decision.proposal_consensus_confirmed = all_proposals_match;
    decision.switch_superiority_evaluated =
        m_pending_proposal.superiority_evaluated;
    decision.switch_clearly_superior =
        m_pending_proposal.clearly_superior;
    if (m_pending_proposal.superiority_evaluated) {
        decision.switch_current_cost =
            m_pending_proposal.current_evaluation.reciprocal_cost_sum;
        decision.switch_proposed_cost =
            m_pending_proposal.evaluation.reciprocal_cost_sum;
        decision.switch_current_minimum_ad_m =
            m_pending_proposal.current_evaluation.minimum_ad_m;
        decision.switch_proposed_minimum_ad_m =
            m_pending_proposal.evaluation.minimum_ad_m;
    }
    decision.new_best_accepted = false;
    decision.previous_best_retained = true;
    if (!all_proposals_match) {
        m_pending_proposal.resolved = true;
        decision.coordination_qualified = m_has_selected_combination;
        m_latest_selection_decision = decision;
        output.decision = decision;
        output.has_decision = true;
        return false;
    }

    const ManeuverActivationStatus activation =
        m_activation_controller.status();
    const std::size_t ownship_index = static_cast<std::size_t>(
        m_params.vehicle_id);
    const bool proposed_ownship_candidate_valid = candidateIsValid(
        m_pending_proposal.candidate_valid_mask, ownship_index);
    if (activation.active && !m_pending_proposal.active_command_change
        && proposed_ownship_candidate_valid
        && (m_pending_proposal.candidate_ids[ownship_index]
                != activation.latched_candidate_id
            || m_pending_proposal.candidate_input_revisions[ownship_index]
                != activation.latched_candidate_input_revision)) {
        m_pending_proposal.resolved = true;
        decision.proposal_consensus_confirmed = false;
        decision.coordination_qualified = m_has_selected_combination;
        m_latest_selection_decision = decision;
        output.decision = decision;
        output.has_decision = true;
        return false;
    }
    if (!m_pending_proposal.component_graph
        && m_pending_proposal.active_command_change
        && require_revision_consensus
        && !allRevisionSensitiveParticipantsReady()) {
        decision.coordination_qualified = m_has_selected_combination;
        m_latest_selection_decision = decision;
        const std::uint64_t last_publish =
            m_pending_proposal.last_readiness_publish_timestamp_us;
        if (last_publish == 0
            || (m_latest_state_timestamp_us >= last_publish
                && m_latest_state_timestamp_us - last_publish
                    >= m_params.trajectory_refresh_period_us)) {
            m_pending_proposal.last_readiness_publish_timestamp_us =
                m_latest_state_timestamp_us;
            output.decision = decision;
            output.has_decision = true;
        }
        return false;
    }
    if (m_params.execution_policy == ManeuverExecutionPolicy::HorizonGatedV4
        && m_v4_horizon_gate_active && m_v4_horizon_latch_valid
        && (m_pending_proposal.candidate_ids[ownship_index]
                != m_v4_horizon_latched_candidate_id
            || m_pending_proposal.candidate_input_revisions[ownship_index]
                != m_v4_horizon_latched_input_revision)) {
        m_pending_proposal.resolved = true;
        decision.proposal_consensus_confirmed = false;
        decision.coordination_qualified = m_has_selected_combination;
        m_latest_selection_decision = decision;
        output.decision = decision;
        output.has_decision = true;
        return false;
    }

    const bool changed = !m_has_selected_combination
        || m_pending_proposal.candidate_valid_mask
            != m_selected_candidate_valid_mask
        || m_pending_proposal.candidate_ids != m_selected_candidate_ids
        || m_pending_proposal.candidate_input_revisions
            != m_selected_candidate_input_revisions;
    if (activation.active && proposed_ownship_candidate_valid
        && (m_pending_proposal.candidate_ids[ownship_index]
                != activation.latched_candidate_id
            || m_pending_proposal.candidate_input_revisions[ownship_index]
                != activation.latched_candidate_input_revision)
        && !m_activation_controller.replaceActiveCommand(
            m_pending_proposal.candidate_ids[ownship_index],
            m_pending_proposal.candidate_input_revisions[ownship_index],
            m_pending_proposal.ownship_input)) {
        return false;
    }

    m_selected_candidate_ids = m_pending_proposal.candidate_ids;
    m_selected_candidate_valid_mask =
        m_pending_proposal.candidate_valid_mask;
    m_selected_candidate_input_revisions = authoritative_input_revisions;
    m_selected_candidate_source_timestamps_us =
        authoritative_source_timestamps_us;
    m_selected_v4_cutover = m_pending_proposal.v4_cutover;
    m_selected_component_graph = m_pending_proposal.component_graph;
    m_selected_component_ids = m_pending_proposal.component_graph
        ? m_pending_proposal.component_ids
        : std::array<std::uint8_t, kMaximumSelectionAircraft>{};
    m_selected_component_count = m_pending_proposal.component_graph
        ? m_pending_proposal.component_count : 0;
    m_has_selected_combination = true;
    const ManeuverActivationStatus committed_activation =
        m_activation_controller.status();
    m_current_best_id = committed_activation.active
        ? committed_activation.latched_candidate_id
        : m_v4_horizon_gate_active && m_v4_horizon_latch_valid
            ? m_v4_horizon_latched_candidate_id
            : candidateIsValid(
                    m_selected_candidate_valid_mask, ownship_index)
                ? m_selected_candidate_ids[ownship_index]
                : m_current_best_id;

    // Proposal agreement can arrive after the next 4 Hz candidate refresh.
    // Keep the newly committed ownship command in that already-open epoch's
    // Current Best + Alternate set and immediately rebroadcast its intents.
    if (!m_params.exhaustive_test_mode && !m_selected_v4_cutover
        && std::find(
            m_held_candidate_ids.begin(),
            m_held_candidate_ids.begin()
                + static_cast<std::ptrdiff_t>(activeCandidateCount()),
            m_current_best_id)
            == m_held_candidate_ids.begin()
                + static_cast<std::ptrdiff_t>(activeCandidateCount())) {
        chooseAlternates(m_latest_state_timestamp_us);
        buildCurrentIntentSet(m_latest_state_timestamp_us, output);
    }

    decision.selection_timestamp_us = m_pending_proposal.timestamp_us;
    decision.local_selection_epoch = m_pending_proposal.epoch;
    decision.remote_selection_epoch = m_pending_proposal.epoch;
    decision.selection_epochs_by_aircraft.fill(m_pending_proposal.epoch);
    decision.selected_combination_index =
        m_pending_proposal.combination_index;
    decision.evaluated_combination_count =
        m_pending_proposal.combination_count;
    decision.evaluated_valid_combination_count =
        m_pending_proposal.valid_combination_count;
    decision.evaluated_safe_combination_count =
        m_pending_proposal.safe_combination_count;
    decision.maximum_evaluated_minimum_ad_m =
        m_pending_proposal.maximum_minimum_ad_m;
    decision.selected_combination_safe =
        m_pending_proposal.evaluation.all_pairs_feasible;
    decision.selected_nominal_rejoin_cost =
        m_pending_proposal.evaluation.nominal_rejoin_cost;
    decision.safe_rejoin_objective_applied =
        std::isfinite(decision.selected_nominal_rejoin_cost);
    decision.selected_candidate_ids = m_selected_candidate_ids;
    decision.selected_candidate_valid_mask =
        m_selected_candidate_valid_mask;
    decision.selected_candidate_input_revisions =
        m_selected_candidate_input_revisions;
    decision.selected_candidate_source_timestamps_us =
        m_selected_candidate_source_timestamps_us;
    decision.selected_v4_cutover = m_selected_v4_cutover;
    decision.ownship_candidate_id = committed_activation.active
        ? committed_activation.latched_candidate_id : m_current_best_id;
    decision.ownship_candidate_valid = committed_activation.active
        || candidateIsValid(m_selected_candidate_valid_mask, ownship_index);
    decision.pmr_m = m_pending_proposal.evaluation.minimum_pmr_m;
    decision.masd_m = m_pending_proposal.evaluation.minimum_masd_m;
    decision.communication_delay_margin_m =
        m_params.evaluator_params.communication_delay_margin_m;
    decision.ad_m = m_pending_proposal.evaluation.minimum_ad_m;
    decision.reciprocal_cost_sum =
        m_pending_proposal.evaluation.reciprocal_cost_sum;
    decision.coordination_qualified = true;
    decision.new_best_accepted = changed;
    decision.previous_best_retained = !changed;
    decision.ownship_input = committed_activation.active
        ? committed_activation.latched_input : m_pending_proposal.ownship_input;
    for (int aircraft = 0;
         aircraft < m_params.total_agent_count; ++aircraft) {
        if (aircraft != m_params.vehicle_id
            && candidateIsValid(
                m_selected_candidate_valid_mask,
                static_cast<std::size_t>(aircraft))) {
            decision.threat_candidate_id = m_selected_candidate_ids[
                static_cast<std::size_t>(aircraft)];
            break;
        }
    }

    m_pending_proposal.resolved = true;
    m_latest_selection_decision = decision;
    output.decision = decision;
    output.has_decision = true;
    return true;
}


}  // namespace collision_avoidance::selection

