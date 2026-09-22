#include "ManeuverSelectionWorkerInternal.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace collision_avoidance::selection
{
using namespace worker_detail;

void ManeuverSelectionWorker::initializeCandidateSet(std::uint64_t now_us)
{
    m_current_best_id = kRollZeroId;
    refreshCandidateSet(now_us);
    m_candidate_set_initialized = true;
}

void ManeuverSelectionWorker::refreshCandidateSet(std::uint64_t now_us)
{
    m_selection_epoch = now_us / m_params.candidate_refresh_period_us;
    if (!v4CutoverMode() || !m_v4_cutover_ready) {
        chooseAlternates(now_us);
    }
    m_epoch_generation_timestamp_us =
        m_selection_epoch * m_params.candidate_refresh_period_us;
    m_next_candidate_refresh_timestamp_us =
        (m_selection_epoch + 1) * m_params.candidate_refresh_period_us;
    m_next_trajectory_refresh_timestamp_us = now_us;
    m_epoch_evaluated = false;
    m_ownship_candidates_complete = false;
    m_ownship_candidate_count = 0;
    // Reuse storage; only ready entries from this epoch may be submitted.
    m_epoch_certification_candidate_counts.fill(0);
    m_epoch_certification_candidate_ready.fill(false);
    for (int remote_vehicle_id = 0;
         remote_vehicle_id < m_params.total_agent_count;
         ++remote_vehicle_id) {
        freezeRemoteCertificationCandidatesForCurrentEpoch(remote_vehicle_id);
    }
}

void ManeuverSelectionWorker::chooseAlternates(std::uint64_t now_us)
{
    if (m_params.exhaustive_test_mode) {
        std::copy(
            m_params.eligible_candidate_ids.begin(),
            m_params.eligible_candidate_ids.end(),
            m_held_candidate_ids.begin());
        std::sort(
            m_held_candidate_ids.begin(), m_held_candidate_ids.end());
        return;
    }
    const auto scores = scoreEligibleCandidates(now_us);
    const auto selected = m_candidate_selector.select(m_current_best_id, scores);
    std::copy(selected.begin(), selected.end(), m_held_candidate_ids.begin());
}

std::array<CandidateSafetyScore, estimation::kManeuverCandidateCount>
ManeuverSelectionWorker::scoreEligibleCandidates(std::uint64_t now_us)
{
    std::array<CandidateSafetyScore, estimation::kManeuverCandidateCount>
        scores{};
    for (std::size_t index = 0; index < scores.size(); ++index) {
        const std::uint8_t candidate_id = static_cast<std::uint8_t>(index);
        scores[index].candidate_id = candidate_id;
        scores[index].eligible = std::find(
            m_params.eligible_candidate_ids.begin(),
            m_params.eligible_candidate_ids.end(),
            candidate_id) != m_params.eligible_candidate_ids.end();
        const estimation::PredictInput * input = m_candidate_table.find(candidate_id);
        if (input != nullptr) {
            scores[index].lateral_acceleration_mps2 = input->a_lat_cmd;
        }

        if (!scores[index].eligible) {
            continue;
        }

        estimation::TrajectoryIntentPacket packet;
        estimation::ReceivedTrajectoryIntent own_candidate;
        if (!m_sender.buildForSelectedCandidate(
                now_us,
                candidate_id,
                m_latest_state,
                m_latest_covariance,
                packet,
                m_selection_epoch)
            || !m_receiver.receive(packet, own_candidate)) {
            continue;
        }

        if (m_params.evaluator_params.positive_margin_filter_enabled) {
            bool barrier_compared = false;
            bool barrier_valid = true;
            bool left_admissible = true;
            bool right_admissible = true;
            for (int remote_id = 0;
                 remote_id < m_params.total_agent_count; ++remote_id) {
                if (remote_id == m_params.vehicle_id) {
                    continue;
                }
                const RemoteCandidateCache & remote_cache =
                    m_remote_caches[static_cast<std::size_t>(remote_id)];
                if (remote_cache.count != activeCandidateCount()) {
                    continue;
                }
                for (std::size_t remote_index = 0;
                     remote_index < activeCandidateCount(); ++remote_index) {
                    BarrierDirectionEvaluation left;
                    BarrierDirectionEvaluation right;
                    if (!m_barrier_evaluator.evaluateDirection(
                            now_us,
                            own_candidate,
                            remote_cache.candidates[remote_index],
                            BarrierDirection::Left,
                            left)
                        || !m_barrier_evaluator.evaluateDirection(
                            now_us,
                            own_candidate,
                            remote_cache.candidates[remote_index],
                            BarrierDirection::Right,
                            right)) {
                        barrier_valid = false;
                        break;
                    }
                    barrier_compared = true;
                    left_admissible = left_admissible && left.admissible;
                    right_admissible = right_admissible && right.admissible;
                }
                if (!barrier_valid) {
                    break;
                }
            }

            constexpr double acceleration_tolerance = 1.0e-9;
            const bool direction_admissible =
                scores[index].lateral_acceleration_mps2
                    < -acceleration_tolerance
                ? left_admissible
                : scores[index].lateral_acceleration_mps2
                        > acceleration_tolerance
                    ? right_admissible
                    : left_admissible || right_admissible;
            if (!barrier_compared || !barrier_valid
                || !direction_admissible) {
                continue;
            }
        }

        bool compared = false;
        bool valid = true;
        double worst_ad_m = std::numeric_limits<double>::infinity();
        for (int remote_id = 0;
             remote_id < m_params.total_agent_count; ++remote_id) {
            if (remote_id == m_params.vehicle_id) {
                continue;
            }
            const RemoteCandidateCache & remote_cache =
                m_remote_caches[static_cast<std::size_t>(remote_id)];
            if (remote_cache.count != activeCandidateCount()) {
                continue;
            }
            for (std::size_t remote_index = 0;
                 remote_index < activeCandidateCount(); ++remote_index) {
                const auto & remote_candidate =
                    remote_cache.candidates[remote_index];
                CombinationEvaluation pair;
                if (!m_pair_evaluator.evaluatePair(
                        now_us, own_candidate, remote_candidate, pair)) {
                    valid = false;
                    break;
                }
                compared = true;
                worst_ad_m = std::min(worst_ad_m, pair.ad_m);
            }
            if (!valid) {
                break;
            }
        }
        scores[index].valid = compared && valid;
        if (scores[index].valid) {
            scores[index].worst_ad_m = worst_ad_m;
        }
    }
    return scores;
}

bool ManeuverSelectionWorker::buildCurrentIntentSet(
    std::uint64_t now_us,
    ManeuverSelectionWorkerOutput & output)
{
    ExhaustiveCandidateIntentSet received_candidates{};
    std::array<
        estimation::TrajectoryIntentPacket,
        kExhaustiveCandidatesPerAircraft> packets{};
    const std::size_t candidate_count = activeCandidateCount();
    estimation::PredictInput execution_input{};
    const bool execution_input_available = publishedInputAt(now_us, execution_input);
    for (std::size_t index = 0; index < candidate_count; ++index) {
        const bool built = m_sender.buildForSelectedCandidate(
            now_us,
            m_held_candidate_ids[index],
            m_latest_state,
            m_latest_covariance,
            packets[index],
            m_selection_epoch);
        if (!built) {
            m_ownship_candidates_complete = false;
            m_ownship_candidate_count = 0;
            return false;
        }
        packets[index].candidate_set_size = static_cast<std::uint8_t>(
            candidate_count);
        packets[index].candidate_set_kind =
            estimation::CandidateSetKind::LegacyRoll;
        packets[index].source_execution_input_available = execution_input_available;
        packets[index].source_execution_input = {
            static_cast<float>(execution_input.V_cmd),
            static_cast<float>(execution_input.h_cmd),
            static_cast<float>(execution_input.h_dot_cmd),
            static_cast<float>(execution_input.a_lat_cmd)};
        if (!m_receiver.receive(packets[index], received_candidates[index])) {
            m_ownship_candidates_complete = false;
            m_ownship_candidate_count = 0;
            return false;
        }
    }

    std::sort(
        received_candidates.begin(),
        received_candidates.begin()
            + static_cast<std::ptrdiff_t>(candidate_count),
        [](const auto & lhs, const auto & rhs) {
            return lhs.candidate_id < rhs.candidate_id;
        });

    m_ownship_candidates = received_candidates;
    m_ownship_candidates_complete = true;
    m_ownship_candidate_count = candidate_count;
    m_ownship_candidate_set_kind =
        estimation::CandidateSetKind::LegacyRoll;
    const std::uint64_t delivery_budget_us =
        3 * m_params.trajectory_refresh_period_us;
    const std::uint64_t freeze_offset_us = m_params.coordination_delay_us
            > delivery_budget_us
        ? m_params.coordination_delay_us - delivery_budget_us
        : 0;
    const std::uint64_t freeze_cutoff_timestamp_us =
        m_epoch_generation_timestamp_us + freeze_offset_us;
    if (m_params.interaction_graph_params.enabled
        && candidate_count == kExhaustiveCandidatesPerAircraft
        && now_us <= freeze_cutoff_timestamp_us) {
        const std::size_t ownship_index = static_cast<std::size_t>(
            m_params.vehicle_id);
        if (!m_epoch_certification_candidate_ready[ownship_index]
            || now_us > (*m_epoch_certification_candidate_sets)
                [ownship_index][0].source_timestamp_us) {
            (*m_epoch_certification_candidate_sets)[ownship_index] =
                received_candidates;
            m_epoch_certification_candidate_counts[ownship_index] =
                candidate_count;
            m_epoch_certification_candidate_ready[ownship_index] = true;
        }
    }
    std::copy_n(
        packets.begin(), candidate_count, output.intent_packets.begin());
    output.intent_packet_count = candidate_count;
    output.generated_timestamp_us = now_us;
    output.selection_epoch = m_selection_epoch;
    return true;
}

void ManeuverSelectionWorker::submitSelectionEvaluation(std::uint64_t now_us)
{
    const auto begin = m_stopped_stage_timing ? StoppedStageTiming::now() : 0;
    auto * available = m_evaluation_worker.beginRequest();
    if (!available) {
        ++m_selection_busy; // bounded backpressure; never wait or overwrite
        return;
    }
    auto & request = *available;
    request.timestamp_us = now_us;
    request.epoch = m_selection_epoch;
    request.aircraft_count = static_cast<std::size_t>(m_params.total_agent_count);
    request.exhaustive = m_params.exhaustive_test_mode;
    request.graph = InteractionGraphDiagnostics{};
    request.counts.fill(0);
    request.nominal_timestamp_us = m_has_latest_nominal ? m_latest_nominal.timestamp_us : 0;
    request.nominal_available = m_has_latest_nominal && nominalPredictInput(
        m_latest_nominal, now_us, m_params.v4_maximum_nominal_age_us, request.nominal_input);
    auto & candidate_sets = request.candidates;
    auto & candidate_counts = request.counts;
    const auto required_set_kind = m_ownship_candidate_set_kind;
    request.set_kind = required_set_kind;
    if (m_params.interaction_graph_params.enabled) {
        // The graph evaluates the frozen common-epoch library, not live caches.
        prepareInteractionGraph(request);
    } else {
        bool all_candidate_sets_complete = m_ownship_candidates_complete
            && m_ownship_candidate_count > 0
            && (!v4CutoverMode() || !m_v4_cutover_ready
                || required_set_kind
                    == estimation::CandidateSetKind::V4SafeControl);
        for (int aircraft = 0;
             aircraft < m_params.total_agent_count; ++aircraft) {
            if (aircraft == m_params.vehicle_id) {
                const std::size_t aircraft_index = static_cast<std::size_t>(
                    aircraft);
                candidate_sets[aircraft_index] = m_ownship_candidates;
                candidate_counts[aircraft_index] = m_ownship_candidate_count;
                continue;
            }
            const std::size_t aircraft_index = static_cast<std::size_t>(
                aircraft);
            const RemoteCandidateCache * remote_cache =
                &m_remote_caches[aircraft_index];
            const bool current_matches =
                remote_cache->selection_epoch == m_selection_epoch
                && remote_cache->candidate_set_kind == required_set_kind;
            const RemoteCandidateCache & previous_cache =
                m_remote_previous_caches[aircraft_index];
            if (!current_matches
                && previous_cache.selection_epoch == m_selection_epoch
                && previous_cache.candidate_set_kind == required_set_kind) {
                remote_cache = &m_remote_previous_caches[aircraft_index];
            }
            if (remote_cache->count == 0
                || remote_cache->count != remote_cache->expected_count) {
                all_candidate_sets_complete = false;
                continue;
            }
            if (remote_cache->selection_epoch != m_selection_epoch
                || remote_cache->candidate_set_kind != required_set_kind) {
                all_candidate_sets_complete = false;
                continue;
            }
            candidate_sets[aircraft_index] = remote_cache->candidates;
            candidate_counts[aircraft_index] = remote_cache->count;
        }

        if (all_candidate_sets_complete
            && m_params.execution_policy
                == ManeuverExecutionPolicy::HorizonGatedV4
            && required_set_kind
                == estimation::CandidateSetKind::V4SafeControl) {
            static_cast<void>(evaluateV4HorizonGate(
                now_us, candidate_sets, candidate_counts, m_latest_selection_decision));
        }
        request.complete = all_candidate_sets_complete;
    }

    if (request.complete)
        request.complete = constrainV4ActiveAircraftCandidates(candidate_sets, candidate_counts);
    if (m_stopped_stage_timing) {
        m_selection_snapshot_timing = StageTimingRecord{
            now_us, request.epoch, m_params.candidate_refresh_period_us,
            begin, StoppedStageTiming::now(), 4,
            static_cast<std::uint8_t>(activeCandidateCount()), request.complete, false};
    }
    m_evaluation_worker.submit();
    ++m_selection_submitted;
}

bool ManeuverSelectionWorker::consumeSelectionEvaluation(
    std::uint64_t now_us, ManeuverSelectionWorkerOutput & output)
{
    const auto * task = m_evaluation_worker.readyResult();
    if (!task) return false;
    const auto begin = m_stopped_stage_timing ? StoppedStageTiming::now() : 0;
    const auto & request = task->request;
    const auto & result = task->result;
    // The job evaluates epoch e at the start of e+1. Advancing the live
    // trajectory epoch to e+1 is normal, not grounds to discard the result.
    // Once the next decision boundary is reached it must not replace a newer
    // proposal. This deadline is the existing selection interval, not a margin.
    const bool expired = now_us >= request.timestamp_us
        && now_us - request.timestamp_us
            >= m_params.candidate_refresh_period_us;
    if (expired) {
        ++m_selection_expired;
    } else {
        applySelectionEvaluation(*task, output);
        ++m_selection_completed;
    }
    if (m_stopped_stage_timing) {
        m_selection_apply_timing = StageTimingRecord{
            request.timestamp_us, request.epoch, m_params.candidate_refresh_period_us,
            begin, StoppedStageTiming::now(), 5,
            static_cast<std::uint8_t>(activeCandidateCount()), !expired, false};
        // Stage 2 measures the dedicated graph/search kernel, NOT queue wait.
        // Only this state-owner appends records, after the output handoff.
        m_completed_selection_timing = StageTimingRecord{
            request.timestamp_us, request.epoch, m_params.candidate_refresh_period_us,
            result.start_ns, result.end_ns, 2,
            static_cast<std::uint8_t>(activeCandidateCount()),
            !expired && output.has_decision && output.decision.proposal_valid, false};
    }
    m_evaluation_worker.release();
    return !expired;
}

void ManeuverSelectionWorker::applySelectionEvaluation(
    const ManeuverEvaluationTask & task, ManeuverSelectionWorkerOutput & output)
{
    const auto & request = task.request;
    const auto & result = task.result;
    const auto now_us = request.timestamp_us;
    const auto & candidate_sets = request.candidates;
    const auto & candidate_counts = request.counts;
    const auto required_set_kind = request.set_kind;
    m_pending_interaction_graph_diagnostics.reset();
    if (m_params.interaction_graph_params.enabled)
        m_pending_interaction_graph_diagnostics = result.graph;
    ManeuverSelectionDecision decision = m_has_selected_combination
        ? m_latest_selection_decision
        : ManeuverSelectionDecision{};
    // The V4 supervisory gate runs on the state owner before dispatch. Preserve
    // its first evaluation even when no maneuver has been committed yet.
    if (!m_has_selected_combination && request.complete
        && m_params.execution_policy == ManeuverExecutionPolicy::HorizonGatedV4
        && required_set_kind == estimation::CandidateSetKind::V4SafeControl) {
        decision.v4_horizon_gate_evaluated = m_latest_selection_decision.v4_horizon_gate_evaluated;
        decision.v4_horizon_gate_valid = m_latest_selection_decision.v4_horizon_gate_valid;
        decision.v4_horizon_local_gate_active = m_latest_selection_decision.v4_horizon_local_gate_active;
        decision.v4_horizon_gate_active = m_latest_selection_decision.v4_horizon_gate_active;
        decision.v4_horizon_h_worst_m = m_latest_selection_decision.v4_horizon_h_worst_m;
        decision.v4_horizon_trigger_m = m_latest_selection_decision.v4_horizon_trigger_m;
        decision.v4_horizon_worst_time_offset_s = m_latest_selection_decision.v4_horizon_worst_time_offset_s;
        decision.v4_horizon_worst_first_vehicle_id = m_latest_selection_decision.v4_horizon_worst_first_vehicle_id;
        decision.v4_horizon_worst_second_vehicle_id = m_latest_selection_decision.v4_horizon_worst_second_vehicle_id;
    }
    decision.vehicle_id = m_params.vehicle_id;
    decision.aircraft_count = static_cast<std::size_t>(
        m_params.total_agent_count);
    decision.proposal_timestamp_us = now_us;
    decision.proposal_epoch = request.epoch;
    decision.proposal_valid = false;
    decision.proposed_candidate_valid_mask = 0U;
    decision.proposed_v4_cutover = false;
    decision.proposed_component_graph = false;
    decision.proposed_candidate_library_hash = 0;
    decision.proposed_graph_hash = 0;
    decision.proposed_component_hash = 0;
    decision.proposed_component_solution_hash = 0;
    decision.proposal_consensus_confirmed = false;
    decision.communication_delay_margin_m =
        m_params.evaluator_params.communication_delay_margin_m;
    decision.switch_superiority_evaluated = false;
    decision.switch_clearly_superior = false;
    decision.switch_current_cost =
        std::numeric_limits<double>::quiet_NaN();
    decision.switch_proposed_cost =
        std::numeric_limits<double>::quiet_NaN();
    decision.switch_current_minimum_ad_m =
        std::numeric_limits<double>::quiet_NaN();
    decision.switch_proposed_minimum_ad_m =
        std::numeric_limits<double>::quiet_NaN();
    decision.evaluated_combination_count = 0;
    decision.evaluated_valid_combination_count = 0;
    decision.evaluated_safe_combination_count = 0;
    decision.maximum_evaluated_minimum_ad_m =
        std::numeric_limits<double>::quiet_NaN();
    decision.selected_combination_safe = false;
    decision.nominal_setpoint_available = request.nominal_available;
    decision.nominal_setpoint_timestamp_us = request.nominal_timestamp_us;
    decision.nominal_ground_speed_command_mps = request.nominal_available
        ? request.nominal_input.V_cmd : std::numeric_limits<double>::quiet_NaN();
    decision.nominal_altitude_command_m = request.nominal_available
        ? request.nominal_input.h_cmd : std::numeric_limits<double>::quiet_NaN();
    decision.nominal_lateral_acceleration_mps2 = request.nominal_available
        ? request.nominal_input.a_lat_cmd : std::numeric_limits<double>::quiet_NaN();
    decision.safe_rejoin_active = m_safe_rejoin_active;
    decision.safe_rejoin_objective_applied = false;
    decision.selected_nominal_rejoin_cost =
        std::numeric_limits<double>::quiet_NaN();
    decision.new_best_accepted = false;
    decision.previous_best_retained = true;
    decision.coordination_qualified = m_has_selected_combination;
    decision.selected_candidate_ids = m_selected_candidate_ids;
    decision.selected_candidate_valid_mask =
        m_selected_candidate_valid_mask;
    decision.selected_candidate_input_revisions =
        m_selected_candidate_input_revisions;
    decision.selected_candidate_source_timestamps_us =
        m_selected_candidate_source_timestamps_us;
    decision.selected_v4_cutover = m_selected_v4_cutover;
    decision.ownship_candidate_id = m_current_best_id;
    decision.ownship_candidate_valid = candidateIsValid(
        m_selected_candidate_valid_mask,
        static_cast<std::size_t>(m_params.vehicle_id));
    if (m_has_selected_combination) {
        const std::size_t ownship_index = static_cast<std::size_t>(
            m_params.vehicle_id);
        const auto found = std::find_if(
            request.candidates[ownship_index].begin(),
            request.candidates[ownship_index].begin()
                + static_cast<std::ptrdiff_t>(request.counts[ownship_index]),
            [this, ownship_index](const auto & candidate) {
                return candidate.candidate_id == m_current_best_id
                    && candidate.candidate_input_revision
                        == m_selected_candidate_input_revisions[ownship_index];
            });
        if (found != request.candidates[ownship_index].begin()
                + static_cast<std::ptrdiff_t>(request.counts[ownship_index])) {
            decision.ownship_input = found->candidate_input;
        }
    } else if (const auto * input = m_candidate_table.find(m_current_best_id)) {
        decision.ownship_input = *input;
    }


    if (request.complete) {
        const auto & best = result.best;
        const auto best_combination_index = result.best_index;
        const auto combination_count = result.combination_count;
        const auto valid_combination_count = result.valid_count;
        const auto safe_combination_count = result.safe_count;
        const auto maximum_minimum_ad_m = result.maximum_minimum_ad_m;
        const bool evaluated = result.evaluated;
        const bool component_cutover = m_params.interaction_graph_params.enabled;
        const auto & component_candidate_ids = result.graph.assembled_candidate_ids;
        const auto component_candidate_valid_mask = result.graph.assembled_candidate_valid_mask;
        decision.safe_rejoin_objective_applied = evaluated
            && std::isfinite(best.nominal_rejoin_cost);
        decision.evaluated_combination_count = combination_count;
        decision.evaluated_valid_combination_count = valid_combination_count;
        decision.evaluated_safe_combination_count = safe_combination_count;
        decision.maximum_evaluated_minimum_ad_m = maximum_minimum_ad_m;
        if (evaluated) {
            decision.selected_nominal_rejoin_cost =
                best.nominal_rejoin_cost;
            std::array<std::uint8_t, kMaximumSelectionAircraft>
                proposed_candidate_ids{};
            const std::uint32_t proposed_candidate_valid_mask =
                component_cutover
                ? component_candidate_valid_mask
                : candidateMaskForAircraftCount(static_cast<std::size_t>(
                    m_params.total_agent_count));
            std::array<std::uint64_t, kMaximumSelectionAircraft>
                proposed_candidate_input_revisions{};
            std::array<std::uint64_t, kMaximumSelectionAircraft>
                proposed_candidate_source_timestamps_us{};
            std::array<std::uint8_t, kMaximumSelectionAircraft>
                proposed_candidate_slots{};
            bool proposal_library_mapping_valid = true;
            for (int aircraft = 0;
                 aircraft < m_params.total_agent_count; ++aircraft) {
                const std::size_t aircraft_index =
                    static_cast<std::size_t>(aircraft);
                if (!candidateIsValid(
                        proposed_candidate_valid_mask, aircraft_index)) {
                    continue;
                }
                std::uint8_t candidate_slot =
                    best.candidate_slots[aircraft_index];
                if (component_cutover) {
                    const std::uint8_t candidate_id =
                        component_candidate_ids[aircraft_index];
                    const auto begin = candidate_sets[aircraft_index].begin();
                    const auto end = begin + static_cast<std::ptrdiff_t>(
                        candidate_counts[aircraft_index]);
                    const auto found = std::find_if(
                        begin, end, [candidate_id](const auto & candidate) {
                            return candidate.candidate_id == candidate_id;
                        });
                    if (found == end) {
                        proposal_library_mapping_valid = false;
                        break;
                    }
                    candidate_slot = static_cast<std::uint8_t>(
                        std::distance(begin, found));
                }
                proposed_candidate_slots[aircraft_index] = candidate_slot;
                const auto & proposed_candidate =
                    candidate_sets[aircraft_index][candidate_slot];
                proposed_candidate_ids[aircraft_index] =
                    proposed_candidate.candidate_id;
                proposed_candidate_input_revisions[aircraft_index] =
                    proposed_candidate.candidate_input_revision;
                proposed_candidate_source_timestamps_us[aircraft_index] =
                    proposed_candidate.source_timestamp_us;
            }
            if (!proposal_library_mapping_valid) {
                m_pending_proposal = PendingSelectionProposal{};
                m_latest_selection_decision = decision;
                output.decision = decision;
                output.has_decision = true;
                publishPendingInteractionGraphDiagnostics();
                return;
            }
            if (component_cutover
                && m_pending_interaction_graph_diagnostics) {
                m_pending_interaction_graph_diagnostics
                    ->component_proposal_used = true;
            }
            const bool active_command_change =
                component_cutover
                ? m_has_selected_combination
                    && (proposed_candidate_valid_mask
                            != m_selected_candidate_valid_mask
                        || proposed_candidate_ids != m_selected_candidate_ids)
                : proposalChangesActiveCommand(
                    proposed_candidate_ids,
                    proposed_candidate_input_revisions,
                    proposed_candidate_valid_mask);
            JointCombinationEvaluation current_evaluation{};
            std::array<std::uint8_t, kMaximumSelectionAircraft>
                incumbent_candidate_ids = m_selected_candidate_ids;
            const bool common_incumbent_available =
                component_cutover
                ? m_has_selected_combination
                : (m_params.execution_policy
                        != ManeuverExecutionPolicy::AmacAdThreshold
                    || buildCommonIncumbentCandidateIds(
                        incumbent_candidate_ids));
            bool current_evaluation_available = false;
            if (active_command_change && common_incumbent_available) {
                if (component_cutover
                    && result.certifications.valid
                    && result.certifications.selection_epoch
                        == request.epoch) {
                    // Both tuples use the same execution-aligned 7x7 matrices.
                    // The immutable request still supplies original IDs/input
                    // revisions/source timestamps; it is NOT re-evaluated here.
                    current_evaluation_available =
                        m_certified_component_evaluator.evaluateTuple(
                            result.certifications,
                            candidate_sets,
                            incumbent_candidate_ids,
                            current_evaluation);
                } else {
                    current_evaluation_available =
                        m_params.execution_policy
                                == ManeuverExecutionPolicy::AmacAdThreshold
                            ? evaluateCandidateIdTuple(
                                now_us,
                                candidate_sets,
                                candidate_counts,
                                incumbent_candidate_ids,
                                current_evaluation)
                            : evaluateSelectedTuple(
                                now_us,
                                candidate_sets,
                                candidate_counts,
                                current_evaluation);
                }
            }
            const bool current_evaluation_valid =
                current_evaluation_available && current_evaluation.valid;
            const bool mode_b_role_recertification = active_command_change
                && m_selected_v4_cutover
                && m_params.execution_policy
                    == ManeuverExecutionPolicy::ContinuousV4;
            const bool superiority_evaluated = active_command_change
                && (current_evaluation_available
                    || mode_b_role_recertification);
            const bool clearly_superior = superiority_evaluated
                && m_params.active_switching_enabled
                && ((!current_evaluation_valid
                        && current_evaluation_available
                        && m_params.execution_policy
                            == ManeuverExecutionPolicy::AmacAdThreshold
                        && best.valid)
                    || (!current_evaluation_valid
                        && mode_b_role_recertification)
                    || clearlySuperior(current_evaluation, best));
            decision.switch_superiority_evaluated = superiority_evaluated;
            decision.switch_clearly_superior = clearly_superior;
            if (superiority_evaluated) {
                decision.switch_proposed_cost = best.reciprocal_cost_sum;
                decision.switch_proposed_minimum_ad_m = best.minimum_ad_m;
                if (current_evaluation_valid) {
                    decision.switch_current_cost =
                        current_evaluation.reciprocal_cost_sum;
                    decision.switch_current_minimum_ad_m =
                        current_evaluation.minimum_ad_m;
                }
            }
            // The component result is authoritative as the proposed tuple,
            // and every changed component tuple is qualified BEFORE proposing,
            // irrespective of a local activation transition during agreement.
            // Compare the committed incumbent on the same frozen library.
            const bool active_change_allowed = !active_command_change
                || clearly_superior;
            if (!active_change_allowed) {
                m_pending_proposal = PendingSelectionProposal{};
                m_latest_selection_decision = decision;
                output.decision = decision;
                output.has_decision = true;
                publishPendingInteractionGraphDiagnostics();
                return;
            }
            m_pending_proposal = PendingSelectionProposal{};
            m_pending_proposal.timestamp_us = now_us;
            m_pending_proposal.epoch = request.epoch;
            m_pending_proposal.candidate_ids = proposed_candidate_ids;
            m_pending_proposal.candidate_valid_mask =
                proposed_candidate_valid_mask;
            m_pending_proposal.candidate_input_revisions =
                proposed_candidate_input_revisions;
            m_pending_proposal.candidate_source_timestamps_us =
                proposed_candidate_source_timestamps_us;
            m_pending_proposal.v4_cutover =
                required_set_kind
                == estimation::CandidateSetKind::V4SafeControl;
            if (m_params.interaction_graph_params.enabled
                && m_pending_interaction_graph_diagnostics
                && m_pending_interaction_graph_diagnostics
                    ->component_search_evaluated) {
                const auto & graph_diagnostics =
                    *m_pending_interaction_graph_diagnostics;
                m_pending_proposal.component_graph = true;
                m_pending_proposal.candidate_library_hash =
                    graph_diagnostics.graph.candidate_library_hash;
                m_pending_proposal.graph_hash =
                    graph_diagnostics.graph.graph_hash;
                m_pending_proposal.component_hash =
                    graph_diagnostics.graph.component_hash;
                m_pending_proposal.component_solution_hash =
                    graph_diagnostics.component_solution_hash;
                m_pending_proposal.component_ids =
                    graph_diagnostics.graph.component_ids;
                m_pending_proposal.component_count =
                    static_cast<std::uint8_t>(
                        graph_diagnostics.graph.component_count);
            }
            const std::uint8_t ownship_slot = proposed_candidate_slots[
                static_cast<std::size_t>(m_params.vehicle_id)];
            const std::size_t ownship_index = static_cast<std::size_t>(
                m_params.vehicle_id);
            if (candidateIsValid(
                    proposed_candidate_valid_mask, ownship_index)) {
                m_pending_proposal.ownship_input =
                    candidate_sets[ownship_index][ownship_slot].candidate_input;
            } else if (request.nominal_available) {
                m_pending_proposal.ownship_input = request.nominal_input;
            } else {
                m_pending_proposal = PendingSelectionProposal{};
                m_latest_selection_decision = decision;
                output.decision = decision;
                output.has_decision = true;
                publishPendingInteractionGraphDiagnostics();
                return;
            }
            m_pending_proposal.current_evaluation = current_evaluation;
            m_pending_proposal.evaluation = best;
            m_pending_proposal.active_command_change =
                active_command_change;
            m_pending_proposal.superiority_evaluated =
                superiority_evaluated;
            m_pending_proposal.clearly_superior = clearly_superior;
            m_pending_proposal.combination_index = best_combination_index;
            m_pending_proposal.combination_count = combination_count;
            m_pending_proposal.valid_combination_count =
                valid_combination_count;
            m_pending_proposal.safe_combination_count =
                safe_combination_count;
            m_pending_proposal.maximum_minimum_ad_m = maximum_minimum_ad_m;
            m_pending_proposal.valid = true;
            m_pending_proposal.resolved = false;
            if (m_params.masd_diagnostics_enabled) {
                ManeuverBudgetTrace trace;
                trace.event = 1;
                trace.epoch = m_pending_proposal.epoch;
                trace.evaluation_timestamp_us = m_pending_proposal.timestamp_us;
                const auto index = static_cast<std::size_t>(m_params.vehicle_id);
                trace.candidate_id = proposed_candidate_ids[index];
                trace.input_revision = proposed_candidate_input_revisions[index];
                trace.source_timestamp_us = proposed_candidate_source_timestamps_us[index];
                trace.ad_m = best.minimum_ad_m;
                recordBudgetTrace(trace);
            }

            decision.proposed_candidate_ids = proposed_candidate_ids;
            decision.proposed_candidate_valid_mask =
                proposed_candidate_valid_mask;
            decision.proposed_candidate_input_revisions =
                proposed_candidate_input_revisions;
            decision.proposed_candidate_source_timestamps_us =
                proposed_candidate_source_timestamps_us;
            decision.proposed_v4_cutover =
                m_pending_proposal.v4_cutover;
            decision.proposed_component_graph =
                m_pending_proposal.component_graph;
            decision.proposed_candidate_library_hash =
                m_pending_proposal.candidate_library_hash;
            decision.proposed_graph_hash = m_pending_proposal.graph_hash;
            decision.proposed_component_hash =
                m_pending_proposal.component_hash;
            decision.proposed_component_solution_hash =
                m_pending_proposal.component_solution_hash;
            decision.proposal_valid = true;
            decision.evaluated_combination_count = combination_count;
            decision.evaluated_valid_combination_count =
                valid_combination_count;
            decision.evaluated_safe_combination_count =
                safe_combination_count;
            decision.maximum_evaluated_minimum_ad_m = maximum_minimum_ad_m;
            decision.selected_combination_safe = best.all_pairs_feasible;
        }
    }

    m_latest_selection_decision = decision;
    output.decision = decision;
    output.has_decision = true;
    publishPendingInteractionGraphDiagnostics();
}


}  // namespace collision_avoidance::selection
