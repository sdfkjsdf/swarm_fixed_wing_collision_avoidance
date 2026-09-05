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
    m_epoch_certification_candidate_sets.reset();
    m_epoch_pairwise_ad_certifications.reset();
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
    estimation::PredictInput nominal_input{};
    const bool nominal_input_available = m_has_latest_nominal
        && nominalPredictInput(
            m_latest_nominal,
            now_us,
            m_params.v4_maximum_nominal_age_us,
            nominal_input);
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
        packets[index].nominal_lateral_acceleration_mps2 =
            nominal_input_available
            ? static_cast<float>(nominal_input.a_lat_cmd)
            : std::numeric_limits<float>::quiet_NaN();
        packets[index].safe_rejoin_requested = m_safe_rejoin_active;
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
        if (!m_epoch_certification_candidate_sets) {
            m_epoch_certification_candidate_sets = std::make_unique<
                MultiAircraftExhaustiveCandidateIntentSets>();
        }
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

void ManeuverSelectionWorker::evaluateCurrentSet(
    std::uint64_t now_us,
    ManeuverSelectionWorkerOutput & output)
{
    ManeuverSelectionDecision decision = m_has_selected_combination
        ? m_latest_selection_decision
        : ManeuverSelectionDecision{};
    decision.vehicle_id = m_params.vehicle_id;
    decision.aircraft_count = static_cast<std::size_t>(
        m_params.total_agent_count);
    decision.proposal_timestamp_us = now_us;
    decision.proposal_epoch = m_selection_epoch;
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
            m_ownship_candidates.begin(),
            m_ownship_candidates.begin()
                + static_cast<std::ptrdiff_t>(m_ownship_candidate_count),
            [this, ownship_index](const auto & candidate) {
                return candidate.candidate_id == m_current_best_id
                    && candidate.candidate_input_revision
                        == m_selected_candidate_input_revisions[ownship_index];
            });
        if (found != m_ownship_candidates.begin()
                + static_cast<std::ptrdiff_t>(m_ownship_candidate_count)) {
            decision.ownship_input = found->candidate_input;
        }
    } else if (const auto * input = m_candidate_table.find(m_current_best_id)) {
        decision.ownship_input = *input;
    }

    MultiAircraftExhaustiveCandidateIntentSets candidate_sets{};
    std::array<std::size_t, kMaximumSelectionAircraft> candidate_counts{};
    const estimation::CandidateSetKind required_set_kind =
        m_ownship_candidate_set_kind;
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
            candidate_sets[aircraft_index] =
                m_ownship_candidates;
            candidate_counts[aircraft_index] = m_ownship_candidate_count;
            continue;
        }
        const std::size_t aircraft_index =
            static_cast<std::size_t>(aircraft);
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

    if (all_candidate_sets_complete) {
        if (m_params.execution_policy
                == ManeuverExecutionPolicy::HorizonGatedV4
            && required_set_kind
                == estimation::CandidateSetKind::V4SafeControl) {
            static_cast<void>(evaluateV4HorizonGate(
                now_us, candidate_sets, candidate_counts, decision));
        }
    }

    evaluateInteractionGraph(now_us);

    // Component search obtains its fleet-wide rejoin objective from the
    // certified candidate library.  A local objective is only meaningful on
    // the legacy evaluator path.
    ManeuverRejoinObjective legacy_rejoin_objective;
    const ManeuverRejoinObjective * legacy_rejoin_objective_ptr = nullptr;
    if (!m_params.interaction_graph_params.enabled
        && m_safe_rejoin_active
        && buildManeuverRejoinObjective(now_us, legacy_rejoin_objective)) {
        legacy_rejoin_objective_ptr = &legacy_rejoin_objective;
    }

    // Active component search must execute from the exact candidate library
    // that was certified. The 20 Hz live cache may already contain a newer
    // trajectory revision from the same 4 Hz epoch.
    if (m_params.interaction_graph_params.enabled
        && m_epoch_certification_candidate_sets
        && m_pending_interaction_graph_diagnostics
        && m_pending_interaction_graph_diagnostics
            ->component_search_evaluated) {
        candidate_sets = *m_epoch_certification_candidate_sets;
        candidate_counts = m_epoch_certification_candidate_counts;
        all_candidate_sets_complete = true;
    }

    if (all_candidate_sets_complete
        && constrainV4ActiveAircraftCandidates(
            candidate_sets, candidate_counts)) {
        JointCombinationEvaluation best{};
        std::size_t best_combination_index = 0;
        std::size_t combination_count = 0;
        std::size_t valid_combination_count = 0;
        std::size_t safe_combination_count = 0;
        double maximum_minimum_ad_m =
            std::numeric_limits<double>::quiet_NaN();
        bool evaluated = false;
        const bool component_cutover =
            m_params.interaction_graph_params.enabled;
        ManeuverRejoinObjective component_rejoin_objective;
        const ManeuverRejoinObjective * selection_rejoin_objective_ptr =
            legacy_rejoin_objective_ptr;
        if (component_cutover) {
            bool rejoin_requested = false;
            bool rejoin_values_valid = true;
            for (int aircraft = 0;
                 aircraft < m_params.total_agent_count; ++aircraft) {
                const std::size_t aircraft_index =
                    static_cast<std::size_t>(aircraft);
                const auto & intent = candidate_sets[aircraft_index][0];
                rejoin_requested = rejoin_requested
                    || intent.safe_rejoin_requested;
                component_rejoin_objective
                    .nominal_lateral_acceleration_mps2[aircraft_index] =
                    intent.nominal_lateral_acceleration_mps2;
                rejoin_values_valid = rejoin_values_valid
                    && std::isfinite(
                        intent.nominal_lateral_acceleration_mps2);
            }
            if (rejoin_requested && rejoin_values_valid) {
                component_rejoin_objective.enabled = true;
                selection_rejoin_objective_ptr =
                    &component_rejoin_objective;
            }
        }
        std::array<std::uint8_t, kMaximumSelectionAircraft>
            component_candidate_ids{};
        std::uint32_t component_candidate_valid_mask{0};
        if (component_cutover) {
            const bool component_result_ready =
                m_pending_interaction_graph_diagnostics
                && m_pending_interaction_graph_diagnostics
                    ->component_search_evaluated
                && m_pending_interaction_graph_diagnostics
                    ->global_crosscheck_pass;
            if (component_result_ready) {
                const auto & component =
                    *m_pending_interaction_graph_diagnostics;
                best = component.global_crosscheck_evaluation;
                best_combination_index = 0;
                combination_count =
                    component.graph.component_evaluation_count;
                valid_combination_count =
                    component.component_valid_evaluation_count;
                safe_combination_count =
                    component.component_safe_evaluation_count;
                maximum_minimum_ad_m = best.minimum_ad_m;
                evaluated = best.valid;
                component_candidate_ids = component.assembled_candidate_ids;
                component_candidate_valid_mask =
                    component.assembled_candidate_valid_mask;
            }
        } else if (m_params.exhaustive_test_mode) {
            ExhaustiveManeuverEvaluation evaluation;
            evaluated = m_exhaustive_evaluator.evaluate(
                now_us,
                candidate_sets,
                static_cast<std::size_t>(m_params.total_agent_count),
                evaluation,
                legacy_rejoin_objective_ptr);
            combination_count = evaluation.combination_count;
            valid_combination_count = evaluation.valid_combination_count;
            safe_combination_count = evaluation.safe_combination_count;
            maximum_minimum_ad_m = evaluation.maximum_minimum_ad_m;
            if (evaluated) {
                best = evaluation.best_combination;
                best_combination_index = evaluation.best_combination_index;
            }
        } else {
            MultiAircraftCandidateIntentSets reduced_candidate_sets{};
            for (int aircraft = 0;
                 aircraft < m_params.total_agent_count; ++aircraft) {
                const std::size_t aircraft_index =
                    static_cast<std::size_t>(aircraft);
                std::copy_n(
                    candidate_sets[aircraft_index].begin(),
                    candidate_counts[aircraft_index],
                    reduced_candidate_sets[aircraft_index].begin());
                std::fill(
                    reduced_candidate_sets[aircraft_index].begin()
                        + static_cast<std::ptrdiff_t>(
                            candidate_counts[aircraft_index]),
                    reduced_candidate_sets[aircraft_index].end(),
                    reduced_candidate_sets[aircraft_index][0]);
            }
            JointManeuverEvaluation evaluation;
            evaluated = m_joint_evaluator.evaluate(
                now_us,
                reduced_candidate_sets,
                candidate_counts,
                static_cast<std::size_t>(m_params.total_agent_count),
                evaluation,
                legacy_rejoin_objective_ptr);
            combination_count = evaluation.combination_count;
            valid_combination_count = evaluation.valid_combination_count;
            safe_combination_count = evaluation.safe_combination_count;
            maximum_minimum_ad_m = evaluation.maximum_minimum_ad_m;
            if (evaluated) {
                best = evaluation.combinations[
                    evaluation.best_combination_index];
                best_combination_index = evaluation.best_combination_index;
            }
        }
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
                proposalChangesActiveCommand(
                    proposed_candidate_ids,
                    proposed_candidate_input_revisions,
                    proposed_candidate_valid_mask);
            JointCombinationEvaluation current_evaluation{};
            std::array<std::uint8_t, kMaximumSelectionAircraft>
                incumbent_candidate_ids{};
            const bool common_incumbent_available =
                m_params.execution_policy
                        != ManeuverExecutionPolicy::AmacAdThreshold
                    || buildCommonIncumbentCandidateIds(
                        incumbent_candidate_ids);
            bool current_evaluation_available = false;
            if (active_command_change && common_incumbent_available) {
                if (component_cutover
                    && m_epoch_pairwise_ad_certifications
                    && m_epoch_pairwise_ad_certifications->valid
                    && m_epoch_pairwise_ad_certifications->selection_epoch
                        == m_selection_epoch) {
                    // The incumbent and proposed tuples refer to the same
                    // frozen candidate library and selection timestamp. Reuse
                    // the already certified 7x7 pair matrices instead of
                    // propagating the incumbent's ten aircraft pairs again.
                    current_evaluation_available =
                        m_certified_component_evaluator.evaluateTuple(
                            *m_epoch_pairwise_ad_certifications,
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
            if (current_evaluation_valid
                && selection_rejoin_objective_ptr != nullptr) {
                current_evaluation.nominal_rejoin_cost = 0.0;
                for (int aircraft = 0;
                     aircraft < m_params.total_agent_count; ++aircraft) {
                    const std::size_t aircraft_index =
                        static_cast<std::size_t>(aircraft);
                    const auto begin = candidate_sets[aircraft_index].begin();
                    const auto end = begin + static_cast<std::ptrdiff_t>(
                        candidate_counts[aircraft_index]);
                    const std::uint8_t selected_id =
                        incumbent_candidate_ids[aircraft_index];
                    const auto found = std::find_if(
                        begin, end, [selected_id](const auto & candidate) {
                            return candidate.candidate_id == selected_id;
                        });
                    if (found == end) {
                        current_evaluation.nominal_rejoin_cost =
                            std::numeric_limits<double>::infinity();
                        break;
                    }
                    const double error = found->candidate_input.a_lat_cmd
                        - selection_rejoin_objective_ptr
                            ->nominal_lateral_acceleration_mps2[
                                aircraft_index];
                    current_evaluation.nominal_rejoin_cost += error * error;
                }
            }
            const bool mode_b_role_recertification = active_command_change
                && m_selected_v4_cutover
                && m_params.execution_policy
                    == ManeuverExecutionPolicy::ContinuousV4;
            const bool rejoin_improves = active_command_change
                && selection_rejoin_objective_ptr != nullptr
                && best.valid && best.all_pairs_feasible
                && (!current_evaluation_valid
                    || !current_evaluation.all_pairs_feasible
                    || (std::isfinite(best.nominal_rejoin_cost)
                        && std::isfinite(
                            current_evaluation.nominal_rejoin_cost)
                        && best.nominal_rejoin_cost
                            < current_evaluation.nominal_rejoin_cost
                                - 1.0e-12));
            const bool superiority_evaluated = active_command_change
                && (current_evaluation_available
                    || mode_b_role_recertification
                    || rejoin_improves);
            const bool clearly_superior = superiority_evaluated
                && m_params.active_switching_enabled
                && ((!current_evaluation_valid
                        && current_evaluation_available
                        && m_params.execution_policy
                            == ManeuverExecutionPolicy::AmacAdThreshold
                        && best.valid)
                    || (!current_evaluation_valid
                        && mode_b_role_recertification)
                    || rejoin_improves
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
            // but changing an active command still uses the same incumbent
            // persistence rule as the legacy evaluator. Both tuples are
            // compared on this epoch's frozen candidate library.
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
            m_pending_proposal.epoch = m_selection_epoch;
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
            } else if (!nominalPredictInput(
                    m_latest_nominal,
                    now_us,
                    m_params.v4_maximum_nominal_age_us,
                    m_pending_proposal.ownship_input)) {
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
