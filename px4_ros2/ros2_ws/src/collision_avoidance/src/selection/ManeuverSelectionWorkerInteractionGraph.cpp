#include "ManeuverSelectionWorkerInternal.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace collision_avoidance::selection
{
using namespace worker_detail;

void ManeuverSelectionWorker::evaluateInteractionGraph(
    std::uint64_t now_us)
{
    if (!m_params.interaction_graph_params.enabled) {
        return;
    }

    using Clock = std::chrono::steady_clock;
    const auto total_start = Clock::now();
    InteractionGraphDiagnostics diagnostics;
    diagnostics.vehicle_id = m_params.vehicle_id;
    diagnostics.enabled = true;
    diagnostics.dropped_ownship_belief_count =
        m_dropped_ownship_beliefs.load(std::memory_order_relaxed);
    diagnostics.dropped_remote_intent_count =
        m_dropped_remote_intents.load(std::memory_order_relaxed);
    diagnostics.dropped_remote_decision_count =
        m_dropped_remote_decisions.load(std::memory_order_relaxed);
    const std::size_t aircraft_count = static_cast<std::size_t>(
        m_params.total_agent_count);
    for (std::size_t aircraft = 0; aircraft < aircraft_count; ++aircraft) {
        if (m_epoch_certification_candidate_ready[aircraft]) {
            diagnostics.candidate_ready_mask |= (1U << aircraft);
            diagnostics.candidate_counts[aircraft] = static_cast<std::uint8_t>(
                m_epoch_certification_candidate_counts[aircraft]);
            if (m_epoch_certification_candidate_sets) {
                diagnostics.candidate_source_timestamps_us[aircraft] =
                    (*m_epoch_certification_candidate_sets)[aircraft][0]
                        .source_timestamp_us;
            }
            continue;
        }
        if (aircraft == static_cast<std::size_t>(m_params.vehicle_id)) {
            diagnostics.candidate_counts[aircraft] =
                static_cast<std::uint8_t>(m_ownship_candidate_count);
            if (m_ownship_candidate_count > 0) {
                diagnostics.candidate_source_timestamps_us[aircraft] =
                    m_ownship_candidates[0].source_timestamp_us;
            }
            continue;
        }
        const auto recordObservedCache = [
            this, &diagnostics, aircraft](const RemoteCandidateCache & cache) {
            if (cache.selection_epoch != m_selection_epoch
                || cache.count == 0
                || cache.source_timestamp_us
                    < diagnostics.candidate_source_timestamps_us[aircraft]) {
                return;
            }
            diagnostics.candidate_counts[aircraft] =
                static_cast<std::uint8_t>(cache.count);
            diagnostics.candidate_source_timestamps_us[aircraft] =
                cache.source_timestamp_us;
        };
        recordObservedCache(m_remote_caches[aircraft]);
        recordObservedCache(m_remote_staging_caches[aircraft]);
    }
    std::fill_n(
        diagnostics.assembled_candidate_ids.begin(),
        aircraft_count,
        kRollZeroId);
    diagnostics.assembled_candidate_valid_mask =
        candidateMaskForAircraftCount(aircraft_count);
    const bool frozen_library_complete =
        m_epoch_certification_candidate_sets != nullptr
        && std::all_of(
            m_epoch_certification_candidate_ready.begin(),
            m_epoch_certification_candidate_ready.begin()
                + static_cast<std::ptrdiff_t>(aircraft_count),
            [](bool ready) { return ready; })
        && std::all_of(
            m_epoch_certification_candidate_counts.begin(),
            m_epoch_certification_candidate_counts.begin()
                + static_cast<std::ptrdiff_t>(aircraft_count),
            [](std::size_t count) {
                return count == kExhaustiveCandidatesPerAircraft;
            });
    if (!frozen_library_complete) {
        diagnostics.graph.status =
            InteractionGraphStatus::InvalidCertification;
        diagnostics.graph.evaluation_timestamp_us = now_us;
        diagnostics.graph.selection_epoch = m_selection_epoch;
        diagnostics.graph.aircraft_count = aircraft_count;
        diagnostics.status =
            InteractionGraphEvaluationStatus::CandidateSetsIncomplete;
        diagnostics.total_evaluation_time_ns = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                Clock::now() - total_start).count());
        m_pending_interaction_graph_diagnostics =
            std::make_shared<InteractionGraphDiagnostics>(diagnostics);
        return;
    }
    const auto & certified_candidate_sets =
        *m_epoch_certification_candidate_sets;

    auto certifications = std::make_unique<PairwiseAdCertificationSet>();
    if (!m_pairwise_ad_certifier.evaluate(
            now_us,
            m_selection_epoch,
            m_params.interaction_graph_params.trajectory_library_version,
            m_params.interaction_graph_params.ad_masd_config_version,
            certified_candidate_sets,
            aircraft_count,
            *certifications)) {
        m_epoch_pairwise_ad_certifications.reset();
        diagnostics.graph.status =
            InteractionGraphStatus::InvalidCertification;
        diagnostics.graph.evaluation_timestamp_us = now_us;
        diagnostics.graph.selection_epoch = m_selection_epoch;
        diagnostics.graph.aircraft_count = aircraft_count;
        diagnostics.status =
            InteractionGraphEvaluationStatus::GraphInvalid;
        diagnostics.total_evaluation_time_ns = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                Clock::now() - total_start).count());
        m_pending_interaction_graph_diagnostics =
            std::make_shared<InteractionGraphDiagnostics>(diagnostics);
        return;
    }
    m_epoch_pairwise_ad_certifications = std::move(certifications);
    const PairwiseAdCertificationSet & certified_pairs =
        *m_epoch_pairwise_ad_certifications;

    diagnostics.graph = m_interaction_graph_builder.build(certified_pairs);
    if (!diagnostics.graph.valid()) {
        diagnostics.status = InteractionGraphEvaluationStatus::GraphInvalid;
        diagnostics.total_evaluation_time_ns = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                Clock::now() - total_start).count());
        m_pending_interaction_graph_diagnostics =
            std::make_shared<InteractionGraphDiagnostics>(diagnostics);
        return;
    }

    const auto search_start = Clock::now();
    bool component_search_valid = true;
    std::uint32_t actual_evaluation_count = 0;
    for (std::size_t component = 0;
        component < diagnostics.graph.component_count; ++component) {
        std::array<std::size_t, kMaximumSelectionAircraft> members{};
        std::size_t member_count = 0;
        for (std::size_t aircraft = 0; aircraft < aircraft_count;
             ++aircraft) {
            if (diagnostics.graph.component_ids[aircraft] == component) {
                members[member_count++] = aircraft;
            }
        }
        if (member_count == 1) {
            continue;
        }
        CertifiedComponentEvaluation component_evaluation;
        if (!m_certified_component_evaluator.evaluate(
                certified_pairs,
                certified_candidate_sets,
                members,
                member_count,
                component_evaluation)
            || !component_evaluation.has_best) {
            component_search_valid = false;
            break;
        }
        actual_evaluation_count += static_cast<std::uint32_t>(
            component_evaluation.combination_count);
        diagnostics.component_valid_evaluation_count +=
            component_evaluation.valid_combination_count;
        diagnostics.component_safe_evaluation_count +=
            component_evaluation.safe_combination_count;
        for (std::size_t member = 0; member < member_count; ++member) {
            const std::size_t aircraft = members[member];
            const std::uint8_t candidate_slot =
                component_evaluation.best_combination.candidate_slots[member];
            diagnostics.assembled_candidate_ids[aircraft] =
                certified_candidate_sets[aircraft][candidate_slot]
                    .candidate_id;
        }
    }
    diagnostics.component_search_time_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            Clock::now() - search_start).count());
    if (!component_search_valid
        || actual_evaluation_count
            != diagnostics.graph.component_evaluation_count) {
        diagnostics.status =
            InteractionGraphEvaluationStatus::ComponentEvaluationFailed;
        diagnostics.total_evaluation_time_ns = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                Clock::now() - total_start).count());
        m_pending_interaction_graph_diagnostics =
            std::make_shared<InteractionGraphDiagnostics>(diagnostics);
        return;
    }
    diagnostics.component_search_evaluated = true;
    diagnostics.assembled_candidate_hash = assembledCandidateHash(
        diagnostics.graph.graph_hash,
        diagnostics.assembled_candidate_ids,
        diagnostics.assembled_candidate_valid_mask,
        aircraft_count);
    diagnostics.component_solution_hash = assembledCandidateHash(
        diagnostics.graph.component_hash,
        diagnostics.assembled_candidate_ids,
        diagnostics.assembled_candidate_valid_mask,
        aircraft_count);

    const auto crosscheck_start = Clock::now();
    JointCombinationEvaluation crosscheck;
    diagnostics.global_crosscheck_evaluated =
        m_certified_component_evaluator.evaluateTuple(
            certified_pairs,
            certified_candidate_sets,
            diagnostics.assembled_candidate_ids,
            crosscheck);
    diagnostics.global_crosscheck_time_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            Clock::now() - crosscheck_start).count());
    if (diagnostics.global_crosscheck_evaluated) {
        bool cross_component_pairs_safe = true;
        for (std::size_t first = 0; first < aircraft_count; ++first) {
            for (std::size_t second = first + 1;
                 second < aircraft_count; ++second) {
                if (diagnostics.graph.component_ids[first]
                    == diagnostics.graph.component_ids[second]) {
                    continue;
                }
                const auto * pair = certified_pairs.findPair(first, second);
                const auto * evaluation = pair == nullptr ? nullptr
                    : pair->find(
                        crosscheck.candidate_slots[first],
                        crosscheck.candidate_slots[second]);
                if (evaluation == nullptr
                    || evaluation->validity != CombinationValidity::Valid
                    || !evaluation->feasible) {
                    cross_component_pairs_safe = false;
                    break;
                }
            }
            if (!cross_component_pairs_safe) {
                break;
            }
        }
        diagnostics.global_crosscheck_minimum_ad_m = crosscheck.minimum_ad_m;
        diagnostics.global_crosscheck_pass =
            crosscheck.valid && cross_component_pairs_safe;
        diagnostics.global_crosscheck_evaluation = crosscheck;
    }
    diagnostics.status = diagnostics.global_crosscheck_pass
        ? InteractionGraphEvaluationStatus::Evaluated
        : InteractionGraphEvaluationStatus::GlobalCrosscheckFailed;
    diagnostics.total_evaluation_time_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            Clock::now() - total_start).count());
    m_pending_interaction_graph_diagnostics =
        std::make_shared<InteractionGraphDiagnostics>(diagnostics);
}

void ManeuverSelectionWorker::publishPendingInteractionGraphDiagnostics()
    noexcept
{
    if (!m_pending_interaction_graph_diagnostics) {
        return;
    }
    const std::shared_ptr<const InteractionGraphDiagnostics> diagnostics =
        m_pending_interaction_graph_diagnostics;
    m_interaction_graph_diagnostics_queue.try_push(diagnostics);
    m_pending_interaction_graph_diagnostics.reset();
}


}  // namespace collision_avoidance::selection
