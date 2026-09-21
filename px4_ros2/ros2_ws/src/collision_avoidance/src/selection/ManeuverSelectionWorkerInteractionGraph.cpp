#include "ManeuverSelectionWorkerInternal.hpp"

#include <algorithm>
#include <chrono>

namespace collision_avoidance::selection
{
void ManeuverSelectionWorker::prepareInteractionGraph(ManeuverEvaluationRequest & request)
{
    auto & diagnostics = request.graph;
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
        // Partial reconstruction belongs to the remote worker. Diagnostics
        // here describe complete sets available to the selection state owner.
    }
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

    request.complete = frozen_library_complete;
    if (frozen_library_complete) {
        request.candidates = *m_epoch_certification_candidate_sets;
        request.counts = m_epoch_certification_candidate_counts;
    }
}
void ManeuverSelectionWorker::publishPendingInteractionGraphDiagnostics()
    noexcept
{
    if (!m_pending_interaction_graph_diagnostics) {
        return;
    }
    if (m_graph_records) {
        const auto wall = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        m_graph_records->append({static_cast<std::uint64_t>(wall),
                                 *m_pending_interaction_graph_diagnostics});
    }
    m_pending_interaction_graph_diagnostics.reset();
}


}  // namespace collision_avoidance::selection
