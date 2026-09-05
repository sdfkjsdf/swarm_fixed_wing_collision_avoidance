#include <collision_avoidance/selection/ManeuverSelectionWorker.hpp>
#include "ManeuverSelectionWorkerInternal.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>
#include <limits>
#include <memory>
#include <thread>

namespace collision_avoidance::selection
{
using namespace worker_detail;

ManeuverSelectionWorker::ManeuverSelectionWorker(
    const ManeuverSelectionWorkerParams & params)
: m_params(params),
  m_predictor(params.predictor_params),
  m_candidate_table(estimation::makeLevelTurnCandidateTable(
      params.ground_speed_command_mps,
      std::numeric_limits<double>::quiet_NaN(),
      params.gravity_mps2)),
  m_sender(m_predictor, m_candidate_table),
  m_receiver(m_predictor, params.uncertainty_params),
  m_uncertainty(params.uncertainty_params),
  m_pair_evaluator(params.evaluator_params),
  m_barrier_evaluator(params.evaluator_params),
  m_joint_evaluator(params.evaluator_params),
  m_exhaustive_evaluator(params.evaluator_params),
  m_pairwise_ad_certifier(params.evaluator_params),
  m_interaction_graph_builder(params.interaction_graph_params),
  m_activation_controller(params.activation_params),
  m_v4_safe_control(params.v4_safe_control_params),
  m_mode_b_interpolator(params.mode_b_interpolator_params),
  m_mode_b_intent_adapter(params.mode_b_intent_adapter_params),
  m_v4_candidate_adapter(params.v4_candidate_adapter_params)
{
    if (m_params.formation_discrimination_enabled) {
        m_formation_discriminator.emplace(
            m_params.formation_boundary_config);
    }
    m_selected_candidate_ids.fill(0U);
    m_latest_selection_decision.selected_candidate_ids.fill(0U);
    m_latest_selection_decision.proposed_candidate_ids.fill(0U);
}

ManeuverSelectionWorker::~ManeuverSelectionWorker()
{
    stop();
}

bool ManeuverSelectionWorker::start()
{
    if (!validParams(m_params)) {
        return false;
    }
    bool expected = false;
    if (!m_running.compare_exchange_strong(expected, true)) {
        return false;
    }
    m_thread = std::thread(&ManeuverSelectionWorker::workerLoop, this);
    return true;
}

void ManeuverSelectionWorker::stop()
{
    m_running.store(false, std::memory_order_release);
    if (m_thread.joinable()) {
        m_thread.join();
    }
}

bool ManeuverSelectionWorker::running() const noexcept
{
    return m_running.load(std::memory_order_acquire);
}

bool ManeuverSelectionWorker::pushOwnshipBelief(
    const ManeuverSelectionBeliefSnapshot & snapshot) noexcept
{
    WorkerInput input;
    input.kind = InputKind::OwnshipBelief;
    input.belief = snapshot;
    if (!m_input_queue.try_push(input)) {
        m_dropped_inputs.fetch_add(1, std::memory_order_relaxed);
        m_dropped_ownship_beliefs.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    return true;
}

bool ManeuverSelectionWorker::pushAirspeed(
    const ManeuverSelectionAirspeedSnapshot & snapshot) noexcept
{
    WorkerInput input;
    input.kind = InputKind::Airspeed;
    input.airspeed = snapshot;
    if (!m_input_queue.try_push(input)) {
        m_dropped_inputs.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    return true;
}

bool ManeuverSelectionWorker::pushNominalSetpoint(
    const ManeuverSelectionNominalSetpointSnapshot & snapshot) noexcept
{
    WorkerInput input;
    input.kind = InputKind::NominalSetpoint;
    input.nominal = snapshot;
    if (!m_input_queue.try_push(input)) {
        m_dropped_inputs.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    return true;
}

bool ManeuverSelectionWorker::pushRemoteIntent(
    int remote_vehicle_id,
    const estimation::TrajectoryIntentPacket & packet) noexcept
{
    WorkerInput input;
    input.kind = InputKind::RemoteIntent;
    input.remote_vehicle_id = remote_vehicle_id;
    input.packet = packet;
    if (!m_input_queue.try_push(input)) {
        m_dropped_inputs.fetch_add(1, std::memory_order_relaxed);
        m_dropped_remote_intents.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    return true;
}

bool ManeuverSelectionWorker::pushRemoteDecision(
    int remote_vehicle_id,
    const ManeuverSelectionPeerDecision & decision) noexcept
{
    WorkerInput input;
    input.kind = InputKind::RemoteDecision;
    input.remote_vehicle_id = remote_vehicle_id;
    input.decision = decision;
    if (!m_input_queue.try_push(input)) {
        m_dropped_inputs.fetch_add(1, std::memory_order_relaxed);
        m_dropped_remote_decisions.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    return true;
}

void ManeuverSelectionWorker::setActivationEnabled(bool enabled) noexcept
{
    m_activation_enabled.store(enabled, std::memory_order_release);
}

std::optional<ManeuverSelectionWorkerOutput>
ManeuverSelectionWorker::tryPopOutput() noexcept
{
    return m_output_queue.try_pop();
}

std::optional<std::shared_ptr<const InteractionGraphDiagnostics>>
ManeuverSelectionWorker::tryPopInteractionGraphDiagnostics() noexcept
{
    return m_interaction_graph_diagnostics_queue.try_pop();
}

bool ManeuverSelectionWorker::processPendingForTest()
{
    if (running() || !validParams(m_params)) {
        return false;
    }
    return processPending();
}

std::uint64_t ManeuverSelectionWorker::droppedInputCount() const noexcept
{
    return m_dropped_inputs.load(std::memory_order_relaxed);
}

std::uint64_t ManeuverSelectionWorker::droppedOutputCount() const noexcept
{
    return m_dropped_outputs.load(std::memory_order_relaxed);
}

std::size_t ManeuverSelectionWorker::activeCandidateCount() const noexcept
{
    return m_params.exhaustive_test_mode
        ? kExhaustiveCandidatesPerAircraft
        : kCandidatesPerAircraft;
}

bool ManeuverSelectionWorker::v4CutoverMode() const noexcept
{
    return m_params.v4_safe_control_enabled && !m_params.v4_shadow_only;
}

bool ManeuverSelectionWorker::allV4CutoverParticipantsReady(
    const ManeuverSelectionDecision & local_decision) const noexcept
{
    if (!v4CutoverMode() || !v4CutoverCandidateReady(local_decision)) {
        return false;
    }
    for (int aircraft = 0;
         aircraft < m_params.total_agent_count; ++aircraft) {
        if (aircraft == m_params.vehicle_id) {
            continue;
        }
        const RemoteDecisionCache & peer = m_remote_decision_caches[
            static_cast<std::size_t>(aircraft)];
        if (!peer.valid
            || !peer.decision.coordination_qualified
            || !peer.decision.v4_cutover_candidate_ready
            || peer.decision.v4_control_architecture
                != m_params.v4_control_architecture) {
            return false;
        }
    }
    return true;
}

void ManeuverSelectionWorker::workerLoop()
{
    using namespace std::chrono_literals;
    while (m_running.load(std::memory_order_acquire)) {
        processPending();
        std::this_thread::sleep_for(1ms);
    }
    processPending();
}

bool ManeuverSelectionWorker::processPending()
{
    bool consumed_input = false;
    while (const auto input = m_input_queue.try_pop()) {
        consumed_input = true;
        if (input->kind == InputKind::OwnshipBelief) {
            acceptOwnshipBelief(input->belief);
        } else if (input->kind == InputKind::Airspeed) {
            acceptAirspeed(input->airspeed);
        } else if (input->kind == InputKind::NominalSetpoint) {
            acceptNominalSetpoint(input->nominal);
        } else if (input->kind == InputKind::RemoteIntent) {
            acceptRemoteIntent(input->remote_vehicle_id, input->packet);
        } else {
            acceptRemoteDecision(input->remote_vehicle_id, input->decision);
        }
    }

    if (!m_has_latest_state) {
        return consumed_input;
    }

    const std::uint64_t now_us = m_latest_state_timestamp_us;
    if (!m_candidate_set_initialized) {
        initializeCandidateSet(now_us);
    }

    ManeuverSelectionWorkerOutput output;
    output.generated_timestamp_us = now_us;
    output.selection_epoch = m_selection_epoch;

    const bool selection_due = !m_epoch_evaluated
        && now_us >= m_epoch_generation_timestamp_us
        && now_us - m_epoch_generation_timestamp_us
            >= m_params.coordination_delay_us;
    if (selection_due) {
        const std::uint64_t common_evaluation_timestamp_us =
            m_epoch_generation_timestamp_us
            + m_params.coordination_delay_us;
        evaluateCurrentSet(common_evaluation_timestamp_us, output);
        m_epoch_evaluated = true;
    }

    if (now_us >= m_next_candidate_refresh_timestamp_us) {
        refreshCandidateSet(now_us);
        output.selection_epoch = m_selection_epoch;
    }

    bool trajectory_refreshed = false;
    if (now_us >= m_next_trajectory_refresh_timestamp_us) {
        if (!v4CutoverMode() || !m_v4_cutover_ready) {
            buildCurrentIntentSet(now_us, output);
        }
        if (m_params.v4_safe_control_enabled) {
            evaluateV4(now_us, output);
        }
        if (v4CutoverMode()) {
            const bool v4_candidates_valid = output.has_decision
                && v4CutoverCandidateReady(output.decision);
            if (v4_candidates_valid
                && (!m_v4_epoch_candidates_valid
                    || m_v4_epoch_candidate_selection_epoch
                        != m_selection_epoch)) {
                // Candidate commands are a 4 Hz selection-epoch snapshot.
                // Their trajectories are still regenerated from the latest
                // belief at 20 Hz below, but the command revisions must stay
                // fixed long enough for distributed proposal consensus.
                m_v4_epoch_candidates = output.decision.v4_candidates;
                m_v4_epoch_candidate_selection_epoch = m_selection_epoch;
                m_v4_epoch_candidates_valid = true;
            }
            if (!m_v4_cutover_ready
                && v4_candidates_valid
                && allV4CutoverParticipantsReady(output.decision)) {
                // Phase 1 keeps publishing the selected legacy bootstrap
                // intent. Only after every participant has independently
                // advertised a usable V4 set do we enter phase 2 and publish
                // V4 intents for the existing proposal/consensus path.
                m_v4_cutover_ready = true;
            }
            const bool active = m_activation_controller.status().active;
            const bool retain_selected_v4 =
                m_has_selected_combination && m_selected_v4_cutover;
            const bool epoch_candidates_available =
                m_v4_epoch_candidates_valid
                && m_v4_epoch_candidate_selection_epoch
                    == m_selection_epoch;
            if (m_v4_cutover_ready
                && (v4_candidates_valid || active || retain_selected_v4)) {
                buildV4IntentSet(
                    now_us,
                    v4_candidates_valid && epoch_candidates_available
                        ? m_v4_epoch_candidates
                        : output.decision.v4_candidates,
                    output);
            } else if (m_v4_cutover_ready) {
                // Once cut over, never fall back to a legacy/best-unsafe set.
                m_ownship_candidates_complete = false;
                m_ownship_candidate_count = 0;
                output.intent_packet_count = 0;
            }
        }
        trajectory_refreshed = true;
        do {
            m_next_trajectory_refresh_timestamp_us +=
                m_params.trajectory_refresh_period_us;
        } while (m_next_trajectory_refresh_timestamp_us <= now_us);
    }

    const bool coordination_committed = finalizePendingCoordination(output);
    if ((selection_due || trajectory_refreshed || coordination_committed)
        && m_has_selected_combination) {
        // AMAC peers use the current decision message for post-release safety
        // acknowledgement and selected-intent awareness. Publish that state
        // with the 20 Hz trajectory refresh; an unrelated V4 shadow evaluator
        // must never be the mechanism that supplies this heartbeat.
        updateActivationState(
            now_us,
            selection_due || trajectory_refreshed || coordination_committed,
            output);
    }

    if (output.has_decision) {
        output.decision.command_execution_requested =
            m_activation_enabled.load(std::memory_order_acquire)
            && maneuverCommandExecutionRequested(
                m_params.execution_policy, output.decision);
        m_latest_selection_decision.command_execution_requested =
            output.decision.command_execution_requested;
    }
    if (output.intent_packet_count > 0 || output.has_decision) {
        publishOutput(output);
    }
    return consumed_input;
}

bool ManeuverSelectionWorker::acceptOwnshipBelief(
    const ManeuverSelectionBeliefSnapshot & snapshot)
{
    if (!snapshot.valid || snapshot.timestamp_us < snapshot.timestamp_sample_us
        || snapshot.timestamp_us < m_latest_state_timestamp_us) {
        return false;
    }
    const std::uint64_t delay_us =
        snapshot.timestamp_us - snapshot.timestamp_sample_us;
    if (delay_us > m_params.maximum_belief_delay_us) {
        return false;
    }

    estimation::PredictState state;
    estimation::PredictStateCovariance covariance;
    if (!m_uncertainty.initializeFromEstimatorBelief(
            snapshot.belief, state, covariance)) {
        return false;
    }

    const estimation::PredictInput * compensation_input =
        m_candidate_table.find(m_current_best_id);
    if (compensation_input == nullptr
        || !m_uncertainty.compensateFusionHorizonDelay(
            m_predictor,
            *compensation_input,
            static_cast<double>(delay_us) * 1.0e-6,
            state,
            covariance)) {
        return false;
    }

    m_latest_state = state;
    m_latest_covariance = covariance;
    m_latest_state_timestamp_us = snapshot.timestamp_us;
    m_has_latest_state = true;
    return true;
}

bool ManeuverSelectionWorker::acceptAirspeed(
    const ManeuverSelectionAirspeedSnapshot & snapshot)
{
    if (m_has_latest_airspeed
        && snapshot.timestamp_us < m_latest_airspeed.timestamp_us) {
        return false;
    }
    m_latest_airspeed = snapshot;
    m_has_latest_airspeed = true;
    return true;
}

bool ManeuverSelectionWorker::acceptNominalSetpoint(
    const ManeuverSelectionNominalSetpointSnapshot & snapshot)
{
    if (m_has_latest_nominal
        && snapshot.timestamp_us < m_latest_nominal.timestamp_us) {
        return false;
    }
    m_latest_nominal = snapshot;
    m_has_latest_nominal = true;
    return true;
}

bool ManeuverSelectionWorker::acceptRemoteIntent(
    int remote_vehicle_id,
    const estimation::TrajectoryIntentPacket & packet)
{
    if (remote_vehicle_id < 0
        || remote_vehicle_id >= m_params.total_agent_count
        || remote_vehicle_id == m_params.vehicle_id) {
        return false;
    }
    RemoteCandidateCache & remote_cache =
        m_remote_caches[static_cast<std::size_t>(remote_vehicle_id)];
    RemoteCandidateCache & staging_cache =
        m_remote_staging_caches[static_cast<std::size_t>(remote_vehicle_id)];

    const auto keyLess = [](
                             std::uint64_t lhs_epoch,
                             std::uint64_t lhs_timestamp,
                             std::uint64_t rhs_epoch,
                             std::uint64_t rhs_timestamp) {
        return lhs_epoch < rhs_epoch
            || (lhs_epoch == rhs_epoch && lhs_timestamp < rhs_timestamp);
    };

    const std::size_t required_candidate_count = packet.candidate_set_size;
    const bool set_metadata_valid = required_candidate_count > 0
        && required_candidate_count <= kExhaustiveCandidatesPerAircraft
        && (packet.candidate_set_kind
                == estimation::CandidateSetKind::LegacyRoll
            || packet.candidate_set_kind
                == estimation::CandidateSetKind::V4SafeControl)
        && ((packet.candidate_set_kind
                    == estimation::CandidateSetKind::LegacyRoll
                && required_candidate_count == activeCandidateCount())
            || (packet.candidate_set_kind
                    == estimation::CandidateSetKind::V4SafeControl
                && required_candidate_count
                    <= kMaximumSafeControlCandidates
                && packet.candidate_id
                    < kMaximumSafeControlCandidates));
    if (!set_metadata_valid) {
        return false;
    }
    if (remote_cache.count == remote_cache.expected_count
        && remote_cache.count > 0
        && keyLess(
            packet.selection_epoch,
            packet.source_timestamp_us,
            remote_cache.selection_epoch,
            remote_cache.source_timestamp_us)) {
        return false;
    }

    estimation::ReceivedTrajectoryIntent received;
    if (!m_receiver.receive(packet, received)) {
        return false;
    }

    const bool staging_key_matches =
        staging_cache.selection_epoch == packet.selection_epoch
        && staging_cache.source_timestamp_us
            == packet.source_timestamp_us
        && staging_cache.candidate_set_kind == packet.candidate_set_kind
        && staging_cache.expected_count == required_candidate_count;
    if (!staging_key_matches) {
        if (staging_cache.count > 0
            && keyLess(
                packet.selection_epoch,
                packet.source_timestamp_us,
                staging_cache.selection_epoch,
                staging_cache.source_timestamp_us)) {
            return false;
        }
        staging_cache = RemoteCandidateCache{};
        staging_cache.selection_epoch = packet.selection_epoch;
        staging_cache.source_timestamp_us = packet.source_timestamp_us;
        staging_cache.candidate_set_kind = packet.candidate_set_kind;
        staging_cache.expected_count = required_candidate_count;
    }

    for (std::size_t index = 0;
         index < staging_cache.candidates.size(); ++index) {
        if (staging_cache.occupied[index]
            && staging_cache.candidates[index].candidate_id
                == packet.candidate_id) {
            staging_cache.candidates[index] = received;
            return true;
        }
    }
    for (std::size_t index = 0;
         index < staging_cache.candidates.size(); ++index) {
        if (!staging_cache.occupied[index]) {
            staging_cache.occupied[index] = true;
            staging_cache.candidates[index] = received;
            ++staging_cache.count;
            if (staging_cache.count == required_candidate_count) {
                std::sort(
                    staging_cache.candidates.begin(),
                    staging_cache.candidates.begin()
                        + static_cast<std::ptrdiff_t>(required_candidate_count),
                    [](const auto & lhs, const auto & rhs) {
                        return lhs.candidate_id < rhs.candidate_id;
                    });
                const bool current_complete = remote_cache.count > 0
                    && remote_cache.count == remote_cache.expected_count;
                const bool set_key_changed =
                    remote_cache.selection_epoch
                            != staging_cache.selection_epoch
                    || remote_cache.source_timestamp_us
                            != staging_cache.source_timestamp_us
                    || remote_cache.candidate_set_kind
                            != staging_cache.candidate_set_kind;
                if (current_complete && set_key_changed) {
                    RemoteCandidateCache & previous_cache =
                        m_remote_previous_caches[
                            static_cast<std::size_t>(remote_vehicle_id)];
                    RemoteSelectedIntentCache & selected_cache =
                        m_remote_selected_caches[
                            static_cast<std::size_t>(remote_vehicle_id)];
                    const RemoteDecisionCache & peer =
                        m_remote_decision_caches[
                            static_cast<std::size_t>(remote_vehicle_id)];
                    const auto findPeerSelection = [remote_vehicle_id, &peer](
                                                       const RemoteCandidateCache & cache)
                        -> const estimation::ReceivedTrajectoryIntent * {
                        if (!peer.valid
                            || !peer.decision.coordination_qualified
                            || !peer.decision.ownship_candidate_valid) {
                            return nullptr;
                        }
                        const std::size_t peer_index =
                            static_cast<std::size_t>(remote_vehicle_id);
                        const std::uint8_t selected_id =
                            peer.decision.ownship_candidate_id;
                        const std::uint64_t selected_revision =
                            peer.decision
                                .selected_candidate_input_revisions[peer_index];
                        const auto found = std::find_if(
                            cache.candidates.begin(),
                            cache.candidates.begin()
                                + static_cast<std::ptrdiff_t>(cache.count),
                            [selected_id, selected_revision](
                                const auto & candidate) {
                                return candidate.candidate_id == selected_id
                                    && candidate.candidate_input_revision
                                        == selected_revision;
                            });
                        return found == cache.candidates.begin()
                                + static_cast<std::ptrdiff_t>(cache.count)
                            ? nullptr
                            : &(*found);
                    };
                    previous_cache = remote_cache;
                    if (const auto * selected =
                            findPeerSelection(remote_cache)) {
                        selected_cache.intent = *selected;
                        selected_cache.valid = true;
                    }
                }
                remote_cache = staging_cache;
                freezeRemoteCertificationCandidatesForCurrentEpoch(
                    remote_vehicle_id);
                const RemoteDecisionCache & peer =
                    m_remote_decision_caches[
                        static_cast<std::size_t>(remote_vehicle_id)];
                if (peer.valid && peer.decision.coordination_qualified
                    && peer.decision.ownship_candidate_valid) {
                    const std::size_t peer_index =
                        static_cast<std::size_t>(remote_vehicle_id);
                    const std::uint8_t selected_id =
                        peer.decision.ownship_candidate_id;
                    const std::uint64_t selected_revision =
                        peer.decision.selected_candidate_input_revisions[
                            peer_index];
                    const auto selected = std::find_if(
                        remote_cache.candidates.begin(),
                        remote_cache.candidates.begin()
                            + static_cast<std::ptrdiff_t>(remote_cache.count),
                        [selected_id, selected_revision](const auto & candidate) {
                            return candidate.candidate_id == selected_id
                                && candidate.candidate_input_revision
                                    == selected_revision;
                        });
                    if (selected != remote_cache.candidates.begin()
                            + static_cast<std::ptrdiff_t>(remote_cache.count)) {
                        m_remote_selected_caches[peer_index].intent = *selected;
                        m_remote_selected_caches[peer_index].valid = true;
                    }
                }
            }
            return true;
        }
    }
    return false;
}

void ManeuverSelectionWorker::freezeRemoteCertificationCandidatesForCurrentEpoch(
    int remote_vehicle_id)
{
    if (!m_params.interaction_graph_params.enabled
        || remote_vehicle_id < 0
        || remote_vehicle_id >= m_params.total_agent_count
        || remote_vehicle_id == m_params.vehicle_id) {
        return;
    }
    const std::uint64_t delivery_budget_us =
        3 * m_params.trajectory_refresh_period_us;
    const std::uint64_t freeze_offset_us =
        m_params.coordination_delay_us > delivery_budget_us
        ? m_params.coordination_delay_us - delivery_budget_us
        : 0;
    const std::uint64_t freeze_cutoff_timestamp_us =
        m_epoch_generation_timestamp_us + freeze_offset_us;
    const std::size_t remote_index = static_cast<std::size_t>(
        remote_vehicle_id);
    const RemoteCandidateCache * selected_cache = nullptr;
    for (const RemoteCandidateCache * cache : {
            &m_remote_caches[remote_index],
            &m_remote_previous_caches[remote_index]}) {
        if (cache->selection_epoch != m_selection_epoch
            || cache->candidate_set_kind
                != estimation::CandidateSetKind::LegacyRoll
            || cache->count != kExhaustiveCandidatesPerAircraft
            || cache->source_timestamp_us > freeze_cutoff_timestamp_us) {
            continue;
        }
        if (selected_cache == nullptr
            || cache->source_timestamp_us
                > selected_cache->source_timestamp_us) {
            selected_cache = cache;
        }
    }
    if (selected_cache == nullptr) {
        return;
    }
    if (!m_epoch_certification_candidate_sets) {
        m_epoch_certification_candidate_sets = std::make_unique<
            MultiAircraftExhaustiveCandidateIntentSets>();
    }
    if (m_epoch_certification_candidate_ready[remote_index]
        && selected_cache->source_timestamp_us
            <= (*m_epoch_certification_candidate_sets)[remote_index][0]
                .source_timestamp_us) {
        return;
    }
    (*m_epoch_certification_candidate_sets)[remote_index] =
        selected_cache->candidates;
    m_epoch_certification_candidate_counts[remote_index] =
        selected_cache->count;
    m_epoch_certification_candidate_ready[remote_index] = true;
}

bool ManeuverSelectionWorker::acceptRemoteDecision(
    int remote_vehicle_id,
    const ManeuverSelectionPeerDecision & decision)
{
    if (remote_vehicle_id < 0
        || remote_vehicle_id >= m_params.total_agent_count
        || remote_vehicle_id == m_params.vehicle_id
        || decision.vehicle_id != remote_vehicle_id) {
        return false;
    }
    const auto candidatesValid = [this](
                                      const std::array<
                                          std::uint8_t,
                                          kMaximumSelectionAircraft> & ids,
                                      std::uint32_t valid_mask) {
        for (int aircraft = 0;
             aircraft < m_params.total_agent_count; ++aircraft) {
            const std::size_t aircraft_index =
                static_cast<std::size_t>(aircraft);
            if (candidateIsValid(valid_mask, aircraft_index)
                && ids[aircraft_index]
                >= estimation::kManeuverCandidateCount) {
                return false;
            }
        }
        return true;
    };
    const std::uint32_t allowed_mask = candidateMaskForAircraftCount(
        static_cast<std::size_t>(m_params.total_agent_count));
    if ((decision.selected_candidate_valid_mask & ~allowed_mask) != 0U
        || (decision.proposed_candidate_valid_mask & ~allowed_mask) != 0U
        || (decision.ownship_candidate_valid
            && decision.ownship_candidate_id
                >= estimation::kManeuverCandidateCount)
        || (decision.v4_control_architecture
                != V4ControlArchitecture::LegacySafeControlSet
            && decision.v4_control_architecture
                != V4ControlArchitecture::ClosedFormBackupModeB)
        || !candidatesValid(
            decision.selected_candidate_ids,
            decision.selected_candidate_valid_mask)
        || (decision.proposal_valid
            && !candidatesValid(
                decision.proposed_candidate_ids,
                decision.proposed_candidate_valid_mask))
        || (decision.proposal_valid
            && decision.proposed_component_graph
            && (decision.proposed_candidate_library_hash == 0
                || decision.proposed_graph_hash == 0
                || decision.proposed_component_hash == 0
                || decision.proposed_component_solution_hash == 0))) {
        return false;
    }
    const auto v4RolesValid = [this](
                                      const std::array<
                                          std::uint8_t,
                                          kMaximumSelectionAircraft> & ids) {
        for (int aircraft = 0;
             aircraft < m_params.total_agent_count; ++aircraft) {
            if (ids[static_cast<std::size_t>(aircraft)]
                >= kMaximumSafeControlCandidates) {
                return false;
            }
        }
        return true;
    };
    if ((decision.selected_v4_cutover
            && !v4RolesValid(decision.selected_candidate_ids))
        || (decision.proposal_valid && decision.proposed_v4_cutover
            && !v4RolesValid(decision.proposed_candidate_ids))) {
        return false;
    }
    const std::size_t remote_index = static_cast<std::size_t>(
        remote_vehicle_id);
    const bool selected_ownship_bit = candidateIsValid(
        decision.selected_candidate_valid_mask, remote_index);
    if (selected_ownship_bit != decision.ownship_candidate_valid
        || (decision.coordination_qualified
            && (!selected_ownship_bit
                || decision.selected_candidate_ids[remote_index]
                    != decision.ownship_candidate_id
                || decision.selected_candidate_input_revisions[remote_index]
                    == 0))) {
        return false;
    }

    RemoteDecisionCache & cache = m_remote_decision_caches[
        static_cast<std::size_t>(remote_vehicle_id)];
    if (cache.valid
        && decision.proposal_epoch < cache.decision.proposal_epoch) {
        return false;
    }
    const bool retain_bootstrap_readiness = cache.valid
        && cache.decision.v4_cutover_candidate_ready
        && !cache.decision.selected_v4_cutover
        && !decision.selected_v4_cutover
        && cache.decision.v4_control_architecture
            == decision.v4_control_architecture;
    if (decision.activation_just_started) {
        // Preserve the edge until the next local 20 Hz belief update. A newer
        // peer heartbeat may otherwise overwrite this one-shot event before
        // the component activation roll-up consumes it.
        cache.activation_start_pending = true;
    }
    cache.decision = decision;
    // Readiness advertises that this peer has demonstrated the selected V4
    // architecture, not that its latest 20 Hz diagnostic sample is a command.
    // Retain that capability across transient missing/stale intent samples.
    // The local candidate must still be valid at the instant the barrier opens,
    // and normal proposal consensus still gates actual command execution.
    cache.decision.v4_cutover_candidate_ready =
        decision.v4_cutover_candidate_ready || retain_bootstrap_readiness;
    cache.valid = true;
    if (m_params.execution_policy
        == ManeuverExecutionPolicy::HorizonGatedV4) {
        rollupV4HorizonGate(m_latest_state_timestamp_us);
    }
    if (decision.coordination_qualified
        && decision.ownship_candidate_valid) {
        const auto find_selected = [remote_index, &decision](
                                       const RemoteCandidateCache & candidate_cache)
            -> const estimation::ReceivedTrajectoryIntent * {
            if (candidate_cache.count == 0
                || candidate_cache.count != candidate_cache.expected_count) {
                return nullptr;
            }
            const auto found = std::find_if(
                candidate_cache.candidates.begin(),
                candidate_cache.candidates.begin()
                    + static_cast<std::ptrdiff_t>(candidate_cache.count),
                [remote_index, &decision](const auto & candidate) {
                    return candidate.candidate_id
                            == decision.ownship_candidate_id
                        && candidate.candidate_input_revision
                            == decision.selected_candidate_input_revisions[
                                remote_index];
                });
            return found == candidate_cache.candidates.begin()
                    + static_cast<std::ptrdiff_t>(candidate_cache.count)
                ? nullptr
                : &(*found);
        };
        for (const RemoteCandidateCache * candidate_cache : {
                 &m_remote_caches[remote_index],
                 &m_remote_previous_caches[remote_index]}) {
            if (const auto * selected = find_selected(*candidate_cache)) {
                m_remote_selected_caches[remote_index].intent = *selected;
                m_remote_selected_caches[remote_index].valid = true;
                break;
            }
        }
    }
    return true;
}

bool ManeuverSelectionWorker::publishOutput(
    const ManeuverSelectionWorkerOutput & output) noexcept
{
    if (!m_output_queue.try_push(output)) {
        m_dropped_outputs.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    return true;
}

const char * v4ShadowEvaluationStatusName(
    V4ShadowEvaluationStatus status) noexcept
{
    switch (status) {
    case V4ShadowEvaluationStatus::Disabled:
        return "disabled";
    case V4ShadowEvaluationStatus::MissingPeerDecision:
        return "missing_peer_decision";
    case V4ShadowEvaluationStatus::MissingPeerIntent:
        return "missing_peer_intent";
    case V4ShadowEvaluationStatus::FuturePeerIntent:
        return "future_peer_intent";
    case V4ShadowEvaluationStatus::StalePeerIntent:
        return "stale_peer_intent";
    case V4ShadowEvaluationStatus::InvalidPeerIntent:
        return "invalid_peer_intent";
    case V4ShadowEvaluationStatus::CoreEvaluated:
        return "core_evaluated";
    case V4ShadowEvaluationStatus::CandidateGenerationFailed:
        return "candidate_generation_failed";
    }
    return "unknown";
}

const char * v4SnapshotStatusName(V4SnapshotStatus status) noexcept
{
    switch (status) {
    case V4SnapshotStatus::Missing:
        return "missing";
    case V4SnapshotStatus::Valid:
        return "valid";
    case V4SnapshotStatus::Invalid:
        return "invalid";
    case V4SnapshotStatus::Future:
        return "future";
    case V4SnapshotStatus::Stale:
        return "stale";
    }
    return "unknown";
}

const char * v4AirspeedSourceName(V4AirspeedSource source) noexcept
{
    switch (source) {
    case V4AirspeedSource::Unavailable:
        return "unavailable";
    case V4AirspeedSource::ActualTas:
        return "actual_tas";
    case V4AirspeedSource::TrimFallback:
        return "trim_fallback";
    }
    return "unknown";
}

}  // namespace collision_avoidance::selection
