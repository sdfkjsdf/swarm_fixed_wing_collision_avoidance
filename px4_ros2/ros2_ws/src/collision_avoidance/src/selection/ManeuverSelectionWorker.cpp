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

namespace
{
std::uint64_t steadyNowNs() noexcept
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

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
  m_v4_safe_control(params.v4_safe_control_params),
  m_mode_b_interpolator(params.mode_b_interpolator_params),
  m_mode_b_intent_adapter(params.mode_b_intent_adapter_params),
  m_v4_candidate_adapter(params.v4_candidate_adapter_params),
  m_evaluation_worker(params.evaluator_params, params.interaction_graph_params,
      params.predictor_params, params.uncertainty_params),
  m_remote_trajectory_worker(params.vehicle_id, params.total_agent_count,
      params.exhaustive_test_mode ? kExhaustiveCandidatesPerAircraft : kCandidatesPerAircraft,
      params.predictor_params, params.uncertainty_params, params.stopped_stage_timing_enabled),
  m_input_storage(std::make_unique<InputStorage>())
{
    if (m_params.interaction_graph_params.enabled) {
        m_epoch_certification_candidate_sets = std::make_unique<
            MultiAircraftExhaustiveCandidateIntentSets>();
    }
    if (m_params.stopped_stage_timing_enabled) {
        m_stopped_stage_timing = std::make_unique<StoppedStageTiming>();
    }
    if (m_params.masd_diagnostics_enabled) {
        m_budget_records = std::make_unique<StoppedBudgetRecords>();
        if (m_params.interaction_graph_params.enabled)
            m_graph_records = std::make_unique<StoppedGraphRecords>();
    }
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
    try {
        m_evaluation_worker.start();
        m_remote_trajectory_worker.start();
        m_thread = std::thread(&ManeuverSelectionWorker::workerLoop, this);
    } catch (...) {
        m_running.store(false, std::memory_order_release);
        m_evaluation_worker.stop();
        m_remote_trajectory_worker.stop();
        throw;
    }
    return true;
}

void ManeuverSelectionWorker::stop()
{
    m_running.store(false, std::memory_order_release);
    if (m_thread.joinable()) {
        m_thread.join();
    }
    m_evaluation_worker.stop();
    m_remote_trajectory_worker.stop();
}

bool ManeuverSelectionWorker::running() const noexcept
{
    return m_running.load(std::memory_order_acquire);
}

void ManeuverSelectionWorker::stopAndWriteStageTiming(std::ostream & out)
{
    stop();
    if (m_stopped_stage_timing) {
        m_stopped_stage_timing->write(out, m_params.vehicle_id, 2);
        out << "[stop-selection-worker]," << m_params.vehicle_id << ','
            << m_selection_submitted.load() << ',' << m_selection_completed.load() << ','
            << m_selection_busy.load() << ',' << m_selection_expired.load() << '\n';
        m_remote_trajectory_worker.writeStoppedStatistics(out);
    }
}

bool ManeuverSelectionWorker::enqueueInput(const WorkerInput & input) noexcept
{
    std::size_t partition = 0;
    if (input.kind == InputKind::RemoteDecision) {
        const int peer = input.remote_vehicle_id;
        if (peer < 0 || peer >= m_params.total_agent_count || peer == m_params.vehicle_id) {
            return false;
        }
        partition = static_cast<std::size_t>(peer < m_params.vehicle_id ? peer + 1 : peer);
    }
    return m_input_storage->inbox.try_push(partition, input);
}

bool ManeuverSelectionWorker::pushOwnshipBelief(
    const ManeuverSelectionBeliefSnapshot & snapshot,
    const BeliefArrivalTiming & arrival) noexcept
{
    WorkerInput input;
    input.kind = InputKind::OwnshipBelief;
    input.belief = snapshot;
    // This arrival anchor drives scheduling, independently of diagnostics.
    // Retain the common-state clock; do not substitute the Pi's wall clock.
    input.belief_enqueue_ns = steadyNowNs();
    if (m_stopped_stage_timing) {
        input.arrival = arrival;
    }
    if (!enqueueInput(input)) {
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
    if (!enqueueInput(input)) {
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
    if (!enqueueInput(input)) {
        m_dropped_inputs.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    return true;
}

bool ManeuverSelectionWorker::pushRemoteIntent(
    int remote_vehicle_id,
    const estimation::TrajectoryIntentPacket & packet) noexcept
{
    if (!m_remote_trajectory_worker.push(remote_vehicle_id, packet)) {
        m_dropped_inputs.fetch_add(1, std::memory_order_relaxed);
        m_dropped_remote_intents.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    return true;
}

bool ManeuverSelectionWorker::pushPublishedSetpoint(
    const ManeuverSelectionPublishedSetpointSnapshot & snapshot) noexcept
{
    WorkerInput input;
    input.kind = InputKind::PublishedSetpoint;
    input.published = snapshot;
    if (!enqueueInput(input)) {
        m_published_input_history_lost.store(true, std::memory_order_release);
        m_dropped_inputs.fetch_add(1, std::memory_order_relaxed);
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
    if (!enqueueInput(input)) {
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

bool ManeuverSelectionWorker::processPendingForTest(
    std::uint64_t belief_elapsed_us, bool run_selection, bool run_remote)
{
    if (running() || !validParams(m_params)) {
        return false;
    }
    const bool remote_processed = run_remote
        && m_remote_trajectory_worker.processAvailableForTest();
    return processPending(belief_elapsed_us, run_selection) || remote_processed;
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

bool ManeuverSelectionWorker::processPending(
    std::optional<std::uint64_t> belief_elapsed_us, bool run_selection_inline)
{
    const bool measure = static_cast<bool>(m_stopped_stage_timing);
    PipelineTimingRecord pipeline{};
    if (measure) pipeline.start_ns = StoppedStageTiming::now();
    bool consumed_input = false;
    if (run_selection_inline) m_evaluation_worker.processOneForTest();
    bool accepted_belief = false;
    const auto input_count = m_input_storage->inbox.drainTo(m_input_storage->batch);
    for (std::size_t index = 0; index < input_count; ++index) {
        const auto * input = &m_input_storage->batch[index];
        consumed_input = true;
        if (measure) ++pipeline.input_count;
        if (input->kind == InputKind::OwnshipBelief) {
            const auto begin = measure ? StoppedStageTiming::now() : 0;
            const bool newer_belief = !m_has_latest_belief
                || input->belief.timestamp_us > m_latest_belief_timestamp_us;
            const bool accepted = acceptOwnshipBelief(input->belief);
            if (accepted && newer_belief) {
                accepted_belief = true;
                m_latest_belief_received_steady_ns = input->belief_enqueue_ns;
            }
            if (measure) {
                const auto end = StoppedStageTiming::now();
                ++pipeline.belief_count;
                pipeline.belief_processing_ns += end - begin;
                m_stopped_stage_timing->appendBelief({input->belief.timestamp_us,
                    input->belief.timestamp_sample_us, input->arrival,
                    input->belief_enqueue_ns, begin, end, accepted});
            }
        } else if (input->kind == InputKind::Airspeed) {
            acceptAirspeed(input->airspeed);
        } else if (input->kind == InputKind::NominalSetpoint) {
            acceptNominalSetpoint(input->nominal);
        } else if (input->kind == InputKind::PublishedSetpoint) {
            acceptPublishedSetpoint(input->published);
        } else {
            acceptRemoteDecision(input->remote_vehicle_id, input->decision);
        }
    }

    // Only completed immutable batches cross from the reconstruction thread.
    // Capture a finite batch; arrivals cannot extend this owner pass. Ownship
    // inputs and peer activation/decision events above never wait for decoding.
    const auto remote_count = m_remote_trajectory_worker.readyCount();
    for (std::size_t index = 0; index < remote_count; ++index) {
        const auto * completed = m_remote_trajectory_worker.readyResult();
        const auto begin = measure ? StoppedStageTiming::now() : 0;
        const auto candidate_count = completed->count;
        acceptRemoteCandidateSet(*completed);
        m_remote_trajectory_worker.releaseResult();
        consumed_input = true;
        if (measure) {
            // Owner installation only now; reconstruction has separate stopped
            // counters and is not attributed to this owner's input processing.
            pipeline.remote_processing_ns += StoppedStageTiming::now() - begin;
            pipeline.remote_count += candidate_count;
        }
    }

    if (measure) pipeline.drain_end_ns = StoppedStageTiming::now();

    if (m_published_input_history_lost.exchange(false, std::memory_order_acq_rel)) {
        // A dropped command could be a switch; never propagate through that
        // unknown interval using the previous command as if it were confirmed.
        m_published_input_head = 0;
        m_published_input_count = 0;
        m_has_latest_belief = false;
        m_has_latest_state = false;
    }
    const auto finish_without_frame = [&]() {
        if (measure && consumed_input) {
            pipeline.source_us = m_latest_state_timestamp_us;
            pipeline.end_ns = StoppedStageTiming::now();
            m_stopped_stage_timing->appendPipeline(pipeline);
        }
        return consumed_input;
    };
    if (!m_has_latest_belief) {
        return finish_without_frame();
    }

    // Fresh input keeps its source time. Between inputs, let due frames run
    // using elapsed monotonic time, as the existing command-history publisher
    // does. This is not a network-age estimate or a new clock conversion.
    const std::uint64_t elapsed_us = belief_elapsed_us.value_or(
        accepted_belief ? 0 :
        (steadyNowNs() - m_latest_belief_received_steady_ns) / 1000);
    if (elapsed_us > m_params.maximum_belief_delay_us
        || elapsed_us > std::numeric_limits<std::uint64_t>::max()
            - m_latest_belief_timestamp_us) {
        m_has_latest_state = false;
        return finish_without_frame();
    }
    const std::uint64_t due_time_us = std::max(m_last_processing_timestamp_us,
        m_latest_belief_timestamp_us + elapsed_us);
    if (due_time_us - m_latest_belief_sample_timestamp_us
        > m_params.maximum_belief_delay_us) {
        m_has_latest_state = false;
        return finish_without_frame();
    }
    const bool frame_due = !m_candidate_set_initialized
        || due_time_us >= m_next_candidate_refresh_timestamp_us
        || due_time_us >= m_next_trajectory_refresh_timestamp_us
        || (!m_epoch_evaluated && due_time_us >= m_epoch_generation_timestamp_us
            && due_time_us - m_epoch_generation_timestamp_us >= m_params.coordination_delay_us);
    const bool frame_ready = frame_due && prepareStateAt(due_time_us);
    // Peer agreement is an input event, not a new prediction frame. It may use
    // the existing valid state/trajectory at their unchanged source times;
    // do not make it wait for history needed only to advance the next frame.
    if (!frame_ready && !(((consumed_input && m_pending_proposal.valid
            && !m_pending_proposal.resolved) || m_evaluation_worker.readyResult())
            && m_has_latest_state
            && m_latest_state_timestamp_us >= m_last_processing_timestamp_us)) {
        return finish_without_frame();
    }
    const std::uint64_t now_us = m_latest_state_timestamp_us;
    m_last_processing_timestamp_us = now_us;
    if (!m_candidate_set_initialized) {
        initializeCandidateSet(now_us);
    }

    ManeuverSelectionWorkerOutput output;
    output.generated_timestamp_us = now_us;
    output.selection_epoch = m_selection_epoch;
    bool selection_completed = consumeSelectionEvaluation(due_time_us, output);

    // Timestamps only here; persist records after the existing output handoff.
    std::array<StageTimingRecord, 3> timing{};
    std::size_t timing_count = 0;

    const bool selection_due = frame_ready && !m_epoch_evaluated
        && now_us >= m_epoch_generation_timestamp_us
        && now_us - m_epoch_generation_timestamp_us
            >= m_params.coordination_delay_us;
    if (selection_due) {
        const std::uint64_t common_evaluation_timestamp_us =
            m_epoch_generation_timestamp_us
            + m_params.coordination_delay_us;
        submitSelectionEvaluation(common_evaluation_timestamp_us, output);
        if (run_selection_inline) {
            m_evaluation_worker.processOneForTest();
            selection_completed |= consumeSelectionEvaluation(due_time_us, output);
        }
        m_epoch_evaluated = true;
    }

    if (frame_ready && now_us >= m_next_candidate_refresh_timestamp_us) {
        refreshCandidateSet(now_us);
        output.selection_epoch = m_selection_epoch;
    }

    bool trajectory_refreshed = false;
    if (frame_ready && now_us >= m_next_trajectory_refresh_timestamp_us) {
        const auto begin = measure ? StoppedStageTiming::now() : 0;
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
        if (measure) {
            const auto end = StoppedStageTiming::now();
            timing[timing_count++] = {now_us, m_selection_epoch,
                m_params.trajectory_refresh_period_us, begin, end, 1,
                static_cast<std::uint8_t>(output.intent_packet_count), output.intent_packet_count > 0, false};
        }
        trajectory_refreshed = true;
        do {
            m_next_trajectory_refresh_timestamp_us +=
                m_params.trajectory_refresh_period_us;
        } while (m_next_trajectory_refresh_timestamp_us <= now_us);
    }

    const bool coordination_committed = finalizePendingCoordination(output);
    if ((selection_due || selection_completed || trajectory_refreshed || coordination_committed)
        && m_has_selected_combination) {
        // AMAC peers use the current decision message for post-release safety
        // acknowledgement and selected-intent awareness. Publish that state
        // with the 20 Hz trajectory refresh; an unrelated V4 shadow evaluator
        // must never be the mechanism that supplies this heartbeat.
        const auto begin = measure ? StoppedStageTiming::now() : 0;
        updateActivationState(
            now_us,
            selection_due || selection_completed || trajectory_refreshed || coordination_committed,
            output);
        if (measure) {
            const auto end = StoppedStageTiming::now();
            timing[timing_count++] = {now_us, m_selection_epoch, 0, begin, end, 3, 0, false, false};
        }
    }

    if (output.has_decision) {
        output.decision.command_execution_requested =
            m_activation_enabled.load(std::memory_order_acquire)
            && maneuverCommandExecutionRequested(
                m_params.execution_policy, output.decision);
        m_latest_selection_decision.command_execution_requested =
            output.decision.command_execution_requested;
    }
    bool output_queued = false;
    if (output.intent_packet_count > 0 || output.has_decision) {
        output_queued = publishOutput(output);
    }
    if (measure) {
        const bool selection_work_recorded = m_selection_snapshot_timing.has_value()
            || m_selection_apply_timing.has_value();
        for (auto * record : {&m_selection_snapshot_timing, &m_selection_apply_timing}) {
            if (*record) {
                (*record)->output_queued = output_queued;
                m_stopped_stage_timing->append(**record);
                record->reset();
            }
        }
        if (m_completed_selection_timing) {
            m_completed_selection_timing->output_queued = output_queued;
            m_stopped_stage_timing->append(*m_completed_selection_timing);
            m_completed_selection_timing.reset();
        }
        if (consumed_input || timing_count > 0 || selection_work_recorded) {
            pipeline.source_us = now_us;
            pipeline.end_ns = StoppedStageTiming::now();
            m_stopped_stage_timing->appendPipeline(pipeline);
        }
        for (std::size_t i = 0; i < timing_count; ++i) {
            timing[i].output_queued = output_queued;
            m_stopped_stage_timing->append(timing[i]);
        }
    }
    return consumed_input || selection_due || selection_completed || trajectory_refreshed
        || coordination_committed;
}

bool ManeuverSelectionWorker::prepareStateAt(std::uint64_t timestamp_us)
{
    if (!m_has_latest_belief || timestamp_us < m_latest_belief_timestamp_us
        || timestamp_us - m_latest_belief_sample_timestamp_us
            > m_params.maximum_belief_delay_us) {
        m_has_latest_state = false;
        return false;
    }
    auto state = m_latest_belief_state;
    auto covariance = m_latest_belief_covariance;
    if (timestamp_us > m_latest_belief_timestamp_us
        && !compensateUsingPublishedInputs(
            m_latest_belief_timestamp_us, timestamp_us, state, covariance)) {
        return false;
    }
    m_latest_state = state;
    m_latest_covariance = covariance;
    m_latest_state_timestamp_us = timestamp_us;
    m_latest_state_sample_timestamp_us = m_latest_belief_sample_timestamp_us;
    m_has_latest_state = true;
    return true;
}

bool ManeuverSelectionWorker::acceptOwnshipBelief(
    const ManeuverSelectionBeliefSnapshot & snapshot)
{
    if (!snapshot.valid || snapshot.timestamp_us < snapshot.timestamp_sample_us
        || snapshot.timestamp_us < m_latest_belief_timestamp_us) {
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

    if (!compensateUsingPublishedInputs(
            snapshot.timestamp_sample_us, snapshot.timestamp_us,
            state, covariance)) {
        return false;
    }

    m_latest_state = state;
    m_latest_covariance = covariance;
    m_latest_state_timestamp_us = snapshot.timestamp_us;
    m_latest_state_sample_timestamp_us = snapshot.timestamp_sample_us;
    m_has_latest_state = true;
    m_latest_belief_state = state;
    m_latest_belief_covariance = covariance;
    m_latest_belief_timestamp_us = snapshot.timestamp_us;
    m_latest_belief_sample_timestamp_us = snapshot.timestamp_sample_us;
    m_has_latest_belief = true;
    return true;
}

bool ManeuverSelectionWorker::acceptPublishedSetpoint(
    const ManeuverSelectionPublishedSetpointSnapshot & snapshot)
{
    if (snapshot.timestamp_us == 0
        || snapshot.timestamp_us < m_latest_published_input_timestamp_us) return false;
    m_latest_published_input_timestamp_us = snapshot.timestamp_us;
    PublishedInputEntry entry;
    static_cast<ManeuverSelectionPublishedSetpointSnapshot &>(entry) = snapshot;
    const auto & u = entry.input;
    entry.valid = entry.valid && std::isfinite(u.V_cmd) && u.V_cmd > 0.0
        && (std::isfinite(u.h_cmd) || std::isnan(u.h_cmd))
        && std::isfinite(u.h_dot_cmd) && std::isfinite(u.a_lat_cmd);
    if (m_published_input_count > 0) {
        auto & last = (*m_published_inputs)[(m_published_input_head
            + m_published_inputs->size() - 1) % m_published_inputs->size()];
        if (entry.timestamp_us < last.timestamp_us) return false;
        if (entry.timestamp_us == last.timestamp_us) {
            entry.roll_setpoint_rad = last.roll_setpoint_rad;
            last = entry;
            return true;
        }
        entry.roll_setpoint_rad = last.valid
            ? m_predictor.rollSetpointAfter(last.roll_setpoint_rad, last.input,
                static_cast<double>(entry.timestamp_us-last.timestamp_us)*1.0e-6)
            : (m_has_latest_state ? m_latest_state.phi : 0.0);
        const auto & previous = last.input;
        if (entry.valid == last.valid && (!entry.valid
            || (u.V_cmd == previous.V_cmd
                && (u.h_cmd == previous.h_cmd
                    || (std::isnan(u.h_cmd) && std::isnan(previous.h_cmd)))
                && u.h_dot_cmd == previous.h_dot_cmd
                && u.a_lat_cmd == previous.a_lat_cmd))) {
            return true; // Store changes, not identical publication heartbeats.
        }
    }
    (*m_published_inputs)[m_published_input_head] = entry;
    m_published_input_head =
        (m_published_input_head + 1) % m_published_inputs->size();
    m_published_input_count = std::min(
        m_published_input_count + 1, m_published_inputs->size());
    return true;
}

bool ManeuverSelectionWorker::compensateUsingPublishedInputs(
    std::uint64_t start_us, std::uint64_t end_us,
    estimation::PredictState & state,
    estimation::PredictStateCovariance & covariance)
{
    const PublishedInputEntry * held = nullptr;
    auto cursor_us = start_us;
    const auto oldest = (m_published_input_head + m_published_inputs->size()
        - m_published_input_count) % m_published_inputs->size();
    const auto advance = [&](std::uint64_t until_us) {
        if (held && held->valid && cursor_us == start_us) {
            state.phi_setpoint = m_predictor.rollSetpointAfter(
                held->roll_setpoint_rad, held->input,
                static_cast<double>(start_us-held->timestamp_us)*1.0e-6);
        }
        if (until_us == cursor_us) return true;
        if (!held || !held->valid) return false;
        return m_uncertainty.compensateFusionHorizonDelay(
            m_predictor, held->input,
            static_cast<double>(until_us - cursor_us) * 1.0e-6,
            state, covariance);
    };
    for (std::size_t i = 0; i < m_published_input_count; ++i) {
        const auto & entry = (*m_published_inputs)[
            (oldest + i) % m_published_inputs->size()];
        if (entry.timestamp_us <= start_us) {
            held = &entry;
        } else if (entry.timestamp_us < end_us) {
            if (!advance(entry.timestamp_us)) return false;
            cursor_us = entry.timestamp_us;
            held = &entry;
        } else {
            break;
        }
    }
    return advance(end_us);
}

bool ManeuverSelectionWorker::publishedInputAt(
    std::uint64_t timestamp_us, estimation::PredictInput & input) const noexcept
{
    if (m_latest_published_input_timestamp_us < timestamp_us
        && timestamp_us - m_latest_published_input_timestamp_us
            > m_params.maximum_belief_delay_us) {
        return false;
    }
    for (std::size_t offset = 0; offset < m_published_input_count; ++offset) {
        const auto & entry = (*m_published_inputs)[(m_published_input_head
            + m_published_inputs->size() - 1 - offset) % m_published_inputs->size()];
        if (entry.timestamp_us <= timestamp_us) {
            if (!entry.valid) return false;
            input = entry.input;
            return true;
        }
    }
    return false;
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

bool ManeuverSelectionWorker::acceptRemoteCandidateSet(
    const RemoteTrajectoryCandidateSet & completed)
{
    const int remote_vehicle_id = completed.vehicle_id;
    if (remote_vehicle_id < 0 || remote_vehicle_id >= m_params.total_agent_count
        || remote_vehicle_id == m_params.vehicle_id || completed.count == 0
        || completed.count != completed.expected_count
        || completed.count > kExhaustiveCandidatesPerAircraft) {
        return false;
    }
    const auto index = static_cast<std::size_t>(remote_vehicle_id);
    RemoteCandidateCache & remote_cache = m_remote_caches[index];
    const bool current_complete = remote_cache.count > 0
        && remote_cache.count == remote_cache.expected_count;
    if (current_complete
        && (completed.selection_epoch < remote_cache.selection_epoch
            || (completed.selection_epoch == remote_cache.selection_epoch
                && completed.source_timestamp_us < remote_cache.source_timestamp_us))) {
        return false;
    }
    const bool set_key_changed = remote_cache.selection_epoch != completed.selection_epoch
        || remote_cache.source_timestamp_us != completed.source_timestamp_us
        || remote_cache.candidate_set_kind != completed.candidate_set_kind;
    const RemoteDecisionCache & peer = m_remote_decision_caches[index];
    const auto retain_selected = [&](const RemoteCandidateCache & cache) {
        if (!peer.valid || !peer.decision.coordination_qualified
            || !peer.decision.ownship_candidate_valid) return;
        const auto selected_id = peer.decision.ownship_candidate_id;
        const auto revision = peer.decision.selected_candidate_input_revisions[index];
        const auto end = cache.candidates.begin() + cache.count;
        const auto selected = std::find_if(cache.candidates.begin(), end,
            [selected_id, revision](const auto & candidate) {
                return candidate.candidate_id == selected_id
                    && candidate.candidate_input_revision == revision;
            });
        if (selected != end) {
            m_remote_selected_caches[index].intent = *selected;
            m_remote_selected_caches[index].valid = true;
        }
    };
    if (current_complete && set_key_changed) {
        m_remote_previous_caches[index] = remote_cache;
        retain_selected(remote_cache);
    }
    remote_cache = completed;
    freezeRemoteCertificationCandidatesForCurrentEpoch(remote_vehicle_id);
    retain_selected(remote_cache);
    return true;
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
        || decision.vehicle_id != remote_vehicle_id
        || (decision.local_activation_request_timestamp_us != 0
            && (decision.activation_timestamp_us == 0
                || decision.local_activation_request_timestamp_us
                    < decision.activation_timestamp_us))) {
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
    // A delayed active heartbeat cannot reopen an ended episode. The start
    // timestamp is already present in the wire message and is stable across
    // maneuver switches. No extra packet or acknowledgement is needed.
    const bool stale_activation_status =
        m_params.interaction_graph_params.enabled
        && m_params.execution_policy == ManeuverExecutionPolicy::AmacAdThreshold
        && cache.valid
        && (decision.activation_timestamp_us
                < cache.decision.activation_timestamp_us
            || (decision.activation_requested
                && decision.activation_timestamp_us != 0
                && decision.activation_timestamp_us
                    <= cache.activation_ended_through_us));
    const auto previous_activation_timestamp =
        cache.decision.activation_timestamp_us;
    const bool previous_activation_requested = cache.decision.activation_requested;
    const auto previous_local_request_timestamp =
        cache.decision.local_activation_request_timestamp_us;
    const bool retain_bootstrap_readiness = cache.valid
        && cache.decision.v4_cutover_candidate_ready
        && !cache.decision.selected_v4_cutover
        && !decision.selected_v4_cutover
        && cache.decision.v4_control_architecture
            == decision.v4_control_architecture;
    if (!stale_activation_status && !decision.activation_requested) {
        cache.activation_ended_through_us = std::max(
            cache.activation_ended_through_us, decision.activation_timestamp_us);
        cache.local_activation_request_consumed_through_us = std::max(
            cache.local_activation_request_consumed_through_us,
            decision.local_activation_request_timestamp_us);
    }
    cache.decision = decision;
    if (stale_activation_status) {
        // Preserve episode ordering without dropping an otherwise valid
        // proposal carried by this message.
        cache.decision.activation_timestamp_us = previous_activation_timestamp;
        cache.decision.activation_requested = previous_activation_requested;
        cache.decision.activation_just_started = false;
        cache.decision.local_activation_request_timestamp_us =
            previous_local_request_timestamp;
    } else if (decision.activation_timestamp_us == previous_activation_timestamp) {
        // A delayed peer-only heartbeat must not erase a local request that
        // originated later within the same execution episode.
        cache.decision.local_activation_request_timestamp_us = std::max(
            previous_local_request_timestamp,
            decision.local_activation_request_timestamp_us);
    }
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

void ManeuverSelectionWorker::recordBudgetTrace(ManeuverBudgetTrace trace)
{
    if (!m_budget_records) return;
    stampBudgetTrace(trace);
    trace.vehicle_id = m_params.vehicle_id;
    trace.state_timestamp_us = m_latest_state_timestamp_us;
    trace.state_sample_timestamp_us = m_latest_state_sample_timestamp_us;
    trace.dropped_trace_count = m_budget_records->dropped;
    m_budget_records->append(trace);
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
