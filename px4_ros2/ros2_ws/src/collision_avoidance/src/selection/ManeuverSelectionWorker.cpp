#include <collision_avoidance/selection/ManeuverSelectionWorker.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <thread>

namespace collision_avoidance::selection
{
namespace
{

constexpr std::uint8_t kRollZeroId = static_cast<std::uint8_t>(
    estimation::ManeuverCandidateId::RollZero);

bool finitePositive(double value) noexcept
{
    return std::isfinite(value) && value > 0.0;
}

bool validParams(const ManeuverSelectionWorkerParams & params) noexcept
{
    if (!finitePositive(params.ground_speed_command_mps)
        || !finitePositive(params.gravity_mps2)
        || params.trajectory_refresh_period_us == 0
        || params.candidate_refresh_period_us == 0
        || params.coordination_delay_us == 0
        || params.maximum_belief_delay_us == 0) {
        return false;
    }

    std::array<bool, estimation::kManeuverCandidateCount> seen{};
    for (const std::uint8_t candidate_id : params.eligible_candidate_ids) {
        if (candidate_id >= estimation::kManeuverCandidateCount
            || seen[candidate_id]) {
            return false;
        }
        seen[candidate_id] = true;
    }
    return seen[kRollZeroId];
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
  m_receiver(m_predictor, m_candidate_table, params.uncertainty_params),
  m_uncertainty(params.uncertainty_params),
  m_evaluator(params.evaluator_params)
{
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
        return false;
    }
    return true;
}

bool ManeuverSelectionWorker::pushRemoteIntent(
    const estimation::TrajectoryIntentPacket & packet) noexcept
{
    WorkerInput input;
    input.kind = InputKind::RemoteIntent;
    input.packet = packet;
    if (!m_input_queue.try_push(input)) {
        m_dropped_inputs.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    return true;
}

std::optional<ManeuverSelectionWorkerOutput>
ManeuverSelectionWorker::tryPopOutput() noexcept
{
    return m_output_queue.try_pop();
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
        } else {
            acceptRemoteIntent(input->packet);
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
        evaluateCurrentSet(now_us, output);
        m_epoch_evaluated = true;
    }

    if (now_us >= m_next_candidate_refresh_timestamp_us) {
        refreshCandidateSet(now_us);
        output.selection_epoch = m_selection_epoch;
    }

    if (now_us >= m_next_trajectory_refresh_timestamp_us) {
        buildCurrentIntentSet(now_us, output);
        do {
            m_next_trajectory_refresh_timestamp_us +=
                m_params.trajectory_refresh_period_us;
        } while (m_next_trajectory_refresh_timestamp_us <= now_us);
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

bool ManeuverSelectionWorker::acceptRemoteIntent(
    const estimation::TrajectoryIntentPacket & packet)
{
    const auto keyLess = [](
                             std::uint64_t lhs_epoch,
                             std::uint64_t lhs_timestamp,
                             std::uint64_t rhs_epoch,
                             std::uint64_t rhs_timestamp) {
        return lhs_epoch < rhs_epoch
            || (lhs_epoch == rhs_epoch && lhs_timestamp < rhs_timestamp);
    };

    if (m_remote_cache.count == kCandidatesPerAircraft
        && keyLess(
            packet.selection_epoch,
            packet.source_timestamp_us,
            m_remote_cache.selection_epoch,
            m_remote_cache.source_timestamp_us)) {
        return false;
    }

    estimation::ReceivedTrajectoryIntent received;
    if (!m_receiver.receive(packet, received)) {
        return false;
    }

    const bool staging_key_matches =
        m_remote_staging_cache.selection_epoch == packet.selection_epoch
        && m_remote_staging_cache.source_timestamp_us
            == packet.source_timestamp_us;
    if (!staging_key_matches) {
        if (m_remote_staging_cache.count > 0
            && keyLess(
                packet.selection_epoch,
                packet.source_timestamp_us,
                m_remote_staging_cache.selection_epoch,
                m_remote_staging_cache.source_timestamp_us)) {
            return false;
        }
        m_remote_staging_cache = RemoteCandidateCache{};
        m_remote_staging_cache.selection_epoch = packet.selection_epoch;
        m_remote_staging_cache.source_timestamp_us = packet.source_timestamp_us;
    }

    for (std::size_t index = 0;
         index < m_remote_staging_cache.candidates.size(); ++index) {
        if (m_remote_staging_cache.occupied[index]
            && m_remote_staging_cache.candidates[index].candidate_id
                == packet.candidate_id) {
            m_remote_staging_cache.candidates[index] = received;
            return true;
        }
    }
    for (std::size_t index = 0;
         index < m_remote_staging_cache.candidates.size(); ++index) {
        if (!m_remote_staging_cache.occupied[index]) {
            m_remote_staging_cache.occupied[index] = true;
            m_remote_staging_cache.candidates[index] = received;
            ++m_remote_staging_cache.count;
            if (m_remote_staging_cache.count == kCandidatesPerAircraft) {
                m_remote_cache = m_remote_staging_cache;
            }
            return true;
        }
    }
    return false;
}

void ManeuverSelectionWorker::initializeCandidateSet(std::uint64_t now_us)
{
    m_current_best_id = kRollZeroId;
    m_alternate_cursor = 0;
    refreshCandidateSet(now_us);
    m_candidate_set_initialized = true;
}

void ManeuverSelectionWorker::refreshCandidateSet(std::uint64_t now_us)
{
    ++m_selection_epoch;
    m_held_candidate_ids[0] = m_current_best_id;
    chooseAlternates();
    m_epoch_generation_timestamp_us = now_us;
    m_next_candidate_refresh_timestamp_us =
        now_us + m_params.candidate_refresh_period_us;
    m_next_trajectory_refresh_timestamp_us = now_us;
    m_epoch_evaluated = false;
    m_ownship_candidates_complete = false;
}

void ManeuverSelectionWorker::chooseAlternates()
{
    std::size_t selected = 1;
    std::size_t visited = 0;
    while (selected < m_held_candidate_ids.size()
        && visited < m_params.eligible_candidate_ids.size() * 2) {
        const std::uint8_t candidate_id = m_params.eligible_candidate_ids[
            m_alternate_cursor % m_params.eligible_candidate_ids.size()];
        ++m_alternate_cursor;
        ++visited;
        if (candidate_id == m_current_best_id) {
            continue;
        }
        bool duplicate = false;
        for (std::size_t index = 1; index < selected; ++index) {
            duplicate = duplicate || m_held_candidate_ids[index] == candidate_id;
        }
        if (!duplicate) {
            m_held_candidate_ids[selected++] = candidate_id;
        }
    }
}

bool ManeuverSelectionWorker::buildCurrentIntentSet(
    std::uint64_t now_us,
    ManeuverSelectionWorkerOutput & output)
{
    CandidateIntentSet received_candidates{};
    std::array<estimation::TrajectoryIntentPacket, kCandidatesPerAircraft> packets{};
    for (std::size_t index = 0; index < m_held_candidate_ids.size(); ++index) {
        if (!m_sender.buildForSelectedCandidate(
                now_us,
                m_held_candidate_ids[index],
                m_latest_state,
                m_latest_covariance,
                packets[index],
                m_selection_epoch)
            || !m_receiver.receive(packets[index], received_candidates[index])) {
            m_ownship_candidates_complete = false;
            return false;
        }
    }

    m_ownship_candidates = received_candidates;
    m_ownship_candidates_complete = true;
    output.intent_packets = packets;
    output.intent_packet_count = packets.size();
    output.generated_timestamp_us = now_us;
    output.selection_epoch = m_selection_epoch;
    return true;
}

void ManeuverSelectionWorker::evaluateCurrentSet(
    std::uint64_t now_us,
    ManeuverSelectionWorkerOutput & output)
{
    ManeuverSelectionDecision decision;
    decision.selection_timestamp_us = now_us;
    decision.local_selection_epoch = m_selection_epoch;
    decision.remote_selection_epoch = m_remote_cache.selection_epoch;
    decision.ownship_candidate_id = m_current_best_id;
    if (const auto * input = m_candidate_table.find(m_current_best_id)) {
        decision.ownship_input = *input;
    }

    if (m_ownship_candidates_complete
        && m_remote_cache.count == kCandidatesPerAircraft) {
        StaticCombinationEvaluation evaluation;
        if (m_evaluator.evaluate(
                now_us,
                m_ownship_candidates,
                m_remote_cache.candidates,
                evaluation)) {
            const CombinationEvaluation & best = evaluation.combinations[
                evaluation.best_combination_index];
            decision.coordination_qualified = true;
            decision.new_best_accepted = true;
            decision.previous_best_retained = false;
            decision.ownship_candidate_id = best.ownship_candidate_id;
            decision.threat_candidate_id = best.threat_candidate_id;
            decision.pmr_m = best.pmr_m;
            decision.masd_m = best.masd_m;
            decision.ad_m = best.ad_m;
            decision.activation_requested = best.ad_m < 0.0;
            if (const auto * input = m_candidate_table.find(
                    best.ownship_candidate_id)) {
                decision.ownship_input = *input;
                m_current_best_id = best.ownship_candidate_id;
            }
        }
    }

    output.decision = decision;
    output.has_decision = true;
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

}  // namespace collision_avoidance::selection
