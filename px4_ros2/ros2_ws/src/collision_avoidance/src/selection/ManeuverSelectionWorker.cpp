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
constexpr double kTrajectoryStepSeconds =
    estimation::kTrajectoryIntentStepSeconds;
constexpr double kTrajectoryHorizonSeconds =
    static_cast<double>(estimation::kTrajectoryPointCount - 1)
    * kTrajectoryStepSeconds;

struct IntentKinematics
{
    std::array<double, 3> position_ned{};
    std::array<double, 3> velocity_ned{};
};

bool finitePositive(double value) noexcept
{
    return std::isfinite(value) && value > 0.0;
}

bool validParams(const ManeuverSelectionWorkerParams & params) noexcept
{
    if (params.total_agent_count < 2
        || params.total_agent_count
            > static_cast<int>(kMaximumSelectionAircraft)
        || params.vehicle_id < 0
        || params.vehicle_id >= params.total_agent_count
        || !finitePositive(params.ground_speed_command_mps)
        || !finitePositive(params.gravity_mps2)
        || params.trajectory_refresh_period_us == 0
        || params.candidate_refresh_period_us == 0
        || params.coordination_delay_us == 0
        || params.maximum_belief_delay_us == 0
        || params.activation_params.maximum_active_duration_us == 0
        || !std::isfinite(
            params.activation_params.separating_rate_threshold_mps)
        || params.activation_params.separating_rate_threshold_mps < 0.0) {
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

bool interpolateIntentKinematics(
    const estimation::ReceivedTrajectoryIntent & intent,
    std::uint64_t evaluation_timestamp_us,
    IntentKinematics & kinematics) noexcept
{
    if (intent.source_timestamp_us > evaluation_timestamp_us) {
        return false;
    }
    const double age_s = static_cast<double>(
        evaluation_timestamp_us - intent.source_timestamp_us) * 1.0e-6;
    if (!std::isfinite(age_s) || age_s < 0.0
        || age_s > kTrajectoryHorizonSeconds) {
        return false;
    }

    const double fractional_index = age_s / kTrajectoryStepSeconds;
    std::size_t lower = static_cast<std::size_t>(
        std::floor(fractional_index));
    lower = std::min(lower, estimation::kTrajectoryPointCount - 1);
    const std::size_t upper = std::min(
        lower + 1, estimation::kTrajectoryPointCount - 1);
    const double alpha = upper == lower
        ? 0.0
        : std::clamp(
            fractional_index - static_cast<double>(lower), 0.0, 1.0);
    const auto & first = intent.reconstructed_mean[lower];
    const auto & second = intent.reconstructed_mean[upper];
    const auto blend = [alpha](double lhs, double rhs) {
        return (1.0 - alpha) * lhs + alpha * rhs;
    };

    const double p_n = blend(first.p_n, second.p_n);
    const double p_e = blend(first.p_e, second.p_e);
    const double h = blend(first.h, second.h);
    const double speed = blend(first.V, second.V);
    const double climb_rate = blend(first.h_dot, second.h_dot);
    const double course = first.psi + alpha * std::remainder(
        second.psi - first.psi, 2.0 * std::acos(-1.0));
    if (!std::isfinite(p_n) || !std::isfinite(p_e) || !std::isfinite(h)
        || !std::isfinite(speed) || !std::isfinite(climb_rate)
        || !std::isfinite(course)) {
        return false;
    }

    const double horizontal_speed = std::sqrt(std::max(
        0.0, speed * speed - climb_rate * climb_rate));
    kinematics.position_ned = {p_n, p_e, -h};
    kinematics.velocity_ned = {
        horizontal_speed * std::cos(course),
        horizontal_speed * std::sin(course),
        -climb_rate};
    return true;
}

bool relativeSeparationRate(
    const IntentKinematics & ownship,
    const IntentKinematics & threat,
    double & rate_mps) noexcept
{
    std::array<double, 3> relative_position{};
    std::array<double, 3> relative_velocity{};
    double squared_range = 0.0;
    double radial_product = 0.0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
        relative_position[axis] =
            threat.position_ned[axis] - ownship.position_ned[axis];
        relative_velocity[axis] =
            threat.velocity_ned[axis] - ownship.velocity_ned[axis];
        squared_range += relative_position[axis] * relative_position[axis];
        radial_product +=
            relative_position[axis] * relative_velocity[axis];
    }
    if (!std::isfinite(squared_range) || squared_range <= 1.0e-12
        || !std::isfinite(radial_product)) {
        return false;
    }
    rate_mps = radial_product / std::sqrt(squared_range);
    return std::isfinite(rate_mps);
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
  m_pair_evaluator(params.evaluator_params),
  m_joint_evaluator(params.evaluator_params),
  m_exhaustive_evaluator(params.evaluator_params),
  m_activation_controller(params.activation_params)
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
    int remote_vehicle_id,
    const estimation::TrajectoryIntentPacket & packet) noexcept
{
    WorkerInput input;
    input.kind = InputKind::RemoteIntent;
    input.remote_vehicle_id = remote_vehicle_id;
    input.packet = packet;
    if (!m_input_queue.try_push(input)) {
        m_dropped_inputs.fetch_add(1, std::memory_order_relaxed);
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
            acceptRemoteIntent(input->remote_vehicle_id, input->packet);
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
        buildCurrentIntentSet(now_us, output);
        trajectory_refreshed = true;
        do {
            m_next_trajectory_refresh_timestamp_us +=
                m_params.trajectory_refresh_period_us;
        } while (m_next_trajectory_refresh_timestamp_us <= now_us);
    }

    if (selection_due || trajectory_refreshed) {
        updateActivationState(now_us, selection_due, output);
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

    const std::size_t required_candidate_count = activeCandidateCount();
    if (remote_cache.count == required_candidate_count
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
            == packet.source_timestamp_us;
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
                if (remote_cache.count == required_candidate_count
                    && remote_cache.selection_epoch
                        != staging_cache.selection_epoch) {
                    m_remote_previous_caches[
                        static_cast<std::size_t>(remote_vehicle_id)] =
                        remote_cache;
                }
                remote_cache = staging_cache;
            }
            return true;
        }
    }
    return false;
}

void ManeuverSelectionWorker::initializeCandidateSet(std::uint64_t now_us)
{
    m_current_best_id = kRollZeroId;
    refreshCandidateSet(now_us);
    m_candidate_set_initialized = true;
}

void ManeuverSelectionWorker::refreshCandidateSet(std::uint64_t now_us)
{
    m_selection_epoch = now_us / m_params.candidate_refresh_period_us;
    chooseAlternates(now_us);
    m_epoch_generation_timestamp_us =
        m_selection_epoch * m_params.candidate_refresh_period_us;
    m_next_candidate_refresh_timestamp_us =
        (m_selection_epoch + 1) * m_params.candidate_refresh_period_us;
    m_next_trajectory_refresh_timestamp_us = now_us;
    m_epoch_evaluated = false;
    m_ownship_candidates_complete = false;
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
    for (std::size_t index = 0; index < candidate_count; ++index) {
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

    std::sort(
        received_candidates.begin(),
        received_candidates.begin()
            + static_cast<std::ptrdiff_t>(candidate_count),
        [](const auto & lhs, const auto & rhs) {
            return lhs.candidate_id < rhs.candidate_id;
        });

    m_ownship_candidates = received_candidates;
    m_ownship_candidates_complete = true;
    output.intent_packets = packets;
    output.intent_packet_count = candidate_count;
    output.generated_timestamp_us = now_us;
    output.selection_epoch = m_selection_epoch;
    return true;
}

void ManeuverSelectionWorker::evaluateCurrentSet(
    std::uint64_t now_us,
    ManeuverSelectionWorkerOutput & output)
{
    ManeuverSelectionDecision decision;
    decision.vehicle_id = m_params.vehicle_id;
    decision.aircraft_count = static_cast<std::size_t>(
        m_params.total_agent_count);
    decision.selection_timestamp_us = now_us;
    decision.local_selection_epoch = m_selection_epoch;
    decision.ownship_candidate_id = m_current_best_id;
    if (const auto * input = m_candidate_table.find(m_current_best_id)) {
        decision.ownship_input = *input;
    }

    MultiAircraftExhaustiveCandidateIntentSets candidate_sets{};
    const std::size_t required_candidate_count = activeCandidateCount();
    bool all_candidate_sets_complete = m_ownship_candidates_complete;
    for (int aircraft = 0;
         aircraft < m_params.total_agent_count; ++aircraft) {
        if (aircraft == m_params.vehicle_id) {
            candidate_sets[static_cast<std::size_t>(aircraft)] =
                m_ownship_candidates;
            decision.selection_epochs_by_aircraft[
                static_cast<std::size_t>(aircraft)] = m_selection_epoch;
            continue;
        }
        const std::size_t aircraft_index =
            static_cast<std::size_t>(aircraft);
        const RemoteCandidateCache * remote_cache =
            &m_remote_caches[aircraft_index];
        if (remote_cache->selection_epoch != m_selection_epoch
            && m_remote_previous_caches[aircraft_index].selection_epoch
                == m_selection_epoch) {
            remote_cache = &m_remote_previous_caches[aircraft_index];
        }
        if (remote_cache->count != required_candidate_count) {
            all_candidate_sets_complete = false;
            continue;
        }
        if (remote_cache->selection_epoch != m_selection_epoch) {
            all_candidate_sets_complete = false;
            continue;
        }
        candidate_sets[aircraft_index] = remote_cache->candidates;
        decision.selection_epochs_by_aircraft[
            aircraft_index] = remote_cache->selection_epoch;
        if (decision.remote_selection_epoch == 0) {
            decision.remote_selection_epoch = remote_cache->selection_epoch;
        }
    }

    if (all_candidate_sets_complete) {
        JointCombinationEvaluation best{};
        std::size_t best_combination_index = 0;
        std::size_t combination_count = 0;
        bool evaluated = false;
        if (m_params.exhaustive_test_mode) {
            ExhaustiveManeuverEvaluation evaluation;
            evaluated = m_exhaustive_evaluator.evaluate(
                now_us,
                candidate_sets,
                static_cast<std::size_t>(m_params.total_agent_count),
                evaluation);
            if (evaluated) {
                best = evaluation.best_combination;
                best_combination_index = evaluation.best_combination_index;
                combination_count = evaluation.combination_count;
            }
        } else {
            MultiAircraftCandidateIntentSets reduced_candidate_sets{};
            for (int aircraft = 0;
                 aircraft < m_params.total_agent_count; ++aircraft) {
                std::copy_n(
                    candidate_sets[static_cast<std::size_t>(aircraft)].begin(),
                    kCandidatesPerAircraft,
                    reduced_candidate_sets[
                        static_cast<std::size_t>(aircraft)].begin());
            }
            JointManeuverEvaluation evaluation;
            evaluated = m_joint_evaluator.evaluate(
                now_us,
                reduced_candidate_sets,
                static_cast<std::size_t>(m_params.total_agent_count),
                evaluation);
            if (evaluated) {
                best = evaluation.combinations[
                    evaluation.best_combination_index];
                best_combination_index = evaluation.best_combination_index;
                combination_count = evaluation.combination_count;
            }
        }
        if (evaluated) {
            std::array<std::uint8_t, kMaximumSelectionAircraft>
                proposed_candidate_ids{};
            for (int aircraft = 0;
                 aircraft < m_params.total_agent_count; ++aircraft) {
                const std::size_t aircraft_index =
                    static_cast<std::size_t>(aircraft);
                const std::uint8_t candidate_slot =
                    best.candidate_slots[aircraft_index];
                proposed_candidate_ids[aircraft_index] =
                    candidate_sets[aircraft_index][candidate_slot].candidate_id;
            }

            const bool command_is_active =
                m_activation_controller.status().active;
            const bool proposed_is_current = m_has_selected_combination
                && proposed_candidate_ids == m_selected_candidate_ids;
            const bool accept_proposed = !command_is_active
                && (!m_has_selected_combination || !proposed_is_current);

            if (accept_proposed) {
                m_selected_candidate_ids = proposed_candidate_ids;
                m_has_selected_combination = true;
                m_current_best_id = m_selected_candidate_ids[
                    static_cast<std::size_t>(m_params.vehicle_id)];
            }

            if (command_is_active && m_has_selected_combination) {
                decision = m_latest_selection_decision;
                decision.selection_timestamp_us = now_us;
                decision.local_selection_epoch = m_selection_epoch;
                decision.selection_epochs_by_aircraft.fill(m_selection_epoch);
                decision.new_best_accepted = false;
                decision.previous_best_retained = true;
                decision.evaluated_combination_count = combination_count;
            } else {
                decision.coordination_qualified = true;
                decision.new_best_accepted = accept_proposed;
                decision.previous_best_retained = !accept_proposed;
                decision.selected_combination_index = best_combination_index;
                decision.evaluated_combination_count = combination_count;
                decision.pmr_m = best.minimum_pmr_m;
                decision.masd_m = best.minimum_masd_m;
                decision.ad_m = best.minimum_ad_m;
                decision.reciprocal_cost_sum = best.reciprocal_cost_sum;
                decision.selected_candidate_ids = m_selected_candidate_ids;
                decision.ownship_candidate_id = m_current_best_id;
                if (const auto * input = m_candidate_table.find(
                        decision.ownship_candidate_id)) {
                    decision.ownship_input = *input;
                }
                for (int aircraft = 0;
                     aircraft < m_params.total_agent_count; ++aircraft) {
                    if (aircraft != m_params.vehicle_id) {
                        decision.threat_candidate_id =
                            m_selected_candidate_ids[
                                static_cast<std::size_t>(aircraft)];
                        break;
                    }
                }
            }
            m_latest_selection_decision = decision;
        }
    }

    if (!m_has_selected_combination) {
        output.decision = decision;
        output.has_decision = true;
    }
}

bool ManeuverSelectionWorker::buildActivationSample(
    std::uint64_t now_us,
    ManeuverActivationSample & sample,
    ManeuverSelectionDecision & decision) const
{
    sample = ManeuverActivationSample{};
    sample.timestamp_us = now_us;
    sample.separation_rates_mps.fill(
        std::numeric_limits<double>::quiet_NaN());
    if (!m_has_selected_combination || !m_ownship_candidates_complete) {
        return false;
    }

    const std::size_t required_candidate_count = activeCandidateCount();
    const auto find_candidate = [required_candidate_count](
                                    const ExhaustiveCandidateIntentSet & candidates,
                                    std::uint8_t candidate_id)
        -> const estimation::ReceivedTrajectoryIntent * {
        for (std::size_t index = 0;
             index < required_candidate_count; ++index) {
            if (candidates[index].candidate_id == candidate_id) {
                return &candidates[index];
            }
        }
        return nullptr;
    };

    const std::size_t ownship_index = static_cast<std::size_t>(
        m_params.vehicle_id);
    const std::uint8_t ownship_candidate_id =
        m_selected_candidate_ids[ownship_index];
    const estimation::ReceivedTrajectoryIntent * ownship_intent =
        find_candidate(m_ownship_candidates, ownship_candidate_id);
    const estimation::PredictInput * ownship_input =
        m_candidate_table.find(ownship_candidate_id);
    IntentKinematics ownship_kinematics;
    if (ownship_intent == nullptr || ownship_input == nullptr
        || !interpolateIntentKinematics(
            *ownship_intent, now_us, ownship_kinematics)) {
        return false;
    }

    sample.selected_candidate_id = ownship_candidate_id;
    sample.selected_input = *ownship_input;
    double minimum_ad = std::numeric_limits<double>::infinity();
    double reciprocal_cost_sum = 0.0;
    bool reciprocal_cost_defined = true;
    std::size_t evaluated_threat_count = 0;
    for (int remote_id = 0;
         remote_id < m_params.total_agent_count; ++remote_id) {
        if (remote_id == m_params.vehicle_id) {
            continue;
        }
        const std::size_t remote_index = static_cast<std::size_t>(remote_id);
        const std::uint8_t remote_candidate_id =
            m_selected_candidate_ids[remote_index];
        const RemoteCandidateCache * cache = &m_remote_caches[remote_index];
        const estimation::ReceivedTrajectoryIntent * remote_intent = nullptr;
        if (cache->count == required_candidate_count) {
            remote_intent = find_candidate(
                cache->candidates, remote_candidate_id);
        }
        if (remote_intent == nullptr) {
            cache = &m_remote_previous_caches[remote_index];
            if (cache->count == required_candidate_count) {
                remote_intent = find_candidate(
                    cache->candidates, remote_candidate_id);
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
        if (pair.ad_m < minimum_ad) {
            minimum_ad = pair.ad_m;
            decision.pmr_m = pair.pmr_m;
            decision.masd_m = pair.masd_m;
            decision.threat_candidate_id = remote_candidate_id;
        }
        if (pair.ad_m < 0.0) {
            sample.unsafe_threat_mask |= std::uint32_t{1} << remote_index;
        }
        if (pair.reciprocal_cost_defined) {
            reciprocal_cost_sum += pair.reciprocal_cost;
        } else {
            reciprocal_cost_defined = false;
        }

        IntentKinematics remote_kinematics;
        double separation_rate_mps =
            std::numeric_limits<double>::quiet_NaN();
        if (interpolateIntentKinematics(
                *remote_intent, now_us, remote_kinematics)
            && relativeSeparationRate(
                ownship_kinematics,
                remote_kinematics,
                separation_rate_mps)) {
            sample.separation_rates_mps[remote_index] = separation_rate_mps;
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

    ManeuverActivationSample sample;
    const bool sample_valid = buildActivationSample(now_us, sample, decision);
    sample.valid = sample_valid;
    const ManeuverActivationStatus status =
        m_activation_controller.update(sample);
    decision.activation_requested = status.active;
    decision.activation_just_started = status.just_activated;
    decision.activation_just_ended = status.just_deactivated;
    decision.activation_timestamp_us = status.activation_timestamp_us;
    decision.deactivation_reason = status.deactivation_reason;
    if (status.active) {
        decision.ownship_candidate_id = status.latched_candidate_id;
        decision.ownship_input = status.latched_input;
        m_current_best_id = status.latched_candidate_id;
    }

    m_latest_selection_decision = decision;
    if (force_decision_output || status.just_activated
        || status.just_deactivated) {
        output.decision = decision;
        output.has_decision = true;
    }
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
