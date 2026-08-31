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
constexpr std::uint64_t kTrajectoryHorizonMicroseconds =
    static_cast<std::uint64_t>(kTrajectoryHorizonSeconds * 1.0e6);

struct IntentKinematics
{
    std::array<double, 3> position_ned{};
    std::array<double, 3> velocity_ned{};
};

enum class IntentKinematicsStatus : std::uint8_t
{
    Valid,
    Future,
    Stale,
    Invalid,
};

bool finitePositive(double value) noexcept
{
    return std::isfinite(value) && value > 0.0;
}

bool validParams(const ManeuverSelectionWorkerParams & params) noexcept
{
    const bool v4_execution_policy =
        params.execution_policy == ManeuverExecutionPolicy::ContinuousV4
        || params.execution_policy == ManeuverExecutionPolicy::HorizonGatedV4;
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
        || params.activation_params.separating_rate_threshold_mps < 0.0
        || !std::isfinite(params.activation_params.activation_threshold_m)
        || params.activation_params.activation_threshold_m < 0.0
        || (v4_execution_policy
            && (!params.v4_safe_control_enabled || params.v4_shadow_only
                || params.activation_params.activation_threshold_m != 0.0))
        || (params.execution_policy == ManeuverExecutionPolicy::HorizonGatedV4
            && (!finitePositive(params.v4_horizon_trigger_m)
                || !params.evaluator_params.robust_cone_filter_enabled))
        || (params.v4_safe_control_enabled
            && (!finitePositive(params.v4_trim_airspeed_mps)
                || params.v4_maximum_airspeed_age_us == 0
                || params.v4_maximum_nominal_age_us == 0
                || !SafeControlSetV4::validParams(
                    params.v4_safe_control_params)
                || !SafeControlCandidateAdapter::validParams(
                    params.v4_candidate_adapter_params)))
        || (params.v4_safe_control_enabled && !params.v4_shadow_only
            && params.exhaustive_test_mode)
        || ((params.evaluator_params.positive_margin_filter_enabled
                || params.evaluator_params.robust_cone_filter_enabled)
            && (!std::isfinite(
                    params.evaluator_params.positive_margin_gamma)
                || params.evaluator_params.positive_margin_gamma <= 0.0
                || params.evaluator_params.positive_margin_gamma > 1.0
                || !finitePositive(
                    params.evaluator_params.positive_margin_reference_m)
                || (params.evaluator_params.positive_margin_filter_enabled
                    && !finitePositive(
                        params.evaluator_params
                            .maximum_lateral_acceleration_mps2))))) {
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

IntentKinematicsStatus interpolateIntentKinematics(
    const estimation::ReceivedTrajectoryIntent & intent,
    std::uint64_t evaluation_timestamp_us,
    IntentKinematics & kinematics) noexcept
{
    if (intent.source_timestamp_us > evaluation_timestamp_us) {
        return IntentKinematicsStatus::Future;
    }
    const double age_s = static_cast<double>(
        evaluation_timestamp_us - intent.source_timestamp_us) * 1.0e-6;
    if (!std::isfinite(age_s) || age_s < 0.0
        || age_s > kTrajectoryHorizonSeconds) {
        return IntentKinematicsStatus::Stale;
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
        return IntentKinematicsStatus::Invalid;
    }

    const double horizontal_speed = std::sqrt(std::max(
        0.0, speed * speed - climb_rate * climb_rate));
    kinematics.position_ned = {p_n, p_e, -h};
    kinematics.velocity_ned = {
        horizontal_speed * std::cos(course),
        horizontal_speed * std::sin(course),
        -climb_rate};
    return IntentKinematicsStatus::Valid;
}

V4SnapshotStatus classifySnapshot(
    bool present,
    bool value_valid,
    std::uint64_t timestamp_us,
    std::uint64_t evaluation_timestamp_us,
    std::uint64_t maximum_age_us,
    std::uint64_t & age_us) noexcept
{
    age_us = 0;
    if (!present) {
        return V4SnapshotStatus::Missing;
    }
    if (!value_valid) {
        return V4SnapshotStatus::Invalid;
    }
    if (timestamp_us > evaluation_timestamp_us) {
        return V4SnapshotStatus::Future;
    }
    age_us = evaluation_timestamp_us - timestamp_us;
    if (age_us > maximum_age_us) {
        return V4SnapshotStatus::Stale;
    }
    return V4SnapshotStatus::Valid;
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
  m_receiver(m_predictor, params.uncertainty_params),
  m_uncertainty(params.uncertainty_params),
  m_pair_evaluator(params.evaluator_params),
  m_barrier_evaluator(params.evaluator_params),
  m_joint_evaluator(params.evaluator_params),
  m_exhaustive_evaluator(params.evaluator_params),
  m_activation_controller(params.activation_params),
  m_v4_safe_control(params.v4_safe_control_params),
  m_v4_candidate_adapter(params.v4_candidate_adapter_params)
{
    m_selected_candidate_ids.fill(kRollZeroId);
    m_latest_selection_decision.selected_candidate_ids.fill(kRollZeroId);
    m_latest_selection_decision.proposed_candidate_ids.fill(kRollZeroId);
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
            || !peer.decision.v4_cutover_candidate_ready) {
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
            if (m_v4_cutover_ready
                && (v4_candidates_valid || active || retain_selected_v4)) {
                buildV4IntentSet(
                    now_us, output.decision.v4_candidates, output);
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
        updateActivationState(
            now_us, selection_due || coordination_committed, output);
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
                            || !peer.decision.coordination_qualified) {
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
                const RemoteDecisionCache & peer =
                    m_remote_decision_caches[
                        static_cast<std::size_t>(remote_vehicle_id)];
                if (peer.valid && peer.decision.coordination_qualified) {
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
                                          kMaximumSelectionAircraft> & ids) {
        for (int aircraft = 0;
             aircraft < m_params.total_agent_count; ++aircraft) {
            if (ids[static_cast<std::size_t>(aircraft)]
                >= estimation::kManeuverCandidateCount) {
                return false;
            }
        }
        return true;
    };
    if (decision.ownship_candidate_id >= estimation::kManeuverCandidateCount
        || !candidatesValid(decision.selected_candidate_ids)
        || (decision.proposal_valid
            && !candidatesValid(decision.proposed_candidate_ids))) {
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
    if (decision.coordination_qualified
        && (decision.selected_candidate_ids[remote_index]
                != decision.ownship_candidate_id
            || decision.selected_candidate_input_revisions[remote_index]
                == 0)) {
        return false;
    }

    RemoteDecisionCache & cache = m_remote_decision_caches[
        static_cast<std::size_t>(remote_vehicle_id)];
    if (cache.valid
        && decision.proposal_epoch < cache.decision.proposal_epoch) {
        return false;
    }
    cache.decision = decision;
    cache.valid = true;
    if (m_params.execution_policy
        == ManeuverExecutionPolicy::HorizonGatedV4) {
        rollupV4HorizonGate(m_latest_state_timestamp_us);
    }
    if (decision.coordination_qualified) {
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
    for (std::size_t index = 0; index < candidate_count; ++index) {
        if (!m_sender.buildForSelectedCandidate(
                now_us,
                m_held_candidate_ids[index],
                m_latest_state,
                m_latest_covariance,
                packets[index],
                m_selection_epoch)) {
            m_ownship_candidates_complete = false;
            m_ownship_candidate_count = 0;
            return false;
        }
        packets[index].candidate_set_size = static_cast<std::uint8_t>(
            candidate_count);
        packets[index].candidate_set_kind =
            estimation::CandidateSetKind::LegacyRoll;
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
    output.intent_packets = packets;
    output.intent_packet_count = candidate_count;
    output.generated_timestamp_us = now_us;
    output.selection_epoch = m_selection_epoch;
    return true;
}

bool ManeuverSelectionWorker::buildV4IntentSet(
    std::uint64_t now_us,
    const SafeControlCandidateAdapterResult & candidates,
    ManeuverSelectionWorkerOutput & output)
{
    std::array<SafeControlCandidate, kMaximumSafeControlCandidates>
        selected_candidates{};
    std::size_t candidate_count = 0;
    const bool generated_safe_set =
        candidates.status == SafeControlCandidateAdapterStatus::Valid
        && candidates.candidate_count > 0
        && candidates.candidate_count <= selected_candidates.size();
    if (generated_safe_set) {
        candidate_count = candidates.candidate_count;
        std::copy_n(
            candidates.candidates.begin(),
            candidate_count,
            selected_candidates.begin());
    }

    const ManeuverActivationStatus activation =
        m_activation_controller.status();
    const bool retain_selected = activation.active
        || (m_has_selected_combination && m_selected_v4_cutover);
    if (retain_selected) {
        const std::size_t ownship_index = static_cast<std::size_t>(
            m_params.vehicle_id);
        const std::uint8_t retained_candidate_id = activation.active
            ? activation.latched_candidate_id
            : m_selected_candidate_ids[ownship_index];
        const std::uint64_t retained_input_revision = activation.active
            ? activation.latched_candidate_input_revision
            : m_selected_candidate_input_revisions[ownship_index];
        const estimation::PredictInput retained_input = activation.active
            ? activation.latched_input
            : m_latest_selection_decision.ownship_input;
        if (retained_candidate_id >= kMaximumSafeControlCandidates
            || retained_input_revision == 0) {
            return false;
        }
        const auto found = std::find_if(
            selected_candidates.begin(),
            selected_candidates.begin()
                + static_cast<std::ptrdiff_t>(candidate_count),
            [retained_candidate_id](const SafeControlCandidate & candidate) {
                return static_cast<std::uint8_t>(candidate.role)
                    == retained_candidate_id;
            });
        if (found != selected_candidates.begin()
                + static_cast<std::ptrdiff_t>(candidate_count)) {
            found->predictor_input = retained_input;
        } else if (candidate_count < selected_candidates.size()) {
            auto & retained = selected_candidates[candidate_count++];
            retained.role = static_cast<SafeCandidateRole>(
                retained_candidate_id);
            retained.predictor_input = retained_input;
        } else {
            return false;
        }
    }

    if (candidate_count == 0) {
        m_ownship_candidates_complete = false;
        m_ownship_candidate_count = 0;
        return false;
    }

    ExhaustiveCandidateIntentSet received_candidates{};
    std::array<
        estimation::TrajectoryIntentPacket,
        kExhaustiveCandidatesPerAircraft> packets{};
    for (std::size_t index = 0; index < candidate_count; ++index) {
        const std::uint8_t candidate_id = static_cast<std::uint8_t>(
            selected_candidates[index].role);
        if (!m_sender.buildForCandidateInput(
                now_us,
                candidate_id,
                selected_candidates[index].predictor_input,
                m_latest_state,
                m_latest_covariance,
                packets[index],
                m_selection_epoch)) {
                m_ownship_candidates_complete = false;
                m_ownship_candidate_count = 0;
                return false;
        }
        if (retain_selected && candidate_id == (activation.active
                ? activation.latched_candidate_id
                : m_selected_candidate_ids[static_cast<std::size_t>(
                    m_params.vehicle_id)])
            && packets[index].candidate_input_revision
                != (activation.active
                    ? activation.latched_candidate_input_revision
                    : m_selected_candidate_input_revisions[
                        static_cast<std::size_t>(m_params.vehicle_id)])) {
            m_ownship_candidates_complete = false;
            m_ownship_candidate_count = 0;
            return false;
        }
        packets[index].candidate_set_size = static_cast<std::uint8_t>(
            candidate_count);
        packets[index].candidate_set_kind =
            estimation::CandidateSetKind::V4SafeControl;
        if (!m_receiver.receive(packets[index], received_candidates[index])) {
            m_ownship_candidates_complete = false;
            m_ownship_candidate_count = 0;
            return false;
        }
    }

    const auto byCandidateId = [](const auto & lhs, const auto & rhs) {
        return lhs.candidate_id < rhs.candidate_id;
    };
    std::sort(
        received_candidates.begin(),
        received_candidates.begin()
            + static_cast<std::ptrdiff_t>(candidate_count),
        byCandidateId);
    std::sort(
        packets.begin(),
        packets.begin() + static_cast<std::ptrdiff_t>(candidate_count),
        byCandidateId);

    m_ownship_candidates = received_candidates;
    m_ownship_candidates_complete = true;
    m_ownship_candidate_count = candidate_count;
    m_ownship_candidate_set_kind =
        estimation::CandidateSetKind::V4SafeControl;
    output.intent_packets = packets;
    output.intent_packet_count = candidate_count;
    output.generated_timestamp_us = now_us;
    output.selection_epoch = m_selection_epoch;
    return true;
}

void ManeuverSelectionWorker::evaluateV4(
    std::uint64_t now_us,
    ManeuverSelectionWorkerOutput & output)
{
    ManeuverSelectionDecision decision = output.has_decision
        ? output.decision
        : m_latest_selection_decision;
    decision.vehicle_id = m_params.vehicle_id;
    decision.aircraft_count = static_cast<std::size_t>(
        m_params.total_agent_count);
    decision.v4_enabled = true;
    decision.v4_shadow_only = m_params.v4_shadow_only;
    decision.v4_shadow_evaluated = false;
    decision.v4_shadow_status = V4ShadowEvaluationStatus::Disabled;
    decision.v4_airspeed_source = V4AirspeedSource::Unavailable;
    decision.v4_px4_airspeed_source = m_has_latest_airspeed
        ? m_latest_airspeed.px4_airspeed_source
        : -1;
    decision.v4_airspeed_timestamp_us = m_has_latest_airspeed
        ? m_latest_airspeed.timestamp_us
        : 0;
    decision.v4_nominal_timestamp_us = m_has_latest_nominal
        ? m_latest_nominal.timestamp_us
        : 0;
    decision.v4_nominal_available = false;
    decision.v4_safe_control = SafeControlSetV4Result{};
    decision.v4_candidates = SafeControlCandidateAdapterResult{};
    // These are one-output transition events.  A diagnostic-only 20 Hz
    // publication must not repeat the previous activation edge.
    decision.activation_just_started = false;
    decision.activation_just_ended = false;
    decision.deactivation_reason = ManeuverDeactivationReason::None;

    const auto publishDiagnostics = [&]() {
        m_latest_selection_decision = decision;
        output.decision = decision;
        output.has_decision = true;
    };

    const bool airspeed_value_valid = m_has_latest_airspeed
        && m_latest_airspeed.valid
        && finitePositive(m_latest_airspeed.true_airspeed_mps);
    decision.v4_airspeed_snapshot_status = classifySnapshot(
        m_has_latest_airspeed,
        airspeed_value_valid,
        decision.v4_airspeed_timestamp_us,
        now_us,
        m_params.v4_maximum_airspeed_age_us,
        decision.v4_airspeed_age_us);

    double true_airspeed_mps = m_params.v4_trim_airspeed_mps;
    if (decision.v4_airspeed_snapshot_status == V4SnapshotStatus::Valid) {
        true_airspeed_mps = m_latest_airspeed.true_airspeed_mps;
        decision.v4_airspeed_source = V4AirspeedSource::ActualTas;
    } else {
        decision.v4_airspeed_source = V4AirspeedSource::TrimFallback;
    }

    const bool nominal_value_valid = m_has_latest_nominal
        && m_latest_nominal.valid
        && finitePositive(m_latest_nominal.ground_speed_command_mps)
        && (std::isfinite(m_latest_nominal.altitude_command_m)
            || std::isnan(m_latest_nominal.altitude_command_m))
        && std::isfinite(
            m_latest_nominal.lateral_acceleration_px4_mps2);
    decision.v4_nominal_snapshot_status = classifySnapshot(
        m_has_latest_nominal,
        nominal_value_valid,
        decision.v4_nominal_timestamp_us,
        now_us,
        m_params.v4_maximum_nominal_age_us,
        decision.v4_nominal_age_us);
    decision.v4_nominal_available =
        decision.v4_nominal_snapshot_status == V4SnapshotStatus::Valid;

    SafeControlSetV4Input safe_input;
    safe_input.ownship.timestamp_us = now_us;
    safe_input.ownship.north_m = m_latest_state.p_n;
    safe_input.ownship.east_m = m_latest_state.p_e;
    safe_input.ownship.heading_ned_rad = m_latest_state.psi;
    safe_input.ownship.true_airspeed_mps = true_airspeed_mps;
    safe_input.ownship.longitudinal_acceleration_mps2 = 0.0;
    safe_input.ownship.longitudinal_source =
        LongitudinalDriftSource::LocalOneStepFreeze;

    for (int remote_id = 0;
         remote_id < m_params.total_agent_count; ++remote_id) {
        if (remote_id == m_params.vehicle_id) {
            continue;
        }
        const std::size_t remote_index = static_cast<std::size_t>(remote_id);
        const RemoteDecisionCache & peer_decision =
            m_remote_decision_caches[remote_index];
        if (!peer_decision.valid
            || !peer_decision.decision.coordination_qualified) {
            decision.v4_shadow_status =
                V4ShadowEvaluationStatus::MissingPeerDecision;
            publishDiagnostics();
            return;
        }

        const std::uint8_t candidate_id =
            peer_decision.decision.ownship_candidate_id;
        const std::uint64_t candidate_input_revision =
            peer_decision.decision.selected_candidate_input_revisions[
                remote_index];
        const RemoteCandidateCache * selected_cache = nullptr;
        const estimation::ReceivedTrajectoryIntent * selected_intent = nullptr;
        for (const RemoteCandidateCache * cache : {
                 &m_remote_caches[remote_index],
                 &m_remote_previous_caches[remote_index]}) {
            if (cache->count == 0
                || cache->count != cache->expected_count) {
                continue;
            }
            const auto found = std::find_if(
                cache->candidates.begin(),
                cache->candidates.begin()
                    + static_cast<std::ptrdiff_t>(cache->count),
                [candidate_id, candidate_input_revision](
                    const auto & candidate) {
                    return candidate.candidate_id == candidate_id
                        && candidate.candidate_input_revision
                            == candidate_input_revision;
                });
            if (found != cache->candidates.begin()
                    + static_cast<std::ptrdiff_t>(cache->count)) {
                selected_cache = cache;
                selected_intent = &(*found);
                break;
            }
        }
        const RemoteSelectedIntentCache & retained =
            m_remote_selected_caches[remote_index];
        if (selected_intent == nullptr && retained.valid
            && retained.intent.candidate_id == candidate_id
            && retained.intent.candidate_input_revision
                == candidate_input_revision) {
            selected_intent = &retained.intent;
        }
        if (selected_intent == nullptr) {
            decision.v4_shadow_status =
                V4ShadowEvaluationStatus::MissingPeerIntent;
            publishDiagnostics();
            return;
        }
        if (selected_cache != nullptr
            && (selected_intent->source_timestamp_us
                    != selected_cache->source_timestamp_us
                || selected_intent->selection_epoch
                    != selected_cache->selection_epoch)) {
            decision.v4_shadow_status =
                V4ShadowEvaluationStatus::InvalidPeerIntent;
            publishDiagnostics();
            return;
        }

        IntentKinematics kinematics;
        const IntentKinematicsStatus interpolation_status =
            interpolateIntentKinematics(
                *selected_intent, now_us, kinematics);
        if (interpolation_status != IntentKinematicsStatus::Valid) {
            decision.v4_shadow_status = interpolation_status
                    == IntentKinematicsStatus::Future
                ? V4ShadowEvaluationStatus::FuturePeerIntent
                : interpolation_status == IntentKinematicsStatus::Stale
                    ? V4ShadowEvaluationStatus::StalePeerIntent
                    : V4ShadowEvaluationStatus::InvalidPeerIntent;
            publishDiagnostics();
            return;
        }

        auto & threat = safe_input.threats[safe_input.threat_count++];
        threat.vehicle_id = remote_id;
        // The intent was interpolated to the common ownship time above.
        threat.timestamp_us = now_us;
        threat.north_m = kinematics.position_ned[0];
        threat.east_m = kinematics.position_ned[1];
        threat.velocity_north_mps = kinematics.velocity_ned[0];
        threat.velocity_east_mps = kinematics.velocity_ned[1];
        threat.physical_clearance_m =
            m_params.evaluator_params.ownship_half_wingspan_m
            + m_params.evaluator_params.threat_half_wingspan_m;
    }

    decision.v4_safe_control = m_v4_safe_control.evaluate(safe_input);
    decision.v4_shadow_evaluated = true;
    decision.v4_shadow_status = V4ShadowEvaluationStatus::CoreEvaluated;

    if (v4HorizonFailClosedRequested(
            m_params.execution_policy,
            decision.v4_safe_control.status)) {
        // Failing to find a safe sampled-data interval is not evidence that
        // the raw nominal command is safe. Keep the last coordinated ownship
        // V4 command instead of failing open to point convergence.
        m_v4_horizon_local_gate_active = true;
        rollupV4HorizonGate(now_us);
        decision.v4_horizon_gate_evaluated = true;
        decision.v4_horizon_gate_valid = false;
        decision.v4_horizon_local_gate_active = true;
        decision.v4_horizon_gate_active = m_v4_horizon_gate_active;
        decision.v4_horizon_trigger_m = m_params.v4_horizon_trigger_m;
    }

    if (decision.v4_safe_control.status == SafeControlSetStatus::Valid
        || decision.v4_safe_control.status
            == SafeControlSetStatus::SearchSetInfeasible) {
        SafeControlCandidateAdapterInput candidate_input;
        candidate_input.safe_set = decision.v4_safe_control;
        candidate_input.true_airspeed_mps = true_airspeed_mps;
        candidate_input.nominal_rate_available =
            decision.v4_nominal_available;
        if (decision.v4_nominal_available) {
            candidate_input.nominal_heading_rate_v4_radps =
                SafeControlCandidateAdapter::
                    px4LateralAccelerationToV4HeadingRate(
                        true_airspeed_mps,
                        m_latest_nominal.lateral_acceleration_px4_mps2,
                        m_params.v4_candidate_adapter_params
                            .speed_tolerance_mps);
            if (!std::isfinite(
                    candidate_input.nominal_heading_rate_v4_radps)) {
                candidate_input.nominal_rate_available = false;
                decision.v4_nominal_available = false;
                decision.v4_nominal_snapshot_status =
                    V4SnapshotStatus::Invalid;
            }
        }
        candidate_input.ground_speed_command_mps =
            decision.v4_nominal_available
            ? m_latest_nominal.ground_speed_command_mps
            : m_params.ground_speed_command_mps;
        candidate_input.altitude_command_m = decision.v4_nominal_available
            ? m_latest_nominal.altitude_command_m
            : std::numeric_limits<double>::quiet_NaN();
        decision.v4_candidates =
            m_v4_candidate_adapter.generate(candidate_input);
        if (decision.v4_candidates.status
                != SafeControlCandidateAdapterStatus::Valid
            && decision.v4_candidates.status
                != SafeControlCandidateAdapterStatus::SearchSetInfeasible) {
            decision.v4_shadow_status =
                V4ShadowEvaluationStatus::CandidateGenerationFailed;
        }
    }

    publishDiagnostics();
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
    decision.proposed_v4_cutover = false;
    decision.proposal_consensus_confirmed = false;
    decision.new_best_accepted = false;
    decision.previous_best_retained = true;
    decision.coordination_qualified = m_has_selected_combination;
    decision.selected_candidate_ids = m_selected_candidate_ids;
    decision.selected_candidate_input_revisions =
        m_selected_candidate_input_revisions;
    decision.selected_candidate_source_timestamps_us =
        m_selected_candidate_source_timestamps_us;
    decision.selected_v4_cutover = m_selected_v4_cutover;
    decision.ownship_candidate_id = m_current_best_id;
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

    if (all_candidate_sets_complete
        && constrainActiveAircraftCandidates(
            candidate_sets, candidate_counts)) {
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
                proposed_candidate_ids = m_selected_candidate_ids;
            std::array<std::uint64_t, kMaximumSelectionAircraft>
                proposed_candidate_input_revisions =
                    m_selected_candidate_input_revisions;
            std::array<std::uint64_t, kMaximumSelectionAircraft>
                proposed_candidate_source_timestamps_us =
                    m_selected_candidate_source_timestamps_us;
            for (int aircraft = 0;
                 aircraft < m_params.total_agent_count; ++aircraft) {
                const std::size_t aircraft_index =
                    static_cast<std::size_t>(aircraft);
                const std::uint8_t candidate_slot =
                    best.candidate_slots[aircraft_index];
                const auto & proposed_candidate =
                    candidate_sets[aircraft_index][candidate_slot];
                proposed_candidate_ids[aircraft_index] =
                    proposed_candidate.candidate_id;
                proposed_candidate_input_revisions[aircraft_index] =
                    proposed_candidate.candidate_input_revision;
                proposed_candidate_source_timestamps_us[aircraft_index] =
                    proposed_candidate.source_timestamp_us;
            }
            m_pending_proposal.timestamp_us = now_us;
            m_pending_proposal.epoch = m_selection_epoch;
            m_pending_proposal.candidate_ids = proposed_candidate_ids;
            m_pending_proposal.candidate_input_revisions =
                proposed_candidate_input_revisions;
            m_pending_proposal.candidate_source_timestamps_us =
                proposed_candidate_source_timestamps_us;
            m_pending_proposal.v4_cutover =
                required_set_kind
                == estimation::CandidateSetKind::V4SafeControl;
            const std::uint8_t ownship_slot = best.candidate_slots[
                static_cast<std::size_t>(m_params.vehicle_id)];
            m_pending_proposal.ownship_input = candidate_sets[
                static_cast<std::size_t>(m_params.vehicle_id)][ownship_slot]
                    .candidate_input;
            m_pending_proposal.evaluation = best;
            m_pending_proposal.combination_index = best_combination_index;
            m_pending_proposal.combination_count = combination_count;
            m_pending_proposal.valid = true;
            m_pending_proposal.resolved = false;

            decision.proposed_candidate_ids = proposed_candidate_ids;
            decision.proposed_candidate_input_revisions =
                proposed_candidate_input_revisions;
            decision.proposed_candidate_source_timestamps_us =
                proposed_candidate_source_timestamps_us;
            decision.proposed_v4_cutover =
                m_pending_proposal.v4_cutover;
            decision.proposal_valid = true;
            decision.evaluated_combination_count = combination_count;
        }
    }

    m_latest_selection_decision = decision;
    output.decision = decision;
    output.has_decision = true;
}

bool ManeuverSelectionWorker::evaluateV4HorizonGate(
    std::uint64_t now_us,
    const MultiAircraftExhaustiveCandidateIntentSets & candidate_sets,
    const std::array<std::size_t, kMaximumSelectionAircraft>
        & candidate_counts,
    ManeuverSelectionDecision & decision)
{
    decision.v4_horizon_gate_evaluated = true;
    decision.v4_horizon_gate_valid = false;
    decision.v4_horizon_local_gate_active =
        m_v4_horizon_local_gate_active;
    decision.v4_horizon_gate_active = m_v4_horizon_gate_active;
    decision.v4_horizon_h_worst_m =
        std::numeric_limits<double>::quiet_NaN();
    decision.v4_horizon_trigger_m = m_params.v4_horizon_trigger_m;
    decision.v4_horizon_worst_time_offset_s =
        std::numeric_limits<double>::quiet_NaN();
    decision.v4_horizon_worst_first_vehicle_id = -1;
    decision.v4_horizon_worst_second_vehicle_id = -1;

    std::array<
        const estimation::ReceivedTrajectoryIntent *,
        kMaximumSelectionAircraft> near_nominal{};
    const std::uint8_t near_nominal_id = static_cast<std::uint8_t>(
        SafeCandidateRole::NearNominal);
    for (int aircraft = 0; aircraft < m_params.total_agent_count; ++aircraft) {
        const std::size_t aircraft_index = static_cast<std::size_t>(aircraft);
        if (candidate_counts[aircraft_index] == 0
            || candidate_counts[aircraft_index]
                > kExhaustiveCandidatesPerAircraft) {
            return false;
        }
        const auto begin = candidate_sets[aircraft_index].begin();
        const auto end = begin + static_cast<std::ptrdiff_t>(
            candidate_counts[aircraft_index]);
        const auto found = std::find_if(
            begin, end, [near_nominal_id](const auto & candidate) {
                return candidate.candidate_id == near_nominal_id;
            });
        if (found == end) {
            return false;
        }
        near_nominal[aircraft_index] = &*found;
    }

    double h_worst_m = std::numeric_limits<double>::infinity();
    for (int first = 0; first < m_params.total_agent_count; ++first) {
        for (int second = first + 1;
             second < m_params.total_agent_count; ++second) {
            TrajectoryBarrierEvaluation pair;
            if (!m_barrier_evaluator.evaluateTrajectoryPair(
                    now_us,
                    *near_nominal[static_cast<std::size_t>(first)],
                    *near_nominal[static_cast<std::size_t>(second)],
                    pair)) {
                return false;
            }
            if (pair.minimum_clearance_m < h_worst_m) {
                h_worst_m = pair.minimum_clearance_m;
                decision.v4_horizon_worst_time_offset_s =
                    pair.minimum_clearance_time_offset_s;
                decision.v4_horizon_worst_first_vehicle_id = first;
                decision.v4_horizon_worst_second_vehicle_id = second;
            }
        }
    }
    if (!std::isfinite(h_worst_m)) {
        return false;
    }

    m_v4_horizon_local_gate_active = updateV4HorizonGateState(
        m_v4_horizon_local_gate_active,
        true,
        h_worst_m,
        m_params.v4_horizon_trigger_m);
    rollupV4HorizonGate(now_us);
    decision.v4_horizon_gate_valid = true;
    decision.v4_horizon_local_gate_active =
        m_v4_horizon_local_gate_active;
    decision.v4_horizon_gate_active = m_v4_horizon_gate_active;
    decision.v4_horizon_h_worst_m = h_worst_m;
    return true;
}

void ManeuverSelectionWorker::rollupV4HorizonGate(
    std::uint64_t now_us) noexcept
{
    bool aggregate_request = m_v4_horizon_local_gate_active;
    for (int aircraft = 0; aircraft < m_params.total_agent_count; ++aircraft) {
        if (aircraft == m_params.vehicle_id) {
            continue;
        }
        const RemoteDecisionCache & cache = m_remote_decision_caches[
            static_cast<std::size_t>(aircraft)];
        aggregate_request = aggregate_request
            || (cache.valid
                && cache.decision.v4_horizon_local_gate_active);
    }

    if (!m_v4_horizon_gate_active && aggregate_request) {
        if (latchV4HorizonOwnshipCandidate()) {
            m_v4_horizon_gate_active = true;
            m_v4_horizon_activation_timestamp_us = now_us;
        }
    } else if (m_v4_horizon_gate_active && !aggregate_request
        && v4HorizonHoldElapsed(
            m_v4_horizon_activation_timestamp_us,
            now_us,
            kTrajectoryHorizonMicroseconds)) {
        m_v4_horizon_gate_active = false;
        m_v4_horizon_activation_timestamp_us = 0;
        m_v4_horizon_latch_valid = false;
        m_v4_horizon_latched_input_revision = 0;
    }
    m_latest_selection_decision.v4_horizon_local_gate_active =
        m_v4_horizon_local_gate_active;
    m_latest_selection_decision.v4_horizon_gate_active =
        m_v4_horizon_gate_active;
}

bool ManeuverSelectionWorker::latchV4HorizonOwnshipCandidate() noexcept
{
    if (!m_has_selected_combination || !m_selected_v4_cutover) {
        return false;
    }
    const std::size_t ownship_index = static_cast<std::size_t>(
        m_params.vehicle_id);
    const std::uint8_t candidate_id =
        m_selected_candidate_ids[ownship_index];
    const std::uint64_t input_revision =
        m_selected_candidate_input_revisions[ownship_index];
    if (candidate_id >= kMaximumSafeControlCandidates
        || input_revision == 0) {
        return false;
    }
    m_v4_horizon_latched_candidate_id = candidate_id;
    m_v4_horizon_latched_input_revision = input_revision;
    m_v4_horizon_latched_input = m_latest_selection_decision.ownship_input;
    m_v4_horizon_latch_valid = true;
    return true;
}

bool ManeuverSelectionWorker::constrainActiveAircraftCandidates(
    MultiAircraftExhaustiveCandidateIntentSets & candidate_sets,
    const std::array<std::size_t, kMaximumSelectionAircraft>
        & candidate_counts) const
{
    for (int aircraft = 0;
         aircraft < m_params.total_agent_count; ++aircraft) {
        const std::size_t aircraft_index = static_cast<std::size_t>(aircraft);
        const std::size_t candidate_count = candidate_counts[aircraft_index];
        if (candidate_count == 0
            || candidate_count > kExhaustiveCandidatesPerAircraft) {
            return false;
        }
        bool active = false;
        std::uint8_t active_candidate_id = kRollZeroId;
        std::uint64_t active_candidate_input_revision = 0;
        if (aircraft == m_params.vehicle_id) {
            const ManeuverActivationStatus status =
                m_activation_controller.status();
            if (m_params.execution_policy
                    == ManeuverExecutionPolicy::HorizonGatedV4
                && m_v4_horizon_gate_active
                && m_v4_horizon_latch_valid) {
                active = true;
                active_candidate_id = m_v4_horizon_latched_candidate_id;
                active_candidate_input_revision =
                    m_v4_horizon_latched_input_revision;
            } else {
                active = status.active;
                active_candidate_id = status.latched_candidate_id;
                active_candidate_input_revision =
                    status.latched_candidate_input_revision;
            }
        } else {
            const RemoteDecisionCache & cache =
                m_remote_decision_caches[aircraft_index];
            active = cache.valid
                && cache.decision.coordination_qualified
                && cache.decision.selected_v4_cutover
                    == m_selected_v4_cutover
                && (cache.decision.activation_requested
                    || (m_params.execution_policy
                            == ManeuverExecutionPolicy::HorizonGatedV4
                        && cache.decision.command_execution_requested));
            active_candidate_id = cache.decision.ownship_candidate_id;
            active_candidate_input_revision =
                cache.decision.selected_candidate_input_revisions[
                    aircraft_index];
        }
        if (!active) {
            continue;
        }

        auto & candidates = candidate_sets[aircraft_index];
        const auto found = std::find_if(
            candidates.begin(),
            candidates.begin() + static_cast<std::ptrdiff_t>(candidate_count),
            [active_candidate_id, active_candidate_input_revision](
                const auto & candidate) {
                return candidate.candidate_id == active_candidate_id
                    && candidate.candidate_input_revision
                        == active_candidate_input_revision;
            });
        if (found == candidates.begin()
                + static_cast<std::ptrdiff_t>(candidate_count)) {
            return false;
        }
        const estimation::ReceivedTrajectoryIntent active_candidate = *found;
        std::fill_n(candidates.begin(), candidate_count, active_candidate);
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
        if (cache.decision.proposed_candidate_ids
                != m_pending_proposal.candidate_ids
            || cache.decision.proposed_candidate_input_revisions
                != m_pending_proposal.candidate_input_revisions
            || cache.decision.proposed_v4_cutover
                != m_pending_proposal.v4_cutover
            || (cache.decision.activation_requested
                && (cache.decision.proposed_candidate_ids[
                        static_cast<std::size_t>(aircraft)]
                        != cache.decision.ownship_candidate_id
                    || cache.decision.proposed_candidate_input_revisions[
                        static_cast<std::size_t>(aircraft)]
                        != cache.decision
                            .selected_candidate_input_revisions[
                                static_cast<std::size_t>(aircraft)]
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
    if (activation.active
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
        || m_pending_proposal.candidate_ids != m_selected_candidate_ids
        || m_pending_proposal.candidate_input_revisions
            != m_selected_candidate_input_revisions;
    m_selected_candidate_ids = m_pending_proposal.candidate_ids;
    m_selected_candidate_input_revisions =
        m_pending_proposal.candidate_input_revisions;
    m_selected_candidate_source_timestamps_us =
        authoritative_source_timestamps_us;
    m_selected_v4_cutover = m_pending_proposal.v4_cutover;
    m_has_selected_combination = true;
    m_current_best_id = activation.active
        ? activation.latched_candidate_id
        : m_v4_horizon_gate_active && m_v4_horizon_latch_valid
            ? m_v4_horizon_latched_candidate_id
            : m_selected_candidate_ids[ownship_index];

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
    decision.selected_candidate_ids = m_selected_candidate_ids;
    decision.selected_candidate_input_revisions =
        m_selected_candidate_input_revisions;
    decision.selected_candidate_source_timestamps_us =
        m_selected_candidate_source_timestamps_us;
    decision.selected_v4_cutover = m_selected_v4_cutover;
    decision.ownship_candidate_id = m_current_best_id;
    decision.pmr_m = m_pending_proposal.evaluation.minimum_pmr_m;
    decision.masd_m = m_pending_proposal.evaluation.minimum_masd_m;
    decision.ad_m = m_pending_proposal.evaluation.minimum_ad_m;
    decision.reciprocal_cost_sum =
        m_pending_proposal.evaluation.reciprocal_cost_sum;
    decision.coordination_qualified = true;
    decision.new_best_accepted = changed;
    decision.previous_best_retained = !changed;
    decision.ownship_input = m_pending_proposal.ownship_input;
    for (int aircraft = 0;
         aircraft < m_params.total_agent_count; ++aircraft) {
        if (aircraft != m_params.vehicle_id) {
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

bool ManeuverSelectionWorker::buildActivationSample(
    std::uint64_t now_us,
    ManeuverActivationSample & sample,
    ManeuverSelectionDecision & decision) const
{
    sample = ManeuverActivationSample{};
    sample.timestamp_us = now_us;
    sample.separation_rates_mps.fill(
        std::numeric_limits<double>::quiet_NaN());
    if (!m_has_selected_combination || !m_ownship_candidates_complete
        || m_ownship_candidate_count == 0
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
    const std::uint8_t ownship_candidate_id =
        m_selected_candidate_ids[ownship_index];
    const std::uint64_t ownship_candidate_input_revision =
        m_selected_candidate_input_revisions[ownship_index];
    const estimation::ReceivedTrajectoryIntent * ownship_intent =
        find_candidate(
            m_ownship_candidates,
            m_ownship_candidate_count,
            ownship_candidate_id,
            ownship_candidate_input_revision);
    IntentKinematics ownship_kinematics;
    if (ownship_intent == nullptr
        || interpolateIntentKinematics(
            *ownship_intent, now_us, ownship_kinematics)
            != IntentKinematicsStatus::Valid) {
        return false;
    }

    sample.selected_candidate_id = ownship_candidate_id;
    sample.selected_candidate_input_revision =
        ownship_candidate_input_revision;
    sample.selected_input = ownship_intent->candidate_input;
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
        const std::uint64_t remote_candidate_input_revision =
            m_selected_candidate_input_revisions[remote_index];
        const RemoteCandidateCache * cache = &m_remote_caches[remote_index];
        const estimation::ReceivedTrajectoryIntent * remote_intent = nullptr;
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
        if (pair.ad_m < minimum_ad) {
            minimum_ad = pair.ad_m;
            decision.pmr_m = pair.pmr_m;
            decision.masd_m = pair.masd_m;
            decision.threat_candidate_id = remote_candidate_id;
        }
        if (pair.ad_m
            < m_params.activation_params.activation_threshold_m) {
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
                == IntentKinematicsStatus::Valid
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
