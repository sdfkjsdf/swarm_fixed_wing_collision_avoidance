#include <collision_avoidance/selection/ManeuverSelectionWorker.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>
#include <limits>
#include <memory>
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
    estimation::kTrajectoryIntentHorizonSeconds;
constexpr std::uint64_t kTrajectoryHorizonMicroseconds =
    static_cast<std::uint64_t>(kTrajectoryHorizonSeconds * 1.0e6);
constexpr std::uint64_t kInteractionFnvOffset = 14695981039346656037ULL;
constexpr std::uint64_t kInteractionFnvPrime = 1099511628211ULL;

std::uint64_t assembledCandidateHash(
    std::uint64_t graph_hash,
    const std::array<std::uint8_t, kMaximumSelectionAircraft> & candidate_ids,
    std::uint32_t candidate_valid_mask,
    std::size_t aircraft_count) noexcept
{
    std::uint64_t hash = kInteractionFnvOffset;
    const auto mix = [&hash](std::uint8_t byte) {
        hash ^= byte;
        hash *= kInteractionFnvPrime;
    };
    for (std::size_t byte = 0; byte < sizeof(graph_hash); ++byte) {
        mix(static_cast<std::uint8_t>(graph_hash >> (byte * 8U)));
    }
    for (std::size_t index = 0; index < aircraft_count; ++index) {
        mix(static_cast<std::uint8_t>(
            (candidate_valid_mask >> index) & std::uint32_t{1}));
        if ((candidate_valid_mask & (std::uint32_t{1} << index)) == 0U) {
            continue;
        }
        mix(candidate_ids[index]);
    }
    return hash;
}

std::uint32_t candidateMaskForAircraftCount(
    const std::size_t aircraft_count) noexcept
{
    return aircraft_count >= 32U
        ? std::numeric_limits<std::uint32_t>::max()
        : (std::uint32_t{1} << aircraft_count) - std::uint32_t{1};
}

bool candidateIsValid(
    const std::uint32_t candidate_valid_mask,
    const std::size_t aircraft_index) noexcept
{
    return aircraft_index < 32U
        && (candidate_valid_mask & (std::uint32_t{1} << aircraft_index)) != 0U;
}

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

bool nominalPredictInput(
    const ManeuverSelectionNominalSetpointSnapshot & snapshot,
    std::uint64_t now_us,
    std::uint64_t maximum_age_us,
    estimation::PredictInput & input) noexcept
{
    if (!snapshot.valid || snapshot.timestamp_us == 0
        || snapshot.timestamp_us > now_us
        || now_us - snapshot.timestamp_us > maximum_age_us
        || !finitePositive(snapshot.ground_speed_command_mps)
        || (!std::isfinite(snapshot.altitude_command_m)
            && !std::isnan(snapshot.altitude_command_m))
        || !std::isfinite(snapshot.lateral_acceleration_px4_mps2)) {
        return false;
    }
    input.V_cmd = snapshot.ground_speed_command_mps;
    input.h_cmd = snapshot.altitude_command_m;
    input.h_dot_cmd = 0.0;
    input.a_lat_cmd = snapshot.lateral_acceleration_px4_mps2;
    return true;
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
        || (params.interaction_graph_component_cutover_enabled
            && (!params.interaction_graph_params.enabled
                || !params.exhaustive_test_mode
                || params.execution_policy
                    != ManeuverExecutionPolicy::AmacAdThreshold))
        || (params.interaction_graph_params.enabled
            && (!std::isfinite(
                    params.interaction_graph_params.ad_screen_m)
                || params.interaction_graph_params
                        .trajectory_library_version == 0
                || params.interaction_graph_params
                        .ad_masd_config_version == 0
                || params.interaction_graph_params.config_version == 0))
        || !finitePositive(
            params.activation_params
                .relative_speed_epsilon_mps)
        || !finitePositive(params.activation_params.cpa_horizon_s)
        || (params.formation_discrimination_enabled
            && (params.execution_policy
                    != ManeuverExecutionPolicy::AmacAdThreshold
                || !finitePositive(params.formation_target_separation_m)
                || params.formation_target_separation_m
                    <= params.evaluator_params
                            .desired_separation_distance_m
                        + params.evaluator_params.ownship_half_wingspan_m
                        + params.evaluator_params.threat_half_wingspan_m
                || !formation::FormationDiscriminator::validConfig(
                    params.formation_boundary_config)))
        || (params.active_switching_enabled
            && (!finitePositive(params.active_switch_cost_margin)
                || !finitePositive(
                    params.active_switch_minimum_ad_margin_m)))
        || (v4_execution_policy
            && (!params.v4_safe_control_enabled || params.v4_shadow_only))
        || (params.execution_policy == ManeuverExecutionPolicy::HorizonGatedV4
            && (!finitePositive(params.v4_horizon_trigger_m)
                || !params.evaluator_params.robust_cone_filter_enabled
                || params.v4_control_architecture
                    != V4ControlArchitecture::LegacySafeControlSet))
        || (params.v4_safe_control_enabled
            && (!finitePositive(params.v4_trim_airspeed_mps)
                || params.v4_maximum_airspeed_age_us == 0
                || params.v4_maximum_nominal_age_us == 0
                || !SafeControlCandidateAdapter::validParams(
                    params.v4_candidate_adapter_params)
                || (params.v4_control_architecture
                        == V4ControlArchitecture::LegacySafeControlSet
                    && !SafeControlSetV4::validParams(
                        params.v4_safe_control_params))
                || (params.v4_control_architecture
                        == V4ControlArchitecture::ClosedFormBackupModeB
                    && (!BackupControlInterpolatorV4::validParams(
                            params.mode_b_interpolator_params)
                        || !BackupThreatIntentAdapterV4::validParams(
                            params.mode_b_intent_adapter_params)
                        || std::abs(
                            params.mode_b_interpolator_params.certifier.horizon_s
                            - params.mode_b_intent_adapter_params.horizon_s)
                            > params.mode_b_interpolator_params.certifier
                                .threat_time_tolerance_s
                        || std::abs(
                            params.mode_b_interpolator_params.certifier
                                .integration_step_s
                            - params.mode_b_intent_adapter_params
                                .integration_step_s)
                            > params.mode_b_interpolator_params.certifier
                                .threat_time_tolerance_s))))
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

IntentKinematicsStatus propagateIntentInitialKinematics(
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

    // The graph must be independent of a candidate maneuver. Every intent in
    // one aircraft's set has the same t=0 state, so align that measured state
    // to the common evaluation time with one deterministic constant-velocity
    // rule instead of following an arbitrary candidate trajectory.
    const auto & state = intent.reconstructed_mean.front();
    if (!std::isfinite(state.p_n) || !std::isfinite(state.p_e)
        || !std::isfinite(state.h) || !std::isfinite(state.V)
        || !std::isfinite(state.h_dot) || !std::isfinite(state.psi)) {
        return IntentKinematicsStatus::Invalid;
    }
    const double horizontal_speed = std::sqrt(std::max(
        0.0, state.V * state.V - state.h_dot * state.h_dot));
    kinematics.velocity_ned = {
        horizontal_speed * std::cos(state.psi),
        horizontal_speed * std::sin(state.psi),
        -state.h_dot};
    kinematics.position_ned = {
        state.p_n + kinematics.velocity_ned[0] * age_s,
        state.p_e + kinematics.velocity_ned[1] * age_s,
        -state.h + kinematics.velocity_ned[2] * age_s};
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
                const std::size_t remote_index = static_cast<std::size_t>(
                    remote_vehicle_id);
                const std::uint64_t delivery_budget_us =
                    3 * m_params.trajectory_refresh_period_us;
                const std::uint64_t freeze_offset_us =
                    m_params.coordination_delay_us > delivery_budget_us
                    ? m_params.coordination_delay_us - delivery_budget_us
                    : 0;
                const std::uint64_t freeze_cutoff_timestamp_us =
                    m_epoch_generation_timestamp_us + freeze_offset_us;
                if (m_params.interaction_graph_params.enabled
                    && remote_cache.selection_epoch == m_selection_epoch
                    && remote_cache.candidate_set_kind
                        == estimation::CandidateSetKind::LegacyRoll
                    && remote_cache.count
                        == kExhaustiveCandidatesPerAircraft
                    && remote_cache.source_timestamp_us
                        <= freeze_cutoff_timestamp_us
                    && (!m_epoch_certification_candidate_ready[remote_index]
                        || remote_cache.source_timestamp_us
                            > (*m_epoch_certification_candidate_sets)
                                [remote_index][0].source_timestamp_us)) {
                    if (!m_epoch_certification_candidate_sets) {
                        m_epoch_certification_candidate_sets =
                            std::make_unique<
                                MultiAircraftExhaustiveCandidateIntentSets>();
                    }
                    (*m_epoch_certification_candidate_sets)[remote_index] =
                        remote_cache.candidates;
                    m_epoch_certification_candidate_counts[remote_index] =
                        remote_cache.count;
                    m_epoch_certification_candidate_ready[remote_index] = true;
                }
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
    const bool active_isolated_candidate = decision.activation_requested
        && decision.ownship_candidate_valid && !selected_ownship_bit;
    if ((!selected_ownship_bit && decision.ownship_candidate_valid
            && !active_isolated_candidate)
        || (selected_ownship_bit && !decision.ownship_candidate_valid)
        || (decision.coordination_qualified && selected_ownship_bit
            && (decision.selected_candidate_ids[remote_index]
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
    m_epoch_certification_candidate_counts.fill(0);
    m_epoch_certification_candidate_ready.fill(false);
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
    std::copy_n(
        packets.begin(), candidate_count, output.intent_packets.begin());
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
    decision.v4_control_architecture = m_params.v4_control_architecture;
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
    decision.mode_b_threat_status = BackupThreatIntentStatusV4::Valid;
    decision.mode_b_invalid_threat_vehicle_id = -1;
    decision.mode_b_interpolation_status =
        BackupInterpolationStatusV4::InvalidConfiguration;
    decision.mode_b_branch_classification =
        BackupBranchClassificationV4::NeitherCertified;
    decision.mode_b_left_certified = false;
    decision.mode_b_right_certified = false;
    decision.mode_b_left_minimum_path_margin_m =
        std::numeric_limits<double>::quiet_NaN();
    decision.mode_b_right_minimum_path_margin_m =
        std::numeric_limits<double>::quiet_NaN();
    decision.mode_b_left_terminal_turn_margin_m =
        std::numeric_limits<double>::quiet_NaN();
    decision.mode_b_right_terminal_turn_margin_m =
        std::numeric_limits<double>::quiet_NaN();
    decision.mode_b_left_interpolation_status =
        BackupInterpolationBranchStatusV4::NotCertified;
    decision.mode_b_right_interpolation_status =
        BackupInterpolationBranchStatusV4::NotCertified;
    decision.mode_b_left_mu_star = std::numeric_limits<double>::quiet_NaN();
    decision.mode_b_right_mu_star = std::numeric_limits<double>::quiet_NaN();
    decision.mode_b_left_safe_rate_radps =
        std::numeric_limits<double>::quiet_NaN();
    decision.mode_b_right_safe_rate_radps =
        std::numeric_limits<double>::quiet_NaN();
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

    if (m_params.v4_control_architecture
            == V4ControlArchitecture::ClosedFormBackupModeB) {
        if (!decision.v4_nominal_available) {
            // Mode B interpolates from the actual nominal command. Missing or
            // stale nominal data cannot be replaced by an invented zero-rate
            // command; an already coordinated command is retained upstream.
            decision.v4_shadow_status =
                V4ShadowEvaluationStatus::CandidateGenerationFailed;
            publishDiagnostics();
            return;
        }

        const double nominal_heading_rate_v4_radps =
            SafeControlCandidateAdapter::
                px4LateralAccelerationToV4HeadingRate(
                    true_airspeed_mps,
                    m_latest_nominal.lateral_acceleration_px4_mps2,
                    m_params.v4_candidate_adapter_params
                        .speed_tolerance_mps);
        if (!std::isfinite(nominal_heading_rate_v4_radps)) {
            decision.v4_nominal_available = false;
            decision.v4_nominal_snapshot_status = V4SnapshotStatus::Invalid;
            decision.v4_shadow_status =
                V4ShadowEvaluationStatus::CandidateGenerationFailed;
            publishDiagnostics();
            return;
        }

        BackupSafetyCertifierV4Input backup_input;
        backup_input.ownship.timestamp_us = now_us;
        backup_input.ownship.north_m = m_latest_state.p_n;
        backup_input.ownship.east_m = m_latest_state.p_e;
        backup_input.ownship.heading_ned_rad = m_latest_state.psi;
        backup_input.ownship.true_airspeed_mps = true_airspeed_mps;
        backup_input.ownship.longitudinal_acceleration_mps2 = 0.0;
        backup_input.ownship.longitudinal_source =
            LongitudinalDriftSource::LocalOneStepFreeze;

        for (int remote_id = 0;
             remote_id < m_params.total_agent_count; ++remote_id) {
            if (remote_id == m_params.vehicle_id) {
                continue;
            }
            const std::size_t remote_index = static_cast<std::size_t>(
                remote_id);
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
            const estimation::ReceivedTrajectoryIntent * selected_intent =
                nullptr;
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

            const auto aligned = m_mode_b_intent_adapter.alignAndPropagate(
                now_us,
                remote_id,
                m_params.evaluator_params.ownship_half_wingspan_m
                    + m_params.evaluator_params.threat_half_wingspan_m,
                *selected_intent);
            decision.mode_b_threat_status = aligned.status;
            if (aligned.status != BackupThreatIntentStatusV4::Valid) {
                decision.mode_b_invalid_threat_vehicle_id = remote_id;
                decision.v4_shadow_status = aligned.status
                        == BackupThreatIntentStatusV4::FutureIntent
                    ? V4ShadowEvaluationStatus::FuturePeerIntent
                    : aligned.status == BackupThreatIntentStatusV4::StaleIntent
                        ? V4ShadowEvaluationStatus::StalePeerIntent
                        : V4ShadowEvaluationStatus::InvalidPeerIntent;
                publishDiagnostics();
                return;
            }
            if (backup_input.threat_count >= backup_input.threats.size()) {
                decision.mode_b_invalid_threat_vehicle_id = remote_id;
                decision.mode_b_threat_status =
                    BackupThreatIntentStatusV4::InvalidIntent;
                decision.v4_shadow_status =
                    V4ShadowEvaluationStatus::InvalidPeerIntent;
                publishDiagnostics();
                return;
            }
            backup_input.threats[backup_input.threat_count++] =
                aligned.trajectory;
        }

        const auto interpolation = m_mode_b_interpolator.evaluate(
            backup_input, nominal_heading_rate_v4_radps);
        decision.v4_shadow_evaluated = true;
        decision.v4_shadow_status = V4ShadowEvaluationStatus::CoreEvaluated;
        decision.mode_b_interpolation_status = interpolation.status;
        decision.mode_b_branch_classification =
            interpolation.certification.classification;
        decision.mode_b_left_certified =
            interpolation.certification.left.certified;
        decision.mode_b_right_certified =
            interpolation.certification.right.certified;
        decision.mode_b_left_minimum_path_margin_m =
            interpolation.certification.left.minimum_path_margin_m;
        decision.mode_b_right_minimum_path_margin_m =
            interpolation.certification.right.minimum_path_margin_m;
        decision.mode_b_left_terminal_turn_margin_m =
            interpolation.certification.left.terminal_turn_margin_m;
        decision.mode_b_right_terminal_turn_margin_m =
            interpolation.certification.right.terminal_turn_margin_m;
        decision.mode_b_left_interpolation_status =
            interpolation.left.status;
        decision.mode_b_right_interpolation_status =
            interpolation.right.status;
        decision.mode_b_left_mu_star = interpolation.left.mu_star;
        decision.mode_b_right_mu_star = interpolation.right.mu_star;
        decision.mode_b_left_safe_rate_radps =
            interpolation.left.safe_heading_rate_v4_radps;
        decision.mode_b_right_safe_rate_radps =
            interpolation.right.safe_heading_rate_v4_radps;

        BackupControlCandidateAdapterInputV4 candidate_input;
        candidate_input.interpolation = interpolation;
        candidate_input.true_airspeed_mps = true_airspeed_mps;
        candidate_input.ground_speed_command_mps =
            m_latest_nominal.ground_speed_command_mps;
        candidate_input.altitude_command_m =
            m_latest_nominal.altitude_command_m;
        decision.v4_candidates =
            m_v4_candidate_adapter.generateFromBackupInterpolation(
                candidate_input);
        if (decision.v4_candidates.status
                != SafeControlCandidateAdapterStatus::Valid
            && decision.v4_candidates.status
                != SafeControlCandidateAdapterStatus::SearchSetInfeasible) {
            decision.v4_shadow_status =
                V4ShadowEvaluationStatus::CandidateGenerationFailed;
        }
        publishDiagnostics();
        return;
    }

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

    evaluateInteractionGraphShadow(now_us);

    // Component search obtains its fleet-wide rejoin objective from the
    // certified candidate library.  A local objective is only meaningful on
    // the legacy evaluator path.
    ManeuverRejoinObjective legacy_rejoin_objective;
    const ManeuverRejoinObjective * legacy_rejoin_objective_ptr = nullptr;
    if (!m_params.interaction_graph_component_cutover_enabled
        && m_safe_rejoin_active
        && buildManeuverRejoinObjective(now_us, legacy_rejoin_objective)) {
        legacy_rejoin_objective_ptr = &legacy_rejoin_objective;
    }

    // Active component cutover must execute from the exact candidate library
    // that was certified. The 20 Hz live cache may already contain a newer
    // trajectory revision from the same 4 Hz epoch.
    if (m_params.interaction_graph_component_cutover_enabled
        && m_epoch_certification_candidate_sets
        && m_pending_interaction_graph_diagnostics
        && m_pending_interaction_graph_diagnostics->shadow_search_evaluated) {
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
            m_params.interaction_graph_component_cutover_enabled;
        std::array<std::uint8_t, kMaximumSelectionAircraft>
            component_candidate_ids{};
        std::uint32_t component_candidate_valid_mask{0};
        if (component_cutover) {
            const bool component_result_ready =
                m_pending_interaction_graph_diagnostics
                && m_pending_interaction_graph_diagnostics
                    ->shadow_search_evaluated
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
                evaluated = best.valid && best.all_pairs_feasible;
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
            if (m_pending_interaction_graph_diagnostics) {
                auto & graph_diagnostics =
                    *m_pending_interaction_graph_diagnostics;
                if (component_cutover) {
                    graph_diagnostics.component_proposal_used = true;
                } else {
                    graph_diagnostics.legacy_proposal_valid = true;
                    graph_diagnostics.legacy_proposed_candidate_ids =
                        proposed_candidate_ids;
                }
            }
            const bool active_command_change =
                proposalChangesActiveCommand(
                    proposed_candidate_ids,
                    proposed_candidate_input_revisions,
                    proposed_candidate_valid_mask);
            JointCombinationEvaluation current_evaluation{};
            std::array<std::uint8_t, kMaximumSelectionAircraft>
                incumbent_candidate_ids{};
            bool common_incumbent_available = false;
            if (!component_cutover) {
                common_incumbent_available =
                    m_params.execution_policy
                            != ManeuverExecutionPolicy::AmacAdThreshold
                        || buildCommonIncumbentCandidateIds(
                            incumbent_candidate_ids);
            }
            const bool current_evaluation_available = active_command_change
                && !component_cutover
                && common_incumbent_available
                && (m_params.execution_policy
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
                            current_evaluation));
            const bool current_evaluation_valid =
                current_evaluation_available && current_evaluation.valid;
            if (current_evaluation_valid
                && legacy_rejoin_objective_ptr != nullptr) {
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
                        - legacy_rejoin_objective_ptr
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
                && legacy_rejoin_objective_ptr != nullptr
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
                && !component_cutover
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
            // A component proposal has already passed the common certified
            // search and global all-pair check.  Only the legacy path performs
            // an additional incumbent-superiority comparison.
            const bool active_change_allowed = !active_command_change
                || (component_cutover
                    ? m_params.active_switching_enabled : clearly_superior);
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
            if (m_params.interaction_graph_component_cutover_enabled
                && m_pending_interaction_graph_diagnostics
                && m_pending_interaction_graph_diagnostics
                    ->shadow_search_evaluated) {
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

void ManeuverSelectionWorker::evaluateInteractionGraphShadow(
    std::uint64_t now_us)
{
    if (!m_params.interaction_graph_params.enabled) {
        return;
    }

    using Clock = std::chrono::steady_clock;
    const auto total_start = Clock::now();
    InteractionGraphDiagnostics diagnostics;
    diagnostics.vehicle_id = m_params.vehicle_id;
    diagnostics.shadow_enabled = true;
    diagnostics.component_cutover_enabled =
        m_params.interaction_graph_component_cutover_enabled;
    diagnostics.assembled_candidate_ids.fill(0U);

    const std::size_t aircraft_count = static_cast<std::size_t>(
        m_params.total_agent_count);
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
        diagnostics.shadow_status =
            InteractionGraphShadowStatus::CandidateSetsIncomplete;
        diagnostics.total_shadow_time_ns = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                Clock::now() - total_start).count());
        m_pending_interaction_graph_diagnostics =
            std::make_shared<InteractionGraphDiagnostics>(diagnostics);
        return;
    }
    const auto & certified_candidate_sets =
        *m_epoch_certification_candidate_sets;

    PairwiseAdCertificationSet certifications;
    if (!m_pairwise_ad_certifier.evaluate(
            now_us,
            m_selection_epoch,
            m_params.interaction_graph_params.trajectory_library_version,
            m_params.interaction_graph_params.ad_masd_config_version,
            certified_candidate_sets,
            aircraft_count,
            certifications)) {
        diagnostics.graph.status =
            InteractionGraphStatus::InvalidCertification;
        diagnostics.graph.evaluation_timestamp_us = now_us;
        diagnostics.graph.selection_epoch = m_selection_epoch;
        diagnostics.graph.aircraft_count = aircraft_count;
        diagnostics.shadow_status =
            InteractionGraphShadowStatus::GraphInvalid;
        diagnostics.total_shadow_time_ns = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                Clock::now() - total_start).count());
        m_pending_interaction_graph_diagnostics =
            std::make_shared<InteractionGraphDiagnostics>(diagnostics);
        return;
    }

    // Rejoin is a fleet/component objective, so its activation and values
    // must come from the same frozen distributed library as the graph. Using
    // this worker's local activation latch here would let identical graphs
    // produce different component solutions on different aircraft.
    ManeuverRejoinObjective common_rejoin_objective;
    bool safe_rejoin_requested = false;
    bool common_rejoin_objective_valid = true;
    for (std::size_t aircraft = 0; aircraft < aircraft_count; ++aircraft) {
        const auto & intent = certified_candidate_sets[aircraft][0];
        safe_rejoin_requested = safe_rejoin_requested
            || intent.safe_rejoin_requested;
        common_rejoin_objective
            .nominal_lateral_acceleration_mps2[aircraft] =
            intent.nominal_lateral_acceleration_mps2;
        common_rejoin_objective_valid = common_rejoin_objective_valid
            && std::isfinite(intent.nominal_lateral_acceleration_mps2);
    }
    if (safe_rejoin_requested && !common_rejoin_objective_valid) {
        diagnostics.graph.status =
            InteractionGraphStatus::InvalidCertification;
        diagnostics.graph.evaluation_timestamp_us = now_us;
        diagnostics.graph.selection_epoch = m_selection_epoch;
        diagnostics.graph.aircraft_count = aircraft_count;
        diagnostics.shadow_status =
            InteractionGraphShadowStatus::ComponentEvaluationFailed;
        diagnostics.total_shadow_time_ns = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                Clock::now() - total_start).count());
        m_pending_interaction_graph_diagnostics =
            std::make_shared<InteractionGraphDiagnostics>(diagnostics);
        return;
    }
    common_rejoin_objective.enabled = safe_rejoin_requested;
    const ManeuverRejoinObjective * rejoin_objective =
        safe_rejoin_requested ? &common_rejoin_objective : nullptr;

    diagnostics.graph = m_interaction_graph_builder.build(certifications);
    if (!diagnostics.graph.valid()) {
        diagnostics.shadow_status = InteractionGraphShadowStatus::GraphInvalid;
        diagnostics.total_shadow_time_ns = static_cast<std::uint64_t>(
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
            // Isolation affects only this epoch's candidate search. The slot
            // remains explicitly invalid; execution separately retains an
            // active latch or follows the current Formation command.
            continue;
        }
        CertifiedComponentEvaluation component_evaluation;
        if (!m_certified_component_evaluator.evaluate(
                certifications,
                certified_candidate_sets,
                members,
                member_count,
                component_evaluation,
                rejoin_objective)
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
            diagnostics.assembled_candidate_valid_mask |=
                std::uint32_t{1} << aircraft;
        }
    }
    diagnostics.component_search_time_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            Clock::now() - search_start).count());
    if (!component_search_valid
        || actual_evaluation_count
            != diagnostics.graph.component_evaluation_count) {
        diagnostics.shadow_status =
            InteractionGraphShadowStatus::ComponentEvaluationFailed;
        diagnostics.total_shadow_time_ns = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                Clock::now() - total_start).count());
        m_pending_interaction_graph_diagnostics =
            std::make_shared<InteractionGraphDiagnostics>(diagnostics);
        return;
    }
    diagnostics.shadow_search_evaluated = true;
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
    std::array<std::size_t, kMaximumSelectionAircraft>
        certified_candidate_counts{};
    std::fill_n(
        certified_candidate_counts.begin(),
        aircraft_count,
        kExhaustiveCandidatesPerAircraft);
    diagnostics.global_crosscheck_evaluated =
        evaluateCandidateIdTupleWithExecutionFallback(
        now_us,
        certified_candidate_sets,
        certified_candidate_counts,
        diagnostics.assembled_candidate_ids,
        diagnostics.assembled_candidate_valid_mask,
        crosscheck,
        rejoin_objective);
    diagnostics.global_crosscheck_time_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            Clock::now() - crosscheck_start).count());
    if (diagnostics.global_crosscheck_evaluated) {
        diagnostics.global_crosscheck_minimum_ad_m = crosscheck.minimum_ad_m;
        diagnostics.global_crosscheck_pass =
            crosscheck.valid && crosscheck.all_pairs_feasible;
        diagnostics.global_crosscheck_evaluation = crosscheck;
    }
    diagnostics.shadow_status = diagnostics.global_crosscheck_pass
        ? InteractionGraphShadowStatus::Evaluated
        : InteractionGraphShadowStatus::GlobalCrosscheckFailed;
    diagnostics.total_shadow_time_ns = static_cast<std::uint64_t>(
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

bool ManeuverSelectionWorker::constrainV4ActiveAircraftCandidates(
    MultiAircraftExhaustiveCandidateIntentSets & candidate_sets,
    const std::array<std::size_t, kMaximumSelectionAircraft>
        & candidate_counts) const
{
    if (m_params.execution_policy == ManeuverExecutionPolicy::AmacAdThreshold) {
        return true;
    }
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

bool ManeuverSelectionWorker::evaluateCandidateIdTupleWithExecutionFallback(
    std::uint64_t evaluation_timestamp_us,
    const MultiAircraftExhaustiveCandidateIntentSets & candidate_sets,
    const std::array<std::size_t, kMaximumSelectionAircraft>
        & candidate_counts,
    const std::array<std::uint8_t, kMaximumSelectionAircraft>
        & candidate_ids,
    std::uint32_t candidate_valid_mask,
    JointCombinationEvaluation & evaluation,
    const ManeuverRejoinObjective * rejoin_objective)
{
    MultiAircraftCandidateIntentSets nominal_sets{};
    std::array<std::size_t, kMaximumSelectionAircraft> nominal_counts{};
    bool nominal_intents_built = false;

    auto selected_sets =
        std::make_unique<MultiAircraftCandidateIntentSets>();
    const ManeuverActivationStatus ownship_activation =
        m_activation_controller.status();
    for (int aircraft = 0; aircraft < m_params.total_agent_count; ++aircraft) {
        const std::size_t aircraft_index = static_cast<std::size_t>(aircraft);
        if (!candidateIsValid(candidate_valid_mask, aircraft_index)) {
            const estimation::ReceivedTrajectoryIntent * execution_intent =
                nullptr;
            std::uint8_t execution_candidate_id = kRollZeroId;
            bool avoidance_active = false;
            if (aircraft == m_params.vehicle_id) {
                avoidance_active = ownship_activation.active;
                execution_candidate_id =
                    ownship_activation.latched_candidate_id;
            } else {
                const RemoteDecisionCache & peer =
                    m_remote_decision_caches[aircraft_index];
                avoidance_active = peer.valid
                    && peer.decision.coordination_qualified
                    && peer.decision.activation_requested
                    && peer.decision.ownship_candidate_valid;
                execution_candidate_id = peer.decision.ownship_candidate_id;
            }
            if (avoidance_active) {
                const auto begin = candidate_sets[aircraft_index].begin();
                const auto end = begin + static_cast<std::ptrdiff_t>(
                    candidate_counts[aircraft_index]);
                const auto found = std::find_if(
                    begin,
                    end,
                    [execution_candidate_id](const auto & candidate) {
                        return candidate.candidate_id
                            == execution_candidate_id;
                    });
                if (found == end) {
                    return false;
                }
                execution_intent = &(*found);
            } else {
                if (!nominal_intents_built) {
                    if (!buildNominalIntentSet(
                            evaluation_timestamp_us,
                            nominal_sets,
                            nominal_counts)) {
                        return false;
                    }
                    nominal_intents_built = true;
                }
                if (nominal_counts[aircraft_index] != 1U) {
                    return false;
                }
                execution_intent = &nominal_sets[aircraft_index][0];
            }
            (*selected_sets)[aircraft_index][0] =
                *execution_intent;
            continue;
        }

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
    }

    // This is a post-check of one already assembled execution tuple, not a
    // new joint search. Evaluate its ten aircraft pairs directly so isolated
    // Formation/active-latch fallbacks do not have to masquerade as a
    // three-candidate maneuver set.
    JointCombinationEvaluation checked{};
    checked.aircraft_count = static_cast<std::size_t>(
        m_params.total_agent_count);
    checked.valid = true;
    checked.all_pairs_feasible = true;
    checked.minimum_pmr_m = std::numeric_limits<double>::infinity();
    checked.minimum_ad_m = std::numeric_limits<double>::infinity();
    checked.reciprocal_cost_sum = 0.0;
    if (rejoin_objective != nullptr && rejoin_objective->enabled) {
        checked.nominal_rejoin_cost = 0.0;
        for (std::size_t aircraft = 0;
             aircraft < checked.aircraft_count; ++aircraft) {
            const double nominal = rejoin_objective
                ->nominal_lateral_acceleration_mps2[aircraft];
            const double command = (*selected_sets)[aircraft][0]
                .candidate_input.a_lat_cmd;
            if (!std::isfinite(nominal) || !std::isfinite(command)) {
                checked.nominal_rejoin_cost =
                    std::numeric_limits<double>::infinity();
                break;
            }
            const double error = command - nominal;
            checked.nominal_rejoin_cost += error * error;
        }
    }
    for (std::size_t first = 0; first < checked.aircraft_count; ++first) {
        for (std::size_t second = first + 1;
             second < checked.aircraft_count; ++second) {
            CombinationEvaluation pair;
            if (!m_pair_evaluator.evaluatePair(
                    evaluation_timestamp_us,
                    (*selected_sets)[first][0],
                    (*selected_sets)[second][0],
                    pair)) {
                return false;
            }
            ++checked.evaluated_pair_count;
            if (pair.ad_m < checked.minimum_ad_m) {
                checked.minimum_pmr_m = pair.pmr_m;
                checked.minimum_masd_m = pair.masd_m;
                checked.minimum_ad_m = pair.ad_m;
            }
            if (!pair.feasible || !pair.reciprocal_cost_defined) {
                checked.all_pairs_feasible = false;
            } else {
                checked.reciprocal_cost_sum += pair.reciprocal_cost;
            }
        }
    }
    checked.valid = checked.evaluated_pair_count
        == checked.aircraft_count * (checked.aircraft_count - 1U) / 2U;
    checked.selected_best = checked.valid;
    evaluation = checked;
    return evaluation.valid;
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

bool ManeuverSelectionWorker::buildActivationSample(
    std::uint64_t now_us,
    ManeuverActivationSample & sample,
    ManeuverSelectionDecision & decision)
{
    sample = ManeuverActivationSample{};
    sample.timestamp_us = now_us;
    sample.activation_criteria_m.fill(
        std::numeric_limits<double>::quiet_NaN());
    for (auto & relative_position : sample.relative_positions_ned_m) {
        relative_position.fill(std::numeric_limits<double>::quiet_NaN());
    }
    for (auto & relative_velocity : sample.relative_velocities_ned_mps) {
        relative_velocity.fill(std::numeric_limits<double>::quiet_NaN());
    }
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
    const auto find_candidate_by_id = [](
                                          const ExhaustiveCandidateIntentSet
                                              & candidates,
                                          std::size_t candidate_count,
                                          std::uint8_t candidate_id)
        -> const estimation::ReceivedTrajectoryIntent * {
        for (std::size_t index = 0; index < candidate_count; ++index) {
            if (candidates[index].candidate_id == candidate_id) {
                return &candidates[index];
            }
        }
        return nullptr;
    };

    const std::size_t ownship_index = static_cast<std::size_t>(
        m_params.vehicle_id);
    if (!m_has_latest_state || m_latest_state_timestamp_us != now_us) {
        return false;
    }
    const bool selected_ownship_candidate_valid = candidateIsValid(
        m_selected_candidate_valid_mask, ownship_index);
    const ManeuverActivationStatus activation =
        m_activation_controller.status();
    const std::uint8_t ownship_candidate_id =
        selected_ownship_candidate_valid
        ? m_selected_candidate_ids[ownship_index]
        : activation.latched_candidate_id;
    const std::uint64_t ownship_candidate_input_revision =
        selected_ownship_candidate_valid
        ? m_selected_candidate_input_revisions[ownship_index]
        : activation.latched_candidate_input_revision;
    estimation::ReceivedTrajectoryIntent active_ownship_intent;
    const estimation::ReceivedTrajectoryIntent * ownship_intent = nullptr;
    if (selected_ownship_candidate_valid) {
        ownship_intent = find_candidate(
            m_ownship_candidates,
            m_ownship_candidate_count,
            ownship_candidate_id,
            ownship_candidate_input_revision);
    } else if (activation.active) {
        estimation::TrajectoryIntentPacket packet;
        if (!m_sender.buildForCandidateInput(
                now_us,
                ownship_candidate_id,
                activation.latched_input,
                m_latest_state,
                m_latest_covariance,
                packet,
                m_selection_epoch)
            || !m_receiver.receive(packet, active_ownship_intent)) {
            return false;
        }
        ownship_intent = &active_ownship_intent;
    }
    if (ownship_intent == nullptr) {
        return false;
    }

    // The alternate-termination study uses the aircraft's current flight
    // vector.  The selected intent remains the source of AD/PMR/MASD below,
    // but it must not replace the measured ownship state in the CPA test.
    const double horizontal_speed_squared_m2ps2 =
        m_latest_state.V * m_latest_state.V
        - m_latest_state.h_dot * m_latest_state.h_dot;
    if (!std::isfinite(horizontal_speed_squared_m2ps2)) {
        return false;
    }
    IntentKinematics ownship_kinematics;
    const double horizontal_speed_mps = std::sqrt(std::max(
        0.0, horizontal_speed_squared_m2ps2));
    ownship_kinematics.position_ned = {
        m_latest_state.p_n, m_latest_state.p_e, -m_latest_state.h};
    ownship_kinematics.velocity_ned = {
        horizontal_speed_mps * std::cos(m_latest_state.psi),
        horizontal_speed_mps * std::sin(m_latest_state.psi),
        -m_latest_state.h_dot};

    sample.selected_candidate_id = ownship_candidate_id;
    sample.selected_candidate_input_revision =
        ownship_candidate_input_revision;
    sample.selected_input = ownship_intent->candidate_input;
    double minimum_ad = std::numeric_limits<double>::infinity();
    double reciprocal_cost_sum = 0.0;
    bool reciprocal_cost_defined = true;
    std::size_t evaluated_threat_count = 0;
    MultiAircraftCandidateIntentSets nominal_sets{};
    std::array<std::size_t, kMaximumSelectionAircraft> nominal_counts{};
    const bool needs_nominal_fallback =
        m_selected_candidate_valid_mask
        != candidateMaskForAircraftCount(static_cast<std::size_t>(
            m_params.total_agent_count));
    if (needs_nominal_fallback
        && !buildNominalIntentSet(now_us, nominal_sets, nominal_counts)) {
        return false;
    }
    for (int remote_id = 0;
         remote_id < m_params.total_agent_count; ++remote_id) {
        if (remote_id == m_params.vehicle_id) {
            continue;
        }
        const std::size_t remote_index = static_cast<std::size_t>(remote_id);
        std::uint8_t remote_candidate_id =
            m_selected_candidate_ids[remote_index];
        std::uint64_t remote_candidate_input_revision =
            m_selected_candidate_input_revisions[remote_index];
        const RemoteCandidateCache * cache = &m_remote_caches[remote_index];
        const estimation::ReceivedTrajectoryIntent * remote_intent = nullptr;
        const bool remote_candidate_valid = candidateIsValid(
            m_selected_candidate_valid_mask, remote_index);
        if (!remote_candidate_valid) {
            const RemoteDecisionCache & peer =
                m_remote_decision_caches[remote_index];
            const bool remote_avoidance_active = peer.valid
                && peer.decision.coordination_qualified
                && peer.decision.activation_requested
                && peer.decision.ownship_candidate_valid;
            if (remote_avoidance_active) {
                remote_candidate_id = peer.decision.ownship_candidate_id;
                if (cache->count > 0
                    && cache->count == cache->expected_count) {
                    remote_intent = find_candidate_by_id(
                        cache->candidates,
                        cache->count,
                        remote_candidate_id);
                }
                if (remote_intent == nullptr) {
                    cache = &m_remote_previous_caches[remote_index];
                    if (cache->count > 0
                        && cache->count == cache->expected_count) {
                        remote_intent = find_candidate_by_id(
                            cache->candidates,
                            cache->count,
                            remote_candidate_id);
                    }
                }
                if (remote_intent == nullptr) {
                    const RemoteSelectedIntentCache & retained =
                        m_remote_selected_caches[remote_index];
                    if (retained.valid
                        && retained.intent.candidate_id
                            == remote_candidate_id) {
                        remote_intent = &retained.intent;
                    }
                }
            } else {
                if (nominal_counts[remote_index] != 1U) {
                    return false;
                }
                remote_intent = &nominal_sets[remote_index][0];
            }
        } else if (cache->count > 0 && cache->count == cache->expected_count
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
        if (remote_intent == nullptr
            && remote_candidate_valid) {
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
        if (remote_intent == nullptr
            && remote_candidate_valid) {
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
            decision.communication_delay_margin_m =
                pair.communication_delay_margin_m;
            decision.threat_candidate_id = remote_candidate_id;
        }
        if (pair.ad_m < 0.0) {
            sample.unsafe_threat_mask |= std::uint32_t{1} << remote_index;
        }
        // The public alternate-termination study does not publish the exact
        // threshold implementation. This project maps its "original
        // activation criteria" to the pair MASD used by AD = PMR - MASD and
        // freezes that value when the pair first becomes unsafe.
        sample.activation_criteria_m[remote_index] = pair.masd_m;
        if (pair.reciprocal_cost_defined) {
            reciprocal_cost_sum += pair.reciprocal_cost;
        } else {
            reciprocal_cost_defined = false;
        }

        IntentKinematics remote_kinematics;
        // A cooperating peer has no local estimator state in this worker. Its
        // timestamped intent is therefore propagated to the current ownship
        // time before forming the peer's current flight vector.
        if (interpolateIntentKinematics(
                *remote_intent, now_us, remote_kinematics)
                != IntentKinematicsStatus::Valid) {
            return false;
        }
        for (std::size_t axis = 0;
             axis < ownship_kinematics.position_ned.size(); ++axis) {
            sample.relative_positions_ned_m[remote_index][axis] =
                remote_kinematics.position_ned[axis]
                - ownship_kinematics.position_ned[axis];
            sample.relative_velocities_ned_mps[remote_index][axis] =
                remote_kinematics.velocity_ned[axis]
                - ownship_kinematics.velocity_ned[axis];
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

bool ManeuverSelectionWorker::buildNominalIntentSet(
    std::uint64_t now_us,
    MultiAircraftCandidateIntentSets & candidate_sets,
    std::array<std::size_t, kMaximumSelectionAircraft> & candidate_counts)
{
    if (!m_has_latest_state || !m_has_latest_nominal) {
        return false;
    }

    candidate_sets = MultiAircraftCandidateIntentSets{};
    candidate_counts.fill(0U);
    estimation::PredictInput ownship_input{};
    if (!nominalPredictInput(
            m_latest_nominal,
            now_us,
            m_params.v4_maximum_nominal_age_us,
            ownship_input)) {
        return false;
    }

    const auto build_intent = [this](
                                  std::uint64_t source_timestamp_us,
                                  const estimation::PredictInput & input,
                                  const estimation::PredictState & state,
                                  const estimation::PredictStateCovariance
                                      & covariance,
                                  estimation::ReceivedTrajectoryIntent
                                      & received) {
        estimation::TrajectoryIntentPacket packet;
        if (!m_sender.buildForCandidateInput(
                source_timestamp_us,
                kRollZeroId,
                input,
                state,
                covariance,
                packet,
                m_selection_epoch)) {
            return false;
        }
        packet.candidate_set_size = 1;
        packet.candidate_set_kind =
            estimation::CandidateSetKind::LegacyRoll;
        return m_receiver.receive(packet, received);
    };

    const std::size_t ownship_index = static_cast<std::size_t>(
        m_params.vehicle_id);
    if (!build_intent(
            m_latest_state_timestamp_us,
            ownship_input,
            m_latest_state,
            m_latest_covariance,
            candidate_sets[ownship_index][0])) {
        return false;
    }
    candidate_counts[ownship_index] = 1;

    for (int aircraft = 0;
         aircraft < m_params.total_agent_count; ++aircraft) {
        if (aircraft == m_params.vehicle_id) {
            continue;
        }
        const std::size_t aircraft_index = static_cast<std::size_t>(aircraft);
        const RemoteDecisionCache & decision_cache =
            m_remote_decision_caches[aircraft_index];
        if (!decision_cache.valid) {
            return false;
        }
        const ManeuverSelectionPeerDecision & peer = decision_cache.decision;
        ManeuverSelectionNominalSetpointSnapshot peer_nominal;
        peer_nominal.timestamp_us = peer.nominal_setpoint_timestamp_us;
        peer_nominal.ground_speed_command_mps =
            peer.nominal_ground_speed_command_mps;
        peer_nominal.altitude_command_m = peer.nominal_altitude_command_m;
        peer_nominal.lateral_acceleration_px4_mps2 =
            peer.nominal_lateral_acceleration_mps2;
        peer_nominal.valid = peer.nominal_setpoint_available;
        estimation::PredictInput peer_input{};
        if (!nominalPredictInput(
                peer_nominal,
                now_us,
                m_params.v4_maximum_nominal_age_us,
                peer_input)) {
            return false;
        }

        const estimation::ReceivedTrajectoryIntent * anchor = nullptr;
        const auto consider_cache = [&anchor](const RemoteCandidateCache & cache) {
            if (cache.count == 0 || cache.count != cache.expected_count) {
                return;
            }
            for (std::size_t index = 0; index < cache.count; ++index) {
                const auto & candidate = cache.candidates[index];
                if (anchor == nullptr
                    || candidate.source_timestamp_us
                        > anchor->source_timestamp_us) {
                    anchor = &candidate;
                }
            }
        };
        consider_cache(m_remote_caches[aircraft_index]);
        consider_cache(m_remote_previous_caches[aircraft_index]);
        if (anchor == nullptr
            || !build_intent(
                anchor->source_timestamp_us,
                peer_input,
                anchor->cone[0].mean,
                anchor->cone[0].state_covariance,
                candidate_sets[aircraft_index][0])) {
            return false;
        }
        candidate_counts[aircraft_index] = 1;
    }

    return true;
}

bool ManeuverSelectionWorker::evaluateNominalPostRelease(
    std::uint64_t now_us,
    JointCombinationEvaluation & evaluation)
{
    MultiAircraftCandidateIntentSets candidate_sets{};
    std::array<std::size_t, kMaximumSelectionAircraft> candidate_counts{};
    if (!buildNominalIntentSet(now_us, candidate_sets, candidate_counts)) {
        return false;
    }

    JointManeuverEvaluation nominal_evaluation;
    if (!m_joint_evaluator.evaluate(
            now_us,
            candidate_sets,
            candidate_counts,
            static_cast<std::size_t>(m_params.total_agent_count),
            nominal_evaluation)
        || !nominal_evaluation.has_best) {
        return false;
    }
    evaluation = nominal_evaluation.combinations[
        nominal_evaluation.best_combination_index];
    return evaluation.valid;
}

bool ManeuverSelectionWorker::buildManeuverRejoinObjective(
    std::uint64_t now_us,
    ManeuverRejoinObjective & objective) const noexcept
{
    ManeuverRejoinObjective candidate{};
    estimation::PredictInput input{};
    if (!m_has_latest_nominal
        || !nominalPredictInput(
            m_latest_nominal,
            now_us,
            m_params.v4_maximum_nominal_age_us,
            input)) {
        return false;
    }
    candidate.nominal_lateral_acceleration_mps2[
        static_cast<std::size_t>(m_params.vehicle_id)] = input.a_lat_cmd;

    for (int aircraft = 0;
         aircraft < m_params.total_agent_count; ++aircraft) {
        if (aircraft == m_params.vehicle_id) {
            continue;
        }
        const RemoteDecisionCache & cache = m_remote_decision_caches[
            static_cast<std::size_t>(aircraft)];
        if (!cache.valid) {
            return false;
        }
        ManeuverSelectionNominalSetpointSnapshot peer_nominal;
        peer_nominal.timestamp_us =
            cache.decision.nominal_setpoint_timestamp_us;
        peer_nominal.ground_speed_command_mps =
            cache.decision.nominal_ground_speed_command_mps;
        peer_nominal.altitude_command_m =
            cache.decision.nominal_altitude_command_m;
        peer_nominal.lateral_acceleration_px4_mps2 =
            cache.decision.nominal_lateral_acceleration_mps2;
        peer_nominal.valid = cache.decision.nominal_setpoint_available;
        if (!nominalPredictInput(
                peer_nominal,
                now_us,
                m_params.v4_maximum_nominal_age_us,
                input)) {
            return false;
        }
        candidate.nominal_lateral_acceleration_mps2[
            static_cast<std::size_t>(aircraft)] = input.a_lat_cmd;
    }
    candidate.enabled = true;
    objective = candidate;
    return true;
}

bool ManeuverSelectionWorker::allPeersConfirmPostRelease(
    std::uint64_t now_us) const noexcept
{
    // Post-release safety is a fresh nominal-trajectory result, not a selected
    // avoidance-tuple result.  Component membership may change selection
    // epochs without changing that nominal safety fact, so freshness and the
    // peer's explicit safe result are the complete release contract here.
    for (int aircraft = 0;
         aircraft < m_params.total_agent_count; ++aircraft) {
        if (aircraft == m_params.vehicle_id) {
            continue;
        }
        const RemoteDecisionCache & cache = m_remote_decision_caches[
            static_cast<std::size_t>(aircraft)];
        if (!cache.valid || !cache.decision.post_release_evaluated
            || !cache.decision.post_release_safe
            || cache.decision.post_release_evaluation_timestamp_us == 0
            || cache.decision.post_release_evaluation_timestamp_us > now_us
            || now_us - cache.decision.post_release_evaluation_timestamp_us
                > m_params.maximum_belief_delay_us) {
            return false;
        }
    }
    return true;
}

void ManeuverSelectionWorker::applyFormationActivationGate(
    const std::uint64_t now_us,
    ManeuverActivationSample & sample,
    ManeuverSelectionDecision & decision)
{
    sample.allow_new_activation = true;
    decision.formation_evaluated = false;
    decision.formation_inhibit = false;
    decision.formation_allow_new_activation = true;
    decision.formation_inhibited_threat_mask = 0U;
    if (!m_formation_discriminator.has_value() || !sample.valid) {
        return;
    }

    const double horizontal_speed_mps = std::sqrt(std::max(
        0.0,
        m_latest_state.V * m_latest_state.V
            - m_latest_state.h_dot * m_latest_state.h_dot));
    formation::FormationKinematicState ownship;
    ownship.position_ned_m = {
        m_latest_state.p_n, m_latest_state.p_e, -m_latest_state.h};
    ownship.velocity_ned_mps = {
        horizontal_speed_mps * std::cos(m_latest_state.psi),
        horizontal_speed_mps * std::sin(m_latest_state.psi),
        -m_latest_state.h_dot};
    ownship.timestamp_s = static_cast<double>(now_us) * 1.0e-6;
    ownship.valid = true;

    std::vector<formation::FormationResult> results;
    std::vector<std::uint32_t> collision_relevant_threat_ids;
    results.reserve(static_cast<std::size_t>(m_params.total_agent_count - 1));
    collision_relevant_threat_ids.reserve(results.capacity());
    const std::uint32_t original_unsafe_threat_mask =
        sample.unsafe_threat_mask;
    for (int remote_id = 0;
         remote_id < m_params.total_agent_count; ++remote_id) {
        if (remote_id == m_params.vehicle_id) {
            continue;
        }
        const std::size_t remote_index = static_cast<std::size_t>(remote_id);
        formation::FormationKinematicState threat;
        for (std::size_t axis = 0; axis < 3; ++axis) {
            threat.position_ned_m[axis] = ownship.position_ned_m[axis]
                + sample.relative_positions_ned_m[remote_index][axis];
            threat.velocity_ned_mps[axis] = ownship.velocity_ned_mps[axis]
                + sample.relative_velocities_ned_mps[remote_index][axis];
        }
        threat.timestamp_s = ownship.timestamp_s;
        threat.valid = true;

        formation::FormationUpdateInput input;
        input.threat_id = static_cast<std::uint32_t>(remote_id);
        input.evaluation_timestamp_s = ownship.timestamp_s;
        input.ownship = ownship;
        input.threat = threat;
        const auto result = m_formation_discriminator->update(input);
        results.push_back(result);

        const std::uint32_t threat_bit = std::uint32_t{1} << remote_index;
        if ((original_unsafe_threat_mask & threat_bit) == 0U) {
            continue;
        }
        collision_relevant_threat_ids.push_back(
            static_cast<std::uint32_t>(remote_id));
        const double hard_safety_budget_m =
            sample.activation_criteria_m[remote_index];
        double current_separation_squared_m2 = 0.0;
        for (const double component :
             sample.relative_positions_ned_m[remote_index]) {
            current_separation_squared_m2 += component * component;
        }
        const double current_separation_m = std::sqrt(
            current_separation_squared_m2);
        if (result.formation_inhibit && result.timestamp_valid
            && result.geometry_valid
            && formationSpacingCompatible(
                m_params.formation_target_separation_m,
                current_separation_m,
                hard_safety_budget_m)) {
            decision.formation_inhibited_threat_mask |= threat_bit;
        }
    }
    decision.formation_evaluated = true;

    // Formation is strictly a new-activation exemption. Once avoidance is
    // active, keep the complete AD-derived unsafe mask so newly unsafe threats
    // are added to the CPA termination monitor even if they resemble a
    // formation encounter.
    if (m_activation_controller.status().active) {
        return;
    }

    if (m_params.formation_aggregation_policy
            == formation::FormationAggregationPolicy::
                PerThreatExemptionOnly) {
        sample.unsafe_threat_mask &=
            ~decision.formation_inhibited_threat_mask;
        sample.allow_new_activation = sample.unsafe_threat_mask != 0U
            || original_unsafe_threat_mask == 0U;
    } else {
        const auto aggregation = formation::FormationInhibitAggregator::
            aggregate(
                results,
                collision_relevant_threat_ids,
                m_params.formation_aggregation_policy);
        sample.allow_new_activation = aggregation.allow_new_activation;
    }
    decision.formation_allow_new_activation = sample.allow_new_activation;
    decision.formation_inhibit = !sample.allow_new_activation;
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
    decision.cpa_clear = false;
    const bool retained_post_release_evaluation =
        m_has_last_post_release_evaluation
        && m_last_post_release_evaluation_timestamp_us <= now_us
        && now_us - m_last_post_release_evaluation_timestamp_us
            <= m_params.maximum_belief_delay_us;
    decision.post_release_evaluated = retained_post_release_evaluation;
    decision.post_release_safe = retained_post_release_evaluation
        && m_last_post_release_evaluation.all_pairs_feasible;
    decision.post_release_minimum_ad_m = retained_post_release_evaluation
        ? m_last_post_release_evaluation.minimum_ad_m
        : std::numeric_limits<double>::quiet_NaN();
    decision.post_release_evaluation_timestamp_us =
        retained_post_release_evaluation
        ? m_last_post_release_evaluation_timestamp_us : 0;
    decision.post_release_peer_confirmed = false;

    ManeuverActivationSample sample;
    const bool sample_valid = buildActivationSample(now_us, sample, decision);
    sample.valid = sample_valid;
    applyFormationActivationGate(now_us, sample, decision);
    JointCombinationEvaluation post_release_evaluation;
    if (evaluateNominalPostRelease(now_us, post_release_evaluation)) {
        m_last_post_release_evaluation = post_release_evaluation;
        m_last_post_release_evaluation_timestamp_us = now_us;
        m_has_last_post_release_evaluation = true;
        decision.post_release_evaluated = true;
        decision.post_release_safe =
            post_release_evaluation.all_pairs_feasible;
        decision.post_release_minimum_ad_m =
            post_release_evaluation.minimum_ad_m;
        decision.post_release_evaluation_timestamp_us = now_us;
    }
    const ManeuverActivationStatus previous_status =
        m_activation_controller.status();
    if (previous_status.active && sample.valid) {
        decision.cpa_clear = m_activation_controller.futureCpaClear(sample);
    }
    if (previous_status.active && decision.cpa_clear) {
        decision.post_release_peer_confirmed =
            decision.post_release_safe
            && std::isfinite(sample.minimum_ad_m)
            && sample.minimum_ad_m >= 0.0
            && allPeersConfirmPostRelease(now_us);
        sample.allow_deactivation =
            decision.post_release_peer_confirmed;
        m_safe_rejoin_active = !sample.allow_deactivation;
    } else {
        sample.allow_deactivation = true;
        m_safe_rejoin_active = false;
    }
    const ManeuverActivationStatus status =
        m_activation_controller.update(sample);
    decision.activation_requested = status.active;
    decision.activation_just_started = status.just_activated;
    decision.activation_just_ended = status.just_deactivated;
    decision.activation_timestamp_us = status.activation_timestamp_us;
    decision.deactivation_reason = status.deactivation_reason;
    if (!status.active) {
        m_safe_rejoin_active = false;
    }
    decision.safe_rejoin_active = m_safe_rejoin_active;
    if (status.active) {
        decision.ownship_candidate_id = status.latched_candidate_id;
        decision.ownship_candidate_valid = true;
        decision.ownship_input = status.latched_input;
        m_current_best_id = status.latched_candidate_id;
    } else {
        decision.ownship_candidate_valid = candidateIsValid(
            m_selected_candidate_valid_mask,
            static_cast<std::size_t>(m_params.vehicle_id));
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
