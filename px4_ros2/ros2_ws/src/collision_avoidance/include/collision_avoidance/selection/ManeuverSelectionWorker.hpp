#pragma once

#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <thread>

#include <collision_avoidance/common/SpscQueue.hpp>
#include <collision_avoidance/estimation/trajectory_prediction/TrajectoryIntent.hpp>
#include <collision_avoidance/selection/HeuristicCandidateSelector.hpp>
#include <collision_avoidance/selection/ManeuverActivationController.hpp>
#include <collision_avoidance/selection/ManeuverCombinationEvaluator.hpp>
#include <collision_avoidance/selection/SafeControlCandidateAdapter.hpp>

namespace collision_avoidance::selection
{

inline constexpr std::size_t kEligibleLateralCandidateCount = 7;
inline constexpr std::size_t kSelectionWorkerInputCapacity = 64;
inline constexpr std::size_t kSelectionWorkerOutputCapacity = 16;
inline constexpr std::size_t kMaximumIntentPacketsPerOutput =
    kExhaustiveCandidatesPerAircraft + 1;

enum class V4ShadowEvaluationStatus : std::uint8_t
{
    Disabled,
    MissingPeerDecision,
    MissingPeerIntent,
    FuturePeerIntent,
    StalePeerIntent,
    InvalidPeerIntent,
    CoreEvaluated,
    CandidateGenerationFailed,
};

enum class V4SnapshotStatus : std::uint8_t
{
    Missing,
    Valid,
    Invalid,
    Future,
    Stale,
};

enum class V4AirspeedSource : std::uint8_t
{
    Unavailable,
    ActualTas,
    TrimFallback,
};

enum class ManeuverExecutionPolicy : std::uint8_t
{
    AmacAdThreshold = 0,
    ContinuousV4,
    HorizonGatedV4,
};

struct ManeuverSelectionWorkerParams
{
    int vehicle_id{0};
    int total_agent_count{2};
    estimation::PredictParams predictor_params{};
    estimation::UncertaintyParams uncertainty_params{};
    ManeuverCombinationEvaluatorParams evaluator_params{};
    double ground_speed_command_mps{20.0};
    double gravity_mps2{9.80665};
    std::uint64_t trajectory_refresh_period_us{50'000};
    std::uint64_t candidate_refresh_period_us{250'000};
    std::uint64_t coordination_delay_us{250'000};
    std::uint64_t maximum_belief_delay_us{1'000'000};
    ManeuverActivationControllerParams activation_params{};
    // Project-defined active-best hysteresis. Public AMAC sources do not
    // disclose these numerical margins, so runtime switching is opt-in.
    bool active_switching_enabled{false};
    double active_switch_cost_margin{0.0};
    double active_switch_minimum_ad_margin_m{0.0};
    ManeuverExecutionPolicy execution_policy{
        ManeuverExecutionPolicy::AmacAdThreshold};
    bool exhaustive_test_mode{false};
    bool v4_safe_control_enabled{false};
    // True keeps V4 diagnostic-only; false supplies V4 candidates downstream.
    bool v4_shadow_only{true};
    double v4_trim_airspeed_mps{15.0};
    std::uint64_t v4_maximum_airspeed_age_us{1'000'000};
    std::uint64_t v4_maximum_nominal_age_us{1'000'000};
    SafeControlSetV4Params v4_safe_control_params{};
    SafeControlCandidateAdapterParams v4_candidate_adapter_params{};
    // Robust cone clearance h excludes DSD. V4 commands are applied when the
    // worst aligned near-nominal horizon clearance reaches this threshold.
    double v4_horizon_trigger_m{10.0};
    std::array<std::uint8_t, kEligibleLateralCandidateCount> eligible_candidate_ids{
        static_cast<std::uint8_t>(estimation::ManeuverCandidateId::RollZero),
        static_cast<std::uint8_t>(estimation::ManeuverCandidateId::RollMinus15),
        static_cast<std::uint8_t>(estimation::ManeuverCandidateId::RollPlus15),
        static_cast<std::uint8_t>(estimation::ManeuverCandidateId::RollMinus30),
        static_cast<std::uint8_t>(estimation::ManeuverCandidateId::RollPlus30),
        static_cast<std::uint8_t>(estimation::ManeuverCandidateId::RollMinus50),
        static_cast<std::uint8_t>(estimation::ManeuverCandidateId::RollPlus50)};
};

struct ManeuverSelectionBeliefSnapshot
{
    std::uint64_t timestamp_us{0};
    std::uint64_t timestamp_sample_us{0};
    estimation::EstimatorTrajectoryBelief belief{};
    bool valid{false};
};

struct ManeuverSelectionAirspeedSnapshot
{
    std::uint64_t timestamp_us{0};
    double true_airspeed_mps{0.0};
    std::int8_t px4_airspeed_source{-1};
    bool valid{false};
};

struct ManeuverSelectionNominalSetpointSnapshot
{
    std::uint64_t timestamp_us{0};
    double ground_speed_command_mps{0.0};
    // Predictor convention: altitude is positive Up.
    double altitude_command_m{0.0};
    double lateral_acceleration_px4_mps2{0.0};
    bool valid{false};
};

struct ManeuverSelectionDecision
{
    int vehicle_id{0};
    std::size_t aircraft_count{0};
    std::uint64_t selection_timestamp_us{0};
    std::uint64_t local_selection_epoch{0};
    std::uint64_t remote_selection_epoch{0};
    std::array<std::uint64_t, kMaximumSelectionAircraft>
        selection_epochs_by_aircraft{};
    std::size_t selected_combination_index{0};
    std::size_t evaluated_combination_count{0};
    std::array<std::uint8_t, kMaximumSelectionAircraft>
        selected_candidate_ids{};
    std::array<std::uint64_t, kMaximumSelectionAircraft>
        selected_candidate_input_revisions{};
    std::array<std::uint64_t, kMaximumSelectionAircraft>
        selected_candidate_source_timestamps_us{};
    bool selected_v4_cutover{false};
    std::uint8_t ownship_candidate_id{
        static_cast<std::uint8_t>(estimation::ManeuverCandidateId::RollZero)};
    std::uint64_t proposal_timestamp_us{0};
    std::uint64_t proposal_epoch{0};
    std::array<std::uint8_t, kMaximumSelectionAircraft>
        proposed_candidate_ids{};
    std::array<std::uint64_t, kMaximumSelectionAircraft>
        proposed_candidate_input_revisions{};
    std::array<std::uint64_t, kMaximumSelectionAircraft>
        proposed_candidate_source_timestamps_us{};
    bool proposed_v4_cutover{false};
    bool proposal_valid{false};
    bool proposal_consensus_confirmed{false};
    bool switch_superiority_evaluated{false};
    bool switch_clearly_superior{false};
    double switch_current_cost{
        std::numeric_limits<double>::quiet_NaN()};
    double switch_proposed_cost{
        std::numeric_limits<double>::quiet_NaN()};
    double switch_current_minimum_ad_m{
        std::numeric_limits<double>::quiet_NaN()};
    double switch_proposed_minimum_ad_m{
        std::numeric_limits<double>::quiet_NaN()};
    std::uint8_t threat_candidate_id{
        static_cast<std::uint8_t>(estimation::ManeuverCandidateId::RollZero)};
    estimation::PredictInput ownship_input{};
    double pmr_m{0.0};
    double masd_m{0.0};
    double ad_m{0.0};
    double reciprocal_cost_sum{0.0};
    std::uint64_t activation_timestamp_us{0};
    ManeuverDeactivationReason deactivation_reason{
        ManeuverDeactivationReason::None};
    bool coordination_qualified{false};
    bool new_best_accepted{false};
    bool previous_best_retained{true};
    bool activation_requested{false};
    // The command gate actually used by FormationMode. This stays distinct
    // from AMAC activation so continuous V4 does not falsify AMAC state.
    bool command_execution_requested{false};
    bool activation_just_started{false};
    bool activation_just_ended{false};

    std::uint64_t handoff_evaluation_epoch{0};
    bool handoff_cpa_clear{false};
    bool handoff_post_ad_evaluated{false};
    bool handoff_post_ad_safe{false};
    double handoff_post_minimum_ad_m{
        std::numeric_limits<double>::quiet_NaN()};
    bool handoff_ready{false};
    bool handoff_consensus_confirmed{false};

    // Horizon-gated V4 supervision. This is distinct from AMAC AD activation.
    bool v4_horizon_gate_evaluated{false};
    bool v4_horizon_gate_valid{false};
    bool v4_horizon_local_gate_active{false};
    bool v4_horizon_gate_active{false};
    double v4_horizon_h_worst_m{
        std::numeric_limits<double>::quiet_NaN()};
    double v4_horizon_trigger_m{
        std::numeric_limits<double>::quiet_NaN()};
    double v4_horizon_worst_time_offset_s{
        std::numeric_limits<double>::quiet_NaN()};
    int v4_horizon_worst_first_vehicle_id{-1};
    int v4_horizon_worst_second_vehicle_id{-1};

    // V4 diagnostics. They are observation-only in shadow mode; in cutover
    // mode v4_candidates is the source of the downstream candidate set.
    bool v4_enabled{false};
    bool v4_shadow_only{true};
    bool v4_shadow_evaluated{false};
    V4ShadowEvaluationStatus v4_shadow_status{
        V4ShadowEvaluationStatus::Disabled};
    V4SnapshotStatus v4_airspeed_snapshot_status{
        V4SnapshotStatus::Missing};
    V4AirspeedSource v4_airspeed_source{V4AirspeedSource::Unavailable};
    std::int8_t v4_px4_airspeed_source{-1};
    std::uint64_t v4_airspeed_timestamp_us{0};
    std::uint64_t v4_airspeed_age_us{0};
    V4SnapshotStatus v4_nominal_snapshot_status{
        V4SnapshotStatus::Missing};
    bool v4_nominal_available{false};
    std::uint64_t v4_nominal_timestamp_us{0};
    std::uint64_t v4_nominal_age_us{0};
    SafeControlSetV4Result v4_safe_control{};
    SafeControlCandidateAdapterResult v4_candidates{};
};

/* ROS-independent subset of a peer's published decision.  A peer owns only
   its own candidate; the complete proposal is used solely for the distributed
   same-epoch cross-check before any tuple is committed. */
struct ManeuverSelectionPeerDecision
{
    int vehicle_id{-1};
    std::uint64_t selection_timestamp_us{0};
    std::uint64_t local_selection_epoch{0};
    std::array<std::uint8_t, kMaximumSelectionAircraft>
        selected_candidate_ids{};
    std::array<std::uint64_t, kMaximumSelectionAircraft>
        selected_candidate_input_revisions{};
    std::array<std::uint64_t, kMaximumSelectionAircraft>
        selected_candidate_source_timestamps_us{};
    bool selected_v4_cutover{false};
    std::uint8_t ownship_candidate_id{
        static_cast<std::uint8_t>(estimation::ManeuverCandidateId::RollZero)};
    std::uint64_t proposal_timestamp_us{0};
    std::uint64_t proposal_epoch{0};
    std::array<std::uint8_t, kMaximumSelectionAircraft>
        proposed_candidate_ids{};
    std::array<std::uint64_t, kMaximumSelectionAircraft>
        proposed_candidate_input_revisions{};
    std::array<std::uint64_t, kMaximumSelectionAircraft>
        proposed_candidate_source_timestamps_us{};
    bool proposed_v4_cutover{false};
    bool proposal_valid{false};
    bool proposal_consensus_confirmed{false};
    bool coordination_qualified{false};
    bool activation_requested{false};
    bool command_execution_requested{false};
    std::uint64_t handoff_evaluation_epoch{0};
    bool handoff_ready{false};
    bool handoff_consensus_confirmed{false};
    bool v4_horizon_local_gate_active{false};
    // Derived from the peer's published V4 diagnostics. This advertises only
    // bootstrap readiness; it is neither a command nor a committed cutover.
    bool v4_cutover_candidate_ready{false};
};

inline bool v4CutoverCandidateReady(
    const ManeuverSelectionDecision & decision) noexcept
{
    return decision.v4_enabled
        && !decision.v4_shadow_only
        && decision.v4_shadow_evaluated
        && decision.v4_shadow_status
            == V4ShadowEvaluationStatus::CoreEvaluated
        && decision.v4_candidates.status
            == SafeControlCandidateAdapterStatus::Valid
        && decision.v4_candidates.candidate_count > 0
        && decision.v4_candidates.candidate_count
            <= kMaximumSafeControlCandidates;
}

inline bool maneuverCommandExecutionRequested(
    ManeuverExecutionPolicy policy,
    const ManeuverSelectionDecision & decision) noexcept
{
    if (!decision.coordination_qualified) {
        return false;
    }
    if (policy == ManeuverExecutionPolicy::ContinuousV4) {
        return decision.selected_v4_cutover;
    }
    if (policy == ManeuverExecutionPolicy::HorizonGatedV4) {
        return decision.selected_v4_cutover
            && decision.v4_horizon_gate_active;
    }
    return decision.activation_requested;
}

inline bool updateV4HorizonGateState(
    bool previous_active,
    bool evaluation_valid,
    double h_worst_m,
    double trigger_m) noexcept
{
    if (!evaluation_valid || !std::isfinite(h_worst_m)
        || !std::isfinite(trigger_m) || trigger_m <= 0.0) {
        // A transient missing snapshot must not cancel an avoidance command
        // that was already justified by the last complete common horizon.
        return previous_active;
    }
    return h_worst_m <= trigger_m;
}

inline bool v4HorizonHoldElapsed(
    std::uint64_t activation_timestamp_us,
    std::uint64_t now_us,
    std::uint64_t hold_duration_us) noexcept
{
    return hold_duration_us > 0
        && now_us >= activation_timestamp_us
        && now_us - activation_timestamp_us >= hold_duration_us;
}

inline bool v4HorizonFailClosedRequested(
    ManeuverExecutionPolicy policy,
    SafeControlSetStatus safe_control_status) noexcept
{
    return policy == ManeuverExecutionPolicy::HorizonGatedV4
        && safe_control_status == SafeControlSetStatus::SearchSetInfeasible;
}

struct ManeuverSelectionWorkerOutput
{
    std::uint64_t generated_timestamp_us{0};
    std::uint64_t selection_epoch{0};
    std::array<
        estimation::TrajectoryIntentPacket,
        kMaximumIntentPacketsPerOutput> intent_packets{};
    std::size_t intent_packet_count{0};
    ManeuverSelectionDecision decision{};
    bool has_decision{false};
};

class ManeuverSelectionWorker
{
public:
    explicit ManeuverSelectionWorker(
        const ManeuverSelectionWorkerParams & params = {});
    ~ManeuverSelectionWorker();

    ManeuverSelectionWorker(const ManeuverSelectionWorker &) = delete;
    ManeuverSelectionWorker & operator=(const ManeuverSelectionWorker &) = delete;

    bool start();
    void stop();
    bool running() const noexcept;

    bool pushOwnshipBelief(
        const ManeuverSelectionBeliefSnapshot & snapshot) noexcept;
    bool pushAirspeed(
        const ManeuverSelectionAirspeedSnapshot & snapshot) noexcept;
    bool pushNominalSetpoint(
        const ManeuverSelectionNominalSetpointSnapshot & snapshot) noexcept;
    bool pushRemoteIntent(
        int remote_vehicle_id,
        const estimation::TrajectoryIntentPacket & packet) noexcept;
    bool pushRemoteDecision(
        int remote_vehicle_id,
        const ManeuverSelectionPeerDecision & decision) noexcept;
    void setActivationEnabled(bool enabled) noexcept;
    std::optional<ManeuverSelectionWorkerOutput> tryPopOutput() noexcept;

    // Deterministic test/benchmark entry point. Do not call while start() is active.
    bool processPendingForTest();

    std::uint64_t droppedInputCount() const noexcept;
    std::uint64_t droppedOutputCount() const noexcept;

private:
    enum class InputKind : std::uint8_t
    {
        OwnshipBelief,
        Airspeed,
        NominalSetpoint,
        RemoteIntent,
        RemoteDecision,
    };

    struct WorkerInput
    {
        InputKind kind{InputKind::OwnshipBelief};
        int remote_vehicle_id{-1};
        ManeuverSelectionBeliefSnapshot belief{};
        ManeuverSelectionAirspeedSnapshot airspeed{};
        ManeuverSelectionNominalSetpointSnapshot nominal{};
        estimation::TrajectoryIntentPacket packet{};
        ManeuverSelectionPeerDecision decision{};
    };

    struct RemoteCandidateCache
    {
        std::uint64_t selection_epoch{0};
        std::uint64_t source_timestamp_us{0};
        estimation::CandidateSetKind candidate_set_kind{
            estimation::CandidateSetKind::LegacyRoll};
        std::size_t expected_count{0};
        ExhaustiveCandidateIntentSet candidates{};
        std::array<bool, kExhaustiveCandidatesPerAircraft> occupied{};
        std::size_t count{0};
    };

    struct RemoteDecisionCache
    {
        ManeuverSelectionPeerDecision decision{};
        bool valid{false};
    };

    struct RemoteSelectedIntentCache
    {
        estimation::ReceivedTrajectoryIntent intent{};
        bool valid{false};
    };

    struct PendingSelectionProposal
    {
        std::uint64_t timestamp_us{0};
        std::uint64_t epoch{0};
        std::array<std::uint8_t, kMaximumSelectionAircraft>
            candidate_ids{};
        std::array<std::uint64_t, kMaximumSelectionAircraft>
            candidate_input_revisions{};
        std::array<std::uint64_t, kMaximumSelectionAircraft>
            candidate_source_timestamps_us{};
        bool v4_cutover{false};
        estimation::PredictInput ownship_input{};
        JointCombinationEvaluation current_evaluation{};
        JointCombinationEvaluation evaluation{};
        bool active_command_change{false};
        bool superiority_evaluated{false};
        bool clearly_superior{false};
        std::size_t combination_index{0};
        std::size_t combination_count{0};
        std::uint64_t last_readiness_publish_timestamp_us{0};
        bool valid{false};
        bool resolved{false};
    };

    void workerLoop();
    bool processPending();
    bool acceptOwnshipBelief(const ManeuverSelectionBeliefSnapshot & snapshot);
    bool acceptAirspeed(
        const ManeuverSelectionAirspeedSnapshot & snapshot);
    bool acceptNominalSetpoint(
        const ManeuverSelectionNominalSetpointSnapshot & snapshot);
    bool acceptRemoteIntent(
        int remote_vehicle_id,
        const estimation::TrajectoryIntentPacket & packet);
    bool acceptRemoteDecision(
        int remote_vehicle_id,
        const ManeuverSelectionPeerDecision & decision);
    void initializeCandidateSet(std::uint64_t now_us);
    void refreshCandidateSet(std::uint64_t now_us);
    void chooseAlternates(std::uint64_t now_us);
    std::array<CandidateSafetyScore, estimation::kManeuverCandidateCount>
        scoreEligibleCandidates(std::uint64_t now_us);
    bool buildCurrentIntentSet(
        std::uint64_t now_us,
        ManeuverSelectionWorkerOutput & output);
    bool appendFlockingHandoffIntent(
        std::uint64_t now_us,
        ManeuverSelectionWorkerOutput & output);
    bool buildV4IntentSet(
        std::uint64_t now_us,
        const SafeControlCandidateAdapterResult & candidates,
        ManeuverSelectionWorkerOutput & output);
    void evaluateV4(
        std::uint64_t now_us,
        ManeuverSelectionWorkerOutput & output);
    void evaluateCurrentSet(
        std::uint64_t now_us,
        ManeuverSelectionWorkerOutput & output);
    bool evaluateV4HorizonGate(
        std::uint64_t now_us,
        const MultiAircraftExhaustiveCandidateIntentSets & candidate_sets,
        const std::array<std::size_t, kMaximumSelectionAircraft>
            & candidate_counts,
        ManeuverSelectionDecision & decision);
    void rollupV4HorizonGate(std::uint64_t now_us) noexcept;
    bool latchV4HorizonOwnshipCandidate() noexcept;
    bool constrainV4ActiveAircraftCandidates(
        MultiAircraftExhaustiveCandidateIntentSets & candidate_sets,
        const std::array<std::size_t, kMaximumSelectionAircraft>
            & candidate_counts) const;
    bool evaluateSelectedTuple(
        std::uint64_t evaluation_timestamp_us,
        const MultiAircraftExhaustiveCandidateIntentSets & candidate_sets,
        const std::array<std::size_t, kMaximumSelectionAircraft>
            & candidate_counts,
        JointCombinationEvaluation & evaluation) const;
    bool proposalChangesActiveCommand(
        const std::array<std::uint8_t, kMaximumSelectionAircraft>
            & candidate_ids,
        const std::array<std::uint64_t, kMaximumSelectionAircraft>
            & candidate_input_revisions) const noexcept;
    bool clearlySuperior(
        const JointCombinationEvaluation & current,
        const JointCombinationEvaluation & proposed) const noexcept;
    bool allProposalParticipantsReady() const noexcept;
    bool finalizePendingCoordination(
        ManeuverSelectionWorkerOutput & output);
    bool buildActivationSample(
        std::uint64_t now_us,
        ManeuverActivationSample & sample,
        ManeuverSelectionDecision & decision) const;
    bool evaluatePostHandoffTuple(
        std::uint64_t now_us,
        double & minimum_ad_m) const;
    bool allPeerHandoffReady(
        std::uint64_t epoch,
        bool require_consensus) const noexcept;
    void updateActivationState(
        std::uint64_t now_us,
        bool force_decision_output,
        ManeuverSelectionWorkerOutput & output);
    std::size_t activeCandidateCount() const noexcept;
    bool v4CutoverMode() const noexcept;
    bool allV4CutoverParticipantsReady(
        const ManeuverSelectionDecision & local_decision) const noexcept;
    bool publishOutput(const ManeuverSelectionWorkerOutput & output) noexcept;

    ManeuverSelectionWorkerParams m_params;
    estimation::TrajectoryPredict m_predictor;
    estimation::ManeuverCandidateTable m_candidate_table;
    estimation::TrajectoryIntentSender m_sender;
    estimation::TrajectoryIntentReceiver m_receiver;
    estimation::TrajectoryUncertainty m_uncertainty;
    ManeuverCombinationEvaluator m_pair_evaluator;
    PositiveMarginBarrierEvaluator m_barrier_evaluator;
    JointManeuverCombinationEvaluator m_joint_evaluator;
    ExhaustiveManeuverCombinationEvaluator m_exhaustive_evaluator;
    HeuristicCandidateSelector m_candidate_selector;
    ManeuverActivationController m_activation_controller;
    SafeControlSetV4 m_v4_safe_control;
    SafeControlCandidateAdapter m_v4_candidate_adapter;

    common::SpscQueue<WorkerInput, kSelectionWorkerInputCapacity> m_input_queue{};
    common::SpscQueue<
        ManeuverSelectionWorkerOutput, kSelectionWorkerOutputCapacity> m_output_queue{};
    std::thread m_thread;
    std::atomic<bool> m_running{false};
    std::atomic<bool> m_activation_enabled{true};
    std::atomic<std::uint64_t> m_dropped_inputs{0};
    std::atomic<std::uint64_t> m_dropped_outputs{0};

    estimation::PredictState m_latest_state{};
    estimation::PredictStateCovariance m_latest_covariance{};
    std::uint64_t m_latest_state_timestamp_us{0};
    bool m_has_latest_state{false};

    ManeuverSelectionAirspeedSnapshot m_latest_airspeed{};
    bool m_has_latest_airspeed{false};
    ManeuverSelectionNominalSetpointSnapshot m_latest_nominal{};
    bool m_has_latest_nominal{false};

    std::array<std::uint8_t, kExhaustiveCandidatesPerAircraft>
        m_held_candidate_ids{};
    std::uint8_t m_current_best_id{
        static_cast<std::uint8_t>(estimation::ManeuverCandidateId::RollZero)};
    std::array<std::uint8_t, kMaximumSelectionAircraft>
        m_selected_candidate_ids{};
    std::array<std::uint64_t, kMaximumSelectionAircraft>
        m_selected_candidate_input_revisions{};
    std::array<std::uint64_t, kMaximumSelectionAircraft>
        m_selected_candidate_source_timestamps_us{};
    ManeuverSelectionDecision m_latest_selection_decision{};
    bool m_has_selected_combination{false};
    std::uint64_t m_last_activation_monitor_timestamp_us{0};
    std::uint64_t m_selection_epoch{0};
    std::uint64_t m_epoch_generation_timestamp_us{0};
    std::uint64_t m_next_candidate_refresh_timestamp_us{0};
    std::uint64_t m_next_trajectory_refresh_timestamp_us{0};
    bool m_candidate_set_initialized{false};
    bool m_epoch_evaluated{false};

    ExhaustiveCandidateIntentSet m_ownship_candidates{};
    bool m_ownship_candidates_complete{false};
    std::size_t m_ownship_candidate_count{0};
    estimation::CandidateSetKind m_ownship_candidate_set_kind{
        estimation::CandidateSetKind::LegacyRoll};
    bool m_v4_cutover_ready{false};
    SafeControlCandidateAdapterResult m_v4_epoch_candidates{};
    std::uint64_t m_v4_epoch_candidate_selection_epoch{0};
    bool m_v4_epoch_candidates_valid{false};
    bool m_selected_v4_cutover{false};
    bool m_v4_horizon_local_gate_active{false};
    bool m_v4_horizon_gate_active{false};
    std::uint64_t m_v4_horizon_activation_timestamp_us{0};
    std::uint8_t m_v4_horizon_latched_candidate_id{0};
    std::uint64_t m_v4_horizon_latched_input_revision{0};
    estimation::PredictInput m_v4_horizon_latched_input{};
    bool m_v4_horizon_latch_valid{false};
    // After a coordinated return, the actual nominal Flocking trajectory is
    // the executed plan. Continue supervising that plan without a time-based
    // reactivation lockout; leave this state immediately if its AD turns
    // negative or cannot be evaluated.
    bool m_following_verified_flocking_handoff{false};
    std::array<RemoteCandidateCache, kMaximumSelectionAircraft>
        m_remote_caches{};
    std::array<RemoteCandidateCache, kMaximumSelectionAircraft>
        m_remote_previous_caches{};
    // A selected trajectory may outlive both candidate-set epochs.  Retain
    // only that one intent so the previous-set slot remains available for the
    // 4 Hz coordination race without duplicating another full trajectory set.
    std::array<RemoteSelectedIntentCache, kMaximumSelectionAircraft>
        m_remote_selected_caches{};
    // Keep the five full reconstructed handoff cones off the worker's stack.
    // They are allocated once at construction and reused at 20 Hz.
    std::unique_ptr<std::array<
        RemoteSelectedIntentCache, kMaximumSelectionAircraft>>
        m_handoff_intents;
    std::array<RemoteCandidateCache, kMaximumSelectionAircraft>
        m_remote_staging_caches{};
    std::array<RemoteDecisionCache, kMaximumSelectionAircraft>
        m_remote_decision_caches{};
    PendingSelectionProposal m_pending_proposal{};
};

const char * v4ShadowEvaluationStatusName(
    V4ShadowEvaluationStatus status) noexcept;

const char * v4SnapshotStatusName(V4SnapshotStatus status) noexcept;

const char * v4AirspeedSourceName(V4AirspeedSource source) noexcept;

}  // namespace collision_avoidance::selection
