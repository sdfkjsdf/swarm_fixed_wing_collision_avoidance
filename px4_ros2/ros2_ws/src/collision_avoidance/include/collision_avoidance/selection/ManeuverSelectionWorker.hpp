#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <thread>

#include <collision_avoidance/common/SpscQueue.hpp>
#include <collision_avoidance/estimation/trajectory_prediction/TrajectoryIntent.hpp>
#include <collision_avoidance/selection/HeuristicCandidateSelector.hpp>
#include <collision_avoidance/selection/ManeuverActivationController.hpp>
#include <collision_avoidance/selection/ManeuverCombinationEvaluator.hpp>

namespace collision_avoidance::selection
{

inline constexpr std::size_t kEligibleLateralCandidateCount = 7;
inline constexpr std::size_t kSelectionWorkerInputCapacity = 64;
inline constexpr std::size_t kSelectionWorkerOutputCapacity = 16;

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
    bool exhaustive_test_mode{false};
    std::array<std::uint8_t, kEligibleLateralCandidateCount> eligible_candidate_ids{
        static_cast<std::uint8_t>(estimation::ManeuverCandidateId::RollZero),
        static_cast<std::uint8_t>(estimation::ManeuverCandidateId::RollMinus15),
        static_cast<std::uint8_t>(estimation::ManeuverCandidateId::RollPlus15),
        static_cast<std::uint8_t>(estimation::ManeuverCandidateId::RollMinus30),
        static_cast<std::uint8_t>(estimation::ManeuverCandidateId::RollPlus30),
        static_cast<std::uint8_t>(estimation::ManeuverCandidateId::RollMinus45),
        static_cast<std::uint8_t>(estimation::ManeuverCandidateId::RollPlus45)};
};

struct ManeuverSelectionBeliefSnapshot
{
    std::uint64_t timestamp_us{0};
    std::uint64_t timestamp_sample_us{0};
    estimation::EstimatorTrajectoryBelief belief{};
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
    std::uint8_t ownship_candidate_id{
        static_cast<std::uint8_t>(estimation::ManeuverCandidateId::RollZero)};
    std::uint64_t proposal_timestamp_us{0};
    std::uint64_t proposal_epoch{0};
    std::array<std::uint8_t, kMaximumSelectionAircraft>
        proposed_candidate_ids{};
    bool proposal_valid{false};
    bool proposal_consensus_confirmed{false};
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
    bool activation_just_started{false};
    bool activation_just_ended{false};
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
    std::uint8_t ownship_candidate_id{
        static_cast<std::uint8_t>(estimation::ManeuverCandidateId::RollZero)};
    std::uint64_t proposal_timestamp_us{0};
    std::uint64_t proposal_epoch{0};
    std::array<std::uint8_t, kMaximumSelectionAircraft>
        proposed_candidate_ids{};
    bool proposal_valid{false};
    bool proposal_consensus_confirmed{false};
    bool coordination_qualified{false};
    bool activation_requested{false};
};

struct ManeuverSelectionWorkerOutput
{
    std::uint64_t generated_timestamp_us{0};
    std::uint64_t selection_epoch{0};
    std::array<
        estimation::TrajectoryIntentPacket,
        kExhaustiveCandidatesPerAircraft> intent_packets{};
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
        RemoteIntent,
        RemoteDecision,
    };

    struct WorkerInput
    {
        InputKind kind{InputKind::OwnshipBelief};
        int remote_vehicle_id{-1};
        ManeuverSelectionBeliefSnapshot belief{};
        estimation::TrajectoryIntentPacket packet{};
        ManeuverSelectionPeerDecision decision{};
    };

    struct RemoteCandidateCache
    {
        std::uint64_t selection_epoch{0};
        std::uint64_t source_timestamp_us{0};
        ExhaustiveCandidateIntentSet candidates{};
        std::array<bool, kExhaustiveCandidatesPerAircraft> occupied{};
        std::size_t count{0};
    };

    struct RemoteDecisionCache
    {
        ManeuverSelectionPeerDecision decision{};
        bool valid{false};
    };

    struct PendingSelectionProposal
    {
        std::uint64_t timestamp_us{0};
        std::uint64_t epoch{0};
        std::array<std::uint8_t, kMaximumSelectionAircraft>
            candidate_ids{};
        JointCombinationEvaluation evaluation{};
        std::size_t combination_index{0};
        std::size_t combination_count{0};
        bool valid{false};
        bool resolved{false};
    };

    void workerLoop();
    bool processPending();
    bool acceptOwnshipBelief(const ManeuverSelectionBeliefSnapshot & snapshot);
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
    void evaluateCurrentSet(
        std::uint64_t now_us,
        ManeuverSelectionWorkerOutput & output);
    bool constrainActiveAircraftCandidates(
        MultiAircraftExhaustiveCandidateIntentSets & candidate_sets) const;
    bool finalizePendingCoordination(
        ManeuverSelectionWorkerOutput & output);
    bool buildActivationSample(
        std::uint64_t now_us,
        ManeuverActivationSample & sample,
        ManeuverSelectionDecision & decision) const;
    void updateActivationState(
        std::uint64_t now_us,
        bool force_decision_output,
        ManeuverSelectionWorkerOutput & output);
    std::size_t activeCandidateCount() const noexcept;
    bool publishOutput(const ManeuverSelectionWorkerOutput & output) noexcept;

    ManeuverSelectionWorkerParams m_params;
    estimation::TrajectoryPredict m_predictor;
    estimation::ManeuverCandidateTable m_candidate_table;
    estimation::TrajectoryIntentSender m_sender;
    estimation::TrajectoryIntentReceiver m_receiver;
    estimation::TrajectoryUncertainty m_uncertainty;
    ManeuverCombinationEvaluator m_pair_evaluator;
    JointManeuverCombinationEvaluator m_joint_evaluator;
    ExhaustiveManeuverCombinationEvaluator m_exhaustive_evaluator;
    HeuristicCandidateSelector m_candidate_selector;
    ManeuverActivationController m_activation_controller;

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

    std::array<std::uint8_t, kExhaustiveCandidatesPerAircraft>
        m_held_candidate_ids{};
    std::uint8_t m_current_best_id{
        static_cast<std::uint8_t>(estimation::ManeuverCandidateId::RollZero)};
    std::array<std::uint8_t, kMaximumSelectionAircraft>
        m_selected_candidate_ids{};
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
    std::array<RemoteCandidateCache, kMaximumSelectionAircraft>
        m_remote_caches{};
    std::array<RemoteCandidateCache, kMaximumSelectionAircraft>
        m_remote_previous_caches{};
    std::array<RemoteCandidateCache, kMaximumSelectionAircraft>
        m_remote_staging_caches{};
    std::array<RemoteDecisionCache, kMaximumSelectionAircraft>
        m_remote_decision_caches{};
    PendingSelectionProposal m_pending_proposal{};
};

}  // namespace collision_avoidance::selection
