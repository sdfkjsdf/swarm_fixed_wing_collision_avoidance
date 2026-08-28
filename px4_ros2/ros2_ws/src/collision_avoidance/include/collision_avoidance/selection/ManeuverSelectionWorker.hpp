#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <thread>

#include <collision_avoidance/common/SpscQueue.hpp>
#include <collision_avoidance/estimation/trajectory_prediction/TrajectoryIntent.hpp>
#include <collision_avoidance/selection/ManeuverCombinationEvaluator.hpp>

namespace collision_avoidance::selection
{

inline constexpr std::size_t kEligibleLateralCandidateCount = 7;
inline constexpr std::size_t kSelectionWorkerInputCapacity = 64;
inline constexpr std::size_t kSelectionWorkerOutputCapacity = 16;

struct ManeuverSelectionWorkerParams
{
    estimation::PredictParams predictor_params{};
    estimation::UncertaintyParams uncertainty_params{};
    ManeuverCombinationEvaluatorParams evaluator_params{};
    double ground_speed_command_mps{20.0};
    double gravity_mps2{9.80665};
    std::uint64_t trajectory_refresh_period_us{50'000};
    std::uint64_t candidate_refresh_period_us{250'000};
    std::uint64_t coordination_delay_us{250'000};
    std::uint64_t maximum_belief_delay_us{1'000'000};
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
    std::uint64_t selection_timestamp_us{0};
    std::uint64_t local_selection_epoch{0};
    std::uint64_t remote_selection_epoch{0};
    std::uint8_t ownship_candidate_id{
        static_cast<std::uint8_t>(estimation::ManeuverCandidateId::RollZero)};
    std::uint8_t threat_candidate_id{
        static_cast<std::uint8_t>(estimation::ManeuverCandidateId::RollZero)};
    estimation::PredictInput ownship_input{};
    double pmr_m{0.0};
    double masd_m{0.0};
    double ad_m{0.0};
    bool coordination_qualified{false};
    bool new_best_accepted{false};
    bool previous_best_retained{true};
    bool activation_requested{false};
};

struct ManeuverSelectionWorkerOutput
{
    std::uint64_t generated_timestamp_us{0};
    std::uint64_t selection_epoch{0};
    std::array<estimation::TrajectoryIntentPacket, kCandidatesPerAircraft>
        intent_packets{};
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
        const estimation::TrajectoryIntentPacket & packet) noexcept;
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
    };

    struct WorkerInput
    {
        InputKind kind{InputKind::OwnshipBelief};
        ManeuverSelectionBeliefSnapshot belief{};
        estimation::TrajectoryIntentPacket packet{};
    };

    struct RemoteCandidateCache
    {
        std::uint64_t selection_epoch{0};
        std::uint64_t source_timestamp_us{0};
        std::array<estimation::ReceivedTrajectoryIntent, kCandidatesPerAircraft>
            candidates{};
        std::array<bool, kCandidatesPerAircraft> occupied{};
        std::size_t count{0};
    };

    void workerLoop();
    bool processPending();
    bool acceptOwnshipBelief(const ManeuverSelectionBeliefSnapshot & snapshot);
    bool acceptRemoteIntent(const estimation::TrajectoryIntentPacket & packet);
    void initializeCandidateSet(std::uint64_t now_us);
    void refreshCandidateSet(std::uint64_t now_us);
    void chooseAlternates();
    bool buildCurrentIntentSet(
        std::uint64_t now_us,
        ManeuverSelectionWorkerOutput & output);
    void evaluateCurrentSet(
        std::uint64_t now_us,
        ManeuverSelectionWorkerOutput & output);
    bool publishOutput(const ManeuverSelectionWorkerOutput & output) noexcept;

    ManeuverSelectionWorkerParams m_params;
    estimation::TrajectoryPredict m_predictor;
    estimation::ManeuverCandidateTable m_candidate_table;
    estimation::TrajectoryIntentSender m_sender;
    estimation::TrajectoryIntentReceiver m_receiver;
    estimation::TrajectoryUncertainty m_uncertainty;
    ManeuverCombinationEvaluator m_evaluator;

    common::SpscQueue<WorkerInput, kSelectionWorkerInputCapacity> m_input_queue{};
    common::SpscQueue<
        ManeuverSelectionWorkerOutput, kSelectionWorkerOutputCapacity> m_output_queue{};
    std::thread m_thread;
    std::atomic<bool> m_running{false};
    std::atomic<std::uint64_t> m_dropped_inputs{0};
    std::atomic<std::uint64_t> m_dropped_outputs{0};

    estimation::PredictState m_latest_state{};
    estimation::PredictStateCovariance m_latest_covariance{};
    std::uint64_t m_latest_state_timestamp_us{0};
    bool m_has_latest_state{false};

    std::array<std::uint8_t, kCandidatesPerAircraft> m_held_candidate_ids{};
    std::uint8_t m_current_best_id{
        static_cast<std::uint8_t>(estimation::ManeuverCandidateId::RollZero)};
    std::size_t m_alternate_cursor{0};
    std::uint64_t m_selection_epoch{0};
    std::uint64_t m_epoch_generation_timestamp_us{0};
    std::uint64_t m_next_candidate_refresh_timestamp_us{0};
    std::uint64_t m_next_trajectory_refresh_timestamp_us{0};
    bool m_candidate_set_initialized{false};
    bool m_epoch_evaluated{false};

    CandidateIntentSet m_ownship_candidates{};
    bool m_ownship_candidates_complete{false};
    RemoteCandidateCache m_remote_cache{};
    RemoteCandidateCache m_remote_staging_cache{};
};

}  // namespace collision_avoidance::selection
