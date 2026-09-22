#pragma once

#include <atomic>
#include <memory>
#include <thread>

#include <collision_avoidance/common/SpscQueue.hpp>
#include <collision_avoidance/selection/InteractionGraph.hpp>

namespace collision_avoidance::selection
{
// Immutable from submit() until release(): no live worker/cache references.
struct ManeuverEvaluationRequest
{
    std::uint64_t timestamp_us{0}, epoch{0};
    std::size_t aircraft_count{0};
    bool complete{false}, exhaustive{false};
    estimation::CandidateSetKind set_kind{estimation::CandidateSetKind::LegacyRoll};
    MultiAircraftExhaustiveCandidateIntentSets candidates{};
    std::array<std::size_t, kMaximumSelectionAircraft> counts{};
    InteractionGraphDiagnostics graph{};
    estimation::PredictInput nominal_input{};
    std::uint64_t nominal_timestamp_us{0};
    bool nominal_available{false};
};

struct ManeuverEvaluationResult
{
    JointCombinationEvaluation best{};
    std::size_t best_index{0}, combination_count{0}, valid_count{0}, safe_count{0};
    double maximum_minimum_ad_m{std::numeric_limits<double>::quiet_NaN()};
    bool evaluated{false};
    InteractionGraphDiagnostics graph{};
    PairwiseAdCertificationSet certifications{};
    std::uint64_t start_ns{0}, end_ns{0};
};

struct ManeuverEvaluationTask
{
    ManeuverEvaluationRequest request{};
    ManeuverEvaluationResult result{};
};

// One preallocated task and two index-only SPSC handoffs. The owner never
// waits for evaluation and never overwrites an in-flight snapshot.
// start/stop are lifecycle-owner only; stop the submitting worker first.
class ManeuverEvaluationWorker
{
public:
    ManeuverEvaluationWorker(const ManeuverCombinationEvaluatorParams & evaluator,
                             const InteractionGraphParams & graph,
                             const estimation::PredictParams & predictor = {},
                             const estimation::UncertaintyParams & uncertainty = {});
    ~ManeuverEvaluationWorker();
    void start();
    void stop();
    ManeuverEvaluationRequest * beginRequest() noexcept;
    void submit() noexcept;
    const ManeuverEvaluationTask * readyResult() const noexcept;
    void release() noexcept;
    // Same kernel, manually scheduled only while no evaluation thread runs.
    bool processOneForTest();

private:
    bool processOne();
    void evaluate();
    void evaluateGraph();
    bool rebuildGraphLibraryAtEvaluationTime();
    void loop();

    InteractionGraphParams m_graph_params;
    JointManeuverCombinationEvaluator m_joint;
    ExhaustiveManeuverCombinationEvaluator m_exhaustive;
    PairwiseAdCertificationEvaluator m_pairwise;
    CertifiedComponentManeuverEvaluator m_component;
    InteractionGraphBuilder m_graph_builder;
    // Private evaluation-thread scratch: the shared request and live caches
    // keep their original source timestamps and compressed-trajectory meaning.
    estimation::TrajectoryIntentSender m_sender;
    estimation::TrajectoryIntentReceiver m_receiver;
    double m_stale_timeout_s;
    std::unique_ptr<MultiAircraftExhaustiveCandidateIntentSets> m_evaluation_candidates;
    std::unique_ptr<ManeuverEvaluationTask> m_task;
    common::SpscQueue<unsigned, 1> m_requests;
    common::SpscQueue<unsigned, 1> m_results;
    bool m_in_flight{false}; // owner only
    std::atomic<bool> m_running{false};
    std::thread m_thread;
};
}  // namespace collision_avoidance::selection
