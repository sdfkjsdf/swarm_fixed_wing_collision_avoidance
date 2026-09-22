#include <collision_avoidance/selection/ManeuverEvaluationWorker.hpp>

#include <algorithm>
#include <cassert>
#include <chrono>

namespace collision_avoidance::selection
{
namespace
{
constexpr auto kRollZeroId = static_cast<std::uint8_t>(estimation::ManeuverCandidateId::RollZero);
std::uint64_t steadyNs() noexcept
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
constexpr std::uint64_t kInteractionFnvOffset = 14695981039346656037ULL;
constexpr std::uint64_t kInteractionFnvPrime = 1099511628211ULL;

inline std::uint64_t assembledCandidateHash(
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

} // namespace

ManeuverEvaluationWorker::ManeuverEvaluationWorker(
    const ManeuverCombinationEvaluatorParams & evaluator, const InteractionGraphParams & graph,
    const estimation::PredictParams & predictor, const estimation::UncertaintyParams & uncertainty)
: m_graph_params(graph), m_joint(evaluator), m_exhaustive(evaluator),
  m_pairwise(evaluator), m_graph_builder(graph),
  m_sender(estimation::TrajectoryPredict(predictor), {}),
  m_receiver(estimation::TrajectoryPredict(predictor), uncertainty),
  m_stale_timeout_s(evaluator.stale_timeout_s),
  m_evaluation_candidates(graph.enabled
      ? std::make_unique<MultiAircraftExhaustiveCandidateIntentSets>() : nullptr),
  m_task(std::make_unique<ManeuverEvaluationTask>())
{
}

ManeuverEvaluationWorker::~ManeuverEvaluationWorker() { stop(); }

void ManeuverEvaluationWorker::start()
{
    if (m_thread.joinable()) return;
    m_running.store(true, std::memory_order_release);
    m_thread = std::thread(&ManeuverEvaluationWorker::loop, this);
}

void ManeuverEvaluationWorker::stop()
{
    m_running.store(false, std::memory_order_release);
    if (m_thread.joinable()) m_thread.join();
    // Lifecycle owner only: neither producer nor consumer runs here.
    m_requests.try_pop();
    m_results.try_pop();
    m_in_flight = false;
}

ManeuverEvaluationRequest * ManeuverEvaluationWorker::beginRequest() noexcept
{
    return m_in_flight ? nullptr : &m_task->request;
}

void ManeuverEvaluationWorker::submit() noexcept
{
    assert(!m_in_flight);
    m_in_flight = true;
    const bool pushed = m_requests.try_push(0U);
    (void)pushed;
    assert(pushed);
}

const ManeuverEvaluationTask * ManeuverEvaluationWorker::readyResult() const noexcept
{
    return m_results.peekForConsumer() ? m_task.get() : nullptr;
}

void ManeuverEvaluationWorker::release() noexcept
{
    const auto done = m_results.try_pop();
    assert(done.has_value());
    m_in_flight = false;
}

bool ManeuverEvaluationWorker::processOneForTest()
{
    if (m_thread.joinable()) return false;
    return processOne();
}

bool ManeuverEvaluationWorker::processOne()
{
    if (!m_requests.try_pop()) return false;
    evaluate();
    const bool pushed = m_results.try_push(0U);
    (void)pushed;
    assert(pushed);
    return true;
}

void ManeuverEvaluationWorker::loop()
{
    while (m_running.load(std::memory_order_acquire)) {
        if (!processOne()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void ManeuverEvaluationWorker::evaluate()
{
    const auto & request = m_task->request;
    auto & result = m_task->result;
    const auto begin = steadyNs();
    result = ManeuverEvaluationResult{};
    result.start_ns = begin;
    result.graph = request.graph;
    if (m_graph_params.enabled) {
        evaluateGraph();
        const auto & graph = result.graph;
        if (graph.component_search_evaluated && graph.global_crosscheck_pass) {
            result.best = graph.global_crosscheck_evaluation;
            result.combination_count = graph.graph.component_evaluation_count;
            result.valid_count = graph.component_valid_evaluation_count;
            result.safe_count = graph.component_safe_evaluation_count;
            result.maximum_minimum_ad_m = result.best.minimum_ad_m;
            result.evaluated = result.best.valid;
        }
    } else if (request.complete && request.exhaustive) {
        ExhaustiveManeuverEvaluation evaluation;
        result.evaluated = m_exhaustive.evaluate(request.timestamp_us,
            request.candidates, request.aircraft_count, evaluation);
        result.combination_count = evaluation.combination_count;
        result.valid_count = evaluation.valid_combination_count;
        result.safe_count = evaluation.safe_combination_count;
        result.maximum_minimum_ad_m = evaluation.maximum_minimum_ad_m;
        if (result.evaluated) {
            result.best = evaluation.best_combination;
            result.best_index = evaluation.best_combination_index;
        }
    } else if (request.complete) {
        MultiAircraftCandidateIntentSets reduced{};
        for (std::size_t aircraft = 0; aircraft < request.aircraft_count; ++aircraft) {
            std::copy_n(request.candidates[aircraft].begin(), request.counts[aircraft],
                        reduced[aircraft].begin());
            std::fill(reduced[aircraft].begin() + request.counts[aircraft],
                      reduced[aircraft].end(), reduced[aircraft][0]);
        }
        JointManeuverEvaluation evaluation;
        result.evaluated = m_joint.evaluate(request.timestamp_us, reduced,
            request.counts, request.aircraft_count, evaluation);
        result.combination_count = evaluation.combination_count;
        result.valid_count = evaluation.valid_combination_count;
        result.safe_count = evaluation.safe_combination_count;
        result.maximum_minimum_ad_m = evaluation.maximum_minimum_ad_m;
        if (result.evaluated) {
            result.best = evaluation.combinations[evaluation.best_combination_index];
            result.best_index = evaluation.best_combination_index;
        }
    }
    result.end_ns = steadyNs();
}

void ManeuverEvaluationWorker::evaluateGraph()
{
    using Clock = std::chrono::steady_clock;
    const auto total_start = Clock::now();
    const auto & request = m_task->request;
    const auto now_us = request.timestamp_us;
    const auto aircraft_count = request.aircraft_count;
    auto & diagnostics = m_task->result.graph;
    std::fill_n(
        diagnostics.assembled_candidate_ids.begin(),
        aircraft_count,
        kRollZeroId);
    diagnostics.assembled_candidate_valid_mask =
        (std::uint32_t{1} << aircraft_count) - 1U;
    const bool frozen_library_complete = request.complete;
    if (!frozen_library_complete) {
        diagnostics.graph.status =
            InteractionGraphStatus::InvalidCertification;
        diagnostics.graph.evaluation_timestamp_us = now_us;
        diagnostics.graph.selection_epoch = request.epoch;
        diagnostics.graph.aircraft_count = aircraft_count;
        diagnostics.status =
            InteractionGraphEvaluationStatus::CandidateSetsIncomplete;
        diagnostics.total_evaluation_time_ns = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                Clock::now() - total_start).count());

        return;
    }
    // Advancing an old hypothetical candidate along itself credits the new
    // command with execution BEFORE it was selected. First advance the common
    // source state/P under the reported held control, then start every candidate
    // at the evaluation timestamp. Do this once per library, not per pair.
    // No added application delay or distance margin is introduced here.
    if (!rebuildGraphLibraryAtEvaluationTime()) {
        diagnostics.graph.status = InteractionGraphStatus::InvalidCertification;
        diagnostics.graph.evaluation_timestamp_us = now_us;
        diagnostics.graph.selection_epoch = request.epoch;
        diagnostics.graph.aircraft_count = aircraft_count;
        diagnostics.status = InteractionGraphEvaluationStatus::GraphInvalid;
        diagnostics.total_evaluation_time_ns = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                Clock::now() - total_start).count());
        return;
    }
    const auto & certified_candidate_sets = *m_evaluation_candidates;

    auto & certifications = m_task->result.certifications;
    if (!m_pairwise.evaluate(
            now_us,
            request.epoch,
            m_graph_params.trajectory_library_version,
            m_graph_params.ad_masd_config_version,
            certified_candidate_sets,
            aircraft_count,
            certifications)) {
        certifications.valid = false;
        diagnostics.graph.status =
            InteractionGraphStatus::InvalidCertification;
        diagnostics.graph.evaluation_timestamp_us = now_us;
        diagnostics.graph.selection_epoch = request.epoch;
        diagnostics.graph.aircraft_count = aircraft_count;
        diagnostics.status =
            InteractionGraphEvaluationStatus::GraphInvalid;
        diagnostics.total_evaluation_time_ns = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                Clock::now() - total_start).count());

        return;
    }
    const PairwiseAdCertificationSet & certified_pairs = certifications;

    diagnostics.graph = m_graph_builder.build(certified_pairs);
    // Public source timestamps remain packet provenance (for age/replay
    // analysis), not the start time of our private derived rollout.
    for (std::size_t aircraft = 0; aircraft < aircraft_count; ++aircraft)
        diagnostics.graph.source_timestamps_us[aircraft] =
            request.candidates[aircraft][0].source_timestamp_us;
    if (!diagnostics.graph.valid()) {
        diagnostics.status = InteractionGraphEvaluationStatus::GraphInvalid;
        diagnostics.total_evaluation_time_ns = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                Clock::now() - total_start).count());

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
            continue;
        }
        CertifiedComponentEvaluation component_evaluation;
        if (!m_component.evaluate(
                certified_pairs,
                certified_candidate_sets,
                members,
                member_count,
                component_evaluation)
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
        }
    }
    diagnostics.component_search_time_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            Clock::now() - search_start).count());
    if (!component_search_valid
        || actual_evaluation_count
            != diagnostics.graph.component_evaluation_count) {
        diagnostics.status =
            InteractionGraphEvaluationStatus::ComponentEvaluationFailed;
        diagnostics.total_evaluation_time_ns = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                Clock::now() - total_start).count());

        return;
    }
    diagnostics.component_search_evaluated = true;
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
    diagnostics.global_crosscheck_evaluated =
        m_component.evaluateTuple(
            certified_pairs,
            certified_candidate_sets,
            diagnostics.assembled_candidate_ids,
            crosscheck);
    diagnostics.global_crosscheck_time_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            Clock::now() - crosscheck_start).count());
    if (diagnostics.global_crosscheck_evaluated) {
        bool cross_component_pairs_safe = true;
        for (std::size_t first = 0; first < aircraft_count; ++first) {
            for (std::size_t second = first + 1;
                 second < aircraft_count; ++second) {
                if (diagnostics.graph.component_ids[first]
                    == diagnostics.graph.component_ids[second]) {
                    continue;
                }
                const auto * pair = certified_pairs.findPair(first, second);
                const auto * evaluation = pair == nullptr ? nullptr
                    : pair->find(
                        crosscheck.candidate_slots[first],
                        crosscheck.candidate_slots[second]);
                if (evaluation == nullptr
                    || evaluation->validity != CombinationValidity::Valid
                    || !evaluation->feasible) {
                    cross_component_pairs_safe = false;
                    break;
                }
            }
            if (!cross_component_pairs_safe) {
                break;
            }
        }
        diagnostics.global_crosscheck_minimum_ad_m = crosscheck.minimum_ad_m;
        diagnostics.global_crosscheck_pass =
            crosscheck.valid && cross_component_pairs_safe;
        diagnostics.global_crosscheck_evaluation = crosscheck;
    }
    diagnostics.status = diagnostics.global_crosscheck_pass
        ? InteractionGraphEvaluationStatus::Evaluated
        : InteractionGraphEvaluationStatus::GlobalCrosscheckFailed;
    diagnostics.total_evaluation_time_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            Clock::now() - total_start).count());

}

bool ManeuverEvaluationWorker::rebuildGraphLibraryAtEvaluationTime()
{
    const auto & request = m_task->request;
    if (!m_evaluation_candidates || request.aircraft_count < 2
        || request.aircraft_count > kMaximumSelectionAircraft) return false;
    for (std::size_t aircraft = 0; aircraft < request.aircraft_count; ++aircraft) {
        const auto & originals = request.candidates[aircraft];
        const auto & source = originals[0];
        if (source.source_timestamp_us > request.timestamp_us
            || static_cast<double>(request.timestamp_us - source.source_timestamp_us) * 1e-6
                > std::min(m_stale_timeout_s, estimation::kTrajectoryIntentHorizonSeconds)
            || request.counts[aircraft] != kExhaustiveCandidatesPerAircraft) return false;
        estimation::PredictState state;
        estimation::PredictStateCovariance covariance;
        if (!m_receiver.executionStateAt(source, request.timestamp_us, state, covariance))
            return false; // Missing actual-input metadata cannot certify an aged library.
        for (std::size_t slot = 0; slot < kExhaustiveCandidatesPerAircraft; ++slot) {
            const auto & original = originals[slot];
            if (original.source_timestamp_us != source.source_timestamp_us
                || original.selection_epoch != request.epoch
                || original.candidate_id != slot
                || original.candidate_set_size != kExhaustiveCandidatesPerAircraft
                || original.candidate_set_kind != estimation::CandidateSetKind::LegacyRoll)
                return false;
            auto & aligned = (*m_evaluation_candidates)[aircraft][slot];
            if (source.source_timestamp_us == request.timestamp_us) {
                aligned = original;
                continue;
            }
            estimation::TrajectoryIntentPacket packet;
            if (!m_sender.buildForCandidateInput(request.timestamp_us, original.candidate_id,
                    original.candidate_input, state, covariance, packet, request.epoch))
                return false;
            packet.candidate_set_size = original.candidate_set_size;
            if (!m_receiver.receive(packet, aligned)) return false;
        }
    }
    return true;
}
} // namespace collision_avoidance::selection
