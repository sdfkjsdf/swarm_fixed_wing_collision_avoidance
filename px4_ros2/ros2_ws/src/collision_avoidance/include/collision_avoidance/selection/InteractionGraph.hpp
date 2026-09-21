#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

#include <collision_avoidance/selection/PairwiseAdCertification.hpp>

namespace collision_avoidance::selection
{

inline constexpr std::size_t kMaximumInteractionGraphAircraft =
    kMaximumSelectionAircraft;
inline constexpr std::uint8_t kNoInteractionComponent = 0xffU;

enum class InteractionGraphStatus : std::uint8_t
{
    Disabled = 0,
    Valid,
    InvalidConfiguration,
    InvalidCertification,
};

struct InteractionGraphParams
{
    bool enabled{false};
    // Project screening reserve. This is not a published Lockheed parameter.
    double ad_screen_m{0.0};
    // Explicit compatibility identifiers for the frozen candidate library
    // and the PMR/MASD/AD contract used to certify it.
    std::uint64_t trajectory_library_version{1};
    std::uint64_t ad_masd_config_version{1};
    std::uint64_t config_version{1};
};

struct InteractionGraphResult
{
    InteractionGraphStatus status{InteractionGraphStatus::Disabled};
    std::uint64_t evaluation_timestamp_us{0};
    std::uint64_t selection_epoch{0};
    std::uint64_t trajectory_library_version{0};
    std::uint64_t ad_masd_config_version{0};
    std::uint64_t candidate_library_hash{0};
    std::size_t aircraft_count{0};
    std::array<int, kMaximumInteractionGraphAircraft> participant_vehicle_ids{};
    std::array<std::uint64_t, kMaximumInteractionGraphAircraft>
        source_timestamps_us{};
    std::array<double, kMaximumSelectionPairCount> pair_minimum_ad_m{};
    std::array<std::uint8_t, kMaximumSelectionPairCount>
        pair_minimum_first_candidate_id{};
    std::array<std::uint8_t, kMaximumSelectionPairCount>
        pair_minimum_second_candidate_id{};
    std::array<std::uint8_t, kMaximumSelectionPairCount> pair_edge_required{};
    std::uint32_t adjacency_bitmask{0};
    std::array<std::uint8_t, kMaximumInteractionGraphAircraft>
        component_ids{};
    std::array<std::uint8_t, kMaximumInteractionGraphAircraft>
        component_sizes{};
    std::uint8_t component_count{0};
    std::uint8_t edge_count{0};
    std::uint32_t trajectory_generation_count{0};
    std::uint32_t pairwise_ad_evaluation_count{0};
    std::uint32_t naive_evaluation_count{0};
    std::uint32_t component_evaluation_count{0};
    std::uint64_t certification_hash{0};
    std::uint64_t graph_hash{0};
    std::uint64_t component_hash{0};
    std::uint64_t certification_compute_time_ns{0};
    std::uint64_t graph_compute_time_ns{0};

    bool valid() const noexcept
    {
        return status == InteractionGraphStatus::Valid;
    }

    bool adjacent(std::size_t first, std::size_t second) const noexcept;
};

class InteractionGraphBuilder
{
public:
    explicit InteractionGraphBuilder(const InteractionGraphParams & params = {});

    InteractionGraphResult build(
        const PairwiseAdCertificationSet & certifications) const noexcept;

    const InteractionGraphParams & params() const noexcept
    {
        return m_params;
    }

private:
    InteractionGraphParams m_params;
};

enum class InteractionGraphEvaluationStatus : std::uint8_t
{
    Disabled = 0,
    GraphInvalid,
    CandidateSetsIncomplete,
    RequiresSevenCandidates,
    ComponentEvaluationFailed,
    GlobalCrosscheckFailed,
    Evaluated,
};

struct InteractionGraphDiagnostics
{
    int vehicle_id{0};
    bool enabled{false};
    bool component_proposal_used{false};
    InteractionGraphResult graph{};
    InteractionGraphEvaluationStatus status{
        InteractionGraphEvaluationStatus::Disabled};
    bool component_search_evaluated{false};
    std::uint32_t candidate_ready_mask{0};
    std::array<std::uint8_t, kMaximumSelectionAircraft>
        candidate_counts{};
    std::array<std::uint64_t, kMaximumSelectionAircraft>
        candidate_source_timestamps_us{};
    std::uint64_t dropped_ownship_belief_count{0};
    std::uint64_t dropped_remote_intent_count{0};
    std::uint64_t dropped_remote_decision_count{0};
    std::array<std::uint8_t, kMaximumSelectionAircraft>
        assembled_candidate_ids{};
    std::uint32_t assembled_candidate_valid_mask{0};
    std::uint64_t assembled_candidate_hash{0};
    std::uint64_t component_solution_hash{0};
    bool global_crosscheck_evaluated{false};
    bool global_crosscheck_pass{false};
    double global_crosscheck_minimum_ad_m{
        std::numeric_limits<double>::quiet_NaN()};
    std::uint64_t component_search_time_ns{0};
    std::uint64_t global_crosscheck_time_ns{0};
    std::uint64_t total_evaluation_time_ns{0};
    std::size_t component_valid_evaluation_count{0};
    std::size_t component_safe_evaluation_count{0};
    JointCombinationEvaluation global_crosscheck_evaluation{};
};

}  // namespace collision_avoidance::selection
