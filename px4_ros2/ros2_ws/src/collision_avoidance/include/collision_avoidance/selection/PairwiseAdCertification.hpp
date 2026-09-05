#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

#include <collision_avoidance/selection/ManeuverCombinationEvaluator.hpp>

namespace collision_avoidance::selection
{

inline constexpr std::size_t kPairwiseAdMatrixSize =
    kExhaustiveCandidatesPerAircraft * kExhaustiveCandidatesPerAircraft;

struct PairwiseAdCertification
{
    std::size_t first_aircraft{0};
    std::size_t second_aircraft{0};
    std::array<CombinationEvaluation, kPairwiseAdMatrixSize> evaluations{};
    std::size_t evaluated_count{0};
    double minimum_ad_m{std::numeric_limits<double>::quiet_NaN()};
    std::uint8_t minimum_first_candidate_slot{0};
    std::uint8_t minimum_second_candidate_slot{0};
    std::uint8_t minimum_first_candidate_id{0};
    std::uint8_t minimum_second_candidate_id{0};
    bool valid{false};

    const CombinationEvaluation * find(
        std::size_t first_candidate_slot,
        std::size_t second_candidate_slot) const noexcept;
};

struct PairwiseAdCertificationSet
{
    std::uint64_t evaluation_timestamp_us{0};
    std::uint64_t selection_epoch{0};
    std::uint64_t trajectory_library_version{0};
    std::uint64_t ad_masd_config_version{0};
    std::uint64_t candidate_library_hash{0};
    std::size_t aircraft_count{0};
    std::array<int, kMaximumSelectionAircraft> participant_vehicle_ids{};
    std::array<std::uint64_t, kMaximumSelectionAircraft>
        source_timestamps_us{};
    std::array<PairwiseAdCertification, kMaximumSelectionPairCount>
        pair_certifications{};
    std::size_t pair_count{0};
    std::size_t evaluated_pair_candidate_count{0};
    std::uint64_t compute_time_ns{0};
    bool valid{false};

    const PairwiseAdCertification * findPair(
        std::size_t first_aircraft,
        std::size_t second_aircraft) const noexcept;
};

class PairwiseAdCertificationEvaluator
{
public:
    explicit PairwiseAdCertificationEvaluator(
        const ManeuverCombinationEvaluatorParams & params = {});

    bool evaluate(
        std::uint64_t evaluation_timestamp_us,
        std::uint64_t selection_epoch,
        std::uint64_t trajectory_library_version,
        std::uint64_t ad_masd_config_version,
        const MultiAircraftExhaustiveCandidateIntentSets & candidate_sets,
        std::size_t aircraft_count,
        PairwiseAdCertificationSet & result) const;

private:
    ManeuverCombinationEvaluator m_pair_evaluator;
};

struct CertifiedComponentEvaluation
{
    std::array<std::size_t, kMaximumSelectionAircraft> member_aircraft{};
    std::size_t member_count{0};
    std::size_t combination_count{0};
    std::size_t valid_combination_count{0};
    std::size_t safe_combination_count{0};
    double maximum_minimum_ad_m{
        std::numeric_limits<double>::quiet_NaN()};
    bool has_best{false};
    JointCombinationEvaluation best_combination{};
};

class CertifiedComponentManeuverEvaluator
{
public:
    bool evaluate(
        const PairwiseAdCertificationSet & certifications,
        const MultiAircraftExhaustiveCandidateIntentSets & candidate_sets,
        const std::array<std::size_t, kMaximumSelectionAircraft>
            & member_aircraft,
        std::size_t member_count,
        CertifiedComponentEvaluation & result) const;

    bool evaluateTuple(
        const PairwiseAdCertificationSet & certifications,
        const MultiAircraftExhaustiveCandidateIntentSets & candidate_sets,
        const std::array<std::uint8_t, kMaximumSelectionAircraft>
            & candidate_ids,
        JointCombinationEvaluation & result) const;
};

std::uint64_t trajectoryCandidateLibraryHash(
    std::uint64_t selection_epoch,
    std::uint64_t trajectory_library_version,
    std::uint64_t ad_masd_config_version,
    const MultiAircraftExhaustiveCandidateIntentSets & candidate_sets,
    std::size_t aircraft_count) noexcept;

}  // namespace collision_avoidance::selection
