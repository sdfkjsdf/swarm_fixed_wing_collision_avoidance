#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

#include <collision_avoidance/estimation/trajectory_prediction/TrajectoryIntent.hpp>

namespace collision_avoidance::selection
{

inline constexpr std::size_t kCandidatesPerAircraft = 3;
inline constexpr std::size_t kTwoAircraftCombinationCount = 9;
inline constexpr std::size_t kPmrWindowCount = 3;
inline constexpr std::size_t kMaximumSelectionAircraft = 5;
inline constexpr std::size_t kMaximumJointCombinationCount = 243;
inline constexpr std::size_t kMaximumSelectionPairCount = 10;
inline constexpr std::size_t kExhaustiveCandidatesPerAircraft =
    estimation::kManeuverCandidateCount;
inline constexpr std::size_t kMaximumExhaustiveCombinationCount = 16'807;
inline constexpr std::size_t kMaximumExhaustivePairEvaluationCount =
    kMaximumSelectionPairCount
    * kExhaustiveCandidatesPerAircraft
    * kExhaustiveCandidatesPerAircraft;

using CandidateIntentSet = std::array<
    estimation::ReceivedTrajectoryIntent, kCandidatesPerAircraft>;
using MultiAircraftCandidateIntentSets = std::array<
    CandidateIntentSet, kMaximumSelectionAircraft>;
using ExhaustiveCandidateIntentSet = std::array<
    estimation::ReceivedTrajectoryIntent,
    kExhaustiveCandidatesPerAircraft>;
using MultiAircraftExhaustiveCandidateIntentSets = std::array<
    ExhaustiveCandidateIntentSet, kMaximumSelectionAircraft>;

enum class CombinationValidity
{
    Valid,
    FutureTimestamp,
    StaleTimestamp,
    NoCommonHorizon,
    InvalidTrajectory,
};

struct ManeuverCombinationEvaluatorParams
{
    double desired_separation_distance_m{10.0};
    double ownship_half_wingspan_m{0.0};
    double threat_half_wingspan_m{0.0};
    double confidence_chi_squared{7.814727903251179};
    double stale_timeout_s{3.0};
};

struct PmrWindowResult
{
    bool available{false};
    double pmr_m{std::numeric_limits<double>::infinity()};
    double time_offset_s{0.0};
    std::size_t sample_index{0};
};

struct CombinationEvaluation
{
    std::size_t combination_index{0};
    std::uint8_t ownship_candidate_id{0};
    std::uint8_t threat_candidate_id{0};
    CombinationValidity validity{CombinationValidity::InvalidTrajectory};
    std::array<PmrWindowResult, kPmrWindowCount> pmr_windows{};
    std::size_t selected_pmr_window{0};
    std::size_t evaluated_sample_count{0};
    double pmr_m{std::numeric_limits<double>::quiet_NaN()};
    double pmr_time_offset_s{std::numeric_limits<double>::quiet_NaN()};
    std::uint64_t pmr_timestamp_us{0};
    double aircraft_size_margin_m{std::numeric_limits<double>::quiet_NaN()};
    double desired_separation_distance_m{
        std::numeric_limits<double>::quiet_NaN()};
    double uncertainty_margin_95_m{std::numeric_limits<double>::quiet_NaN()};
    double masd_m{std::numeric_limits<double>::quiet_NaN()};
    double ad_m{std::numeric_limits<double>::quiet_NaN()};
    bool feasible{false};
    bool reciprocal_cost_defined{false};
    double reciprocal_cost{std::numeric_limits<double>::quiet_NaN()};
    bool selected_best{false};
};

struct StaticCombinationEvaluation
{
    std::uint64_t evaluation_timestamp_us{0};
    std::array<CombinationEvaluation, kTwoAircraftCombinationCount> combinations{};
    bool has_best{false};
    std::size_t best_combination_index{0};
};

struct JointCombinationEvaluation
{
    std::size_t combination_index{0};
    std::size_t aircraft_count{0};
    std::array<std::uint8_t, kMaximumSelectionAircraft> candidate_slots{};
    std::size_t evaluated_pair_count{0};
    bool valid{false};
    bool all_pairs_feasible{false};
    double minimum_pmr_m{std::numeric_limits<double>::quiet_NaN()};
    double minimum_masd_m{std::numeric_limits<double>::quiet_NaN()};
    double minimum_ad_m{std::numeric_limits<double>::quiet_NaN()};
    double reciprocal_cost_sum{std::numeric_limits<double>::quiet_NaN()};
    bool selected_best{false};
};

struct JointManeuverEvaluation
{
    std::uint64_t evaluation_timestamp_us{0};
    std::size_t aircraft_count{0};
    std::size_t combination_count{0};
    std::array<JointCombinationEvaluation, kMaximumJointCombinationCount>
        combinations{};
    bool has_best{false};
    std::size_t best_combination_index{0};
};

/* Test-only full-search summary. The 16,807 rows are reduced online to the
   selected result; only the 490 unique pair/candidate evaluations are
   computed from trajectory samples. */
struct ExhaustiveManeuverEvaluation
{
    std::uint64_t evaluation_timestamp_us{0};
    std::size_t aircraft_count{0};
    std::size_t combination_count{0};
    std::size_t evaluated_unique_pair_count{0};
    bool has_best{false};
    std::size_t best_combination_index{0};
    JointCombinationEvaluation best_combination{};
};

class ManeuverCombinationEvaluator
{
public:
    explicit ManeuverCombinationEvaluator(
        const ManeuverCombinationEvaluatorParams & params = {});

    bool evaluate(
        std::uint64_t evaluation_timestamp_us,
        const CandidateIntentSet & ownship_candidates,
        const CandidateIntentSet & threat_candidates,
        StaticCombinationEvaluation & evaluation) const;

    bool evaluatePair(
        std::uint64_t evaluation_timestamp_us,
        const estimation::ReceivedTrajectoryIntent & ownship_intent,
        const estimation::ReceivedTrajectoryIntent & threat_intent,
        CombinationEvaluation & evaluation) const;

    static std::string formatTable(
        const StaticCombinationEvaluation & evaluation);

    static const char * validityName(CombinationValidity validity) noexcept;

private:
    ManeuverCombinationEvaluatorParams m_params;
};

class JointManeuverCombinationEvaluator
{
public:
    explicit JointManeuverCombinationEvaluator(
        const ManeuverCombinationEvaluatorParams & params = {});

    bool evaluate(
        std::uint64_t evaluation_timestamp_us,
        const MultiAircraftCandidateIntentSets & candidate_sets,
        std::size_t aircraft_count,
        JointManeuverEvaluation & evaluation) const;

private:
    ManeuverCombinationEvaluator m_pair_evaluator;
};

class ExhaustiveManeuverCombinationEvaluator
{
public:
    explicit ExhaustiveManeuverCombinationEvaluator(
        const ManeuverCombinationEvaluatorParams & params = {});

    bool evaluate(
        std::uint64_t evaluation_timestamp_us,
        const MultiAircraftExhaustiveCandidateIntentSets & candidate_sets,
        std::size_t aircraft_count,
        ExhaustiveManeuverEvaluation & evaluation) const;

private:
    ManeuverCombinationEvaluator m_pair_evaluator;
};

}  // namespace collision_avoidance::selection
