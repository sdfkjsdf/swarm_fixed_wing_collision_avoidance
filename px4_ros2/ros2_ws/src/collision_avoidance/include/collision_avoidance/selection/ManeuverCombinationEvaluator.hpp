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

using CandidateIntentSet = std::array<
    estimation::ReceivedTrajectoryIntent, kCandidatesPerAircraft>;

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

    static std::string formatTable(
        const StaticCombinationEvaluation & evaluation);

    static const char * validityName(CombinationValidity validity) noexcept;

private:
    ManeuverCombinationEvaluatorParams m_params;
};

}  // namespace collision_avoidance::selection
