#include <collision_avoidance/selection/HeuristicCandidateSelector.hpp>

#include <algorithm>
#include <cmath>

namespace collision_avoidance::selection
{
namespace
{

constexpr double kScoreTieToleranceM = 1.0e-6;
constexpr double kAccelerationTolerance = 1.0e-9;

bool betterScore(
    const CandidateSafetyScore & lhs,
    const CandidateSafetyScore & rhs) noexcept
{
    if (lhs.eligible != rhs.eligible) {
        return lhs.eligible;
    }
    if (lhs.valid != rhs.valid) {
        return lhs.valid;
    }
    if (lhs.valid
        && std::abs(lhs.worst_ad_m - rhs.worst_ad_m) > kScoreTieToleranceM) {
        return lhs.worst_ad_m > rhs.worst_ad_m;
    }

    const bool lhs_right = lhs.lateral_acceleration_mps2 > kAccelerationTolerance;
    const bool rhs_right = rhs.lateral_acceleration_mps2 > kAccelerationTolerance;
    if (lhs_right != rhs_right) {
        return lhs_right;
    }

    const double lhs_magnitude = std::abs(lhs.lateral_acceleration_mps2);
    const double rhs_magnitude = std::abs(rhs.lateral_acceleration_mps2);
    if (std::abs(lhs_magnitude - rhs_magnitude) > kAccelerationTolerance) {
        return lhs_magnitude < rhs_magnitude;
    }
    return lhs.candidate_id < rhs.candidate_id;
}

}  // namespace

std::array<std::uint8_t, kCandidatesPerAircraft>
HeuristicCandidateSelector::select(
    std::uint8_t current_best_id,
    const std::array<
        CandidateSafetyScore,
        estimation::kManeuverCandidateCount> & scores) const noexcept
{
    std::array<CandidateSafetyScore, estimation::kManeuverCandidateCount>
        ranked = scores;
    std::stable_sort(ranked.begin(), ranked.end(), betterScore);

    std::array<std::uint8_t, kCandidatesPerAircraft> selected{};
    selected[0] = current_best_id;
    std::size_t selected_count = 1;
    for (const CandidateSafetyScore & score : ranked) {
        if (!score.eligible || score.candidate_id == current_best_id) {
            continue;
        }
        selected[selected_count++] = score.candidate_id;
        if (selected_count == selected.size()) {
            break;
        }
    }
    return selected;
}

}  // namespace collision_avoidance::selection
