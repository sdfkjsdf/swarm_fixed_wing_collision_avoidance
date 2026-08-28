#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include <collision_avoidance/estimation/trajectory_prediction/TrajectoryIntent.hpp>
#include <collision_avoidance/selection/ManeuverCombinationEvaluator.hpp>

namespace collision_avoidance::selection
{

struct CandidateSafetyScore
{
    std::uint8_t candidate_id{0};
    double worst_ad_m{0.0};
    double lateral_acceleration_mps2{0.0};
    bool eligible{true};
    bool valid{false};
};

class HeuristicCandidateSelector
{
public:
    std::array<std::uint8_t, kCandidatesPerAircraft> select(
        std::uint8_t current_best_id,
        const std::array<
            CandidateSafetyScore,
            estimation::kManeuverCandidateCount> & scores) const noexcept;
};

}  // namespace collision_avoidance::selection
