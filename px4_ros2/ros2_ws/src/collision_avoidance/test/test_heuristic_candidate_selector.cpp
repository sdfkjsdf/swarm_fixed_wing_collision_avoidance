#include <gtest/gtest.h>

#include <array>
#include <cstdint>

#include <collision_avoidance/selection/HeuristicCandidateSelector.hpp>

namespace ce = collision_avoidance::estimation;
namespace cs = collision_avoidance::selection;

namespace
{

std::array<cs::CandidateSafetyScore, ce::kManeuverCandidateCount> scores()
{
    constexpr std::array<double, ce::kManeuverCandidateCount> accelerations{
        -9.8, -5.7, -2.6, 0.0, 2.6, 5.7, 9.8};
    std::array<cs::CandidateSafetyScore, ce::kManeuverCandidateCount> result{};
    for (std::size_t index = 0; index < result.size(); ++index) {
        result[index].candidate_id = static_cast<std::uint8_t>(index);
        result[index].lateral_acceleration_mps2 = accelerations[index];
    }
    return result;
}

}  // namespace

TEST(HeuristicCandidateSelector, UsesDeterministicRightTurnStartupFallback)
{
    const auto input = scores();
    cs::HeuristicCandidateSelector selector;
    const auto selected = selector.select(
        static_cast<std::uint8_t>(ce::ManeuverCandidateId::RollZero), input);

    EXPECT_EQ(
        selected[0],
        static_cast<std::uint8_t>(ce::ManeuverCandidateId::RollZero));
    EXPECT_EQ(
        selected[1],
        static_cast<std::uint8_t>(ce::ManeuverCandidateId::RollPlus15));
    EXPECT_EQ(
        selected[2],
        static_cast<std::uint8_t>(ce::ManeuverCandidateId::RollPlus30));
}

TEST(HeuristicCandidateSelector, RanksValidWorstCaseAdButRetainsCurrentBest)
{
    auto input = scores();
    for (auto & score : input) {
        score.valid = true;
        score.worst_ad_m = -20.0;
    }
    input[static_cast<std::size_t>(ce::ManeuverCandidateId::RollMinus45)]
        .worst_ad_m = 5.0;
    input[static_cast<std::size_t>(ce::ManeuverCandidateId::RollPlus45)]
        .worst_ad_m = 4.0;

    cs::HeuristicCandidateSelector selector;
    const auto selected = selector.select(
        static_cast<std::uint8_t>(ce::ManeuverCandidateId::RollPlus15), input);

    EXPECT_EQ(
        selected[0],
        static_cast<std::uint8_t>(ce::ManeuverCandidateId::RollPlus15));
    EXPECT_EQ(
        selected[1],
        static_cast<std::uint8_t>(ce::ManeuverCandidateId::RollMinus45));
    EXPECT_EQ(
        selected[2],
        static_cast<std::uint8_t>(ce::ManeuverCandidateId::RollPlus45));
}
