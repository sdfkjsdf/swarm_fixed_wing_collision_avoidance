#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>

#include <collision_avoidance/selection/ManeuverCombinationEvaluator.hpp>

namespace ce = collision_avoidance::estimation;
namespace cs = collision_avoidance::selection;

namespace
{

ce::ReceivedTrajectoryIntent linearIntent(
    std::uint64_t source_timestamp_us,
    std::uint8_t candidate_id,
    double north_0,
    double east_0,
    double altitude,
    double velocity_north,
    double velocity_east,
    double position_variance)
{
    ce::ReceivedTrajectoryIntent intent{};
    intent.source_timestamp_us = source_timestamp_us;
    intent.candidate_id = candidate_id;
    for (std::size_t index = 0; index < ce::kTrajectoryPointCount; ++index) {
        const double time_s =
            static_cast<double>(index) * ce::kTrajectoryIntentStepSeconds;
        const ce::PredictState mean{
            north_0 + velocity_north * time_s,
            east_0 + velocity_east * time_s,
            altitude,
            std::hypot(velocity_north, velocity_east),
            std::atan2(velocity_east, velocity_north),
            0.0,
            0.0};
        intent.reconstructed_mean[index] = mean;
        intent.cone[index].time_offset_s = time_s;
        intent.cone[index].mean = mean;
        intent.cone[index].position_covariance_ned = {
            position_variance, 0.0, 0.0,
            0.0, position_variance, 0.0,
            0.0, 0.0, position_variance};
    }
    return intent;
}

cs::CandidateIntentSet parallelCandidates(
    std::uint64_t timestamp_us,
    const std::array<double, 3> & east_offsets,
    double variance = 0.0)
{
    cs::CandidateIntentSet candidates{};
    for (std::size_t index = 0; index < candidates.size(); ++index) {
        candidates[index] = linearIntent(
            timestamp_us,
            static_cast<std::uint8_t>(index),
            0.0,
            east_offsets[index],
            100.0,
            10.0,
            0.0,
            variance);
    }
    return candidates;
}

cs::ExhaustiveCandidateIntentSet exhaustiveParallelCandidates(
    std::uint64_t timestamp_us,
    double base_east,
    double candidate_spacing_m)
{
    cs::ExhaustiveCandidateIntentSet candidates{};
    for (std::size_t index = 0; index < candidates.size(); ++index) {
        candidates[index] = linearIntent(
            timestamp_us,
            static_cast<std::uint8_t>(index),
            0.0,
            base_east + candidate_spacing_m * static_cast<double>(index),
            100.0,
            10.0,
            0.0,
            0.0);
    }
    return candidates;
}

}  // namespace

TEST(ManeuverCombinationEvaluator, EvaluatesAllNineAndKeepsWindowPmrValues)
{
    constexpr std::uint64_t timestamp_us = 1'000'000ULL;
    const auto ownship = parallelCandidates(timestamp_us, {0.0, 20.0, 40.0});
    const auto threat = parallelCandidates(timestamp_us, {5.0, 25.0, 60.0});

    cs::ManeuverCombinationEvaluatorParams params;
    params.desired_separation_distance_m = 10.0;
    params.confidence_chi_squared = 7.814727903251179;
    cs::ManeuverCombinationEvaluator evaluator(params);
    cs::StaticCombinationEvaluation evaluation;

    ASSERT_TRUE(evaluator.evaluate(
        timestamp_us, ownship, threat, evaluation));
    ASSERT_TRUE(evaluation.has_best);
    EXPECT_EQ(evaluation.best_combination_index, 2U);

    const auto & first = evaluation.combinations[0];
    EXPECT_EQ(first.validity, cs::CombinationValidity::Valid);
    EXPECT_EQ(first.evaluated_sample_count, ce::kTrajectoryPointCount);
    ASSERT_TRUE(first.pmr_windows[0].available);
    ASSERT_TRUE(first.pmr_windows[1].available);
    ASSERT_TRUE(first.pmr_windows[2].available);
    EXPECT_NEAR(first.pmr_windows[0].pmr_m, 5.0, 1.0e-12);
    EXPECT_NEAR(first.pmr_windows[1].pmr_m, 5.0, 1.0e-12);
    EXPECT_NEAR(first.pmr_windows[2].pmr_m, 5.0, 1.0e-12);
    EXPECT_NEAR(first.ad_m, -5.0, 1.0e-12);
    EXPECT_FALSE(first.feasible);
    EXPECT_FALSE(first.reciprocal_cost_defined);

    const auto & best = evaluation.combinations[2];
    EXPECT_TRUE(best.selected_best);
    EXPECT_NEAR(best.pmr_m, 60.0, 1.0e-12);
    EXPECT_NEAR(best.masd_m, 10.0, 1.0e-12);
    EXPECT_NEAR(best.ad_m, 50.0, 1.0e-12);
    EXPECT_TRUE(best.feasible);
    ASSERT_TRUE(best.reciprocal_cost_defined);
    EXPECT_NEAR(best.reciprocal_cost, 0.02, 1.0e-12);
}

TEST(ManeuverCombinationEvaluator, IncludesAircraftDsdAndRelativeUncertainty)
{
    constexpr std::uint64_t timestamp_us = 2'000'000ULL;
    const auto ownship = parallelCandidates(timestamp_us, {0.0, 0.0, 0.0}, 1.0);
    const auto threat = parallelCandidates(timestamp_us, {20.0, 20.0, 20.0}, 1.0);

    cs::ManeuverCombinationEvaluatorParams params;
    params.desired_separation_distance_m = 10.0;
    params.ownship_half_wingspan_m = 1.072;
    params.threat_half_wingspan_m = 1.072;
    cs::ManeuverCombinationEvaluator evaluator(params);
    cs::StaticCombinationEvaluation evaluation;

    ASSERT_TRUE(evaluator.evaluate(
        timestamp_us, ownship, threat, evaluation));
    const auto & row = evaluation.combinations[0];
    const double expected_uncertainty = std::sqrt(
        params.confidence_chi_squared * 2.0);
    EXPECT_NEAR(row.aircraft_size_margin_m, 2.144, 1.0e-12);
    EXPECT_NEAR(row.desired_separation_distance_m, 10.0, 1.0e-12);
    EXPECT_NEAR(row.uncertainty_margin_95_m, expected_uncertainty, 1.0e-12);
    EXPECT_NEAR(
        row.masd_m,
        2.144 + 10.0 + expected_uncertainty,
        1.0e-12);
    EXPECT_NEAR(row.ad_m, 20.0 - row.masd_m, 1.0e-12);
}

TEST(ManeuverCombinationEvaluator, SelectsLargestAdWhenEveryCombinationIsUnsafe)
{
    constexpr std::uint64_t timestamp_us = 3'000'000ULL;
    const auto ownship = parallelCandidates(timestamp_us, {0.0, 1.0, 2.0});
    const auto threat = parallelCandidates(timestamp_us, {5.0, 6.0, 9.0});

    cs::ManeuverCombinationEvaluator evaluator;
    cs::StaticCombinationEvaluation evaluation;
    ASSERT_TRUE(evaluator.evaluate(
        timestamp_us, ownship, threat, evaluation));

    ASSERT_TRUE(evaluation.has_best);
    const auto & best = evaluation.combinations[
        evaluation.best_combination_index];
    EXPECT_FALSE(best.feasible);
    EXPECT_FALSE(best.reciprocal_cost_defined);
    EXPECT_NEAR(best.pmr_m, 9.0, 1.0e-12);
    EXPECT_NEAR(best.ad_m, -1.0, 1.0e-12);
    EXPECT_TRUE(best.selected_best);
}

TEST(ManeuverCombinationEvaluator, AlignsValidDelayedTrajectoryToEvaluationTime)
{
    constexpr std::uint64_t ownship_timestamp_us = 1'000'000ULL;
    constexpr std::uint64_t threat_timestamp_us = 1'050'000ULL;
    constexpr std::uint64_t evaluation_timestamp_us = threat_timestamp_us;

    cs::CandidateIntentSet ownship{};
    cs::CandidateIntentSet threat{};
    for (std::size_t index = 0; index < ownship.size(); ++index) {
        ownship[index] = linearIntent(
            ownship_timestamp_us,
            static_cast<std::uint8_t>(index),
            0.0, 0.0, 100.0, 10.0, 0.0, 0.0);
        threat[index] = linearIntent(
            threat_timestamp_us,
            static_cast<std::uint8_t>(index),
            100.0, 0.0, 100.0, 0.0, 0.0, 0.0);
    }

    cs::ManeuverCombinationEvaluator evaluator;
    cs::StaticCombinationEvaluation evaluation;
    ASSERT_TRUE(evaluator.evaluate(
        evaluation_timestamp_us, ownship, threat, evaluation));

    const auto & row = evaluation.combinations[0];
    EXPECT_EQ(row.evaluated_sample_count, 45U);
    EXPECT_NEAR(row.pmr_time_offset_s, 4.4, 1.0e-12);
    EXPECT_NEAR(row.pmr_m, 55.5, 1.0e-9);
}

TEST(ManeuverCombinationEvaluator, RejectsFutureAndStaleStaticInputs)
{
    constexpr std::uint64_t evaluation_timestamp_us = 5'000'000ULL;
    const auto current = parallelCandidates(
        evaluation_timestamp_us, {0.0, 1.0, 2.0});
    const auto future = parallelCandidates(
        evaluation_timestamp_us + 1, {5.0, 6.0, 7.0});
    const auto stale = parallelCandidates(
        evaluation_timestamp_us - 3'000'001ULL, {5.0, 6.0, 7.0});

    cs::ManeuverCombinationEvaluator evaluator;
    cs::StaticCombinationEvaluation evaluation;
    EXPECT_FALSE(evaluator.evaluate(
        evaluation_timestamp_us, current, future, evaluation));
    EXPECT_EQ(
        evaluation.combinations[0].validity,
        cs::CombinationValidity::FutureTimestamp);

    EXPECT_FALSE(evaluator.evaluate(
        evaluation_timestamp_us, current, stale, evaluation));
    EXPECT_EQ(
        evaluation.combinations[0].validity,
        cs::CombinationValidity::StaleTimestamp);
}

TEST(ManeuverCombinationEvaluator, FormatsCompleteNineRowTable)
{
    constexpr std::uint64_t timestamp_us = 6'000'000ULL;
    const auto ownship = parallelCandidates(timestamp_us, {0.0, 20.0, 40.0});
    const auto threat = parallelCandidates(timestamp_us, {5.0, 25.0, 60.0});
    cs::ManeuverCombinationEvaluator evaluator;
    cs::StaticCombinationEvaluation evaluation;
    ASSERT_TRUE(evaluator.evaluate(
        timestamp_us, ownship, threat, evaluation));

    const std::string table = cs::ManeuverCombinationEvaluator::formatTable(
        evaluation);
    EXPECT_NE(table.find("PMR1"), std::string::npos);
    EXPECT_NE(table.find("U95"), std::string::npos);
    EXPECT_NE(table.find("MASD"), std::string::npos);
    EXPECT_NE(table.find("YES"), std::string::npos);
    std::size_t row_count = 0;
    for (char value : table) {
        row_count += value == '\n' ? 1U : 0U;
    }
    EXPECT_EQ(row_count, 11U);
}

TEST(JointManeuverCombinationEvaluator, EvaluatesAllFiveAircraftCombinations)
{
    constexpr std::uint64_t timestamp_us = 7'000'000ULL;
    cs::MultiAircraftCandidateIntentSets candidate_sets{};
    for (std::size_t aircraft = 0;
         aircraft < cs::kMaximumSelectionAircraft; ++aircraft) {
        const double base_east = static_cast<double>(aircraft) * 100.0;
        candidate_sets[aircraft] = parallelCandidates(
            timestamp_us,
            {base_east, base_east + 5.0, base_east + 10.0});
    }

    cs::JointManeuverCombinationEvaluator evaluator;
    cs::JointManeuverEvaluation evaluation;
    ASSERT_TRUE(evaluator.evaluate(
        timestamp_us,
        candidate_sets,
        cs::kMaximumSelectionAircraft,
        evaluation));
    EXPECT_EQ(evaluation.aircraft_count, 5U);
    EXPECT_EQ(evaluation.combination_count, 243U);
    ASSERT_TRUE(evaluation.has_best);
    const auto & best = evaluation.combinations[
        evaluation.best_combination_index];
    EXPECT_TRUE(best.valid);
    EXPECT_TRUE(best.all_pairs_feasible);
    EXPECT_TRUE(best.selected_best);
    EXPECT_EQ(best.evaluated_pair_count, 10U);
    EXPECT_GT(best.minimum_ad_m, 0.0);
    for (std::size_t aircraft = 0; aircraft < 5; ++aircraft) {
        EXPECT_LT(best.candidate_slots[aircraft], 3U);
    }
}

TEST(JointManeuverCombinationEvaluator, UsesMaximinFallbackWhenAllAreUnsafe)
{
    constexpr std::uint64_t timestamp_us = 8'000'000ULL;
    cs::MultiAircraftCandidateIntentSets candidate_sets{};
    for (std::size_t aircraft = 0;
         aircraft < cs::kMaximumSelectionAircraft; ++aircraft) {
        const double base_east = static_cast<double>(aircraft);
        candidate_sets[aircraft] = parallelCandidates(
            timestamp_us,
            {base_east, base_east + 1.0, base_east + 2.0});
    }

    cs::JointManeuverCombinationEvaluator evaluator;
    cs::JointManeuverEvaluation evaluation;
    ASSERT_TRUE(evaluator.evaluate(
        timestamp_us, candidate_sets, 5, evaluation));
    const auto & best = evaluation.combinations[
        evaluation.best_combination_index];
    EXPECT_TRUE(best.valid);
    EXPECT_FALSE(best.all_pairs_feasible);
    EXPECT_LE(best.minimum_ad_m, 0.0);
    for (std::size_t index = 0;
         index < evaluation.combination_count; ++index) {
        const auto & combination = evaluation.combinations[index];
        if (combination.valid) {
            EXPECT_GE(best.minimum_ad_m, combination.minimum_ad_m - 1.0e-12);
        }
    }
}

TEST(ExhaustiveManeuverCombinationEvaluator, EvaluatesAllFortyNineTwoAircraftCombinations)
{
    constexpr std::uint64_t timestamp_us = 9'000'000ULL;
    cs::MultiAircraftExhaustiveCandidateIntentSets candidate_sets{};
    candidate_sets[0] = exhaustiveParallelCandidates(timestamp_us, 0.0, 1.0);
    candidate_sets[1] = exhaustiveParallelCandidates(timestamp_us, 100.0, 1.0);

    cs::ExhaustiveManeuverCombinationEvaluator evaluator;
    cs::ExhaustiveManeuverEvaluation evaluation;
    ASSERT_TRUE(evaluator.evaluate(
        timestamp_us, candidate_sets, 2U, evaluation));
    EXPECT_EQ(evaluation.combination_count, 49U);
    EXPECT_EQ(evaluation.evaluated_unique_pair_count, 49U);
    EXPECT_EQ(evaluation.best_combination_index, 6U);
    ASSERT_TRUE(evaluation.has_best);
    EXPECT_EQ(evaluation.best_combination.candidate_slots[0], 0U);
    EXPECT_EQ(evaluation.best_combination.candidate_slots[1], 6U);
    EXPECT_NEAR(evaluation.best_combination.minimum_pmr_m, 106.0, 1.0e-12);
    EXPECT_NEAR(evaluation.best_combination.minimum_ad_m, 96.0, 1.0e-12);
}

TEST(ExhaustiveManeuverCombinationEvaluator, EvaluatesAllFiveAircraftCombinations)
{
    constexpr std::uint64_t timestamp_us = 10'000'000ULL;
    cs::MultiAircraftExhaustiveCandidateIntentSets candidate_sets{};
    for (std::size_t aircraft = 0;
         aircraft < cs::kMaximumSelectionAircraft; ++aircraft) {
        candidate_sets[aircraft] = exhaustiveParallelCandidates(
            timestamp_us, static_cast<double>(aircraft) * 100.0, 1.0);
    }

    cs::ExhaustiveManeuverCombinationEvaluator evaluator;
    cs::ExhaustiveManeuverEvaluation evaluation;
    ASSERT_TRUE(evaluator.evaluate(
        timestamp_us,
        candidate_sets,
        cs::kMaximumSelectionAircraft,
        evaluation));
    EXPECT_EQ(evaluation.combination_count, 16'807U);
    EXPECT_EQ(evaluation.evaluated_unique_pair_count, 490U);
    ASSERT_TRUE(evaluation.has_best);
    EXPECT_TRUE(evaluation.best_combination.valid);
    EXPECT_TRUE(evaluation.best_combination.all_pairs_feasible);
    EXPECT_EQ(evaluation.best_combination.evaluated_pair_count, 10U);
}
