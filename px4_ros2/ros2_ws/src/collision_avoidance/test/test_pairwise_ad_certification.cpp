#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>

#include <collision_avoidance/selection/PairwiseAdCertification.hpp>

namespace ce = collision_avoidance::estimation;
namespace cs = collision_avoidance::selection;

namespace
{

ce::ReceivedTrajectoryIntent linearIntent(
    std::uint64_t source_timestamp_us,
    std::uint64_t selection_epoch,
    std::uint8_t candidate_id,
    double east_m)
{
    ce::ReceivedTrajectoryIntent intent{};
    intent.source_timestamp_us = source_timestamp_us;
    intent.selection_epoch = selection_epoch;
    intent.candidate_id = candidate_id;
    intent.candidate_set_size = cs::kExhaustiveCandidatesPerAircraft;
    intent.candidate_set_kind = ce::CandidateSetKind::LegacyRoll;
    intent.candidate_input_revision = 100 + candidate_id;
    intent.candidate_input.V_cmd = 20.0;
    intent.candidate_input.h_cmd = 100.0;
    intent.candidate_input.h_dot_cmd = 0.0;
    intent.candidate_input.a_lat_cmd = 0.0;
    for (std::size_t sample = 0; sample < ce::kTrajectoryPointCount;
         ++sample) {
        const double time_s = static_cast<double>(sample)
            * ce::kTrajectoryIntentStepSeconds;
        const ce::PredictState state{
            20.0 * time_s, east_m, 100.0, 20.0, 0.0, 0.0, 0.0};
        intent.reconstructed_mean[sample] = state;
        intent.cone[sample].time_offset_s = time_s;
        intent.cone[sample].mean = state;
        intent.cone[sample].position_covariance_ned = {
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0};
    }
    return intent;
}

cs::MultiAircraftExhaustiveCandidateIntentSets twoAircraftLibrary(
    std::uint64_t selection_epoch)
{
    cs::MultiAircraftExhaustiveCandidateIntentSets sets{};
    for (std::size_t slot = 0;
         slot < cs::kExhaustiveCandidatesPerAircraft; ++slot) {
        sets[0][slot] = linearIntent(
            1'000'000, selection_epoch, static_cast<std::uint8_t>(slot),
            10.0 * static_cast<double>(slot));
        sets[1][slot] = linearIntent(
            1'000'000, selection_epoch, static_cast<std::uint8_t>(slot),
            50.0 + 10.0 * static_cast<double>(slot));
    }
    return sets;
}

TEST(PairwiseAdCertification, EvaluatesExactlyFortyNinePairsAndFindsMinimum)
{
    constexpr std::uint64_t epoch = 4;
    const auto sets = twoAircraftLibrary(epoch);
    cs::PairwiseAdCertificationEvaluator evaluator;
    cs::PairwiseAdCertificationSet result;
    ASSERT_TRUE(evaluator.evaluate(
        1'000'000, epoch, 1, 1, sets, 2, result));
    ASSERT_TRUE(result.valid);
    ASSERT_EQ(result.pair_count, 1U);
    EXPECT_EQ(result.evaluated_pair_candidate_count, 49U);
    const auto & pair = result.pair_certifications[0];
    EXPECT_TRUE(pair.valid);
    EXPECT_EQ(pair.evaluated_count, 49U);
    EXPECT_EQ(pair.minimum_first_candidate_id, 5U);
    EXPECT_EQ(pair.minimum_second_candidate_id, 0U);
    EXPECT_TRUE(std::isfinite(pair.minimum_ad_m));
    EXPECT_NE(result.candidate_library_hash, 0U);
}

TEST(PairwiseAdCertification, RejectsMixedEpochLibrary)
{
    constexpr std::uint64_t epoch = 4;
    auto sets = twoAircraftLibrary(epoch);
    sets[1][3].selection_epoch = epoch + 1;
    cs::PairwiseAdCertificationEvaluator evaluator;
    cs::PairwiseAdCertificationSet result;
    EXPECT_FALSE(evaluator.evaluate(
        1'000'000, epoch, 1, 1, sets, 2, result));
    EXPECT_FALSE(result.valid);
}

TEST(PairwiseAdCertification, RejectsNonCanonicalSevenCandidateOrdering)
{
    constexpr std::uint64_t epoch = 4;
    auto sets = twoAircraftLibrary(epoch);
    std::swap(sets[0][0], sets[0][1]);
    cs::PairwiseAdCertificationEvaluator evaluator;
    cs::PairwiseAdCertificationSet result;
    EXPECT_FALSE(evaluator.evaluate(
        1'000'000, epoch, 1, 1, sets, 2, result));
}

TEST(PairwiseAdCertification, RejoinMetadataIsPartOfFrozenLibraryIdentity)
{
    constexpr std::uint64_t epoch = 4;
    auto first_sets = twoAircraftLibrary(epoch);
    auto second_sets = first_sets;
    for (std::size_t aircraft = 0; aircraft < 2; ++aircraft) {
        for (std::size_t slot = 0;
             slot < cs::kExhaustiveCandidatesPerAircraft; ++slot) {
            first_sets[aircraft][slot]
                .nominal_lateral_acceleration_mps2 = 1.0;
            first_sets[aircraft][slot].safe_rejoin_requested = true;
            second_sets[aircraft][slot]
                .nominal_lateral_acceleration_mps2 = 2.0;
            second_sets[aircraft][slot].safe_rejoin_requested = true;
        }
    }

    cs::PairwiseAdCertificationEvaluator evaluator;
    cs::PairwiseAdCertificationSet first;
    cs::PairwiseAdCertificationSet second;
    ASSERT_TRUE(evaluator.evaluate(
        1'000'000, epoch, 1, 1, first_sets, 2, first));
    ASSERT_TRUE(evaluator.evaluate(
        1'000'000, epoch, 1, 1, second_sets, 2, second));
    EXPECT_NE(first.candidate_library_hash, second.candidate_library_hash);

    second_sets[0][3].safe_rejoin_requested = false;
    EXPECT_FALSE(evaluator.evaluate(
        1'000'000, epoch, 1, 1, second_sets, 2, second));
}

TEST(PairwiseAdCertification, CachedComponentSearchMatchesLegacyExhaustive)
{
    constexpr std::uint64_t epoch = 4;
    const auto sets = twoAircraftLibrary(epoch);
    cs::PairwiseAdCertificationEvaluator certifier;
    cs::PairwiseAdCertificationSet certifications;
    ASSERT_TRUE(certifier.evaluate(
        1'000'000, epoch, 1, 1, sets, 2, certifications));

    cs::CertifiedComponentManeuverEvaluator cached;
    cs::CertifiedComponentEvaluation cached_result;
    std::array<std::size_t, cs::kMaximumSelectionAircraft> members{};
    members[0] = 0;
    members[1] = 1;
    ASSERT_TRUE(cached.evaluate(
        certifications, sets, members, 2, cached_result));

    cs::ExhaustiveManeuverCombinationEvaluator legacy;
    cs::ExhaustiveManeuverEvaluation legacy_result;
    ASSERT_TRUE(legacy.evaluate(1'000'000, sets, 2, legacy_result));
    EXPECT_EQ(cached_result.combination_count, 49U);
    EXPECT_EQ(
        cached_result.best_combination.candidate_slots,
        legacy_result.best_combination.candidate_slots);
    EXPECT_NEAR(
        cached_result.best_combination.minimum_ad_m,
        legacy_result.best_combination.minimum_ad_m,
        1.0e-12);
}

TEST(PairwiseAdCertification, ComponentSearchHonorsSafeRejoinObjective)
{
    constexpr std::uint64_t epoch = 5;
    auto sets = twoAircraftLibrary(epoch);
    for (std::size_t aircraft = 0; aircraft < 2; ++aircraft) {
        for (std::size_t slot = 0;
             slot < cs::kExhaustiveCandidatesPerAircraft; ++slot) {
            sets[aircraft][slot].candidate_input.a_lat_cmd =
                static_cast<double>(slot);
        }
    }

    cs::PairwiseAdCertificationEvaluator certifier;
    cs::PairwiseAdCertificationSet certifications;
    ASSERT_TRUE(certifier.evaluate(
        1'000'000, epoch, 1, 1, sets, 2, certifications));

    std::array<std::size_t, cs::kMaximumSelectionAircraft> members{};
    members[0] = 0;
    members[1] = 1;
    cs::ManeuverRejoinObjective objective;
    objective.enabled = true;
    objective.nominal_lateral_acceleration_mps2[0] = 1.0;
    objective.nominal_lateral_acceleration_mps2[1] = 5.0;

    cs::CertifiedComponentManeuverEvaluator evaluator;
    cs::CertifiedComponentEvaluation result;
    ASSERT_TRUE(evaluator.evaluate(
        certifications, sets, members, 2, result, &objective));
    ASSERT_TRUE(result.best_combination.all_pairs_feasible);
    EXPECT_EQ(result.best_combination.candidate_slots[0], 1U);
    EXPECT_EQ(result.best_combination.candidate_slots[1], 5U);
    EXPECT_DOUBLE_EQ(result.best_combination.nominal_rejoin_cost, 0.0);
}

}  // namespace
