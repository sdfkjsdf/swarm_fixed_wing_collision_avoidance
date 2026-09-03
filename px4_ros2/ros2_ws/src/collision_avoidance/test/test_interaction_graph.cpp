#include <gtest/gtest.h>

#include <array>
#include <cstddef>

#include <collision_avoidance/selection/InteractionGraph.hpp>

namespace
{

using collision_avoidance::selection::InteractionGraphBuilder;
using collision_avoidance::selection::InteractionGraphParams;
using collision_avoidance::selection::InteractionGraphStatus;
using collision_avoidance::selection::PairwiseAdCertificationSet;
using collision_avoidance::selection::kPairwiseAdMatrixSize;

PairwiseAdCertificationSet certificationFromPairMinima(
    std::size_t aircraft_count,
    const std::array<double, 10> & pair_minimum_ad_m)
{
    PairwiseAdCertificationSet result;
    result.evaluation_timestamp_us = 1'250'000;
    result.selection_epoch = 4;
    result.trajectory_library_version = 1;
    result.ad_masd_config_version = 1;
    result.candidate_library_hash = 0xabc123ULL;
    result.aircraft_count = aircraft_count;
    result.participant_vehicle_ids.fill(-1);
    for (std::size_t aircraft = 0; aircraft < aircraft_count; ++aircraft) {
        result.participant_vehicle_ids[aircraft] = static_cast<int>(aircraft);
        result.source_timestamps_us[aircraft] = 1'000'000 + aircraft;
    }
    std::size_t pair_index = 0;
    for (std::size_t first = 0; first < aircraft_count; ++first) {
        for (std::size_t second = first + 1;
             second < aircraft_count; ++second) {
            auto & pair = result.pair_certifications[pair_index];
            pair.first_aircraft = first;
            pair.second_aircraft = second;
            pair.evaluated_count = kPairwiseAdMatrixSize;
            pair.minimum_ad_m = pair_minimum_ad_m[pair_index];
            pair.minimum_first_candidate_id = 1;
            pair.minimum_second_candidate_id = 2;
            pair.valid = true;
            ++pair_index;
        }
    }
    result.pair_count = pair_index;
    result.evaluated_pair_candidate_count =
        pair_index * kPairwiseAdMatrixSize;
    result.valid = true;
    return result;
}

InteractionGraphParams enabledParams(double ad_screen_m = 0.0)
{
    InteractionGraphParams params;
    params.enabled = true;
    params.ad_screen_m = ad_screen_m;
    return params;
}

TEST(InteractionGraph, DisabledIsInert)
{
    InteractionGraphBuilder builder;
    const auto result = builder.build(certificationFromPairMinima(
        2, {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
    EXPECT_EQ(result.status, InteractionGraphStatus::Disabled);
}

TEST(InteractionGraph, EdgeExistsOnlyBelowScreeningThreshold)
{
    InteractionGraphBuilder builder(enabledParams(0.0));
    const auto below = builder.build(certificationFromPairMinima(
        2, {-0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
    const auto equal = builder.build(certificationFromPairMinima(
        2, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
    ASSERT_TRUE(below.valid());
    ASSERT_TRUE(equal.valid());
    EXPECT_TRUE(below.adjacent(0, 1));
    EXPECT_FALSE(equal.adjacent(0, 1));
}

TEST(InteractionGraph, TwoComponentsThreeAndTwoRequire392Combinations)
{
    // Pair order: 01,02,03,04,12,13,14,23,24,34.
    InteractionGraphBuilder builder(enabledParams());
    const auto result = builder.build(certificationFromPairMinima(
        5, {-1.0, 1.0, 1.0, 1.0, -1.0,
            1.0, 1.0, 1.0, 1.0, -1.0}));
    ASSERT_TRUE(result.valid());
    EXPECT_EQ(result.component_count, 2);
    EXPECT_EQ(result.component_sizes[0], 3);
    EXPECT_EQ(result.component_sizes[1], 2);
    EXPECT_EQ(result.pairwise_ad_evaluation_count, 490U);
    EXPECT_EQ(result.naive_evaluation_count, 16807U);
    EXPECT_EQ(result.component_evaluation_count, 392U);
}

TEST(InteractionGraph, ChainIsOneConnectedComponent)
{
    InteractionGraphBuilder builder(enabledParams());
    const auto result = builder.build(certificationFromPairMinima(
        3, {-1.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
    ASSERT_TRUE(result.valid());
    EXPECT_EQ(result.component_count, 1);
    EXPECT_EQ(result.component_sizes[0], 3);
    EXPECT_EQ(result.component_evaluation_count, 343U);
}

TEST(InteractionGraph, IsolatedAircraftAddsNoSearchDimension)
{
    InteractionGraphBuilder builder(enabledParams());
    const auto result = builder.build(certificationFromPairMinima(
        3, {-1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
    ASSERT_TRUE(result.valid());
    EXPECT_EQ(result.component_count, 2);
    EXPECT_EQ(result.component_sizes[0], 2);
    EXPECT_EQ(result.component_sizes[1], 1);
    EXPECT_EQ(result.component_evaluation_count, 49U);
}

TEST(InteractionGraph, HashesAreDeterministicAndConfigurationSensitive)
{
    const auto certification = certificationFromPairMinima(
        2, {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    InteractionGraphBuilder first(enabledParams(0.0));
    InteractionGraphBuilder second(enabledParams(0.0));
    InteractionGraphBuilder changed(enabledParams(-2.0));
    const auto a = first.build(certification);
    const auto b = second.build(certification);
    const auto c = changed.build(certification);
    ASSERT_TRUE(a.valid());
    ASSERT_TRUE(b.valid());
    ASSERT_TRUE(c.valid());
    EXPECT_EQ(a.certification_hash, b.certification_hash);
    EXPECT_EQ(a.graph_hash, b.graph_hash);
    EXPECT_EQ(a.component_hash, b.component_hash);
    EXPECT_NE(a.graph_hash, c.graph_hash);
}

TEST(InteractionGraph, RejectsIncompleteCertification)
{
    auto certification = certificationFromPairMinima(
        2, {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    certification.evaluated_pair_candidate_count = 48;
    InteractionGraphBuilder builder(enabledParams());
    EXPECT_EQ(
        builder.build(certification).status,
        InteractionGraphStatus::InvalidCertification);
}

}  // namespace
