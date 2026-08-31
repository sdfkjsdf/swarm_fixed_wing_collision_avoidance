#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <collision_avoidance/formation/FormationCalibration.hpp>
#include <collision_avoidance/formation/FormationDiscrimination.hpp>

namespace formation = collision_avoidance::formation;

namespace
{

formation::FormationBoundaryConfig makeConfig()
{
    formation::FormationBoundaryConfig config{};
    config.profile_name = "unit_test_uav_profile";
    config.profile_kind = formation::FormationProfileKind::UavCalibrated;
    config.representative_wingspan_m = 2.0;
    config.range0_wingspan_scale = 1.5;
    config.uncertainty_margin_m = 1.0;
    config.range1_offset_m = 5.0;
    config.closure_upper_entry_table = {
        {0.0, 1.0}, {50.0, 2.0}, {100.0, 3.0}};
    config.closure_upper_exit_table = {
        {0.0, 1.5}, {50.0, 2.5}, {100.0, 3.5}};
    config.closure_lower_entry_mps = -1.0;
    config.closure_lower_exit_mps = -1.5;
    config.fdz_entry_limit_m = 10.0;
    config.fdz_exit_limit_m = 12.0;
    config.max_range_entry_m = 80.0;
    config.max_range_exit_m = 90.0;
    config.maximum_state_age_s = 0.5;
    config.maximum_future_skew_s = 0.05;
    config.maximum_timestamp_skew_s = 0.1;
    config.lookup_policy =
        formation::FormationLookupPolicy::RejectOutsideTable;
    return config;
}

formation::FormationUpdateInput makeInput(
    const double range_m,
    const double closure_mps,
    const double time_s = 1.0,
    const std::uint32_t threat_id = 7)
{
    formation::FormationUpdateInput input{};
    input.threat_id = threat_id;
    input.evaluation_timestamp_s = time_s;
    input.ownship.valid = true;
    input.ownship.timestamp_s = time_s;
    input.ownship.position_ned_m = {0.0, 0.0, 0.0};
    input.ownship.velocity_ned_mps = {0.0, 0.0, 0.0};
    input.threat.valid = true;
    input.threat.timestamp_s = time_s;
    input.threat.position_ned_m = {range_m, 0.0, 0.0};
    input.threat.velocity_ned_mps = {-closure_mps, 0.0, 0.0};
    return input;
}

}  // namespace

TEST(FormationDiscrimination, ValidatesRelationsAndEntryExitHysteresis)
{
    auto config = makeConfig();
    EXPECT_TRUE(formation::FormationDiscriminator::validConfig(config));
    EXPECT_DOUBLE_EQ(config.range0_m(), 3.0);
    EXPECT_DOUBLE_EQ(config.minimum_distance_m(), 4.0);
    EXPECT_DOUBLE_EQ(config.range1_m(), 8.0);

    config.fdz_exit_limit_m = 9.0;
    EXPECT_FALSE(formation::FormationDiscriminator::validConfig(config));
    config = makeConfig();
    config.max_range_exit_m = 79.0;
    EXPECT_FALSE(formation::FormationDiscriminator::validConfig(config));
}

TEST(FormationDiscrimination, UsesPiecewiseLookupWithoutSilentExtrapolation)
{
    const auto config = makeConfig();
    const auto midpoint = formation::FormationDiscriminator::closureUpperLimit(
        config.closure_upper_entry_table, 25.0, config.lookup_policy);
    ASSERT_TRUE(midpoint.valid);
    EXPECT_FALSE(midpoint.clamped);
    EXPECT_DOUBLE_EQ(midpoint.closure_upper_mps, 1.5);

    const auto rejected = formation::FormationDiscriminator::closureUpperLimit(
        config.closure_upper_entry_table, 101.0, config.lookup_policy);
    EXPECT_FALSE(rejected.valid);
    const auto clamped = formation::FormationDiscriminator::closureUpperLimit(
        config.closure_upper_entry_table,
        101.0,
        formation::FormationLookupPolicy::ClampToEndpoint);
    ASSERT_TRUE(clamped.valid);
    EXPECT_TRUE(clamped.clamped);
    EXPECT_DOUBLE_EQ(clamped.closure_upper_mps, 3.0);
}

TEST(FormationDiscrimination, EntersFdzAndSlowRejoinStandby)
{
    formation::FormationDiscriminator discriminator(makeConfig());
    const auto fdz = discriminator.update(makeInput(9.0, 20.0));
    EXPECT_EQ(fdz.state, formation::FormationState::FormationDeactivationZone);
    EXPECT_TRUE(fdz.formation_inhibit);
    EXPECT_TRUE(fdz.entered_formation);

    discriminator.reset();
    const auto standby = discriminator.update(makeInput(50.0, 2.0));
    EXPECT_EQ(standby.state, formation::FormationState::Standby);
    EXPECT_EQ(
        standby.reason,
        formation::FormationDecisionReason::InsideNominalStandby);
}

TEST(FormationDiscrimination, RejectsExcessClosureAndEntryMaximumRange)
{
    formation::FormationDiscriminator discriminator(makeConfig());
    const auto high_closure = discriminator.update(makeInput(50.0, 2.01));
    EXPECT_EQ(
        high_closure.state, formation::FormationState::OutsideFormation);
    EXPECT_FALSE(high_closure.formation_inhibit);

    const auto outside_range = discriminator.update(makeInput(80.01, 2.0, 2.0));
    EXPECT_EQ(
        outside_range.state, formation::FormationState::OutsideFormation);
}

TEST(FormationDiscrimination, StandbyUsesOnlyItsRelaxedExitBoundary)
{
    formation::FormationDiscriminator discriminator(makeConfig());
    ASSERT_EQ(
        discriminator.update(makeInput(70.0, 2.0)).state,
        formation::FormationState::Standby);

    const auto retained = discriminator.update(makeInput(81.0, 2.0, 2.0));
    EXPECT_EQ(retained.state, formation::FormationState::Standby);
    EXPECT_TRUE(retained.retained_by_hysteresis);
    EXPECT_EQ(
        retained.reason,
        formation::FormationDecisionReason::RetainedInStandbyByExitBoundary);

    const auto exited = discriminator.update(makeInput(91.0, 2.0, 3.0));
    EXPECT_EQ(exited.state, formation::FormationState::OutsideFormation);
    EXPECT_TRUE(exited.exited_formation);
}

TEST(FormationDiscrimination, FdzUsesOnlyItsRelaxedExitBoundary)
{
    formation::FormationDiscriminator discriminator(makeConfig());
    ASSERT_EQ(
        discriminator.update(makeInput(9.9, 20.0)).state,
        formation::FormationState::FormationDeactivationZone);
    const auto retained = discriminator.update(makeInput(11.9, 20.0, 2.0));
    EXPECT_EQ(
        retained.state, formation::FormationState::FormationDeactivationZone);
    EXPECT_TRUE(retained.retained_by_hysteresis);
    const auto exited = discriminator.update(makeInput(12.1, 20.0, 3.0));
    EXPECT_EQ(exited.state, formation::FormationState::OutsideFormation);
}

TEST(FormationDiscrimination, CrossRegionTransitionsUseRelaxedUnionBoundary)
{
    formation::FormationDiscriminator from_fdz(makeConfig());
    ASSERT_EQ(
        from_fdz.update(makeInput(9.0, 20.0)).state,
        formation::FormationState::FormationDeactivationZone);
    const auto retained_in_standby = from_fdz.update(
        makeInput(85.0, 2.5, 2.0));
    EXPECT_EQ(retained_in_standby.state, formation::FormationState::Standby);
    EXPECT_TRUE(retained_in_standby.retained_by_hysteresis);

    formation::FormationDiscriminator from_standby(makeConfig());
    ASSERT_EQ(
        from_standby.update(makeInput(50.0, 2.0)).state,
        formation::FormationState::Standby);
    const auto retained_in_fdz = from_standby.update(
        makeInput(11.0, 20.0, 2.0));
    EXPECT_EQ(
        retained_in_fdz.state,
        formation::FormationState::FormationDeactivationZone);
    EXPECT_TRUE(retained_in_fdz.retained_by_hysteresis);
}

TEST(FormationDiscrimination, EntryAndExitTablesSuppressClosureChattering)
{
    formation::FormationDiscriminator discriminator(makeConfig());
    ASSERT_EQ(
        discriminator.update(makeInput(50.0, 1.99)).state,
        formation::FormationState::Standby);
    for (std::size_t index = 0; index < 8; ++index) {
        const double closure = index % 2 == 0 ? 2.05 : 1.95;
        const auto result = discriminator.update(
            makeInput(50.0, closure, 2.0 + static_cast<double>(index)));
        EXPECT_EQ(result.state, formation::FormationState::Standby);
    }
}

TEST(FormationDiscrimination, FdzEntryAndExitSuppressRangeChattering)
{
    formation::FormationDiscriminator discriminator(makeConfig());
    ASSERT_EQ(
        discriminator.update(makeInput(9.99, 20.0)).state,
        formation::FormationState::FormationDeactivationZone);
    for (std::size_t index = 0; index < 8; ++index) {
        const double range = index % 2 == 0 ? 10.01 : 9.99;
        const auto result = discriminator.update(
            makeInput(range, 20.0, 2.0 + static_cast<double>(index)));
        EXPECT_EQ(
            result.state,
            formation::FormationState::FormationDeactivationZone);
    }
}

TEST(FormationDiscrimination, VerifiesThreeDimensionalRangeAndClosureSign)
{
    formation::FormationDiscriminator discriminator(makeConfig());
    auto input = makeInput(30.0, 0.0);
    input.threat.position_ned_m = {30.0, 40.0, 12.0};
    input.threat.velocity_ned_mps = {3.0, 4.0, 1.2};
    const auto separating = discriminator.update(input);
    EXPECT_NEAR(separating.range_m, 51.4198405, 1.0e-6);
    EXPECT_GT(separating.range_rate_mps, 0.0);
    EXPECT_LT(separating.closure_rate_mps, 0.0);
    EXPECT_DOUBLE_EQ(
        formation::FormationDiscriminator::closureFromRangeRate(4.0), -4.0);
}

TEST(FormationDiscrimination, InvalidAndStaleTimestampsFailOpen)
{
    formation::FormationDiscriminator discriminator(makeConfig());
    ASSERT_TRUE(discriminator.update(makeInput(9.0, 0.0)).formation_inhibit);

    auto stale = makeInput(9.0, 0.0, 2.0);
    stale.threat.timestamp_s = 1.0;
    const auto stale_result = discriminator.update(stale);
    EXPECT_FALSE(stale_result.timestamp_valid);
    EXPECT_FALSE(stale_result.formation_inhibit);
    EXPECT_EQ(stale_result.state, formation::FormationState::OutsideFormation);
    EXPECT_TRUE(stale_result.exited_formation);

    auto skewed = makeInput(9.0, 0.0, 3.0);
    skewed.ownship.timestamp_s = 3.0;
    skewed.threat.timestamp_s = 2.85;
    EXPECT_FALSE(discriminator.update(skewed).timestamp_valid);
}

TEST(FormationDiscrimination, IdenticalEntryExitProfileHasNoHysteresis)
{
    auto config = makeConfig();
    config.closure_upper_exit_table = config.closure_upper_entry_table;
    config.closure_lower_exit_mps = config.closure_lower_entry_mps;
    config.fdz_exit_limit_m = config.fdz_entry_limit_m;
    config.max_range_exit_m = config.max_range_entry_m;
    formation::FormationDiscriminator discriminator(config);
    ASSERT_EQ(
        discriminator.update(makeInput(50.0, 2.0)).state,
        formation::FormationState::Standby);
    const auto exited = discriminator.update(makeInput(50.0, 2.01, 2.0));
    EXPECT_EQ(exited.state, formation::FormationState::OutsideFormation);
    EXPECT_FALSE(exited.retained_by_hysteresis);
}

TEST(FormationDiscrimination, ProfileChangeChangesResultWithoutCodeChange)
{
    auto permissive = makeConfig();
    auto restrictive = makeConfig();
    restrictive.profile_name = "restrictive_uav_profile";
    restrictive.max_range_entry_m = 40.0;
    restrictive.max_range_exit_m = 45.0;
    formation::FormationDiscriminator first(permissive);
    formation::FormationDiscriminator second(restrictive);
    EXPECT_TRUE(first.update(makeInput(50.0, 1.0)).formation_inhibit);
    EXPECT_FALSE(second.update(makeInput(50.0, 1.0)).formation_inhibit);
}

TEST(FormationAggregation, ExplicitPoliciesHandleFormationAndCollisionThreats)
{
    formation::FormationDiscriminator discriminator(makeConfig());
    const auto wingman = discriminator.update(makeInput(9.0, 0.0, 1.0, 1));
    const auto collision = discriminator.update(makeInput(50.0, 20.0, 1.0, 2));
    const std::vector<formation::FormationResult> results{wingman, collision};
    const std::vector<std::uint32_t> relevant{1, 2};

    const auto per_threat = formation::FormationInhibitAggregator::aggregate(
        results,
        relevant,
        formation::FormationAggregationPolicy::PerThreatExemptionOnly);
    EXPECT_FALSE(per_threat.formation_inhibit);
    EXPECT_TRUE(per_threat.allow_new_activation);
    ASSERT_EQ(per_threat.inhibited_threat_ids.size(), 1U);
    EXPECT_EQ(per_threat.inhibited_threat_ids.front(), 1U);

    const auto all = formation::FormationInhibitAggregator::aggregate(
        results,
        relevant,
        formation::FormationAggregationPolicy::AllRelevantThreatsFormation);
    EXPECT_FALSE(all.formation_inhibit);
    EXPECT_TRUE(all.complete_collision_context);

    const auto any = formation::FormationInhibitAggregator::aggregate(
        results,
        relevant,
        formation::FormationAggregationPolicy::AnyRelevantThreatFormation);
    EXPECT_TRUE(any.formation_inhibit);
    EXPECT_FALSE(any.allow_new_activation);
}

TEST(FormationAggregation, MissingThreatCannotSatisfyAllRelevantPolicy)
{
    formation::FormationDiscriminator discriminator(makeConfig());
    const auto wingman = discriminator.update(makeInput(9.0, 0.0, 1.0, 1));
    const auto aggregation = formation::FormationInhibitAggregator::aggregate(
        {wingman},
        {1, 2},
        formation::FormationAggregationPolicy::AllRelevantThreatsFormation);
    EXPECT_FALSE(aggregation.complete_collision_context);
    EXPECT_FALSE(aggregation.formation_inhibit);
    EXPECT_TRUE(aggregation.allow_new_activation);
}

TEST(FormationCalibration, CsvContainsStructuredDecisionAndUnits)
{
    const auto config = makeConfig();
    formation::FormationDiscriminator discriminator(config);
    const auto result = discriminator.update(makeInput(50.0, 1.0));
    const auto header = formation::FormationDecisionCsvFormatter::header();
    const auto row = formation::FormationDecisionCsvFormatter::row(result, config);
    EXPECT_NE(header.find("range_rate_mps"), std::string::npos);
    EXPECT_NE(header.find("retained_by_hysteresis"), std::string::npos);
    EXPECT_NE(row.find("unit_test_uav_profile"), std::string::npos);
    EXPECT_NE(row.find("inside nominal standby envelope"), std::string::npos);
    EXPECT_NE(row.find("\"SI\""), std::string::npos);
}

TEST(FormationCalibration, ReplaySweepAndMetricsPreservePerThreatState)
{
    std::vector<formation::FormationReplaySample> samples;
    for (std::size_t index = 0; index < 4; ++index) {
        formation::FormationReplaySample sample{};
        const double range = index < 3 ? 50.0 : 95.0;
        sample.input = makeInput(range, 1.0, static_cast<double>(index + 1));
        sample.collision_activation_requested = index == 1;
        sample.has_expected_inhibit = true;
        sample.expected_inhibit = index < 3;
        samples.push_back(sample);
    }

    auto second_profile = makeConfig();
    second_profile.profile_name = "short_range_profile";
    second_profile.max_range_entry_m = 40.0;
    second_profile.max_range_exit_m = 45.0;
    const formation::FormationMetricParams params{1.0, 0.1};
    const auto sweep = formation::FormationCalibrationHarness::sweep(
        samples, {makeConfig(), second_profile}, params);
    ASSERT_EQ(sweep.size(), 2U);
    EXPECT_TRUE(sweep[0].config_valid);
    EXPECT_DOUBLE_EQ(sweep[0].metrics.first_entry_time_s, 1.0);
    EXPECT_DOUBLE_EQ(sweep[0].metrics.first_exit_time_s, 4.0);
    EXPECT_EQ(sweep[0].metrics.state_toggle_count, 2U);
    EXPECT_EQ(sweep[0].metrics.nuisance_activation_opportunities, 1U);
    EXPECT_EQ(sweep[0].metrics.missed_inhibit_cases, 0U);
    EXPECT_LT(sweep[1].metrics.first_entry_time_s, 0.0);
    EXPECT_EQ(sweep[1].metrics.missed_inhibit_cases, 1U);
}
