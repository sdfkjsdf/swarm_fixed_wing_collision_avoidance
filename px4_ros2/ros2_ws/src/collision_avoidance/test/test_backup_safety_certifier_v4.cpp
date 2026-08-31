#include <collision_avoidance/selection/BackupSafetyCertifierV4.hpp>

#include <cmath>
#include <cstdint>

#include <gtest/gtest.h>

namespace cs = collision_avoidance::selection;

namespace
{

constexpr std::uint64_t kTimestampUs = 4'000'000;

cs::BackupSafetyCertifierV4Params unitParams()
{
    cs::BackupSafetyCertifierV4Params params;
    params.model.gravity_mps2 = 10.0;
    params.model.maximum_roll_rad = std::atan(2.0);
    params.model.maximum_yaw_rate_radps = 2.0;
    params.horizon_s = 4.5;
    params.integration_step_s = 0.1;
    params.reference_margin_m = 2.0;
    params.certification_tolerance_m = 1.0e-6;
    return params;
}

cs::BackupSafetyCertifierV4Input northboundInput(
    double airspeed_mps = 20.0)
{
    cs::BackupSafetyCertifierV4Input input;
    input.ownship.timestamp_us = kTimestampUs;
    input.ownship.north_m = 0.0;
    input.ownship.east_m = 0.0;
    input.ownship.heading_ned_rad = 0.0;
    input.ownship.true_airspeed_mps = airspeed_mps;
    input.ownship.longitudinal_acceleration_mps2 = 0.0;
    input.ownship.longitudinal_source =
        cs::LongitudinalDriftSource::LocalOneStepFreeze;
    return input;
}

void addLinearThreat(
    cs::BackupSafetyCertifierV4Input & input,
    const cs::BackupSafetyCertifierV4Params & params,
    int vehicle_id,
    double north_m,
    double east_m,
    double velocity_north_mps,
    double velocity_east_mps,
    double physical_clearance_m = 0.0)
{
    ASSERT_LT(input.threat_count, input.threats.size());
    cs::BackupControlModelV4 model(params.model);
    const auto grid = model.propagate(
        input.ownship,
        cs::BackupDirectionV4::Left,
        params.horizon_s,
        params.integration_step_s);
    ASSERT_EQ(grid.status, cs::BackupPropagationStatusV4::Valid);

    auto & threat = input.threats[input.threat_count++];
    threat.vehicle_id = vehicle_id;
    threat.source_timestamp_us = input.ownship.timestamp_us;
    threat.physical_clearance_m = physical_clearance_m;
    threat.point_count = grid.point_count;
    for (std::size_t index = 0; index < threat.point_count; ++index) {
        const double time_s = grid.points[index].time_offset_s;
        threat.points[index].time_offset_s = time_s;
        threat.points[index].north_m =
            north_m + velocity_north_mps * time_s;
        threat.points[index].east_m =
            east_m + velocity_east_mps * time_s;
    }
}

void expectFixedDirection(const cs::BackupSafetyCertifierV4Result & result)
{
    ASSERT_GT(result.left.trajectory.point_count, 1U);
    ASSERT_EQ(
        result.left.trajectory.point_count,
        result.right.trajectory.point_count);
    for (std::size_t index = 0;
         index < result.left.trajectory.point_count;
         ++index) {
        EXPECT_GE(
            result.left.trajectory.points[index]
                .backup_heading_rate_v4_radps,
            0.0);
        EXPECT_LE(
            result.right.trajectory.points[index]
                .backup_heading_rate_v4_radps,
            0.0);
    }
}

}  // namespace

TEST(BackupSafetyCertifierV4, NoThreatCertifiesBothFixedDirections)
{
    const cs::BackupSafetyCertifierV4 certifier(unitParams());
    const auto result = certifier.evaluate(northboundInput());

    ASSERT_EQ(result.status, cs::BackupCertificationStatusV4::Valid);
    EXPECT_EQ(
        result.classification,
        cs::BackupBranchClassificationV4::BothCertified);
    EXPECT_TRUE(result.left.certified);
    EXPECT_TRUE(result.right.certified);
    EXPECT_EQ(result.left.trajectory.point_count, 46U);
    EXPECT_TRUE(std::isinf(result.left.minimum_path_margin_m));
    EXPECT_TRUE(std::isinf(result.left.terminal_turn_margin_m));
    EXPECT_NEAR(result.effective_max_heading_rate_initial_radps, 1.0, 1e-12);
    expectFixedDirection(result);
}

TEST(BackupSafetyCertifierV4, ParallelSameSpeedTrafficCertifiesBoth)
{
    const auto params = unitParams();
    auto input = northboundInput();
    addLinearThreat(input, params, 1, 0.0, 100.0, 20.0, 0.0);

    const auto result = cs::BackupSafetyCertifierV4(params).evaluate(input);

    ASSERT_EQ(result.status, cs::BackupCertificationStatusV4::Valid);
    EXPECT_EQ(
        result.classification,
        cs::BackupBranchClassificationV4::BothCertified);
    EXPECT_GT(result.left.minimum_path_margin_m, 0.0);
    EXPECT_GT(result.right.minimum_path_margin_m, 0.0);
}

TEST(BackupSafetyCertifierV4, HeadOnTrafficIsEvaluatedOnBothBranches)
{
    const auto params = unitParams();
    auto input = northboundInput();
    addLinearThreat(input, params, 1, 200.0, 0.0, -20.0, 0.0);

    const auto result = cs::BackupSafetyCertifierV4(params).evaluate(input);

    ASSERT_EQ(result.status, cs::BackupCertificationStatusV4::Valid);
    EXPECT_EQ(result.evaluated_threat_count, 1U);
    EXPECT_TRUE(std::isfinite(result.left.minimum_path_margin_m));
    EXPECT_TRUE(std::isfinite(result.right.minimum_path_margin_m));
    expectFixedDirection(result);
}

TEST(BackupSafetyCertifierV4, EvaluatesLeftAndRightCrossingTraffic)
{
    const auto params = unitParams();
    auto from_left = northboundInput();
    addLinearThreat(from_left, params, 1, 60.0, -50.0, 0.0, 20.0);
    auto from_right = northboundInput();
    addLinearThreat(from_right, params, 2, 60.0, 50.0, 0.0, -20.0);

    const auto left_result =
        cs::BackupSafetyCertifierV4(params).evaluate(from_left);
    const auto right_result =
        cs::BackupSafetyCertifierV4(params).evaluate(from_right);

    ASSERT_EQ(left_result.status, cs::BackupCertificationStatusV4::Valid);
    ASSERT_EQ(right_result.status, cs::BackupCertificationStatusV4::Valid);
    EXPECT_EQ(left_result.evaluated_threat_count, 1U);
    EXPECT_EQ(right_result.evaluated_threat_count, 1U);
    EXPECT_TRUE(std::isfinite(left_result.left.terminal_turn_margin_m));
    EXPECT_TRUE(std::isfinite(right_result.right.terminal_turn_margin_m));
}

TEST(BackupSafetyCertifierV4, ClassifiesLeftOnlyAndRightOnly)
{
    const auto params = unitParams();
    auto west_threat = northboundInput();
    addLinearThreat(west_threat, params, 1, 20.0, -20.0, 0.0, 0.0);
    auto east_threat = northboundInput();
    addLinearThreat(east_threat, params, 2, 20.0, 20.0, 0.0, 0.0);

    const auto west_result =
        cs::BackupSafetyCertifierV4(params).evaluate(west_threat);
    const auto east_result =
        cs::BackupSafetyCertifierV4(params).evaluate(east_threat);

    ASSERT_EQ(west_result.status, cs::BackupCertificationStatusV4::Valid);
    ASSERT_EQ(east_result.status, cs::BackupCertificationStatusV4::Valid);
    EXPECT_EQ(
        west_result.classification,
        cs::BackupBranchClassificationV4::RightOnly);
    EXPECT_EQ(
        east_result.classification,
        cs::BackupBranchClassificationV4::LeftOnly);
    EXPECT_FALSE(west_result.left.certified);
    EXPECT_TRUE(west_result.right.certified);
    EXPECT_TRUE(east_result.left.certified);
    EXPECT_FALSE(east_result.right.certified);
}

TEST(BackupSafetyCertifierV4, ConflictingThreatsCertifyNeitherBranch)
{
    const auto params = unitParams();
    auto input = northboundInput();
    addLinearThreat(input, params, 1, 20.0, -20.0, 0.0, 0.0);
    addLinearThreat(input, params, 2, 20.0, 20.0, 0.0, 0.0);

    const auto result = cs::BackupSafetyCertifierV4(params).evaluate(input);

    ASSERT_EQ(result.status, cs::BackupCertificationStatusV4::Valid);
    EXPECT_EQ(
        result.classification,
        cs::BackupBranchClassificationV4::NeitherCertified);
    EXPECT_FALSE(result.left.certified);
    EXPECT_FALSE(result.right.certified);
    EXPECT_NE(result.left.limiting_path_threat_id, -1);
    EXPECT_NE(result.right.limiting_path_threat_id, -1);
}

TEST(BackupSafetyCertifierV4, AirspeedChangesTurnRateMagnitude)
{
    const auto params = unitParams();
    const cs::BackupControlModelV4 model(params.model);

    EXPECT_NEAR(model.effectiveMaxHeadingRate(20.0), 1.0, 1e-12);
    EXPECT_NEAR(model.effectiveMaxHeadingRate(40.0), 0.5, 1e-12);

    auto accelerating = northboundInput();
    accelerating.ownship.longitudinal_source =
        cs::LongitudinalDriftSource::ValidatedExternal;
    accelerating.ownship.longitudinal_acceleration_mps2 = 2.0;
    const auto trajectory = model.propagate(
        accelerating.ownship,
        cs::BackupDirectionV4::Left,
        params.horizon_s,
        params.integration_step_s);
    ASSERT_EQ(trajectory.status, cs::BackupPropagationStatusV4::Valid);
    EXPECT_LT(
        trajectory.points[trajectory.point_count - 1]
            .backup_heading_rate_v4_radps,
        trajectory.points[0].backup_heading_rate_v4_radps);
    for (std::size_t index = 0; index < trajectory.point_count; ++index) {
        EXPECT_GE(
            trajectory.points[index].backup_heading_rate_v4_radps,
            0.0);
    }
}

TEST(BackupSafetyCertifierV4, DirectionMayChangeOnlyAcrossNewCycles)
{
    const auto params = unitParams();
    auto cycle_one = northboundInput();
    addLinearThreat(cycle_one, params, 1, 20.0, -20.0, 0.0, 0.0);
    auto cycle_two = northboundInput();
    cycle_two.ownship.timestamp_us += 50'000;
    addLinearThreat(cycle_two, params, 1, 20.0, 20.0, 0.0, 0.0);

    const cs::BackupSafetyCertifierV4 certifier(params);
    const auto first = certifier.evaluate(cycle_one);
    const auto second = certifier.evaluate(cycle_two);

    ASSERT_EQ(first.status, cs::BackupCertificationStatusV4::Valid);
    ASSERT_EQ(second.status, cs::BackupCertificationStatusV4::Valid);
    EXPECT_EQ(
        first.classification,
        cs::BackupBranchClassificationV4::RightOnly);
    EXPECT_EQ(
        second.classification,
        cs::BackupBranchClassificationV4::LeftOnly);
    expectFixedDirection(first);
    expectFixedDirection(second);
}

TEST(BackupSafetyCertifierV4, ThreatTimingAndGridFailuresAreFailClosed)
{
    const auto params = unitParams();
    auto stale = northboundInput();
    addLinearThreat(stale, params, 1, 100.0, 0.0, 0.0, 0.0);
    stale.threats[0].source_timestamp_us -= 1;
    EXPECT_EQ(
        cs::BackupSafetyCertifierV4(params).evaluate(stale).status,
        cs::BackupCertificationStatusV4::StaleThreatTimestamp);

    auto future = northboundInput();
    addLinearThreat(future, params, 1, 100.0, 0.0, 0.0, 0.0);
    future.threats[0].source_timestamp_us += 1;
    EXPECT_EQ(
        cs::BackupSafetyCertifierV4(params).evaluate(future).status,
        cs::BackupCertificationStatusV4::FutureThreatTimestamp);

    auto bad_grid = northboundInput();
    addLinearThreat(bad_grid, params, 1, 100.0, 0.0, 0.0, 0.0);
    bad_grid.threats[0].points[3].time_offset_s += 0.01;
    EXPECT_EQ(
        cs::BackupSafetyCertifierV4(params).evaluate(bad_grid).status,
        cs::BackupCertificationStatusV4::InvalidThreatTrajectory);
}

TEST(BackupSafetyCertifierV4, RejectsInvalidConfigurationAndState)
{
    auto invalid_params = unitParams();
    invalid_params.horizon_s = 0.0;
    EXPECT_FALSE(cs::BackupSafetyCertifierV4::validParams(invalid_params));
    EXPECT_EQ(
        cs::BackupSafetyCertifierV4(invalid_params)
            .evaluate(northboundInput()).status,
        cs::BackupCertificationStatusV4::InvalidConfiguration);

    auto invalid_state = northboundInput(0.0);
    EXPECT_EQ(
        cs::BackupSafetyCertifierV4(unitParams())
            .evaluate(invalid_state).status,
        cs::BackupCertificationStatusV4::InvalidOwnshipState);
}
