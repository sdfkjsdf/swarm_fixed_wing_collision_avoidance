#include <collision_avoidance/selection/SafeControlSetV4.hpp>

#include <cmath>
#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

namespace cs = collision_avoidance::selection;

namespace
{

constexpr std::uint64_t kTimestampUs = 1'000'000;

cs::SafeControlSetV4Params unitParams()
{
    cs::SafeControlSetV4Params params;
    params.margin_reference_m = 10.0;
    params.margin_time_constant_s = 5.0;
    params.control_period_s = 0.05;
    params.gravity_mps2 = 10.0;
    params.maximum_roll_rad = std::atan(2.0);
    params.maximum_yaw_rate_radps = 2.0;
    return params;
}

cs::SafeControlSetV4Input northboundInput(double airspeed_mps = 20.0)
{
    cs::SafeControlSetV4Input input;
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

void addStationaryThreat(
    cs::SafeControlSetV4Input & input,
    int vehicle_id,
    double north_m,
    double east_m,
    double physical_clearance_m = 0.0)
{
    ASSERT_LT(input.threat_count, input.threats.size());
    auto & threat = input.threats[input.threat_count++];
    threat.vehicle_id = vehicle_id;
    threat.timestamp_us = input.ownship.timestamp_us;
    threat.north_m = north_m;
    threat.east_m = east_m;
    threat.velocity_north_mps = 0.0;
    threat.velocity_east_mps = 0.0;
    threat.physical_clearance_m = physical_clearance_m;
}

}  // namespace

TEST(SafeControlSetV4, NoThreatReturnsSeparatePhysicalFamilies)
{
    const cs::SafeControlSetV4 core(unitParams());
    const auto result = core.evaluate(northboundInput());

    EXPECT_EQ(result.status, cs::SafeControlSetStatus::Valid);
    ASSERT_TRUE(result.left_safe.feasible);
    ASSERT_TRUE(result.right_safe.feasible);
    EXPECT_NEAR(result.effective_max_heading_rate_radps, 1.0, 1.0e-12);
    EXPECT_NEAR(result.left_safe.lower_radps, 0.0, 1.0e-12);
    EXPECT_NEAR(result.left_safe.upper_radps, 1.0, 1.0e-12);
    EXPECT_NEAR(result.right_safe.lower_radps, -1.0, 1.0e-12);
    EXPECT_NEAR(result.right_safe.upper_radps, 0.0, 1.0e-12);
    EXPECT_EQ(result.evaluated_threat_count, 0U);
    EXPECT_EQ(result.diagnostic_count, 0U);
    EXPECT_NEAR(result.kappa_per_s, 0.2, 1.0e-12);
    EXPECT_NEAR(
        result.gamma_diagnostic,
        1.0 - std::exp(-0.05 / 5.0),
        1.0e-12);
}

TEST(SafeControlSetV4, WestThreatTrimsPositiveLeftRateInNed)
{
    auto input = northboundInput();
    // Northbound NED: negative East is geometrically Left/West.
    addStationaryThreat(input, 1, 100.0, -20.0);

    const cs::SafeControlSetV4 core(unitParams());
    const auto result = core.evaluate(input);

    EXPECT_EQ(result.status, cs::SafeControlSetStatus::Valid);
    ASSERT_TRUE(result.left_safe.feasible);
    EXPECT_NEAR(result.left_safe.lower_radps, 0.3, 1.0e-12);
    EXPECT_NEAR(result.left_safe.upper_radps, 1.0, 1.0e-12);
    EXPECT_EQ(result.evaluated_threat_count, 1U);
    EXPECT_EQ(result.diagnostic_count, 2U);
}

TEST(SafeControlSetV4, EastThreatTrimsNegativeRightRateInNed)
{
    auto input = northboundInput();
    // Northbound NED: positive East is geometrically Right.
    addStationaryThreat(input, 1, 100.0, 20.0);

    const cs::SafeControlSetV4 core(unitParams());
    const auto result = core.evaluate(input);

    EXPECT_EQ(result.status, cs::SafeControlSetStatus::Valid);
    ASSERT_TRUE(result.right_safe.feasible);
    EXPECT_NEAR(result.right_safe.lower_radps, -1.0, 1.0e-12);
    EXPECT_NEAR(result.right_safe.upper_radps, -0.3, 1.0e-12);
}

TEST(SafeControlSetV4, MultipleThreatsIntersectWithinOneDirection)
{
    auto input = northboundInput();
    addStationaryThreat(input, 1, 100.0, -20.0);  // r_left >= 0.3
    addStationaryThreat(input, 2, 80.0, -20.0);   // r_left >= 0.5

    const cs::SafeControlSetV4 core(unitParams());
    const auto result = core.evaluate(input);

    EXPECT_EQ(result.status, cs::SafeControlSetStatus::Valid);
    ASSERT_TRUE(result.left_safe.feasible);
    EXPECT_NEAR(result.left_safe.lower_radps, 0.5, 1.0e-12);
    EXPECT_NEAR(result.left_safe.upper_radps, 1.0, 1.0e-12);
    EXPECT_EQ(result.evaluated_threat_count, 2U);
    EXPECT_EQ(result.diagnostic_count, 4U);
}

TEST(SafeControlSetV4, DoesNotMixLeftAndRightAcrossThreats)
{
    auto input = northboundInput();
    addStationaryThreat(input, 1, 20.0, -20.0);  // Left requires r > +1.
    addStationaryThreat(input, 2, 20.0, 20.0);   // Right requires r < -1.

    const cs::SafeControlSetV4 core(unitParams());
    const auto result = core.evaluate(input);

    EXPECT_EQ(
        result.status,
        cs::SafeControlSetStatus::SearchSetInfeasible);
    EXPECT_FALSE(result.left_safe.feasible);
    EXPECT_FALSE(result.right_safe.feasible);
    EXPECT_GE(result.first_infeasible_vehicle_id, 0);
}

TEST(SafeControlSetV4, DegenerateRateCoefficientPassesConstantConstraint)
{
    auto input = northboundInput();
    // For the Left circle q is normal to the flight direction, making a_r=0.
    addStationaryThreat(input, 1, 0.0, -120.0);

    const cs::SafeControlSetV4 core(unitParams());
    const auto result = core.evaluate(input);

    EXPECT_EQ(result.status, cs::SafeControlSetStatus::Valid);
    EXPECT_TRUE(result.left_safe.feasible);
    EXPECT_NEAR(result.left_safe.lower_radps, 0.0, 1.0e-12);
    EXPECT_NEAR(result.left_safe.upper_radps, 1.0, 1.0e-12);
    ASSERT_GE(result.diagnostic_count, 1U);
    EXPECT_TRUE(result.diagnostics[0].constraint_degenerate);
    EXPECT_TRUE(result.diagnostics[0].constraint_feasible);
}

TEST(SafeControlSetV4, DegenerateRateCoefficientRejectsViolatedConstraint)
{
    auto input = northboundInput();
    addStationaryThreat(input, 1, 0.0, -30.0);

    const cs::SafeControlSetV4 core(unitParams());
    const auto result = core.evaluate(input);

    EXPECT_EQ(result.status, cs::SafeControlSetStatus::Valid);
    EXPECT_FALSE(result.left_safe.feasible);
    EXPECT_TRUE(result.right_safe.feasible);
    ASSERT_GE(result.diagnostic_count, 1U);
    EXPECT_TRUE(result.diagnostics[0].constraint_degenerate);
    EXPECT_FALSE(result.diagnostics[0].constraint_feasible);
}

TEST(SafeControlSetV4, ExactPhysicalBoundaryRemainsAFeasiblePoint)
{
    auto input = northboundInput();
    addStationaryThreat(input, 1, 30.0, -20.0);  // r_left >= 1.0

    const cs::SafeControlSetV4 core(unitParams());
    const auto result = core.evaluate(input);

    EXPECT_EQ(result.status, cs::SafeControlSetStatus::Valid);
    ASSERT_TRUE(result.left_safe.feasible);
    EXPECT_NEAR(result.left_safe.lower_radps, 1.0, 1.0e-12);
    EXPECT_NEAR(result.left_safe.upper_radps, 1.0, 1.0e-12);
}

TEST(SafeControlSetV4, ToleranceCollapseNeverEscapesPhysicalDomain)
{
    auto input = northboundInput();
    // Produces a lower bound of 1.00000005 rad/s, within eps_interval of 1.
    addStationaryThreat(input, 1, 29.999995, -20.0);

    const cs::SafeControlSetV4 core(unitParams());
    const auto result = core.evaluate(input);

    EXPECT_EQ(result.status, cs::SafeControlSetStatus::Valid);
    ASSERT_TRUE(result.left_safe.feasible);
    EXPECT_NEAR(result.left_safe.lower_radps, 1.0, 1.0e-12);
    EXPECT_NEAR(result.left_safe.upper_radps, 1.0, 1.0e-12);
    EXPECT_LE(
        result.left_safe.upper_radps,
        result.effective_max_heading_rate_radps);
}

TEST(SafeControlSetV4, YawRateAndBankRateLimitsUseTheMinimum)
{
    auto yaw_limited_params = unitParams();
    yaw_limited_params.maximum_yaw_rate_radps = 0.4;
    const auto yaw_limited =
        cs::SafeControlSetV4(yaw_limited_params).evaluate(
            northboundInput());
    EXPECT_NEAR(
        yaw_limited.effective_max_heading_rate_radps,
        0.4,
        1.0e-12);

    auto bank_limited_params = unitParams();
    bank_limited_params.maximum_roll_rad = std::atan(1.0);
    const auto bank_limited =
        cs::SafeControlSetV4(bank_limited_params).evaluate(
            northboundInput());
    EXPECT_NEAR(
        bank_limited.effective_max_heading_rate_radps,
        0.5,
        1.0e-12);
}

TEST(SafeControlSetV4, RecomputesBankLimitedRateFromCurrentAirspeed)
{
    const cs::SafeControlSetV4 core(unitParams());
    const auto at_twenty = core.evaluate(northboundInput(20.0));
    const auto at_forty = core.evaluate(northboundInput(40.0));

    EXPECT_NEAR(
        at_twenty.effective_max_heading_rate_radps,
        1.0,
        1.0e-12);
    EXPECT_NEAR(
        at_forty.effective_max_heading_rate_radps,
        0.5,
        1.0e-12);
}

TEST(SafeControlSetV4, BankLimitedAccelerationUsesStateDependentRadiusDerivative)
{
    auto frozen_input = northboundInput();
    addStationaryThreat(frozen_input, 1, 100.0, -20.0);
    auto accelerated_input = frozen_input;
    accelerated_input.ownship.longitudinal_acceleration_mps2 = 2.0;
    accelerated_input.ownship.longitudinal_source =
        cs::LongitudinalDriftSource::ValidatedExternal;

    const cs::SafeControlSetV4 core(unitParams());
    const auto frozen = core.evaluate(frozen_input);
    const auto accelerated = core.evaluate(accelerated_input);

    EXPECT_NEAR(frozen.left_safe.lower_radps, 0.3, 1.0e-12);
    EXPECT_NEAR(accelerated.left_safe.lower_radps, 0.5, 1.0e-12);
    EXPECT_NEAR(
        frozen.effective_max_heading_rate_radps,
        accelerated.effective_max_heading_rate_radps,
        1.0e-12);
    EXPECT_EQ(
        frozen.longitudinal_source,
        cs::LongitudinalDriftSource::LocalOneStepFreeze);
    EXPECT_EQ(
        accelerated.longitudinal_source,
        cs::LongitudinalDriftSource::ValidatedExternal);
}

TEST(SafeControlSetV4, YawLimitedAccelerationUsesConstantLimitRadiusDerivative)
{
    auto params = unitParams();
    params.maximum_yaw_rate_radps = 0.5;
    auto frozen_input = northboundInput();
    // rho=V/r_yaw=40 m; position q=(-100,0) for the Left circle.
    addStationaryThreat(frozen_input, 1, 100.0, -40.0);
    auto accelerated_input = frozen_input;
    accelerated_input.ownship.longitudinal_acceleration_mps2 = 2.0;
    accelerated_input.ownship.longitudinal_source =
        cs::LongitudinalDriftSource::ValidatedExternal;

    const cs::SafeControlSetV4 core(params);
    const auto frozen = core.evaluate(frozen_input);
    const auto accelerated = core.evaluate(accelerated_input);

    EXPECT_NEAR(frozen.left_safe.lower_radps, 0.25, 1.0e-12);
    EXPECT_NEAR(accelerated.left_safe.lower_radps, 0.35, 1.0e-12);
    EXPECT_NEAR(
        accelerated.effective_max_heading_rate_radps,
        0.5,
        1.0e-12);
}

TEST(SafeControlSetV4, RejectsInvalidAirspeedAndNonFiniteState)
{
    const cs::SafeControlSetV4 core(unitParams());
    auto zero_speed = northboundInput(0.0);
    EXPECT_EQ(
        core.evaluate(zero_speed).status,
        cs::SafeControlSetStatus::InvalidAirspeed);

    auto non_finite = northboundInput();
    non_finite.ownship.heading_ned_rad =
        std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(
        core.evaluate(non_finite).status,
        cs::SafeControlSetStatus::InvalidOwnshipState);
}

TEST(SafeControlSetV4, RejectsFutureStaleAndInvalidThreats)
{
    const cs::SafeControlSetV4 core(unitParams());
    auto future = northboundInput();
    addStationaryThreat(future, 1, 100.0, -20.0);
    future.threats[0].timestamp_us = kTimestampUs + 1;
    EXPECT_EQ(
        core.evaluate(future).status,
        cs::SafeControlSetStatus::FutureThreatTimestamp);

    auto stale = northboundInput();
    addStationaryThreat(stale, 1, 100.0, -20.0);
    stale.threats[0].timestamp_us = kTimestampUs - 1;
    EXPECT_EQ(
        core.evaluate(stale).status,
        cs::SafeControlSetStatus::StaleThreatTimestamp);

    auto invalid = northboundInput();
    addStationaryThreat(invalid, 1, 100.0, -20.0);
    invalid.threats[0].physical_clearance_m = -1.0;
    EXPECT_EQ(
        core.evaluate(invalid).status,
        cs::SafeControlSetStatus::InvalidThreatState);
}

TEST(SafeControlSetV4, RejectsDegenerateTurningCircleGeometry)
{
    auto input = northboundInput();
    // Left center is N=0, E=-20 for the unit test parameters.
    addStationaryThreat(input, 1, 0.0, -20.0);

    const cs::SafeControlSetV4 core(unitParams());
    const auto result = core.evaluate(input);

    EXPECT_EQ(
        result.status,
        cs::SafeControlSetStatus::DegenerateGeometry);
    EXPECT_EQ(result.first_infeasible_vehicle_id, 1);
    EXPECT_EQ(
        result.first_infeasible_direction,
        cs::SafeControlDirection::Left);
}

TEST(SafeControlSetV4, RejectsInvalidConfigurationAndThreatCount)
{
    auto invalid_params = unitParams();
    invalid_params.margin_time_constant_s = 0.0;
    EXPECT_FALSE(cs::SafeControlSetV4::validParams(invalid_params));
    EXPECT_EQ(
        cs::SafeControlSetV4(invalid_params)
            .evaluate(northboundInput()).status,
        cs::SafeControlSetStatus::InvalidConfiguration);

    auto zero_tolerance_params = unitParams();
    zero_tolerance_params.tolerances.constraint_mps = 0.0;
    EXPECT_FALSE(
        cs::SafeControlSetV4::validParams(zero_tolerance_params));

    auto invalid_source = northboundInput();
    invalid_source.ownship.longitudinal_source =
        static_cast<cs::LongitudinalDriftSource>(255);
    EXPECT_EQ(
        cs::SafeControlSetV4(unitParams())
            .evaluate(invalid_source).status,
        cs::SafeControlSetStatus::InvalidOwnshipState);

    auto inconsistent_freeze = northboundInput();
    inconsistent_freeze.ownship.longitudinal_acceleration_mps2 = 0.1;
    EXPECT_EQ(
        cs::SafeControlSetV4(unitParams())
            .evaluate(inconsistent_freeze).status,
        cs::SafeControlSetStatus::InvalidOwnshipState);

    auto too_many = northboundInput();
    too_many.threat_count = too_many.threats.size() + 1;
    EXPECT_EQ(
        cs::SafeControlSetV4(unitParams()).evaluate(too_many).status,
        cs::SafeControlSetStatus::InvalidThreatCount);
}

TEST(SafeControlSetV4, StatusNamesAreStableDiagnostics)
{
    EXPECT_STREQ(
        cs::safeControlSetStatusName(
            cs::SafeControlSetStatus::SearchSetInfeasible),
        "search_set_infeasible");
    EXPECT_STREQ(
        cs::safeControlSetStatusName(
            cs::SafeControlSetStatus::FutureThreatTimestamp),
        "future_threat_timestamp");
}
