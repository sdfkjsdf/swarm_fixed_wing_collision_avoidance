#include <collision_avoidance/selection/BackupControlInterpolatorV4.hpp>

#include <cmath>
#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

namespace cs = collision_avoidance::selection;

namespace
{

constexpr std::uint64_t kTimestampUs = 8'000'000;

cs::BackupControlInterpolatorV4Params unitParams()
{
    cs::BackupControlInterpolatorV4Params params;
    params.certifier.model.gravity_mps2 = 10.0;
    params.certifier.model.maximum_roll_rad = std::atan(2.0);
    params.certifier.model.maximum_yaw_rate_radps = 2.0;
    params.certifier.horizon_s = 4.5;
    params.certifier.integration_step_s = 0.1;
    params.certifier.reference_margin_m = 2.0;
    params.certifier.certification_tolerance_m = 1.0e-6;
    params.path_alpha_gain_per_s = 0.2;
    params.terminal_alpha_gain_per_s = 0.5;
    return params;
}

cs::BackupSafetyCertifierV4Input northboundInput()
{
    cs::BackupSafetyCertifierV4Input input;
    input.ownship.timestamp_us = kTimestampUs;
    input.ownship.north_m = 0.0;
    input.ownship.east_m = 0.0;
    input.ownship.heading_ned_rad = 0.0;
    input.ownship.true_airspeed_mps = 20.0;
    input.ownship.longitudinal_acceleration_mps2 = 0.0;
    input.ownship.longitudinal_source =
        cs::LongitudinalDriftSource::LocalOneStepFreeze;
    return input;
}

void addLinearThreat(
    cs::BackupSafetyCertifierV4Input & input,
    const cs::BackupControlInterpolatorV4Params & params,
    int vehicle_id,
    double north_m,
    double east_m,
    double velocity_north_mps,
    double velocity_east_mps)
{
    ASSERT_LT(input.threat_count, input.threats.size());
    const cs::BackupControlModelV4 model(params.certifier.model);
    const auto grid = model.propagate(
        input.ownship,
        cs::BackupDirectionV4::Left,
        params.certifier.horizon_s,
        params.certifier.integration_step_s);
    ASSERT_EQ(grid.status, cs::BackupPropagationStatusV4::Valid);

    auto & threat = input.threats[input.threat_count++];
    threat.vehicle_id = vehicle_id;
    threat.source_timestamp_us = input.ownship.timestamp_us;
    threat.physical_clearance_m = 0.0;
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

}  // namespace

TEST(BackupControlInterpolatorV4, NoThreatKeepsNominalForBothBranches)
{
    const cs::BackupControlInterpolatorV4 interpolator(unitParams());
    const auto result = interpolator.evaluate(northboundInput(), 0.3);

    ASSERT_EQ(result.status, cs::BackupInterpolationStatusV4::Valid);
    ASSERT_EQ(
        result.left.status,
        cs::BackupInterpolationBranchStatusV4::Feasible);
    ASSERT_EQ(
        result.right.status,
        cs::BackupInterpolationBranchStatusV4::Feasible);
    EXPECT_EQ(result.certified_branch_count, 2U);
    EXPECT_EQ(result.feasible_branch_count, 2U);
    EXPECT_DOUBLE_EQ(result.left.mu_star, 0.0);
    EXPECT_DOUBLE_EQ(result.right.mu_star, 0.0);
    EXPECT_NEAR(result.left.safe_heading_rate_v4_radps, 0.3, 1e-12);
    EXPECT_NEAR(result.right.safe_heading_rate_v4_radps, 0.3, 1e-12);
}

TEST(BackupControlInterpolatorV4, ScalarIntersectionHandlesBothBoundSigns)
{
    cs::ScalarMuIntervalV4 interval;
    const cs::BackupInterpolationTolerancesV4 tolerances;

    ASSERT_TRUE(cs::BackupControlInterpolatorV4::applyScalarConstraint(
        -0.25, 1.0, tolerances, interval));
    EXPECT_NEAR(interval.lower, 0.25, 1e-12);
    EXPECT_NEAR(interval.upper, 1.0, 1e-12);

    ASSERT_TRUE(cs::BackupControlInterpolatorV4::applyScalarConstraint(
        0.75, -1.0, tolerances, interval));
    EXPECT_NEAR(interval.lower, 0.25, 1e-12);
    EXPECT_NEAR(interval.upper, 0.75, 1e-12);
}

TEST(BackupControlInterpolatorV4, ScalarDangerRequiresMoreIntervention)
{
    const cs::BackupInterpolationTolerancesV4 tolerances;
    cs::ScalarMuIntervalV4 moderate;
    cs::ScalarMuIntervalV4 severe;

    ASSERT_TRUE(cs::BackupControlInterpolatorV4::applyScalarConstraint(
        -0.2, 1.0, tolerances, moderate));
    ASSERT_TRUE(cs::BackupControlInterpolatorV4::applyScalarConstraint(
        -0.6, 1.0, tolerances, severe));
    EXPECT_NEAR(moderate.lower, 0.2, 1e-12);
    EXPECT_NEAR(severe.lower, 0.6, 1e-12);
    EXPECT_GT(severe.lower, moderate.lower);
}

TEST(BackupControlInterpolatorV4, DegenerateViolatedConstraintIsInfeasible)
{
    cs::ScalarMuIntervalV4 interval;
    const cs::BackupInterpolationTolerancesV4 tolerances;

    EXPECT_FALSE(cs::BackupControlInterpolatorV4::applyScalarConstraint(
        -1.0, 0.0, tolerances, interval));
    EXPECT_FALSE(interval.feasible);
}

TEST(BackupControlInterpolatorV4, EmptyScalarIntersectionIsInfeasible)
{
    cs::ScalarMuIntervalV4 interval;
    const cs::BackupInterpolationTolerancesV4 tolerances;

    ASSERT_TRUE(cs::BackupControlInterpolatorV4::applyScalarConstraint(
        -0.8, 1.0, tolerances, interval));
    EXPECT_FALSE(cs::BackupControlInterpolatorV4::applyScalarConstraint(
        0.2, -1.0, tolerances, interval));
    EXPECT_FALSE(interval.feasible);
}

TEST(BackupControlInterpolatorV4, DoesNotInterpolateUncertifiedDirection)
{
    const auto params = unitParams();
    auto input = northboundInput();
    addLinearThreat(input, params, 1, 20.0, -20.0, 0.0, 0.0);

    const auto result =
        cs::BackupControlInterpolatorV4(params).evaluate(input, 0.0);

    ASSERT_EQ(
        result.certification.classification,
        cs::BackupBranchClassificationV4::RightOnly);
    EXPECT_EQ(
        result.left.status,
        cs::BackupInterpolationBranchStatusV4::NotCertified);
    EXPECT_NE(
        result.right.status,
        cs::BackupInterpolationBranchStatusV4::NotCertified);
}

TEST(BackupControlInterpolatorV4, GeometricDangerIncreasesIntervention)
{
    const auto params = unitParams();
    const auto evaluateAtEast = [&params](double east_m) {
            auto input = northboundInput();
            addLinearThreat(
                input, params, 1, 10.0, east_m, 10.0, 0.0);
            return cs::BackupControlInterpolatorV4(params).evaluate(
                input, 0.0);
        };
    const auto moderate = evaluateAtEast(-80.0);
    const auto severe = evaluateAtEast(-75.0);

    ASSERT_EQ(
        moderate.left.status,
        cs::BackupInterpolationBranchStatusV4::Feasible);
    ASSERT_EQ(
        severe.left.status,
        cs::BackupInterpolationBranchStatusV4::Feasible);
    ASSERT_EQ(
        moderate.right.status,
        cs::BackupInterpolationBranchStatusV4::Feasible);
    EXPECT_GT(moderate.left.mu_star, 0.0);
    EXPECT_GT(severe.left.mu_star, moderate.left.mu_star);
    EXPECT_DOUBLE_EQ(moderate.right.mu_star, 0.0);
    EXPECT_GT(moderate.left.mu_star, moderate.right.mu_star);
    EXPECT_GE(
        severe.left.safe_heading_rate_v4_radps,
        severe.left.nominal_heading_rate_v4_radps);
    EXPECT_LE(
        severe.left.safe_heading_rate_v4_radps,
        severe.left.backup_heading_rate_v4_radps);
    EXPECT_GE(
        severe.left.minimum_residual_at_mu_star_mps,
        -params.tolerances.residual_mps);
}

TEST(BackupControlInterpolatorV4, DirectionalRatesMatchFiniteDifference)
{
    const auto params = unitParams();
    auto input = northboundInput();
    addLinearThreat(input, params, 1, 10.0, -80.0, 10.0, 0.0);
    const auto interpolation =
        cs::BackupControlInterpolatorV4(params).evaluate(input, 0.0);

    ASSERT_EQ(
        interpolation.left.status,
        cs::BackupInterpolationBranchStatusV4::Feasible);
    ASSERT_TRUE(interpolation.left.lower_bound_diagnostic_valid);
    const auto diagnostic = interpolation.left.lower_bound_diagnostic;

    const auto marginForOwnship = [
        &input,
        &params,
        &diagnostic](const cs::SafeControlOwnshipState & ownship) {
            auto perturbed_input = input;
            perturbed_input.ownship = ownship;
            const auto certification =
                cs::BackupSafetyCertifierV4(params.certifier).evaluate(
                    perturbed_input);
            if (certification.status
                != cs::BackupCertificationStatusV4::Valid) {
                return std::numeric_limits<double>::quiet_NaN();
            }
            if (diagnostic.kind == cs::BackupInterpolationConstraintKindV4::
                    TerminalTurnCertificate) {
                return certification.left.terminal_turn_margin_m;
            }
            for (std::size_t index = 0;
                 index < certification.left.trajectory.point_count;
                 ++index) {
                const auto & ownship_point =
                    certification.left.trajectory.points[index];
                if (std::abs(
                        ownship_point.time_offset_s
                        - diagnostic.time_offset_s) <= 1.0e-9) {
                    const auto & threat_point = input.threats[0].points[index];
                    return std::hypot(
                            ownship_point.north_m - threat_point.north_m,
                            ownship_point.east_m - threat_point.east_m)
                        - input.threats[0].physical_clearance_m
                        - params.certifier.reference_margin_m;
                }
            }
            return std::numeric_limits<double>::quiet_NaN();
        };
    const auto finiteDifference = [
        &input,
        &marginForOwnship](double rate_v4_radps) {
            constexpr double epsilon_s = 1.0e-5;
            const double speed_mps = input.ownship.true_airspeed_mps;
            const double heading_rad = input.ownship.heading_ned_rad;
            const double north_rate_mps = speed_mps * std::cos(heading_rad);
            const double east_rate_mps = speed_mps * std::sin(heading_rad);
            const double heading_ned_rate_radps = -rate_v4_radps;
            const double speed_rate_mps2 =
                input.ownship.longitudinal_acceleration_mps2;
            auto plus = input.ownship;
            auto minus = input.ownship;
            plus.north_m += epsilon_s * north_rate_mps;
            minus.north_m -= epsilon_s * north_rate_mps;
            plus.east_m += epsilon_s * east_rate_mps;
            minus.east_m -= epsilon_s * east_rate_mps;
            plus.heading_ned_rad += epsilon_s * heading_ned_rate_radps;
            minus.heading_ned_rad -= epsilon_s * heading_ned_rate_radps;
            plus.true_airspeed_mps += epsilon_s * speed_rate_mps2;
            minus.true_airspeed_mps -= epsilon_s * speed_rate_mps2;
            return (marginForOwnship(plus) - marginForOwnship(minus))
                / (2.0 * epsilon_s);
        };

    EXPECT_NEAR(
        finiteDifference(0.0),
        diagnostic.hdot_nominal_mps,
        5.0e-3);
    EXPECT_NEAR(
        finiteDifference(
            interpolation.left.backup_heading_rate_v4_radps),
        diagnostic.hdot_backup_mps,
        5.0e-3);
}

TEST(BackupControlInterpolatorV4, ModelLimitDerivativesMatchActiveBranch)
{
    const auto params = unitParams();
    const cs::BackupControlModelV4 bank_limited(params.certifier.model);
    EXPECT_NEAR(
        bank_limited.effectiveMaxHeadingRateSpeedDerivative(20.0),
        -0.05,
        1.0e-12);
    EXPECT_NEAR(
        bank_limited.turningRadiusSpeedDerivative(20.0),
        2.0,
        1.0e-12);

    auto yaw_limited_params = params.certifier.model;
    yaw_limited_params.maximum_yaw_rate_radps = 0.25;
    const cs::BackupControlModelV4 yaw_limited(yaw_limited_params);
    EXPECT_DOUBLE_EQ(
        yaw_limited.effectiveMaxHeadingRateSpeedDerivative(20.0),
        0.0);
    EXPECT_NEAR(
        yaw_limited.turningRadiusSpeedDerivative(20.0),
        4.0,
        1.0e-12);
}

TEST(BackupControlInterpolatorV4, ConflictingThreatsReturnNoCertifiedBranch)
{
    const auto params = unitParams();
    auto input = northboundInput();
    addLinearThreat(input, params, 1, 20.0, -20.0, 0.0, 0.0);
    addLinearThreat(input, params, 2, 20.0, 20.0, 0.0, 0.0);

    const auto result =
        cs::BackupControlInterpolatorV4(params).evaluate(input, 0.0);

    EXPECT_EQ(
        result.status,
        cs::BackupInterpolationStatusV4::NoCertifiedBranch);
    EXPECT_EQ(result.certified_branch_count, 0U);
}

TEST(BackupControlInterpolatorV4, RejectsInvalidNominalAndConfiguration)
{
    const auto params = unitParams();
    const cs::BackupControlInterpolatorV4 interpolator(params);
    EXPECT_EQ(
        interpolator.evaluate(
            northboundInput(),
            std::numeric_limits<double>::quiet_NaN()).status,
        cs::BackupInterpolationStatusV4::InvalidNominalRate);
    EXPECT_EQ(
        interpolator.evaluate(northboundInput(), 1.1).status,
        cs::BackupInterpolationStatusV4::InvalidNominalRate);

    auto invalid_params = params;
    invalid_params.path_alpha_gain_per_s = 0.0;
    EXPECT_FALSE(
        cs::BackupControlInterpolatorV4::validParams(invalid_params));
    EXPECT_EQ(
        cs::BackupControlInterpolatorV4(invalid_params)
            .evaluate(northboundInput(), 0.0).status,
        cs::BackupInterpolationStatusV4::InvalidConfiguration);
}
