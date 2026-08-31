#include <collision_avoidance/selection/SafeControlCandidateAdapter.hpp>

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

namespace cs = collision_avoidance::selection;

namespace
{

cs::SafeControlCandidateAdapterParams unitParams()
{
    cs::SafeControlCandidateAdapterParams params;
    params.robustness_guard_radps = 0.1;
    params.duplicate_tolerance_radps = 1.0e-9;
    params.speed_tolerance_mps = 1.0e-3;
    return params;
}

cs::SafeControlCandidateAdapterInput validInput()
{
    cs::SafeControlCandidateAdapterInput input;
    input.safe_set.status = cs::SafeControlSetStatus::Valid;
    input.safe_set.effective_max_heading_rate_radps = 1.0;
    input.safe_set.left_safe = {0.3, 1.0, true};
    input.safe_set.right_safe = {-1.0, -0.4, true};
    input.true_airspeed_mps = 20.0;
    input.nominal_rate_available = true;
    input.nominal_heading_rate_v4_radps = 0.0;
    input.ground_speed_command_mps = 18.0;
    input.altitude_command_m = 120.0;
    return input;
}

bool inInterval(
    double value,
    const cs::HeadingRateInterval & interval)
{
    return interval.feasible
        && value >= interval.lower_radps
        && value <= interval.upper_radps;
}

}  // namespace

TEST(SafeControlCandidateAdapter, KeepsNominalThatIsAlreadySafe)
{
    auto input = validInput();
    input.nominal_heading_rate_v4_radps = -0.6;

    const auto result =
        cs::SafeControlCandidateAdapter(unitParams()).generate(input);

    ASSERT_EQ(result.status, cs::SafeControlCandidateAdapterStatus::Valid);
    ASSERT_EQ(result.candidate_count, 3U);
    EXPECT_EQ(result.candidates[0].role, cs::SafeCandidateRole::NearNominal);
    EXPECT_NEAR(result.candidates[0].heading_rate_v4_radps, -0.6, 1.0e-12);
}

TEST(SafeControlCandidateAdapter, ProjectsNominalToNearestSeparateFamily)
{
    const auto input = validInput();
    const auto result =
        cs::SafeControlCandidateAdapter(unitParams()).generate(input);

    ASSERT_EQ(result.status, cs::SafeControlCandidateAdapterStatus::Valid);
    ASSERT_GE(result.candidate_count, 1U);
    EXPECT_NEAR(result.candidates[0].heading_rate_v4_radps, 0.3, 1.0e-12);
    EXPECT_TRUE(inInterval(
        result.candidates[0].heading_rate_v4_radps,
        input.safe_set.left_safe));
    EXPECT_FALSE(inInterval(0.0, input.safe_set.left_safe));
    EXPECT_FALSE(inInterval(0.0, input.safe_set.right_safe));
}

TEST(SafeControlCandidateAdapter, ProjectionTieDeterministicallyChoosesLeft)
{
    auto input = validInput();
    input.safe_set.left_safe = {0.2, 1.0, true};
    input.safe_set.right_safe = {-1.0, -0.2, true};

    const auto result =
        cs::SafeControlCandidateAdapter(unitParams()).generate(input);

    ASSERT_EQ(result.status, cs::SafeControlCandidateAdapterStatus::Valid);
    ASSERT_GE(result.candidate_count, 1U);
    EXPECT_NEAR(result.candidates[0].heading_rate_v4_radps, 0.2, 1.0e-12);
}

TEST(SafeControlCandidateAdapter, UsesGuardedDirectionalRepresentatives)
{
    const auto input = validInput();
    const auto result =
        cs::SafeControlCandidateAdapter(unitParams()).generate(input);

    ASSERT_EQ(result.status, cs::SafeControlCandidateAdapterStatus::Valid);
    ASSERT_EQ(result.candidate_count, 3U);
    EXPECT_EQ(result.candidates[1].role, cs::SafeCandidateRole::RobustLeft);
    EXPECT_NEAR(result.candidates[1].heading_rate_v4_radps, 0.9, 1.0e-12);
    EXPECT_EQ(result.candidates[2].role, cs::SafeCandidateRole::RobustRight);
    EXPECT_NEAR(result.candidates[2].heading_rate_v4_radps, -0.9, 1.0e-12);
}

TEST(SafeControlCandidateAdapter, NarrowIntervalUsesMidpoint)
{
    auto input = validInput();
    input.nominal_rate_available = false;
    input.safe_set.left_safe = {0.4, 0.5, true};
    input.safe_set.right_safe.feasible = false;

    const auto result =
        cs::SafeControlCandidateAdapter(unitParams()).generate(input);

    ASSERT_EQ(result.status, cs::SafeControlCandidateAdapterStatus::Valid);
    ASSERT_EQ(result.candidate_count, 1U);
    EXPECT_EQ(result.candidates[0].role, cs::SafeCandidateRole::RobustLeft);
    EXPECT_NEAR(result.candidates[0].heading_rate_v4_radps, 0.45, 1.0e-12);
}

TEST(SafeControlCandidateAdapter, SuppressesDuplicateRatesByRoleOrder)
{
    auto input = validInput();
    input.nominal_heading_rate_v4_radps = 0.9;

    const auto result =
        cs::SafeControlCandidateAdapter(unitParams()).generate(input);

    ASSERT_EQ(result.status, cs::SafeControlCandidateAdapterStatus::Valid);
    ASSERT_EQ(result.candidate_count, 2U);
    EXPECT_EQ(result.candidates[0].role, cs::SafeCandidateRole::NearNominal);
    EXPECT_NEAR(result.candidates[0].heading_rate_v4_radps, 0.9, 1.0e-12);
    EXPECT_EQ(result.candidates[1].role, cs::SafeCandidateRole::RobustRight);
}

TEST(SafeControlCandidateAdapter, MissingNominalStillReturnsDirections)
{
    auto input = validInput();
    input.nominal_rate_available = false;
    input.nominal_heading_rate_v4_radps =
        std::numeric_limits<double>::quiet_NaN();

    const auto result =
        cs::SafeControlCandidateAdapter(unitParams()).generate(input);

    ASSERT_EQ(result.status, cs::SafeControlCandidateAdapterStatus::Valid);
    ASSERT_EQ(result.candidate_count, 2U);
    EXPECT_EQ(result.candidates[0].role, cs::SafeCandidateRole::RobustLeft);
    EXPECT_EQ(result.candidates[1].role, cs::SafeCandidateRole::RobustRight);
}

TEST(SafeControlCandidateAdapter, OneFeasibleFamilyNeverCreatesOtherFamily)
{
    auto input = validInput();
    input.safe_set.right_safe.feasible = false;
    input.nominal_heading_rate_v4_radps = -0.8;

    const auto result =
        cs::SafeControlCandidateAdapter(unitParams()).generate(input);

    ASSERT_EQ(result.status, cs::SafeControlCandidateAdapterStatus::Valid);
    ASSERT_EQ(result.candidate_count, 2U);
    for (std::size_t index = 0; index < result.candidate_count; ++index) {
        EXPECT_TRUE(inInterval(
            result.candidates[index].heading_rate_v4_radps,
            input.safe_set.left_safe));
        EXPECT_GE(result.candidates[index].heading_rate_v4_radps, 0.0);
    }
}

TEST(SafeControlCandidateAdapter, InfeasibleCoreProducesNoCandidate)
{
    auto input = validInput();
    input.safe_set.status = cs::SafeControlSetStatus::SearchSetInfeasible;
    input.safe_set.left_safe.feasible = false;
    input.safe_set.right_safe.feasible = false;

    const auto result =
        cs::SafeControlCandidateAdapter(unitParams()).generate(input);

    EXPECT_EQ(
        result.status,
        cs::SafeControlCandidateAdapterStatus::SearchSetInfeasible);
    EXPECT_EQ(result.candidate_count, 0U);
}

TEST(SafeControlCandidateAdapter, NeverReturnsMoreThanThreeSafeCandidates)
{
    const auto input = validInput();
    const auto result =
        cs::SafeControlCandidateAdapter(unitParams()).generate(input);

    ASSERT_EQ(result.status, cs::SafeControlCandidateAdapterStatus::Valid);
    EXPECT_LE(result.candidate_count, cs::kMaximumSafeControlCandidates);
    for (std::size_t index = 0; index < result.candidate_count; ++index) {
        const double rate = result.candidates[index].heading_rate_v4_radps;
        EXPECT_TRUE(
            inInterval(rate, input.safe_set.left_safe)
            || inInterval(rate, input.safe_set.right_safe));
    }
}

TEST(SafeControlCandidateAdapter, ConvertsV4AndPx4SignsExactlyOnce)
{
    EXPECT_NEAR(
        cs::SafeControlCandidateAdapter::
            v4HeadingRateToPx4LateralAcceleration(20.0, 0.25),
        -5.0,
        1.0e-12);
    EXPECT_NEAR(
        cs::SafeControlCandidateAdapter::
            v4HeadingRateToPx4LateralAcceleration(20.0, -0.25),
        5.0,
        1.0e-12);

    const double original_rate = 0.37;
    const double acceleration = cs::SafeControlCandidateAdapter::
        v4HeadingRateToPx4LateralAcceleration(22.0, original_rate);
    const double recovered_rate = cs::SafeControlCandidateAdapter::
        px4LateralAccelerationToV4HeadingRate(22.0, acceleration);
    EXPECT_NEAR(recovered_rate, original_rate, 1.0e-12);
}

TEST(SafeControlCandidateAdapter, PreservesCommandsAndIntroducesNoClimb)
{
    auto input = validInput();
    input.ground_speed_command_mps = 21.5;
    input.altitude_command_m = 135.0;

    const auto result =
        cs::SafeControlCandidateAdapter(unitParams()).generate(input);

    ASSERT_EQ(result.status, cs::SafeControlCandidateAdapterStatus::Valid);
    ASSERT_GT(result.candidate_count, 0U);
    for (std::size_t index = 0; index < result.candidate_count; ++index) {
        const auto & command = result.candidates[index].predictor_input;
        EXPECT_DOUBLE_EQ(command.V_cmd, 21.5);
        EXPECT_DOUBLE_EQ(command.h_cmd, 135.0);
        EXPECT_DOUBLE_EQ(command.h_dot_cmd, 0.0);
        EXPECT_NEAR(
            command.a_lat_cmd,
            -input.true_airspeed_mps
                * result.candidates[index].heading_rate_v4_radps,
            1.0e-12);
    }
}

TEST(SafeControlCandidateAdapter, SupportsExistingNanAltitudeFallback)
{
    auto input = validInput();
    input.altitude_command_m = std::numeric_limits<double>::quiet_NaN();

    const auto result =
        cs::SafeControlCandidateAdapter(unitParams()).generate(input);

    ASSERT_EQ(result.status, cs::SafeControlCandidateAdapterStatus::Valid);
    ASSERT_GT(result.candidate_count, 0U);
    EXPECT_TRUE(std::isnan(result.candidates[0].predictor_input.h_cmd));
}

TEST(SafeControlCandidateAdapter, MapsModeBBranchesWithoutLegacyIntervals)
{
    cs::BackupControlCandidateAdapterInputV4 input;
    input.interpolation.status = cs::BackupInterpolationStatusV4::Valid;
    input.interpolation.left.status =
        cs::BackupInterpolationBranchStatusV4::Feasible;
    input.interpolation.left.nominal_heading_rate_v4_radps = 0.05;
    input.interpolation.left.safe_heading_rate_v4_radps = 0.2;
    input.interpolation.right.status =
        cs::BackupInterpolationBranchStatusV4::Feasible;
    input.interpolation.right.nominal_heading_rate_v4_radps = 0.05;
    input.interpolation.right.safe_heading_rate_v4_radps = -0.4;
    input.true_airspeed_mps = 20.0;
    input.ground_speed_command_mps = 18.0;
    input.altitude_command_m = 120.0;

    const auto result = cs::SafeControlCandidateAdapter(unitParams())
        .generateFromBackupInterpolation(input);

    ASSERT_EQ(result.status, cs::SafeControlCandidateAdapterStatus::Valid);
    ASSERT_EQ(result.candidate_count, 2U);
    EXPECT_EQ(result.candidates[0].role, cs::SafeCandidateRole::NearNominal);
    EXPECT_NEAR(result.candidates[0].heading_rate_v4_radps, 0.2, 1.0e-12);
    // The nominal-compatible command is identical to the LEFT branch, so the
    // existing adapter's deterministic de-duplication keeps one copy.
    EXPECT_EQ(result.candidates[1].role, cs::SafeCandidateRole::RobustRight);
    for (std::size_t index = 0; index < result.candidate_count; ++index) {
        EXPECT_DOUBLE_EQ(result.candidates[index].predictor_input.h_dot_cmd, 0.0);
        EXPECT_NEAR(
            result.candidates[index].predictor_input.a_lat_cmd,
            -20.0 * result.candidates[index].heading_rate_v4_radps,
            1.0e-12);
    }
}

TEST(SafeControlCandidateAdapter, ModeBDeduplicatesNominalAndBranchRates)
{
    cs::BackupControlCandidateAdapterInputV4 input;
    input.interpolation.status = cs::BackupInterpolationStatusV4::Valid;
    input.interpolation.left.status =
        cs::BackupInterpolationBranchStatusV4::Feasible;
    input.interpolation.left.nominal_heading_rate_v4_radps = 0.1;
    input.interpolation.left.safe_heading_rate_v4_radps = 0.1;
    input.interpolation.right.status =
        cs::BackupInterpolationBranchStatusV4::NotCertified;
    input.true_airspeed_mps = 20.0;
    input.ground_speed_command_mps = 20.0;

    const auto result = cs::SafeControlCandidateAdapter(unitParams())
        .generateFromBackupInterpolation(input);

    ASSERT_EQ(result.status, cs::SafeControlCandidateAdapterStatus::Valid);
    ASSERT_EQ(result.candidate_count, 1U);
    EXPECT_EQ(result.candidates[0].role, cs::SafeCandidateRole::NearNominal);
}

TEST(SafeControlCandidateAdapter, ModeBInfeasibleSetFailsClosed)
{
    cs::BackupControlCandidateAdapterInputV4 input;
    input.interpolation.status =
        cs::BackupInterpolationStatusV4::NoCertifiedBranch;
    input.true_airspeed_mps = 20.0;
    input.ground_speed_command_mps = 20.0;

    const auto result = cs::SafeControlCandidateAdapter(unitParams())
        .generateFromBackupInterpolation(input);

    EXPECT_EQ(
        result.status,
        cs::SafeControlCandidateAdapterStatus::SearchSetInfeasible);
    EXPECT_EQ(result.candidate_count, 0U);
}

TEST(SafeControlCandidateAdapter, RejectsInvalidInputsAndConfiguration)
{
    auto invalid_params = unitParams();
    invalid_params.duplicate_tolerance_radps = 0.0;
    EXPECT_FALSE(cs::SafeControlCandidateAdapter::validParams(invalid_params));
    EXPECT_EQ(
        cs::SafeControlCandidateAdapter(invalid_params)
            .generate(validInput()).status,
        cs::SafeControlCandidateAdapterStatus::InvalidConfiguration);

    auto zero_guard = unitParams();
    zero_guard.robustness_guard_radps = 0.0;
    EXPECT_FALSE(cs::SafeControlCandidateAdapter::validParams(zero_guard));

    auto invalid_set = validInput();
    invalid_set.safe_set.left_safe = {-0.2, 0.5, true};
    EXPECT_EQ(
        cs::SafeControlCandidateAdapter(unitParams())
            .generate(invalid_set).status,
        cs::SafeControlCandidateAdapterStatus::InvalidSafeSet);

    auto invalid_speed = validInput();
    invalid_speed.true_airspeed_mps = 0.0;
    EXPECT_EQ(
        cs::SafeControlCandidateAdapter(unitParams())
            .generate(invalid_speed).status,
        cs::SafeControlCandidateAdapterStatus::InvalidAirspeed);

    auto invalid_command = validInput();
    invalid_command.ground_speed_command_mps =
        std::numeric_limits<double>::infinity();
    EXPECT_EQ(
        cs::SafeControlCandidateAdapter(unitParams())
            .generate(invalid_command).status,
        cs::SafeControlCandidateAdapterStatus::InvalidPredictorCommand);

    auto invalid_nominal = validInput();
    invalid_nominal.nominal_heading_rate_v4_radps =
        std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(
        cs::SafeControlCandidateAdapter(unitParams())
            .generate(invalid_nominal).status,
        cs::SafeControlCandidateAdapterStatus::InvalidNominalRate);

    EXPECT_TRUE(std::isnan(
        cs::SafeControlCandidateAdapter::
            px4LateralAccelerationToV4HeadingRate(0.0, 1.0)));
    EXPECT_TRUE(std::isnan(
        cs::SafeControlCandidateAdapter::
            px4LateralAccelerationToV4HeadingRate(
                2.0e-3,
                std::numeric_limits<double>::max())));
}

TEST(SafeControlCandidateAdapter, StatusNamesAreStableDiagnostics)
{
    EXPECT_STREQ(
        cs::safeControlCandidateAdapterStatusName(
            cs::SafeControlCandidateAdapterStatus::SearchSetInfeasible),
        "search_set_infeasible");
    EXPECT_STREQ(
        cs::safeControlCandidateAdapterStatusName(
            cs::SafeControlCandidateAdapterStatus::InvalidNominalRate),
        "invalid_nominal_rate");
}
