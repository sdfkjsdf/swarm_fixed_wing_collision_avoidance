#include <gtest/gtest.h>

#include <limits>

#include <collision_avoidance/selection/ManeuverActivationController.hpp>

namespace cs = collision_avoidance::selection;
namespace ce = collision_avoidance::estimation;

namespace
{

cs::ManeuverActivationSample sample(
    std::uint64_t timestamp_us,
    double ad_m,
    double relative_position_n_m,
    double relative_velocity_n_mps,
    std::uint8_t candidate_id = 1,
    double activation_criterion_m = 10.0)
{
    cs::ManeuverActivationSample value;
    value.timestamp_us = timestamp_us;
    value.valid = true;
    value.minimum_ad_m = ad_m;
    value.unsafe_threat_mask = std::uint32_t{1} << 1;
    value.activation_criteria_m.fill(
        std::numeric_limits<double>::quiet_NaN());
    value.activation_criteria_m[1] = activation_criterion_m;
    for (auto & position : value.relative_positions_ned_m) {
        position.fill(std::numeric_limits<double>::quiet_NaN());
    }
    for (auto & velocity : value.relative_velocities_ned_mps) {
        velocity.fill(std::numeric_limits<double>::quiet_NaN());
    }
    value.relative_positions_ned_m[1] = {relative_position_n_m, 0.0, 0.0};
    value.relative_velocities_ned_mps[1] = {
        relative_velocity_n_mps, 0.0, 0.0};
    value.selected_candidate_id = candidate_id;
    value.selected_candidate_input_revision = 1000U + candidate_id;
    value.selected_input.V_cmd = 20.0;
    value.selected_input.a_lat_cmd = static_cast<double>(candidate_id);
    return value;
}

}  // namespace

TEST(ManeuverActivationController, LatchesCommandAcrossSelectionChanges)
{
    cs::ManeuverActivationController controller;
    const auto activated = controller.update(
        sample(1'000'000, -0.1, 5.0, -1.0, 1));
    ASSERT_TRUE(activated.active);
    EXPECT_TRUE(activated.just_activated);
    EXPECT_EQ(activated.latched_candidate_id, 1U);
    EXPECT_EQ(activated.latched_candidate_input_revision, 1001U);

    const auto held = controller.update(
        sample(1'250'000, -5.0, 5.0, -1.0, 6));
    EXPECT_TRUE(held.active);
    EXPECT_FALSE(held.just_activated);
    EXPECT_EQ(held.latched_candidate_id, 1U);
    EXPECT_EQ(held.latched_candidate_input_revision, 1001U);
    EXPECT_DOUBLE_EQ(held.latched_input.a_lat_cmd, 1.0);

    const auto positive_ad = controller.update(
        sample(1'500'000, 10.0, 5.0, -1.0, 0));
    EXPECT_TRUE(positive_ad.active);
    EXPECT_FALSE(positive_ad.just_deactivated);
    EXPECT_EQ(positive_ad.latched_candidate_id, 1U);
}

TEST(ManeuverActivationController, DeactivatesWhenFutureCpaIsClear)
{
    cs::ManeuverActivationController controller;
    ASSERT_TRUE(controller.update(sample(2'000'000, -1.0, 5.0, -1.0)).active);
    const auto status = controller.update(sample(2'100'000, 1.0, 20.0, 1.0));
    EXPECT_FALSE(status.active);
    EXPECT_TRUE(status.just_deactivated);
    EXPECT_EQ(
        status.deactivation_reason,
        cs::ManeuverDeactivationReason::FutureCpaClear);
}

TEST(ManeuverActivationController, FormationInhibitBlocksOnlyNewActivation)
{
    cs::ManeuverActivationController controller;
    auto inhibited = sample(2'150'000, -1.0, 5.0, -1.0);
    inhibited.allow_new_activation = false;
    EXPECT_FALSE(controller.update(inhibited).active);

    auto activation = sample(2'250'000, -1.0, 5.0, -1.0);
    ASSERT_TRUE(controller.update(activation).active);
    auto clear_while_inhibited = sample(2'350'000, 1.0, 20.0, 1.0);
    clear_while_inhibited.allow_new_activation = false;
    const auto status = controller.update(clear_while_inhibited);
    EXPECT_FALSE(status.active);
    EXPECT_TRUE(status.just_deactivated);
    EXPECT_EQ(
        status.deactivation_reason,
        cs::ManeuverDeactivationReason::FutureCpaClear);
}

TEST(ManeuverActivationController, RequiresEveryAffectedPairCpaToBeClear)
{
    cs::ManeuverActivationController controller;
    auto activation = sample(2'500'000, -1.0, 5.0, -1.0);
    activation.unsafe_threat_mask =
        (std::uint32_t{1} << 1) | (std::uint32_t{1} << 2);
    activation.activation_criteria_m[2] = 10.0;
    activation.relative_positions_ned_m[2] = {5.0, 0.0, 0.0};
    activation.relative_velocities_ned_mps[2] = {-1.0, 0.0, 0.0};
    ASSERT_TRUE(controller.update(activation).active);

    auto one_cpa_unsafe = sample(2'600'000, 1.0, 20.0, 1.0);
    one_cpa_unsafe.relative_positions_ned_m[2] = {5.0, 0.0, 0.0};
    one_cpa_unsafe.relative_velocities_ned_mps[2] = {-1.0, 0.0, 0.0};
    EXPECT_TRUE(controller.update(one_cpa_unsafe).active);

    auto all_cpa_clear = sample(2'700'000, 1.0, 20.0, 1.0);
    all_cpa_clear.relative_positions_ned_m[2] = {20.0, 0.0, 0.0};
    all_cpa_clear.relative_velocities_ned_mps[2] = {1.0, 0.0, 0.0};
    const auto ended = controller.update(all_cpa_clear);
    EXPECT_FALSE(ended.active);
    EXPECT_TRUE(ended.just_deactivated);
}

TEST(ManeuverActivationController, ZeroRelativeSpeedUsesCurrentDistance)
{
    cs::ManeuverActivationController controller;
    ASSERT_TRUE(controller.update(sample(2'800'000, -1.0, 5.0, -1.0)).active);

    const auto clear = controller.update(sample(2'900'000, 1.0, 20.0, 0.0));
    EXPECT_FALSE(clear.active);
    EXPECT_TRUE(clear.just_deactivated);

    ASSERT_TRUE(controller.update(sample(3'000'000, -1.0, 5.0, 0.0)).active);
    const auto still_inside = controller.update(
        sample(3'100'000, 1.0, 5.0, 0.0));
    EXPECT_TRUE(still_inside.active);
    EXPECT_FALSE(still_inside.just_deactivated);
}

TEST(ManeuverActivationController,
    CalibratedNearZeroRelativeSpeedUsesCurrentDistance)
{
    cs::ManeuverActivationControllerParams params;
    params.relative_speed_epsilon_mps = 0.21;
    cs::ManeuverActivationController controller(params);
    ASSERT_TRUE(controller.update(sample(3'150'000, -1.0, 5.0, -1.0)).active);

    // The unconstrained CPA would be 200 s away. Because 0.1 m/s is below
    // the calibrated near-zero threshold, current separation is used.
    const auto clear = controller.update(
        sample(3'160'000, 1.0, 20.0, -0.1));
    EXPECT_FALSE(clear.active);
    EXPECT_TRUE(clear.just_deactivated);
}

TEST(ManeuverActivationController,
    ApproachingCpaBeyondPredictionHorizonCannotTerminate)
{
    cs::ManeuverActivationControllerParams params;
    params.cpa_horizon_s = 4.5;
    cs::ManeuverActivationController controller(params);
    ASSERT_TRUE(controller.update(sample(3'170'000, -1.0, 5.0, -1.0)).active);

    auto beyond_horizon = sample(3'180'000, 1.0, 20.0, -1.0);
    beyond_horizon.relative_positions_ned_m[1] = {20.0, 15.0, 0.0};
    beyond_horizon.relative_velocities_ned_mps[1] = {-1.0, 0.0, 0.0};
    const auto status = controller.update(beyond_horizon);
    EXPECT_TRUE(status.active);
    EXPECT_FALSE(status.just_deactivated);
}

TEST(ManeuverActivationController,
    CpaExactlyOnPredictionHorizonUsesCpaDistance)
{
    cs::ManeuverActivationControllerParams params;
    params.cpa_horizon_s = 4.5;
    cs::ManeuverActivationController controller(params);
    ASSERT_TRUE(controller.update(sample(3'190'000, -1.0, 5.0, -1.0)).active);

    auto at_horizon = sample(3'200'000, 1.0, 4.5, -1.0);
    at_horizon.relative_positions_ned_m[1] = {4.5, 15.0, 0.0};
    at_horizon.relative_velocities_ned_mps[1] = {-1.0, 0.0, 0.0};
    const auto status = controller.update(at_horizon);
    EXPECT_FALSE(status.active);
    EXPECT_TRUE(status.just_deactivated);
}

TEST(ManeuverActivationController, UsesThreeDimensionalFutureCpaDistance)
{
    cs::ManeuverActivationController controller;
    auto activation = sample(3'200'000, -1.0, 5.0, -1.0);
    activation.relative_positions_ned_m[1] = {8.0, 15.0, 0.0};
    activation.relative_velocities_ned_mps[1] = {-2.0, 0.0, 0.0};
    ASSERT_TRUE(controller.update(activation).active);

    auto future_cpa_clear = sample(3'300'000, 1.0, 8.0, -2.0);
    future_cpa_clear.relative_positions_ned_m[1] = {8.0, 15.0, 0.0};
    future_cpa_clear.relative_velocities_ned_mps[1] = {-2.0, 0.0, 0.0};
    const auto status = controller.update(future_cpa_clear);
    EXPECT_FALSE(status.active);
    EXPECT_TRUE(status.just_deactivated);
}

TEST(ManeuverActivationController, UsesStrictActivationCriterionBoundary)
{
    cs::ManeuverActivationController controller;
    ASSERT_TRUE(controller.update(sample(3'400'000, -1.0, 5.0, -1.0)).active);
    const auto on_boundary = controller.update(
        sample(3'500'000, 1.0, 10.0, 0.0));
    EXPECT_TRUE(on_boundary.active);
    EXPECT_FALSE(on_boundary.just_deactivated);
}

TEST(ManeuverActivationController, AddsNewUnsafeThreatDuringActiveEpisode)
{
    cs::ManeuverActivationController controller;
    ASSERT_TRUE(controller.update(sample(3'600'000, -1.0, 5.0, -1.0)).active);

    auto new_threat = sample(3'700'000, -1.0, 20.0, 1.0);
    new_threat.unsafe_threat_mask = std::uint32_t{1} << 2;
    new_threat.activation_criteria_m[2] = 10.0;
    new_threat.relative_positions_ned_m[2] = {5.0, 0.0, 0.0};
    new_threat.relative_velocities_ned_mps[2] = {-1.0, 0.0, 0.0};
    new_threat.allow_new_activation = false;
    const auto retained = controller.update(new_threat);
    EXPECT_TRUE(retained.active);
    EXPECT_EQ(
        retained.affected_threat_mask,
        (std::uint32_t{1} << 1) | (std::uint32_t{1} << 2));
}

TEST(ManeuverActivationController, ElapsedTimeCannotDeactivateClosingConflict)
{
    cs::ManeuverActivationController controller;
    ASSERT_TRUE(controller.update(sample(3'000'000, -1.0, 5.0, -1.0)).active);
    EXPECT_TRUE(controller.update(sample(7'500'000, -1.0, 5.0, -1.0)).active);
    const auto status = controller.update(
        sample(13'000'000, -1.0, 5.0, -1.0));
    EXPECT_TRUE(status.active);
    EXPECT_FALSE(status.just_deactivated);
    EXPECT_EQ(status.deactivation_reason, cs::ManeuverDeactivationReason::None);
}

TEST(ManeuverActivationController, InvalidUpdatesCannotInterruptActiveCommand)
{
    cs::ManeuverActivationController controller;
    ASSERT_TRUE(controller.update(sample(4'000'000, -1.0, 5.0, -1.0)).active);
    auto invalid = sample(4'100'000, 1.0, 100.0, 1.0, 6);
    invalid.valid = false;
    const auto status = controller.update(invalid);
    EXPECT_TRUE(status.active);
    EXPECT_EQ(status.latched_candidate_id, 1U);
}

TEST(ManeuverActivationController,
    ProjectReconstructionImmediatelyRearmsWhenConflictPersists)
{
    cs::ManeuverActivationController controller;
    ASSERT_TRUE(controller.update(sample(5'000'000, -1.0, 5.0, -1.0)).active);
    const auto ended = controller.update(sample(5'100'000, -0.5, 20.0, 1.0));
    ASSERT_TRUE(ended.just_deactivated);

    const auto still_penetrating = controller.update(
        sample(5'150'000, -2.0, 20.0, 1.0, 6));
    EXPECT_TRUE(still_penetrating.active);
    EXPECT_TRUE(still_penetrating.just_activated);
    EXPECT_EQ(still_penetrating.latched_candidate_id, 6U);
}

TEST(ManeuverActivationController, UsesStrictZeroAdActivationBoundary)
{
    cs::ManeuverActivationController controller;
    EXPECT_FALSE(controller.update(sample(6'000'000, 0.1, 5.0, -1.0)).active);
    EXPECT_FALSE(controller.update(sample(6'050'000, 0.0, 5.0, -1.0)).active);
    const auto activated = controller.update(
        sample(6'100'000, -1.0e-9, 5.0, -1.0));
    EXPECT_TRUE(activated.active);
    EXPECT_TRUE(activated.just_activated);
}

TEST(ManeuverActivationController,
    CoordinatedReplacementPreservesActiveEpisode)
{
    cs::ManeuverActivationController controller;
    const auto activated = controller.update(
        sample(7'000'000, -1.0, 5.0, -1.0, 1));
    ASSERT_TRUE(activated.active);
    const auto activation_timestamp = activated.activation_timestamp_us;
    const auto affected_mask = activated.affected_threat_mask;

    ce::PredictInput replacement;
    replacement.V_cmd = 22.0;
    replacement.a_lat_cmd = -4.0;
    ASSERT_TRUE(controller.replaceActiveCommand(6, 2006, replacement));

    const auto status = controller.status();
    EXPECT_TRUE(status.active);
    EXPECT_FALSE(status.just_activated);
    EXPECT_FALSE(status.just_deactivated);
    EXPECT_EQ(status.activation_timestamp_us, activation_timestamp);
    EXPECT_EQ(status.affected_threat_mask, affected_mask);
    EXPECT_EQ(status.latched_candidate_id, 6U);
    EXPECT_EQ(status.latched_candidate_input_revision, 2006U);
    EXPECT_DOUBLE_EQ(status.latched_input.V_cmd, 22.0);
    EXPECT_DOUBLE_EQ(status.latched_input.a_lat_cmd, -4.0);
}

TEST(ManeuverActivationController, CannotReplaceInactiveCommand)
{
    cs::ManeuverActivationController controller;
    ce::PredictInput replacement;
    EXPECT_FALSE(controller.replaceActiveCommand(2, 1002, replacement));
    EXPECT_FALSE(controller.status().active);
}
