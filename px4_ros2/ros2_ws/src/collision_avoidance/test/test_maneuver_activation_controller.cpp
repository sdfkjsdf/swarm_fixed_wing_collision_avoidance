#include <gtest/gtest.h>

#include <collision_avoidance/selection/ManeuverActivationController.hpp>

namespace cs = collision_avoidance::selection;
namespace ce = collision_avoidance::estimation;

namespace
{
cs::ManeuverActivationSample sample(
    std::uint64_t timestamp_us, double ad_m, std::uint8_t candidate_id = 1)
{
    cs::ManeuverActivationSample value;
    value.timestamp_us = timestamp_us;
    value.valid = true;
    value.minimum_ad_m = ad_m;
    value.unsafe_threat_mask = std::uint32_t{1} << 1;
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
    const auto activated = controller.update(sample(1'000'000, -0.1));
    ASSERT_TRUE(activated.active);
    EXPECT_TRUE(activated.just_activated);
    const auto held = controller.update(sample(1'250'000, -5.0, 6));
    EXPECT_TRUE(held.active);
    EXPECT_FALSE(held.just_activated);
    EXPECT_EQ(held.latched_candidate_id, 1U);
    EXPECT_EQ(held.latched_candidate_input_revision, 1001U);
    EXPECT_DOUBLE_EQ(held.latched_input.a_lat_cmd, 1.0);
    EXPECT_TRUE(controller.update(sample(1'500'000, 10.0, 0)).active);
}

TEST(ManeuverActivationController, ReleasePermissionDefaultsToFalse)
{
    EXPECT_FALSE(cs::ManeuverActivationSample{}.allow_deactivation);
    cs::ManeuverActivationController controller;
    ASSERT_TRUE(controller.update(sample(2'000'000, -1.0)).active);
    EXPECT_TRUE(controller.update(sample(2'100'000, 100.0)).active);
}

TEST(ManeuverActivationController, ExplicitSafeNominalReturnReleasesWithoutCpa)
{
    cs::ManeuverActivationController controller;
    ASSERT_TRUE(controller.update(sample(2'000'000, -1.0)).active);
    auto release = sample(2'100'000, -0.5);
    // Current straight-line flight vectors intersect. They must not veto
    // the worker-certified, curved nominal return and transition trajectories.
    release.relative_positions_ned_m[1] = {20.0, 0.0, 0.0};
    release.relative_velocities_ned_mps[1] = {-10.0, 0.0, 0.0};
    release.allow_deactivation = true;
    const auto status = controller.update(release);
    EXPECT_FALSE(status.active);
    EXPECT_TRUE(status.just_deactivated);
    EXPECT_EQ(status.deactivation_reason,
              cs::ManeuverDeactivationReason::CoordinatedNominalReturnSafe);
}

TEST(ManeuverActivationController, FormationInhibitBlocksOnlyNewActivation)
{
    cs::ManeuverActivationController controller;
    auto inhibited = sample(2'150'000, -1.0);
    inhibited.allow_new_activation = false;
    EXPECT_FALSE(controller.update(inhibited).active);
    ASSERT_TRUE(controller.update(sample(2'250'000, -1.0)).active);
    auto release = sample(2'350'000, 1.0);
    release.allow_new_activation = false;
    release.allow_deactivation = true;
    EXPECT_TRUE(controller.update(release).just_deactivated);
}

TEST(ManeuverActivationController, CoordinatedTriggerStillNeedsValidInputAndThreat)
{
    cs::ManeuverActivationController controller;
    auto coordinated = sample(2'400'000, 5.0, 6);
    coordinated.coordinated_activation_requested = true;
    coordinated.unsafe_threat_mask = 0;
    EXPECT_FALSE(controller.update(coordinated).active);
    coordinated.unsafe_threat_mask = 2;
    coordinated.valid = false;
    EXPECT_FALSE(controller.update(coordinated).active);
    coordinated.valid = true;
    const auto status = controller.update(coordinated);
    ASSERT_TRUE(status.active);
    EXPECT_TRUE(status.just_activated);
    EXPECT_EQ(status.latched_candidate_id, 6U);
}

TEST(ManeuverActivationController, ElapsedTimeCannotGrantRelease)
{
    cs::ManeuverActivationController controller;
    ASSERT_TRUE(controller.update(sample(3'000'000, -1.0)).active);
    EXPECT_TRUE(controller.update(sample(7'500'000, 1.0)).active);
    const auto status = controller.update(sample(13'000'000, 1.0));
    EXPECT_TRUE(status.active);
    EXPECT_FALSE(status.just_deactivated);
    EXPECT_EQ(status.deactivation_reason, cs::ManeuverDeactivationReason::None);
}

TEST(ManeuverActivationController, InvalidOrOutOfOrderUpdateCannotRelease)
{
    cs::ManeuverActivationController controller;
    ASSERT_TRUE(controller.update(sample(4'000'000, -1.0)).active);
    auto invalid = sample(4'100'000, 1.0, 6);
    invalid.valid = false;
    invalid.allow_deactivation = true;
    EXPECT_TRUE(controller.update(invalid).active);
    invalid.valid = true;
    invalid.timestamp_us = 4'050'000;
    const auto status = controller.update(invalid);
    EXPECT_TRUE(status.active);
    EXPECT_EQ(status.latched_candidate_id, 1U);
}

TEST(ManeuverActivationController, ImmediatelyRearmsWhenNewConflictPersists)
{
    cs::ManeuverActivationController controller;
    ASSERT_TRUE(controller.update(sample(5'000'000, -1.0)).active);
    auto release = sample(5'100'000, -0.5);
    release.allow_deactivation = true;
    ASSERT_TRUE(controller.update(release).just_deactivated);
    const auto next = controller.update(sample(5'150'000, -2.0, 6));
    EXPECT_TRUE(next.active);
    EXPECT_TRUE(next.just_activated);
    EXPECT_EQ(next.latched_candidate_id, 6U);
}

TEST(ManeuverActivationController, UsesStrictZeroAdActivationBoundary)
{
    cs::ManeuverActivationController controller;
    EXPECT_FALSE(controller.update(sample(6'000'000, 0.1)).active);
    EXPECT_FALSE(controller.update(sample(6'050'000, 0.0)).active);
    EXPECT_TRUE(controller.update(sample(6'100'000, -1.0e-9)).just_activated);
}

TEST(ManeuverActivationController, CoordinatedReplacementPreservesActiveEpisode)
{
    cs::ManeuverActivationController controller;
    const auto activated = controller.update(sample(7'000'000, -1.0));
    ASSERT_TRUE(activated.active);
    ce::PredictInput replacement;
    replacement.V_cmd = 22.0;
    replacement.a_lat_cmd = -4.0;
    ASSERT_TRUE(controller.replaceActiveCommand(6, 2006, replacement));
    const auto status = controller.status();
    EXPECT_TRUE(status.active);
    EXPECT_FALSE(status.just_activated);
    EXPECT_FALSE(status.just_deactivated);
    EXPECT_EQ(status.activation_timestamp_us, activated.activation_timestamp_us);
    EXPECT_EQ(status.latched_candidate_id, 6U);
    EXPECT_EQ(status.latched_candidate_input_revision, 2006U);
    EXPECT_DOUBLE_EQ(status.latched_input.V_cmd, 22.0);
    EXPECT_DOUBLE_EQ(status.latched_input.a_lat_cmd, -4.0);
}

TEST(ManeuverActivationController, CannotReplaceInactiveCommand)
{
    cs::ManeuverActivationController controller;
    EXPECT_FALSE(controller.replaceActiveCommand(2, 1002, ce::PredictInput{}));
    EXPECT_FALSE(controller.status().active);
}

TEST(ManeuverActivationController, ResetClearsEpisodeAndTimestamp)
{
    cs::ManeuverActivationController controller;
    ASSERT_TRUE(controller.update(sample(8'000'000, -1.0)).active);
    controller.reset();
    EXPECT_FALSE(controller.status().active);
    EXPECT_TRUE(controller.update(sample(1'000'000, -1.0)).just_activated);
}
