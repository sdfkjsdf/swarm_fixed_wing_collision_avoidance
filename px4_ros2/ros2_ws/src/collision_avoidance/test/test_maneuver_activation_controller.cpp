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
    double separation_rate_mps,
    std::uint8_t candidate_id = 1)
{
    cs::ManeuverActivationSample value;
    value.timestamp_us = timestamp_us;
    value.valid = true;
    value.minimum_ad_m = ad_m;
    value.unsafe_threat_mask = std::uint32_t{1} << 1;
    value.separation_rates_mps.fill(
        std::numeric_limits<double>::quiet_NaN());
    value.separation_rates_mps[1] = separation_rate_mps;
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
    const auto activated = controller.update(sample(1'000'000, -0.1, -0.1, 1));
    ASSERT_TRUE(activated.active);
    EXPECT_TRUE(activated.just_activated);
    EXPECT_EQ(activated.latched_candidate_id, 1U);
    EXPECT_EQ(activated.latched_candidate_input_revision, 1001U);

    const auto held = controller.update(sample(1'250'000, -5.0, -0.1, 6));
    EXPECT_TRUE(held.active);
    EXPECT_FALSE(held.just_activated);
    EXPECT_EQ(held.latched_candidate_id, 1U);
    EXPECT_EQ(held.latched_candidate_input_revision, 1001U);
    EXPECT_DOUBLE_EQ(held.latched_input.a_lat_cmd, 1.0);

    const auto positive_ad = controller.update(
        sample(1'500'000, 10.0, -0.1, 0));
    EXPECT_TRUE(positive_ad.active);
    EXPECT_FALSE(positive_ad.just_deactivated);
    EXPECT_EQ(positive_ad.latched_candidate_id, 1U);
}

TEST(ManeuverActivationController, DeactivatesWhenAffectedThreatSeparates)
{
    cs::ManeuverActivationController controller;
    ASSERT_TRUE(controller.update(sample(2'000'000, -1.0, -0.1)).active);
    const auto status = controller.update(sample(2'100'000, 1.0, 0.0));
    EXPECT_FALSE(status.active);
    EXPECT_TRUE(status.just_deactivated);
    EXPECT_EQ(
        status.deactivation_reason,
        cs::ManeuverDeactivationReason::Separating);
}

TEST(ManeuverActivationController, RequiresEveryAffectedPairToBeNonClosing)
{
    cs::ManeuverActivationController controller;
    auto activation = sample(2'500'000, -1.0, -0.1);
    activation.unsafe_threat_mask =
        (std::uint32_t{1} << 1) | (std::uint32_t{1} << 2);
    activation.separation_rates_mps[2] = -0.2;
    ASSERT_TRUE(controller.update(activation).active);

    auto one_still_closing = sample(2'600'000, 1.0, 0.0);
    one_still_closing.separation_rates_mps[2] = -1.0e-6;
    EXPECT_TRUE(controller.update(one_still_closing).active);

    auto all_non_closing = sample(2'700'000, 1.0, 0.0);
    all_non_closing.separation_rates_mps[2] = 0.0;
    const auto ended = controller.update(all_non_closing);
    EXPECT_FALSE(ended.active);
    EXPECT_TRUE(ended.just_deactivated);
}

TEST(ManeuverActivationController, ElapsedTimeCannotDeactivateClosingConflict)
{
    cs::ManeuverActivationController controller;
    ASSERT_TRUE(controller.update(sample(3'000'000, -1.0, -0.1)).active);
    EXPECT_TRUE(controller.update(sample(7'500'000, -1.0, -0.1)).active);
    const auto status = controller.update(
        sample(13'000'000, -1.0, -0.1));
    EXPECT_TRUE(status.active);
    EXPECT_FALSE(status.just_deactivated);
    EXPECT_EQ(status.deactivation_reason, cs::ManeuverDeactivationReason::None);
}

TEST(ManeuverActivationController, InvalidUpdatesCannotInterruptActiveCommand)
{
    cs::ManeuverActivationController controller;
    ASSERT_TRUE(controller.update(sample(4'000'000, -1.0, 0.0)).active);
    auto invalid = sample(4'100'000, 1.0, 100.0, 6);
    invalid.valid = false;
    const auto status = controller.update(invalid);
    EXPECT_TRUE(status.active);
    EXPECT_EQ(status.latched_candidate_id, 1U);
}

TEST(ManeuverActivationController,
    ReactivatesOnNextEvaluationWhenConflictPersists)
{
    cs::ManeuverActivationController controller;
    ASSERT_TRUE(controller.update(sample(5'000'000, -1.0, -0.1)).active);
    const auto ended = controller.update(sample(5'100'000, -0.5, 0.0));
    ASSERT_TRUE(ended.just_deactivated);

    const auto still_penetrating = controller.update(
        sample(5'150'000, -2.0, 0.0, 6));
    EXPECT_TRUE(still_penetrating.active);
    EXPECT_TRUE(still_penetrating.just_activated);
    EXPECT_EQ(still_penetrating.latched_candidate_id, 6U);
}

TEST(ManeuverActivationController, UsesStrictZeroAdActivationBoundary)
{
    cs::ManeuverActivationController controller;
    EXPECT_FALSE(controller.update(sample(6'000'000, 0.1, -0.1)).active);
    EXPECT_FALSE(controller.update(sample(6'050'000, 0.0, -0.1)).active);
    const auto activated = controller.update(sample(6'100'000, -1.0e-9, -0.1));
    EXPECT_TRUE(activated.active);
    EXPECT_TRUE(activated.just_activated);
}

TEST(ManeuverActivationController,
    CoordinatedReplacementPreservesActiveEpisode)
{
    cs::ManeuverActivationController controller;
    const auto activated = controller.update(
        sample(7'000'000, -1.0, -0.1, 1));
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
