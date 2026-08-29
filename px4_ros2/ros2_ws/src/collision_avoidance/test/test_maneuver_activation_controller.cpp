#include <gtest/gtest.h>

#include <limits>

#include <collision_avoidance/selection/ManeuverActivationController.hpp>

namespace cs = collision_avoidance::selection;

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
    const auto activated = controller.update(sample(1'000'000, -0.1, 0.0, 1));
    ASSERT_TRUE(activated.active);
    EXPECT_TRUE(activated.just_activated);
    EXPECT_EQ(activated.latched_candidate_id, 1U);
    EXPECT_EQ(activated.latched_candidate_input_revision, 1001U);

    const auto held = controller.update(sample(1'250'000, -5.0, 0.0, 6));
    EXPECT_TRUE(held.active);
    EXPECT_FALSE(held.just_activated);
    EXPECT_EQ(held.latched_candidate_id, 1U);
    EXPECT_EQ(held.latched_candidate_input_revision, 1001U);
    EXPECT_DOUBLE_EQ(held.latched_input.a_lat_cmd, 1.0);

    const auto positive_ad = controller.update(
        sample(1'500'000, 10.0, 5.0, 0));
    EXPECT_TRUE(positive_ad.active);
    EXPECT_FALSE(positive_ad.just_deactivated);
    EXPECT_EQ(positive_ad.latched_candidate_id, 1U);
}

TEST(ManeuverActivationController, DeactivatesWhenAffectedThreatSeparates)
{
    cs::ManeuverActivationController controller;
    ASSERT_TRUE(controller.update(sample(2'000'000, -1.0, 0.0)).active);
    const auto status = controller.update(sample(2'100'000, 1.0, 30.48));
    EXPECT_FALSE(status.active);
    EXPECT_TRUE(status.just_deactivated);
    EXPECT_EQ(
        status.deactivation_reason,
        cs::ManeuverDeactivationReason::Separating);
}

TEST(ManeuverActivationController, DeactivatesAtFourPointFiveSecondTimeout)
{
    cs::ManeuverActivationController controller;
    ASSERT_TRUE(controller.update(sample(3'000'000, -1.0, 0.0)).active);
    EXPECT_TRUE(controller.update(sample(7'499'999, 1.0, 0.0)).active);
    const auto status = controller.update(sample(7'500'000, -1.0, 0.0));
    EXPECT_FALSE(status.active);
    EXPECT_TRUE(status.just_deactivated);
    EXPECT_EQ(
        status.deactivation_reason,
        cs::ManeuverDeactivationReason::Timeout);
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

TEST(ManeuverActivationController, RequiresClearConeBeforeNewConflictRearms)
{
    cs::ManeuverActivationController controller;
    ASSERT_TRUE(controller.update(sample(5'000'000, -1.0, 0.0)).active);
    const auto ended = controller.update(sample(5'100'000, -0.5, 30.48));
    ASSERT_TRUE(ended.just_deactivated);

    const auto still_penetrating = controller.update(
        sample(5'150'000, -2.0, 0.0, 6));
    EXPECT_FALSE(still_penetrating.active);
    EXPECT_FALSE(still_penetrating.just_activated);

    const auto cleared = controller.update(sample(5'200'000, 0.1, 0.0, 6));
    EXPECT_FALSE(cleared.active);
    EXPECT_FALSE(cleared.just_activated);

    const auto new_conflict = controller.update(
        sample(5'250'000, -0.1, 0.0, 6));
    EXPECT_TRUE(new_conflict.active);
    EXPECT_TRUE(new_conflict.just_activated);
    EXPECT_EQ(new_conflict.latched_candidate_id, 6U);
}
