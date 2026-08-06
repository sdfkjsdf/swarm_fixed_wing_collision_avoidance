#include <gtest/gtest.h>

#include <array>

#include <collision_avoidance/coordinate/CommonNedTransform.hpp>

namespace cc = collision_avoidance::coordinate;

TEST(CommonNedTransform, ConvertsReferenceOffsetIntoCommonNedTranslation)
{
    const cc::CommonNedTransform transform({47.397742, 8.545594, 488.0});
    ASSERT_TRUE(transform.valid());

    std::array<double, 3> translation{};
    ASSERT_TRUE(transform.translationFrom(
        {47.397742, 8.545594, 488.83}, translation));
    EXPECT_NEAR(translation[0], 0.0, 1.0e-6);
    EXPECT_NEAR(translation[1], 0.0, 1.0e-6);
    EXPECT_NEAR(translation[2], -0.83, 1.0e-9);
}

TEST(CommonNedTransform, PreservesExpectedNorthEastAxisConvention)
{
    const cc::CommonNedTransform transform({47.397742, 8.545594, 488.0});
    std::array<double, 3> north_translation{};
    std::array<double, 3> east_translation{};
    ASSERT_TRUE(transform.translationFrom(
        {47.397752, 8.545594, 488.0}, north_translation));
    ASSERT_TRUE(transform.translationFrom(
        {47.397742, 8.545604, 488.0}, east_translation));
    EXPECT_GT(north_translation[0], 1.0);
    EXPECT_NEAR(north_translation[1], 0.0, 1.0e-6);
    EXPECT_GT(east_translation[1], 0.7);
    EXPECT_NEAR(east_translation[0], 0.0, 1.0e-6);
}

TEST(CommonNedTransform, RejectsInvalidReference)
{
    const cc::CommonNedTransform transform({47.397742, 8.545594, 488.0});
    std::array<double, 3> translation{};
    EXPECT_FALSE(transform.translationFrom({91.0, 8.0, 0.0}, translation));
}
