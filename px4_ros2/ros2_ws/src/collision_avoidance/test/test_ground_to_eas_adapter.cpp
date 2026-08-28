#include <cmath>

#include <gtest/gtest.h>

#include <collision_avoidance/control/GroundToEasAdapter.hpp>

namespace control = collision_avoidance::control;

TEST(GroundToEasAdapter, MatchesExistingWindTriangleCases)
{
    EXPECT_FLOAT_EQ(
        control::computeRequiredEquivalentAirspeed(
            20.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
            control::kSeaLevelAirDensityKgM3),
        20.0F);
    EXPECT_FLOAT_EQ(
        control::computeRequiredEquivalentAirspeed(
            20.0F, 0.0F, 0.0F, 5.0F, 0.0F, 0.0F,
            control::kSeaLevelAirDensityKgM3),
        15.0F);
    EXPECT_FLOAT_EQ(
        control::computeRequiredEquivalentAirspeed(
            20.0F, 0.0F, 0.0F, -5.0F, 0.0F, 0.0F,
            control::kSeaLevelAirDensityKgM3),
        25.0F);
    EXPECT_FLOAT_EQ(
        control::computeRequiredEquivalentAirspeed(
            20.0F, 0.0F, 0.0F, 0.0F, 4.0F, 0.0F,
            control::kSeaLevelAirDensityKgM3),
        std::sqrt(416.0F));
}

TEST(GroundToEasAdapter, IsEquivalentToLegacyImplementation)
{
    const auto legacy = [](float ground_speed, float course,
                           float wind_north, float wind_east) {
        const float gs_n = ground_speed * std::cos(course);
        const float gs_e = ground_speed * std::sin(course);
        const float as_n = gs_n - wind_north;
        const float as_e = gs_e - wind_east;
        return std::sqrt(as_n * as_n + as_e * as_e);
    };

    EXPECT_EQ(
        control::computeRequiredEquivalentAirspeed(
            18.0F, 0.7F, 0.0F, 3.0F, -2.0F, 0.0F,
            control::kSeaLevelAirDensityKgM3),
        legacy(18.0F, 0.7F, 3.0F, -2.0F));
}

TEST(GroundToEasAdapter, IncludesVerticalGroundAndWindComponents)
{
    EXPECT_FLOAT_EQ(
        control::computeRequiredEquivalentAirspeed(
            25.0F, 0.0F, 15.0F, 4.0F, 3.0F, 0.0F,
            control::kSeaLevelAirDensityKgM3),
        std::sqrt(490.0F));
    EXPECT_FLOAT_EQ(
        control::computeRequiredEquivalentAirspeed(
            25.0F, 0.0F, -15.0F, 4.0F, 3.0F, 0.0F,
            control::kSeaLevelAirDensityKgM3),
        std::sqrt(490.0F));
    EXPECT_FLOAT_EQ(
        control::computeRequiredEquivalentAirspeed(
            25.0F, 0.0F, 15.0F, 4.0F, 3.0F, 2.0F,
            control::kSeaLevelAirDensityKgM3),
        std::sqrt(554.0F));
}

TEST(GroundToEasAdapter, ConvertsTrueToEquivalentAirspeedUsingDensity)
{
    EXPECT_FLOAT_EQ(
        control::computeRequiredEquivalentAirspeed(
            20.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
            control::kSeaLevelAirDensityKgM3 / 4.0F),
        10.0F);
}

TEST(GroundToEasAdapter, RejectsPhysicallyInvalidInputs)
{
    EXPECT_TRUE(std::isnan(control::computeRequiredEquivalentAirspeed(
        10.0F, 0.0F, 11.0F, 0.0F, 0.0F, 0.0F,
        control::kSeaLevelAirDensityKgM3)));
    EXPECT_TRUE(std::isnan(control::computeRequiredEquivalentAirspeed(
        10.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F)));
    EXPECT_FLOAT_EQ(
        control::computeRequiredEquivalentAirspeed(
            10.0F, 0.0F, 10.0F, 0.0F, 0.0F, 0.0F,
            control::kSeaLevelAirDensityKgM3),
        10.0F);
}

TEST(GroundToEasAdapter, AppliesCoordinatedTurnMinimumEquivalentAirspeed)
{
    constexpr float gravity = 9.80665F;
    constexpr float minimum_level_eas = 12.0F;

    EXPECT_FLOAT_EQ(
        control::applyTurnMinimumEquivalentAirspeed(
            20.0F, minimum_level_eas, gravity, gravity),
        20.0F);

    const float expected_45_degree_minimum =
        minimum_level_eas * std::sqrt(std::sqrt(2.0F));
    EXPECT_NEAR(
        control::applyTurnMinimumEquivalentAirspeed(
            10.0F, minimum_level_eas, gravity, gravity),
        expected_45_degree_minimum,
        1.0e-5F);
    EXPECT_NEAR(
        control::applyTurnMinimumEquivalentAirspeed(
            10.0F, minimum_level_eas, -gravity, gravity),
        expected_45_degree_minimum,
        1.0e-5F);
}

TEST(GroundToEasAdapter, RejectsInvalidTurnMinimumInputs)
{
    EXPECT_TRUE(std::isnan(control::applyTurnMinimumEquivalentAirspeed(
        NAN, 12.0F, 0.0F, 9.80665F)));
    EXPECT_TRUE(std::isnan(control::applyTurnMinimumEquivalentAirspeed(
        20.0F, -1.0F, 0.0F, 9.80665F)));
    EXPECT_TRUE(std::isnan(control::applyTurnMinimumEquivalentAirspeed(
        20.0F, 12.0F, 0.0F, 0.0F)));
}
