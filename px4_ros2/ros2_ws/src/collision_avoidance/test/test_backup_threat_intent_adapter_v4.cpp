#include <collision_avoidance/selection/BackupThreatIntentAdapterV4.hpp>

#include <cmath>

#include <gtest/gtest.h>

namespace cs = collision_avoidance::selection;
namespace ce = collision_avoidance::estimation;

namespace
{

ce::ReceivedTrajectoryIntent straightIntent(std::uint64_t timestamp_us)
{
    ce::ReceivedTrajectoryIntent intent;
    intent.source_timestamp_us = timestamp_us;
    intent.candidate_input = {20.0, 100.0, 0.0, 0.0};
    for (std::size_t index = 0;
         index < intent.reconstructed_mean.size(); ++index) {
        const double time_s = static_cast<double>(index)
            * ce::kTrajectoryIntentStepSeconds;
        intent.reconstructed_mean[index] = {
            20.0 * time_s, 0.0, 100.0, 20.0, 0.0, 0.0, 0.0};
    }
    return intent;
}

}  // namespace

TEST(BackupThreatIntentAdapterV4, AlignsThenBuildsFullCommonTimeHorizon)
{
    const cs::BackupThreatIntentAdapterV4 adapter;
    const auto result = adapter.alignAndPropagate(
        1'200'000, 4, 2.5, straightIntent(1'000'000));

    ASSERT_EQ(result.status, cs::BackupThreatIntentStatusV4::Valid);
    EXPECT_EQ(result.source_age_us, 200'000U);
    EXPECT_EQ(result.trajectory.source_timestamp_us, 1'200'000U);
    EXPECT_EQ(result.trajectory.vehicle_id, 4);
    EXPECT_DOUBLE_EQ(result.trajectory.physical_clearance_m, 2.5);
    ASSERT_EQ(result.trajectory.point_count, 46U);
    EXPECT_NEAR(result.trajectory.points.front().north_m, 4.0, 1.0e-12);
    EXPECT_NEAR(result.trajectory.points.front().time_offset_s, 0.0, 1.0e-12);
    EXPECT_NEAR(result.trajectory.points[45].time_offset_s, 4.5, 1.0e-12);
    EXPECT_NEAR(result.trajectory.points[45].north_m, 94.0, 1.0e-8);
    EXPECT_NEAR(result.trajectory.points[45].east_m, 0.0, 1.0e-12);
}

TEST(BackupThreatIntentAdapterV4, RejectsFutureAndStaleIntents)
{
    cs::BackupThreatIntentAdapterV4Params params;
    params.maximum_intent_age_us = 250'000;
    const cs::BackupThreatIntentAdapterV4 adapter(params);

    EXPECT_EQ(
        adapter.alignAndPropagate(
            900'000, 1, 2.0, straightIntent(1'000'000)).status,
        cs::BackupThreatIntentStatusV4::FutureIntent);
    EXPECT_EQ(
        adapter.alignAndPropagate(
            1'300'001, 1, 2.0, straightIntent(1'000'000)).status,
        cs::BackupThreatIntentStatusV4::StaleIntent);
}

TEST(BackupThreatIntentAdapterV4, RejectsInvalidTransmittedCommand)
{
    auto intent = straightIntent(1'000'000);
    intent.candidate_input.V_cmd = std::nan("");
    const auto result = cs::BackupThreatIntentAdapterV4{}.alignAndPropagate(
        1'000'000, 1, 2.0, intent);
    EXPECT_EQ(result.status, cs::BackupThreatIntentStatusV4::InvalidIntent);
}
