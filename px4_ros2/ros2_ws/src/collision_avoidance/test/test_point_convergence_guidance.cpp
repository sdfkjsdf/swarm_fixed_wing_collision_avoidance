#include <gtest/gtest.h>

#include <cmath>

#include <collision_avoidance/guidance/PointConvergenceGuidance.hpp>

namespace cg = collision_avoidance::guidance;
namespace ct = collision_avoidance::types;

TEST(PointConvergenceGuidance, CommandsHorizontalTurnTowardTarget)
{
    cg::PointConvergenceGuidance::Parameters params;
    params.target_north_m = 100.0F;
    params.target_east_m = 100.0F;
    params.ground_speed_command_mps = 20.0F;
    cg::PointConvergenceGuidance guidance(params);

    ct::AgentState state;
    state.pos_n = 100.0F;
    state.pos_e = 0.0F;
    state.vel_n = 20.0F;
    state.psi = 0.0F;

    const auto output = guidance.computeFwSetpoint(state, -100.0F);
    EXPECT_GT(output.lateral_acceleration, 0.0F);
    EXPECT_FLOAT_EQ(output.airspeed, 20.0F);
    EXPECT_FLOAT_EQ(output.height_rate, 0.0F);
    EXPECT_FLOAT_EQ(output.height_setpoint, -100.0F);
    EXPECT_FALSE(output.is_fallback);
}

TEST(PointConvergenceGuidance, HoldsCourseInsideTargetRadius)
{
    cg::PointConvergenceGuidance::Parameters params;
    params.target_north_m = 1.0F;
    params.target_east_m = 1.0F;
    cg::PointConvergenceGuidance guidance(params);

    ct::AgentState state;
    state.pos_n = 1.2F;
    state.pos_e = 1.1F;
    state.vel_n = 10.0F;
    state.vel_e = 10.0F;
    state.psi = static_cast<float>(M_PI) / 4.0F;

    const auto output = guidance.computeFwSetpoint(state, -100.0F);
    EXPECT_NEAR(output.lateral_acceleration, 0.0F, 1.0e-5F);
}
