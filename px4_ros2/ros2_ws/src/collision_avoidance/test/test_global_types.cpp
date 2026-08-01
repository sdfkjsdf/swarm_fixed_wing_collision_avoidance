#include <cmath>
#include <type_traits>

#include <gtest/gtest.h>

#include <collision_avoidance/common/GlobalTypes.hpp>

namespace types = collision_avoidance::types;

static_assert(types::kMaxAgents == 8);
static_assert(std::is_trivially_copyable_v<types::ControlState>);
static_assert(std::is_trivially_copyable_v<types::ControlSnapshot>);
static_assert(std::is_trivially_copyable_v<types::AgentState>);
static_assert(std::is_trivially_copyable_v<types::FwSetpoint>);

TEST(GlobalTypes, UsesSafeSetpointDefaults)
{
    const types::FwSetpoint setpoint{};

    EXPECT_FLOAT_EQ(setpoint.course, 0.0f);
    EXPECT_FLOAT_EQ(setpoint.airspeed, 0.0f);
    EXPECT_FLOAT_EQ(setpoint.height_rate, 0.0f);
    EXPECT_TRUE(std::isnan(setpoint.height_setpoint));
    EXPECT_FLOAT_EQ(setpoint.lateral_acceleration, 0.0f);
    EXPECT_TRUE(setpoint.is_fallback);
}

TEST(GlobalTypes, TransfersControlSnapshotsThroughSharedQueue)
{
    types::ControlInputQueue queue;
    types::ControlSnapshot input{};
    input.num_agents = 2;
    input.timestamp = 123.0;
    input.agents[1].position = {10.0f, 20.0f, -30.0f};

    ASSERT_TRUE(queue.try_push(input));
    const auto output = queue.try_pop();

    ASSERT_TRUE(output.has_value());
    EXPECT_EQ(output->num_agents, 2);
    EXPECT_DOUBLE_EQ(output->timestamp, 123.0);
    EXPECT_FLOAT_EQ(output->agents[1].position[0], 10.0f);
    EXPECT_FLOAT_EQ(output->agents[1].position[1], 20.0f);
    EXPECT_FLOAT_EQ(output->agents[1].position[2], -30.0f);
    EXPECT_FALSE(queue.try_pop().has_value());
}

TEST(GlobalTypes, TransfersSetpointsThroughSharedQueue)
{
    types::FwSetpointQueue queue;
    types::FwSetpoint input{};
    input.airspeed = 18.0f;
    input.height_rate = 1.5f;
    input.lateral_acceleration = -2.0f;
    input.is_fallback = false;

    ASSERT_TRUE(queue.try_push(input));
    const auto output = queue.try_pop();

    ASSERT_TRUE(output.has_value());
    EXPECT_FLOAT_EQ(output->airspeed, 18.0f);
    EXPECT_FLOAT_EQ(output->height_rate, 1.5f);
    EXPECT_FLOAT_EQ(output->lateral_acceleration, -2.0f);
    EXPECT_FALSE(output->is_fallback);
}
