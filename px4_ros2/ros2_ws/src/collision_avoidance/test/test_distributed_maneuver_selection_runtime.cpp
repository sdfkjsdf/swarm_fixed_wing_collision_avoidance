#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <collision_avoidance/communication/DistributedManeuverSelectionRuntime.hpp>
#include <collision_avoidance/msg/trajectory_intent.hpp>
#include <px4_msgs/msg/estimator_trajectory_belief.hpp>

namespace cc = collision_avoidance::communication;
namespace ce = collision_avoidance::estimation;
namespace cs = collision_avoidance::selection;

namespace
{

px4_msgs::msg::EstimatorTrajectoryBelief beliefMessage(
    std::uint64_t timestamp_us,
    double north,
    double velocity_north)
{
    px4_msgs::msg::EstimatorTrajectoryBelief message;
    message.timestamp = timestamp_us;
    message.timestamp_sample = timestamp_us;
    message.attitude_q = {1.0F, 0.0F, 0.0F, 0.0F};
    message.velocity = {
        static_cast<float>(velocity_north), 0.0F, 0.0F};
    message.position = {static_cast<float>(north), 0.0F, -100.0F};
    message.valid = true;

    std::size_t packed_index = 0;
    for (std::size_t row = 0; row < ce::kEstimatorBeliefDimension; ++row) {
        for (std::size_t column = row;
             column < ce::kEstimatorBeliefDimension; ++column) {
            message.covariance_upper_triangle[packed_index++] =
                row == column ? 0.01F : 0.0F;
        }
    }
    return message;
}

void spinFor(
    rclcpp::executors::SingleThreadedExecutor & executor,
    std::chrono::milliseconds duration)
{
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

}  // namespace

TEST(DistributedManeuverSelectionRuntime, ExchangesIntentsAndScoresIndependently)
{
    if (!rclcpp::ok()) {
        int argc = 0;
        char ** argv = nullptr;
        rclcpp::init(argc, argv);
    }

    auto node = std::make_shared<rclcpp::Node>(
        "distributed_maneuver_selection_runtime_test");
    auto belief_a_publisher =
        node->create_publisher<px4_msgs::msg::EstimatorTrajectoryBelief>(
            "/common/px4_0/trans_estimator_trajectory_belief",
            rclcpp::SensorDataQoS());
    auto belief_b_publisher =
        node->create_publisher<px4_msgs::msg::EstimatorTrajectoryBelief>(
            "/common/px4_1/trans_estimator_trajectory_belief",
            rclcpp::SensorDataQoS());

    std::size_t intent_a_count = 0;
    std::size_t intent_b_count = 0;
    auto intent_a_subscription =
        node->create_subscription<collision_avoidance::msg::TrajectoryIntent>(
            "/common/px4_0/trajectory_intent",
            rclcpp::SensorDataQoS(),
            [&intent_a_count](
                collision_avoidance::msg::TrajectoryIntent::ConstSharedPtr) {
                ++intent_a_count;
            });
    auto intent_b_subscription =
        node->create_subscription<collision_avoidance::msg::TrajectoryIntent>(
            "/common/px4_1/trajectory_intent",
            rclcpp::SensorDataQoS(),
            [&intent_b_count](
                collision_avoidance::msg::TrajectoryIntent::ConstSharedPtr) {
                ++intent_b_count;
            });

    cs::ManeuverSelectionWorkerParams params;
    params.predictor_params.V_min = 10.0;
    params.predictor_params.V_max = 25.0;
    params.evaluator_params.desired_separation_distance_m = 10.0;
    params.evaluator_params.ownship_half_wingspan_m = 1.072;
    params.evaluator_params.threat_half_wingspan_m = 1.072;

    std::optional<cs::ManeuverSelectionDecision> decision_a;
    std::optional<cs::ManeuverSelectionDecision> decision_b;
    auto runtime_a = std::make_unique<cc::DistributedManeuverSelectionRuntime>(
        *node, 0, 2, params,
        [&decision_a](const cs::ManeuverSelectionDecision & decision) {
            decision_a = decision;
        });
    auto runtime_b = std::make_unique<cc::DistributedManeuverSelectionRuntime>(
        *node, 1, 2, params,
        [&decision_b](const cs::ManeuverSelectionDecision & decision) {
            decision_b = decision;
        });
    ASSERT_TRUE(runtime_a->enabled());
    ASSERT_TRUE(runtime_b->enabled());

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    spinFor(executor, std::chrono::milliseconds(30));

    constexpr std::uint64_t start = 10'000'000ULL;
    for (const std::uint64_t offset : {
             0ULL, 50'000ULL, 100'000ULL,
             150'000ULL, 200'000ULL, 250'000ULL}) {
        const double elapsed_s = static_cast<double>(offset) * 1.0e-6;
        belief_a_publisher->publish(beliefMessage(
            start + offset, -45.0 + 20.0 * elapsed_s, 20.0));
        belief_b_publisher->publish(beliefMessage(
            start + offset, 45.0 - 20.0 * elapsed_s, -20.0));
        spinFor(executor, std::chrono::milliseconds(35));
    }

    ASSERT_TRUE(decision_a.has_value());
    ASSERT_TRUE(decision_b.has_value());
    EXPECT_TRUE(decision_a->coordination_qualified);
    EXPECT_TRUE(decision_b->coordination_qualified);
    EXPECT_TRUE(decision_a->activation_requested);
    EXPECT_TRUE(decision_b->activation_requested);
    EXPECT_LT(decision_a->ad_m, 0.0);
    EXPECT_LT(decision_b->ad_m, 0.0);
    EXPECT_GE(intent_a_count, 15U);
    EXPECT_GE(intent_b_count, 15U);
    EXPECT_EQ(decision_a->local_selection_epoch, 1U);
    EXPECT_EQ(decision_b->local_selection_epoch, 1U);

    executor.remove_node(node);
    runtime_b.reset();
    runtime_a.reset();
    static_cast<void>(intent_a_subscription);
    static_cast<void>(intent_b_subscription);
    rclcpp::shutdown();
}
