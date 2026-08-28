#include <gtest/gtest.h>

#include <chrono>
#include <array>
#include <cmath>
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
    double velocity_north,
    double east = 0.0,
    double velocity_east = 0.0)
{
    px4_msgs::msg::EstimatorTrajectoryBelief message;
    message.timestamp = timestamp_us;
    message.timestamp_sample = timestamp_us;
    message.attitude_q = {1.0F, 0.0F, 0.0F, 0.0F};
    message.velocity = {
        static_cast<float>(velocity_north),
        static_cast<float>(velocity_east), 0.0F};
    message.position = {
        static_cast<float>(north), static_cast<float>(east), -100.0F};
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
    runtime_a->setActivationEnabled(true);
    runtime_b->setActivationEnabled(true);

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
    spinFor(executor, std::chrono::milliseconds(120));

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
    EXPECT_EQ(decision_a->local_selection_epoch, 40U);
    EXPECT_EQ(decision_b->local_selection_epoch, 40U);

    executor.remove_node(node);
    runtime_b.reset();
    runtime_a.reset();
    static_cast<void>(intent_a_subscription);
    static_cast<void>(intent_b_subscription);
    rclcpp::shutdown();
}

TEST(DistributedManeuverSelectionRuntime, FiveRuntimesPublishSameJointDecision)
{
    if (!rclcpp::ok()) {
        int argc = 0;
        char ** argv = nullptr;
        rclcpp::init(argc, argv);
    }

    constexpr std::size_t aircraft_count = 5;
    constexpr double radius_m = 45.0;
    constexpr double speed_mps = 20.0;
    auto node = std::make_shared<rclcpp::Node>(
        "distributed_five_aircraft_runtime_test");
    std::array<rclcpp::Publisher<
        px4_msgs::msg::EstimatorTrajectoryBelief>::SharedPtr, aircraft_count>
        belief_publishers;
    for (std::size_t aircraft = 0; aircraft < aircraft_count; ++aircraft) {
        belief_publishers[aircraft] = node->create_publisher<
            px4_msgs::msg::EstimatorTrajectoryBelief>(
            "/common/px4_" + std::to_string(aircraft)
                + "/trans_estimator_trajectory_belief",
            rclcpp::SensorDataQoS());
    }

    cs::ManeuverSelectionWorkerParams params;
    params.predictor_params.V_min = 10.0;
    params.predictor_params.V_max = 25.0;
    params.evaluator_params.desired_separation_distance_m = 10.0;
    params.evaluator_params.ownship_half_wingspan_m = 1.072;
    params.evaluator_params.threat_half_wingspan_m = 1.072;

    std::array<std::optional<cs::ManeuverSelectionDecision>, aircraft_count>
        decisions;
    std::array<std::unique_ptr<cc::DistributedManeuverSelectionRuntime>,
        aircraft_count> runtimes;
    for (std::size_t aircraft = 0; aircraft < aircraft_count; ++aircraft) {
        runtimes[aircraft] = std::make_unique<
            cc::DistributedManeuverSelectionRuntime>(
            *node, static_cast<int>(aircraft), aircraft_count, params,
            [&decisions, aircraft](
                const cs::ManeuverSelectionDecision & decision) {
                decisions[aircraft] = decision;
            });
        ASSERT_TRUE(runtimes[aircraft]->enabled());
    }

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    spinFor(executor, std::chrono::milliseconds(40));

    constexpr std::uint64_t start = 20'000'000ULL;
    for (const std::uint64_t offset : {
             0ULL, 50'000ULL, 100'000ULL,
             150'000ULL, 200'000ULL, 250'000ULL}) {
        const double elapsed_s = static_cast<double>(offset) * 1.0e-6;
        for (std::size_t aircraft = 0;
             aircraft < aircraft_count; ++aircraft) {
            const double angle = 2.0 * M_PI * static_cast<double>(aircraft)
                / static_cast<double>(aircraft_count);
            const double unit_north = std::cos(angle);
            const double unit_east = std::sin(angle);
            belief_publishers[aircraft]->publish(beliefMessage(
                start + offset,
                (radius_m - speed_mps * elapsed_s) * unit_north,
                -speed_mps * unit_north,
                (radius_m - speed_mps * elapsed_s) * unit_east,
                -speed_mps * unit_east));
        }
        spinFor(executor, std::chrono::milliseconds(80));
    }
    spinFor(executor, std::chrono::milliseconds(120));

    ASSERT_TRUE(decisions[0].has_value());
    const auto expected_tuple = decisions[0]->selected_candidate_ids;
    for (std::size_t aircraft = 0; aircraft < aircraft_count; ++aircraft) {
        ASSERT_TRUE(decisions[aircraft].has_value());
        EXPECT_TRUE(decisions[aircraft]->coordination_qualified);
        EXPECT_EQ(decisions[aircraft]->aircraft_count, aircraft_count);
        EXPECT_EQ(decisions[aircraft]->evaluated_combination_count, 243U);
        EXPECT_EQ(decisions[aircraft]->selected_candidate_ids, expected_tuple);
    }

    executor.remove_node(node);
    for (auto & runtime : runtimes) {
        runtime.reset();
    }
    rclcpp::shutdown();
}
