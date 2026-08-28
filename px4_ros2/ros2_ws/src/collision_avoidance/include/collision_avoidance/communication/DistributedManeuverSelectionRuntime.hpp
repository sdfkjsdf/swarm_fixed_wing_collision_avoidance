#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/estimator_trajectory_belief.hpp>

#include <collision_avoidance/communication/TrajectoryIntentTransport.hpp>
#include <collision_avoidance/msg/maneuver_selection_decision.hpp>
#include <collision_avoidance/selection/ManeuverSelectionWorker.hpp>

namespace collision_avoidance::communication
{

class DistributedManeuverSelectionRuntime
{
public:
    using DecisionCallback =
        std::function<void(const selection::ManeuverSelectionDecision &)>;

    DistributedManeuverSelectionRuntime(
        rclcpp::Node & node,
        int vehicle_id,
        int total_agent_count,
        const selection::ManeuverSelectionWorkerParams & worker_params,
        DecisionCallback decision_callback);
    ~DistributedManeuverSelectionRuntime();

    DistributedManeuverSelectionRuntime(
        const DistributedManeuverSelectionRuntime &) = delete;
    DistributedManeuverSelectionRuntime & operator=(
        const DistributedManeuverSelectionRuntime &) = delete;

    bool enabled() const noexcept;
    void setActivationEnabled(bool enabled) noexcept;

private:
    void onBelief(
        const px4_msgs::msg::EstimatorTrajectoryBelief & message);
    void drainWorkerOutput();

    rclcpp::Node & m_node;
    int m_vehicle_id{0};
    int m_total_agent_count{0};
    bool m_enabled{false};
    DecisionCallback m_decision_callback;
    selection::ManeuverSelectionWorker m_worker;

    std::unique_ptr<TrajectoryIntentPublisher> m_intent_publisher;
    rclcpp::Publisher<
        collision_avoidance::msg::ManeuverSelectionDecision>::SharedPtr
        m_decision_publisher;
    std::vector<std::unique_ptr<TrajectoryIntentSubscription>>
        m_intent_subscriptions;
    std::vector<rclcpp::Subscription<
        collision_avoidance::msg::ManeuverSelectionDecision>::SharedPtr>
        m_decision_subscriptions;
    rclcpp::Subscription<px4_msgs::msg::EstimatorTrajectoryBelief>::SharedPtr
        m_belief_subscription;
    rclcpp::TimerBase::SharedPtr m_output_timer;
};

}  // namespace collision_avoidance::communication
