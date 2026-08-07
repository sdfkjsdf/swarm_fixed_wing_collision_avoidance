#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <collision_avoidance/estimation/trajectory_prediction/TrajectoryUncertainty.hpp>
#include <collision_avoidance/msg/trajectory_cone.hpp>
#include <px4_msgs/msg/estimator_trajectory_belief.hpp>

class BeliefConePublisher
{
public:
    BeliefConePublisher(
        rclcpp::Node & node,
        std::string topic_namespace_prefix,
        std::shared_ptr<collision_avoidance::estimation::TrajectoryPredict> predictor,
        const collision_avoidance::estimation::UncertaintyParams & uncertainty_params,
        double estimator_output_delay_s);

    bool publish(
        const collision_avoidance::estimation::PredictionInputTrajectory & inputs,
        double dt);

private:
    rclcpp::Node & m_node;
    std::shared_ptr<collision_avoidance::estimation::TrajectoryPredict> m_predictor;
    collision_avoidance::estimation::TrajectoryUncertainty m_uncertainty;
    double m_estimator_output_delay_s{0.0};

    rclcpp::Subscription<px4_msgs::msg::EstimatorTrajectoryBelief>::SharedPtr m_belief_sub;
    rclcpp::Publisher<collision_avoidance::msg::TrajectoryCone>::SharedPtr m_cone_pub;

    std::mutex m_belief_mutex;
    px4_msgs::msg::EstimatorTrajectoryBelief m_latest_belief{};
    bool m_has_belief{false};
    std::uint64_t m_last_published_source_timestamp{0};
};
