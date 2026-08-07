#include <trajectory_prediction_hils/estimation/BeliefConePublisher.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

using collision_avoidance::estimation::EstimatorTrajectoryBelief;
using collision_avoidance::estimation::PredictState;
using collision_avoidance::estimation::PredictStateCovariance;
using collision_avoidance::estimation::TrajectoryCone;
using collision_avoidance::estimation::kEstimatorBeliefDimension;
using collision_avoidance::estimation::kTrajectoryIntervalCount;
using collision_avoidance::estimation::kTrajectoryPointCount;

BeliefConePublisher::BeliefConePublisher(
    rclcpp::Node & node,
    std::string topic_namespace_prefix,
    std::shared_ptr<collision_avoidance::estimation::TrajectoryPredict> predictor,
    const collision_avoidance::estimation::UncertaintyParams & uncertainty_params,
    double estimator_output_delay_s)
: m_node(node),
  m_predictor(std::move(predictor)),
  m_uncertainty(uncertainty_params),
  m_estimator_output_delay_s(estimator_output_delay_s)
{
    const auto qos = rclcpp::SensorDataQoS();
    const std::string belief_topic =
        topic_namespace_prefix + "/fmu/out/estimator_trajectory_belief";
    const std::string cone_topic =
        topic_namespace_prefix + "/collision_estimation/trajectory_cone";

    m_belief_sub = m_node.create_subscription<px4_msgs::msg::EstimatorTrajectoryBelief>(
        belief_topic, qos,
        [this](px4_msgs::msg::EstimatorTrajectoryBelief::UniquePtr message) {
            std::lock_guard<std::mutex> lock(m_belief_mutex);
            m_latest_belief = *message;
            m_has_belief = true;
        });
    m_cone_pub = m_node.create_publisher<collision_avoidance::msg::TrajectoryCone>(
        cone_topic, qos);

    RCLCPP_INFO(m_node.get_logger(),
        "[cone] belief='%s', output='%s', estimator_output_delay=%.3fs",
        belief_topic.c_str(), cone_topic.c_str(), m_estimator_output_delay_s);
}

bool BeliefConePublisher::publish(
    const collision_avoidance::estimation::PredictionInputTrajectory & inputs,
    double dt)
{
    px4_msgs::msg::EstimatorTrajectoryBelief source;
    {
        std::lock_guard<std::mutex> lock(m_belief_mutex);
        if (!m_has_belief || m_latest_belief.timestamp == m_last_published_source_timestamp) {
            return false;
        }
        source = m_latest_belief;
        m_last_published_source_timestamp = source.timestamp;
    }

    collision_avoidance::msg::TrajectoryCone message;
    message.timestamp = static_cast<std::uint64_t>(m_node.now().nanoseconds() / 1000);
    message.source_timestamp = source.timestamp;
    message.source_timestamp_sample = source.timestamp_sample;
    message.confidence_level = 0.95F;
    message.chi_square_quantile = 7.8147279F;
    message.point_count = static_cast<std::uint8_t>(kTrajectoryPointCount);
    message.valid = false;

    const std::uint64_t delay_us = source.timestamp >= source.timestamp_sample
        ? source.timestamp - source.timestamp_sample : 0;
    const double timestamp_delay_s = static_cast<double>(delay_us) * 1.0e-6;
    const double delay_s = timestamp_delay_s + m_estimator_output_delay_s;
    message.source_delay_s = static_cast<float>(delay_s);

    for (std::size_t interval = 0; interval < kTrajectoryIntervalCount; ++interval) {
        const auto & input = inputs[interval];
        const std::size_t base = interval * 4;
        message.prediction_inputs[base] = static_cast<float>(input.V_cmd);
        message.prediction_inputs[base + 1] = static_cast<float>(input.h_cmd);
        message.prediction_inputs[base + 2] = static_cast<float>(input.h_dot_cmd);
        message.prediction_inputs[base + 3] = static_cast<float>(input.a_lat_cmd);
    }

    if (!source.valid || delay_s > 1.0) {
        m_cone_pub->publish(message);
        return false;
    }

    EstimatorTrajectoryBelief belief;
    std::copy(source.attitude_q.begin(), source.attitude_q.end(), belief.attitude_q.begin());
    std::copy(source.velocity.begin(), source.velocity.end(), belief.velocity_ned.begin());
    std::copy(source.position.begin(), source.position.end(), belief.position_ned.begin());

    std::size_t packed_index = 0;
    for (std::size_t row = 0; row < kEstimatorBeliefDimension; ++row) {
        for (std::size_t column = row; column < kEstimatorBeliefDimension; ++column) {
            const double value = source.covariance_upper_triangle[packed_index++];
            belief.covariance[row * kEstimatorBeliefDimension + column] = value;
            belief.covariance[column * kEstimatorBeliefDimension + row] = value;
        }
    }

    PredictState state;
    PredictStateCovariance covariance;
    TrajectoryCone cone;
    bool valid = m_uncertainty.initializeFromEstimatorBelief(belief, state, covariance);
    valid = valid && m_uncertainty.compensateFusionHorizonDelay(
        *m_predictor, inputs.front(), delay_s, state, covariance);
    valid = valid && m_uncertainty.propagate(
        *m_predictor, state, covariance, inputs, dt, cone);

    if (valid) {
        for (std::size_t point = 0; point < kTrajectoryPointCount; ++point) {
            message.time_offset_s[point] = static_cast<float>(cone[point].time_offset_s);
            const std::size_t position_base = point * 3;
            message.mean_position_ned[position_base] = static_cast<float>(cone[point].mean.p_n);
            message.mean_position_ned[position_base + 1] = static_cast<float>(cone[point].mean.p_e);
            message.mean_position_ned[position_base + 2] = static_cast<float>(-cone[point].mean.h);

            const std::size_t covariance_base = point * 9;
            for (std::size_t entry = 0; entry < 9; ++entry) {
                message.position_covariance_ned[covariance_base + entry] =
                    static_cast<float>(cone[point].position_covariance_ned[entry]);
            }
        }
        message.valid = true;
    }

    m_cone_pub->publish(message);
    if (!valid) {
        RCLCPP_WARN_THROTTLE(
            m_node.get_logger(), *m_node.get_clock(), 5000,
            "[cone] invalid EKF belief or propagated covariance");
    }
    return valid;
}
