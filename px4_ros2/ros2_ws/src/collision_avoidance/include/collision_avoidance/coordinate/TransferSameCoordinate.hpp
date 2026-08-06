#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <collision_avoidance/coordinate/CommonNedTransform.hpp>
#include <collision_avoidance/msg/trajectory_cone.hpp>
#include <px4_msgs/msg/estimator_trajectory_belief.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

namespace Transfer_coordinate
{

struct SpawnPosition
{
    float x{0.0F};
    float y{0.0F};
    float z{0.0F};
};

class TransferSameCoordinate : public rclcpp::Node
{
public:
    TransferSameCoordinate();

private:
    enum class TransformMode
    {
        SpawnOffset,
        GeodeticReference,
    };

    struct AgentReference
    {
        collision_avoidance::coordinate::GeodeticReference geodetic{};
        bool valid{false};
    };

    bool translationForAgent(int agent, std::array<double, 3> & translation) const;
    bool translationFromReference(
        double latitude_deg,
        double longitude_deg,
        double altitude_m,
        std::array<double, 3> & translation) const;
    void warnMissingReference(int agent);

    void onLocalPosition(
        int agent,
        const px4_msgs::msg::VehicleLocalPosition::UniquePtr message);
    void onGroundTruth(
        int agent,
        const px4_msgs::msg::VehicleLocalPosition::UniquePtr message);
    void onOdometry(
        int agent,
        const px4_msgs::msg::VehicleOdometry::UniquePtr message);
    void onBelief(
        int agent,
        const px4_msgs::msg::EstimatorTrajectoryBelief::UniquePtr message);
    void onCone(
        int agent,
        const collision_avoidance::msg::TrajectoryCone::UniquePtr message);

    int m_total_agent_num{0};
    TransformMode m_transform_mode{TransformMode::SpawnOffset};
    bool m_apply_spawn_z_offset{false};
    collision_avoidance::coordinate::GeodeticReference m_common_reference{};
    std::unique_ptr<collision_avoidance::coordinate::CommonNedTransform>
        m_common_transform;
    std::vector<SpawnPosition> m_spawn_offsets;
    std::vector<AgentReference> m_agent_references;

    std::vector<rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr>
        m_odometry_subscriptions;
    std::vector<rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr>
        m_local_position_subscriptions;
    std::vector<rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr>
        m_ground_truth_subscriptions;
    std::vector<rclcpp::Subscription<px4_msgs::msg::EstimatorTrajectoryBelief>::SharedPtr>
        m_belief_subscriptions;
    std::vector<rclcpp::Subscription<collision_avoidance::msg::TrajectoryCone>::SharedPtr>
        m_cone_subscriptions;

    std::vector<rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr>
        m_odometry_publishers;
    std::vector<rclcpp::Publisher<px4_msgs::msg::VehicleLocalPosition>::SharedPtr>
        m_local_position_publishers;
    std::vector<rclcpp::Publisher<px4_msgs::msg::VehicleLocalPosition>::SharedPtr>
        m_ground_truth_publishers;
    std::vector<rclcpp::Publisher<px4_msgs::msg::EstimatorTrajectoryBelief>::SharedPtr>
        m_belief_publishers;
    std::vector<rclcpp::Publisher<collision_avoidance::msg::TrajectoryCone>::SharedPtr>
        m_cone_publishers;
};

}  // namespace Transfer_coordinate
