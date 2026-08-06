#include <collision_avoidance/coordinate/TransferSameCoordinate.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace Transfer_coordinate
{
namespace
{

template<typename MessageT>
void translatePosition(MessageT & message, const std::array<double, 3> & translation)
{
    message.position[0] += static_cast<float>(translation[0]);
    message.position[1] += static_cast<float>(translation[1]);
    message.position[2] += static_cast<float>(translation[2]);
}

}  // namespace

TransferSameCoordinate::TransferSameCoordinate()
: Node("coordinate_transformer_node")
{
    declare_parameter<int>("total_agent_num", 0);
    declare_parameter<std::vector<double>>("spawn_offset_x", std::vector<double>{});
    declare_parameter<std::vector<double>>("spawn_offset_y", std::vector<double>{});
    declare_parameter<std::vector<double>>("spawn_offset_z", std::vector<double>{});
    declare_parameter<std::string>("transform_mode", "spawn_offset");
    declare_parameter<bool>("apply_spawn_z_offset", false);
    declare_parameter<double>("common_origin_lat", 47.397742);
    declare_parameter<double>("common_origin_lon", 8.545594);
    declare_parameter<double>("common_origin_alt", 488.0);

    m_total_agent_num = get_parameter("total_agent_num").as_int();
    const auto spawn_offset_x = get_parameter("spawn_offset_x").as_double_array();
    const auto spawn_offset_y = get_parameter("spawn_offset_y").as_double_array();
    const auto spawn_offset_z = get_parameter("spawn_offset_z").as_double_array();
    const std::string transform_mode = get_parameter("transform_mode").as_string();
    m_apply_spawn_z_offset = get_parameter("apply_spawn_z_offset").as_bool();
    m_common_reference = {
        get_parameter("common_origin_lat").as_double(),
        get_parameter("common_origin_lon").as_double(),
        get_parameter("common_origin_alt").as_double(),
    };

    if (m_total_agent_num <= 0) {
        throw std::runtime_error("total_agent_num must be positive");
    }
    const auto required_size = static_cast<std::size_t>(m_total_agent_num);
    if (spawn_offset_x.size() < required_size
        || spawn_offset_y.size() < required_size
        || spawn_offset_z.size() < required_size) {
        throw std::runtime_error("spawn offset arrays must contain every configured agent");
    }
    if (transform_mode == "spawn_offset") {
        m_transform_mode = TransformMode::SpawnOffset;
    } else if (transform_mode == "geodetic_reference") {
        m_transform_mode = TransformMode::GeodeticReference;
        m_common_transform =
            std::make_unique<collision_avoidance::coordinate::CommonNedTransform>(
                m_common_reference);
        if (!m_common_transform->valid()) {
            throw std::runtime_error("invalid common geodetic origin");
        }
    } else {
        throw std::runtime_error(
            "transform_mode must be 'spawn_offset' or 'geodetic_reference'");
    }

    m_spawn_offsets.resize(required_size);
    m_agent_references.resize(required_size);
    for (std::size_t agent = 0; agent < required_size; ++agent) {
        m_spawn_offsets[agent] = {
            static_cast<float>(spawn_offset_x[agent]),
            static_cast<float>(spawn_offset_y[agent]),
            static_cast<float>(spawn_offset_z[agent]),
        };
    }

    m_odometry_subscriptions.resize(required_size);
    m_local_position_subscriptions.resize(required_size);
    m_ground_truth_subscriptions.resize(required_size);
    m_belief_subscriptions.resize(required_size);
    m_cone_subscriptions.resize(required_size);
    m_odometry_publishers.resize(required_size);
    m_local_position_publishers.resize(required_size);
    m_ground_truth_publishers.resize(required_size);
    m_belief_publishers.resize(required_size);
    m_cone_publishers.resize(required_size);

    const auto qos = rclcpp::SensorDataQoS();
    for (int agent = 0; agent < m_total_agent_num; ++agent) {
        const std::string px4_namespace = "/px4_" + std::to_string(agent);
        const std::string common_namespace = "/common/px4_" + std::to_string(agent);

        m_odometry_publishers[agent] = create_publisher<px4_msgs::msg::VehicleOdometry>(
            common_namespace + "/trans_vehicle_odometry", qos);
        m_local_position_publishers[agent] =
            create_publisher<px4_msgs::msg::VehicleLocalPosition>(
                common_namespace + "/trans_vehicle_local_position", qos);
        m_ground_truth_publishers[agent] =
            create_publisher<px4_msgs::msg::VehicleLocalPosition>(
                common_namespace + "/trans_vehicle_local_position_groundtruth", qos);
        m_belief_publishers[agent] =
            create_publisher<px4_msgs::msg::EstimatorTrajectoryBelief>(
                common_namespace + "/trans_estimator_trajectory_belief", qos);
        m_cone_publishers[agent] =
            create_publisher<collision_avoidance::msg::TrajectoryCone>(
                common_namespace + "/trans_trajectory_cone", qos);

        m_local_position_subscriptions[agent] =
            create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                px4_namespace + "/fmu/out/vehicle_local_position_v1", qos,
                [this, agent](px4_msgs::msg::VehicleLocalPosition::UniquePtr message) {
                    onLocalPosition(agent, std::move(message));
                });
        m_ground_truth_subscriptions[agent] =
            create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                px4_namespace + "/fmu/out/vehicle_local_position_groundtruth_v1", qos,
                [this, agent](px4_msgs::msg::VehicleLocalPosition::UniquePtr message) {
                    onGroundTruth(agent, std::move(message));
                });
        m_odometry_subscriptions[agent] =
            create_subscription<px4_msgs::msg::VehicleOdometry>(
                px4_namespace + "/fmu/out/vehicle_odometry", qos,
                [this, agent](px4_msgs::msg::VehicleOdometry::UniquePtr message) {
                    onOdometry(agent, std::move(message));
                });
        m_belief_subscriptions[agent] =
            create_subscription<px4_msgs::msg::EstimatorTrajectoryBelief>(
                px4_namespace + "/fmu/out/estimator_trajectory_belief", qos,
                [this, agent](px4_msgs::msg::EstimatorTrajectoryBelief::UniquePtr message) {
                    onBelief(agent, std::move(message));
                });
        m_cone_subscriptions[agent] =
            create_subscription<collision_avoidance::msg::TrajectoryCone>(
                px4_namespace + "/collision_estimation/trajectory_cone", qos,
                [this, agent](collision_avoidance::msg::TrajectoryCone::UniquePtr message) {
                    onCone(agent, std::move(message));
                });
    }

    RCLCPP_INFO(
        get_logger(),
        "Common NED transformer started: agents=%d mode=%s origin=(%.8f, %.8f, %.3f)",
        m_total_agent_num, transform_mode.c_str(),
        m_common_reference.latitude_deg, m_common_reference.longitude_deg,
        m_common_reference.altitude_m);
}

bool TransferSameCoordinate::translationForAgent(
    int agent, std::array<double, 3> & translation) const
{
    if (m_transform_mode == TransformMode::SpawnOffset) {
        const auto & spawn = m_spawn_offsets[agent];
        translation = {
            static_cast<double>(spawn.y),
            static_cast<double>(spawn.x),
            m_apply_spawn_z_offset ? -static_cast<double>(spawn.z) : 0.0,
        };
        return true;
    }
    if (!m_agent_references[agent].valid) {
        return false;
    }
    return m_common_transform->translationFrom(
        m_agent_references[agent].geodetic, translation);
}

bool TransferSameCoordinate::translationFromReference(
    double latitude_deg,
    double longitude_deg,
    double altitude_m,
    std::array<double, 3> & translation) const
{
    if (m_transform_mode == TransformMode::SpawnOffset) {
        return false;
    }
    return m_common_transform->translationFrom(
        {latitude_deg, longitude_deg, altitude_m}, translation);
}

void TransferSameCoordinate::warnMissingReference(int agent)
{
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "agent %d EKF local reference unavailable; common-frame message skipped", agent);
}

void TransferSameCoordinate::onLocalPosition(
    int agent, const px4_msgs::msg::VehicleLocalPosition::UniquePtr message)
{
    if (m_transform_mode == TransformMode::GeodeticReference) {
        if (!message->xy_global || !message->z_global
            || !std::isfinite(message->ref_lat)
            || !std::isfinite(message->ref_lon)
            || !std::isfinite(message->ref_alt)) {
            warnMissingReference(agent);
            return;
        }
        m_agent_references[agent] = {
            {message->ref_lat, message->ref_lon, message->ref_alt}, true};
    }

    std::array<double, 3> translation{};
    if (!translationForAgent(agent, translation)) {
        warnMissingReference(agent);
        return;
    }
    auto transformed = *message;
    transformed.x += static_cast<float>(translation[0]);
    transformed.y += static_cast<float>(translation[1]);
    transformed.z += static_cast<float>(translation[2]);
    if (m_transform_mode == TransformMode::GeodeticReference) {
        transformed.ref_lat = m_common_reference.latitude_deg;
        transformed.ref_lon = m_common_reference.longitude_deg;
        transformed.ref_alt = static_cast<float>(m_common_reference.altitude_m);
    }
    m_local_position_publishers[agent]->publish(transformed);
}

void TransferSameCoordinate::onGroundTruth(
    int agent, const px4_msgs::msg::VehicleLocalPosition::UniquePtr message)
{
    std::array<double, 3> translation{};
    if (m_transform_mode == TransformMode::GeodeticReference) {
        if (!message->xy_global || !message->z_global
            || !translationFromReference(
                message->ref_lat, message->ref_lon, message->ref_alt, translation)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "agent %d ground-truth reference unavailable; common-frame message skipped",
                agent);
            return;
        }
    } else if (!translationForAgent(agent, translation)) {
        return;
    }

    auto transformed = *message;
    transformed.x += static_cast<float>(translation[0]);
    transformed.y += static_cast<float>(translation[1]);
    transformed.z += static_cast<float>(translation[2]);
    if (m_transform_mode == TransformMode::GeodeticReference) {
        transformed.ref_lat = m_common_reference.latitude_deg;
        transformed.ref_lon = m_common_reference.longitude_deg;
        transformed.ref_alt = static_cast<float>(m_common_reference.altitude_m);
    }
    m_ground_truth_publishers[agent]->publish(transformed);
}

void TransferSameCoordinate::onOdometry(
    int agent, const px4_msgs::msg::VehicleOdometry::UniquePtr message)
{
    std::array<double, 3> translation{};
    if (!translationForAgent(agent, translation)) {
        warnMissingReference(agent);
        return;
    }
    auto transformed = *message;
    translatePosition(transformed, translation);
    m_odometry_publishers[agent]->publish(transformed);
}

void TransferSameCoordinate::onBelief(
    int agent, const px4_msgs::msg::EstimatorTrajectoryBelief::UniquePtr message)
{
    std::array<double, 3> translation{};
    if (!translationForAgent(agent, translation)) {
        warnMissingReference(agent);
        return;
    }
    auto transformed = *message;
    translatePosition(transformed, translation);
    m_belief_publishers[agent]->publish(transformed);
}

void TransferSameCoordinate::onCone(
    int agent, const collision_avoidance::msg::TrajectoryCone::UniquePtr message)
{
    std::array<double, 3> translation{};
    if (!translationForAgent(agent, translation)) {
        warnMissingReference(agent);
        return;
    }
    auto transformed = *message;
    const std::size_t point_count = std::min<std::size_t>(
        transformed.point_count, transformed.mean_position_ned.size() / 3);
    for (std::size_t point = 0; point < point_count; ++point) {
        const std::size_t base = point * 3;
        transformed.mean_position_ned[base] += static_cast<float>(translation[0]);
        transformed.mean_position_ned[base + 1] += static_cast<float>(translation[1]);
        transformed.mean_position_ned[base + 2] += static_cast<float>(translation[2]);
    }
    // A deterministic translation does not rotate or enlarge covariance.
    m_cone_publishers[agent]->publish(transformed);
}

}  // namespace Transfer_coordinate
