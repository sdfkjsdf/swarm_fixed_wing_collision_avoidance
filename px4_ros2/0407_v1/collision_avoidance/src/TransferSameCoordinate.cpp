#include <collision_avoidance/TransferSameCoordinate.hpp>

using namespace px4_msgs::msg;

namespace Transfer_coordinate
{

TransferSameCoordinate::TransferSameCoordinate() : Node("coordinate_transformer_node")
{
    /*파라미터 선언*/
    this->declare_parameter<int>("total_agent_num", 0);
    this->declare_parameter<std::vector<double>>("spawn_offset_x", std::vector<double>{});
    this->declare_parameter<std::vector<double>>("spawn_offset_y", std::vector<double>{});
    this->declare_parameter<std::vector<double>>("spawn_offset_z", std::vector<double>{});

    /*파라미터 값 할당*/
    this->get_parameter("total_agent_num", m_total_agent_num);
    this->get_parameter("spawn_offset_x", m_spawn_offset_x);
    this->get_parameter("spawn_offset_y", m_spawn_offset_y);
    this->get_parameter("spawn_offset_z", m_spawn_offset_z);

    /*벡터 크기 지정*/
    m_spawn_offsets.resize(m_total_agent_num);
    m_raw_odometry.resize(m_total_agent_num);
    raw_odom_subs_.resize(m_total_agent_num);
    trans_odom_pubs_.resize(m_total_agent_num);

    /*yaml에서 받은 값을 구조체에 할당*/
    for (int n = 0; n < m_total_agent_num; n++) {
        m_spawn_offsets[n].x = (float)m_spawn_offset_x[n];
        m_spawn_offsets[n].y = (float)m_spawn_offset_y[n];
        m_spawn_offsets[n].z = (float)m_spawn_offset_z[n];
    }

    /*QoS 설정*/
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    /*반복문으로 서브스크라이버 + 퍼블리셔 생성*/
    for (int n = 0; n < m_total_agent_num; n++) {

        /*퍼블리셔 설정*/
        trans_odom_pubs_[n] =
            this->create_publisher<VehicleOdometry>(
                "/common/px4_" + std::to_string(n) + "/trans_vehicle_odometry", 10);

        /*서브스크라이버 설정: 받자마자 오프셋 더해서 바로 퍼블리시*/
        raw_odom_subs_[n] =
            this->create_subscription<VehicleOdometry>(
                "/px4_" + std::to_string(n) + "/fmu/out/vehicle_odometry", qos,
                [this, n](const VehicleOdometry::UniquePtr msg)
                {
                    /*원본 복사*/
                    VehicleOdometry trans_msg = *msg;

                    /*position에 스폰 오프셋 더하기 (공통 좌표계로 변환)*/
                    trans_msg.position[0] = msg->position[0] + m_spawn_offsets[n].x;
                    trans_msg.position[1] = msg->position[1] + m_spawn_offsets[n].y;
                    /* z는 NED라서 스폰 높이는 더하지 않음 (지면 높이 보정 불필요) */

                    /*공통 좌표 퍼블리시*/
                    trans_odom_pubs_[n]->publish(trans_msg);
                }
            );
    }

    RCLCPP_INFO(this->get_logger(), "Coordinate transformer started: %d agents", m_total_agent_num);
}

} // namespace Transfer_coordinate
