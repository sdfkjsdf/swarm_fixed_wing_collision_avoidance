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

        /*퍼블리셔 설정 — PX4 표준 sensor_data QoS (BEST_EFFORT) 와 통일
          이렇게 해야 vtol_guidance 노드의 BEST_EFFORT 구독자와 매칭됨 */
        trans_odom_pubs_[n] =
            this->create_publisher<VehicleOdometry>(
                "/common/px4_" + std::to_string(n) + "/trans_vehicle_odometry", qos);

        /*서브스크라이버 설정: 받자마자 오프셋 더해서 바로 퍼블리시*/
        raw_odom_subs_[n] =
            this->create_subscription<VehicleOdometry>(
                "/px4_" + std::to_string(n) + "/fmu/out/vehicle_odometry", qos,
                [this, n](const VehicleOdometry::UniquePtr msg)
                {
                    /*원본 복사*/
                    VehicleOdometry trans_msg = *msg;

                    /* ── ENU(spawn yaml) → NED(PX4 odometry) 좌표 변환 후 더하기 ──
                       Gazebo 월드 좌표계는 ENU 로, yaml 의 spawn_offset 도 ENU:
                         m_spawn_offsets[n].x = ENU 동쪽 (East)
                         m_spawn_offsets[n].y = ENU 북쪽 (North)
                         m_spawn_offsets[n].z = ENU 위쪽 (Up)
                       PX4 의 vehicle_odometry.position 은 NED:
                         position[0] = NED 북쪽 (North)
                         position[1] = NED 동쪽 (East)
                         position[2] = NED 아래 (Down)
                       따라서 다음과 같이 매핑해야 함:
                         NED.N (position[0]) ← spawn.y (ENU North)
                         NED.E (position[1]) ← spawn.x (ENU East)
                         NED.D (position[2]) ← -spawn.z (ENU Up)  ← (현재는 무시) */
                    trans_msg.position[0] = msg->position[0] + m_spawn_offsets[n].y;  // N += ENU.N
                    trans_msg.position[1] = msg->position[1] + m_spawn_offsets[n].x;  // E += ENU.E
                    /* z는 모든 기체 동일 지면 가정으로 보정하지 않음 */

                    /*공통 좌표 퍼블리시*/
                    trans_odom_pubs_[n]->publish(trans_msg);
                }
            );
    }

    RCLCPP_INFO(this->get_logger(), "Coordinate transformer started: %d agents", m_total_agent_num);
}

} // namespace Transfer_coordinate
