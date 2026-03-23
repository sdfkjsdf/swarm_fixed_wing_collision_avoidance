#pragma once

#include <rclcpp/rclcpp.hpp>

/*px4 msg관련 내용 경로 설정 해주기*/
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>


namespace safetyfilter
{


class CollisionAvoidanceControl: public rclcpp::Node

{

    public:
        CollisionAvoidanceControl();

    private:
        /*퍼블리셔에 대한 맴버변수 정의 */
        rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr  position_setpoint_pub_;
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr  velocity_setpoint_pub_;
        rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr      vehicle_command_pub_;


        /*서브스크라이버에 대한 맴버변수 정의*/
        rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
      


        /*서브스크라이브 하고 임시저장할 맴버 변수 정의*/
        rclcpp::TimerBase::SharedPtr timer_;


        /*토픽이름 제작을 위한 맴버 변수 정의*/
        int m_vehicle_ID=1;
        uint64_t offboard_counter_{0};

        /*비행 단계 정의*/
        enum class FlightPhase {
            IDLE,           // 초기 상태: offboard setpoint 스트리밍
            TAKEOFF,        // 멀티콥터 모드로 이륙 + 고도 확보
            TRANSITION,     // 고정익 전환 명령 전송
            FW_CRUISE       // 고정익 순항
        };
        FlightPhase m_flight_phase{FlightPhase::IDLE};
        bool m_transition_commanded{false};
        float m_cruise_altitude{0.0f};  // 고정익 전환 시점의 고도 저장용

        /*프라이빗 변수를 생성 */
        px4_msgs::msg::VehicleStatus m_vehicle_status{};
        px4_msgs::msg::VehicleLocalPosition m_local_position{};
        px4_msgs::msg::TrajectorySetpoint m_desired_setpoint{};
        /*TrajectorySetpoint의 position은 각 기체의 EKF 로컬 원점 기준*/


        /*ROS2 퍼블리시나 서브스크라이브 관련 맴버함수 정의*/
        void publish_offboard_control_mode(); /* OFFBOARD 명령이 실행되었을 때 어떤 유형의 SETPOINT을 줄 예정이다를 알리는 역활임*/
        void publish_vehicle_command(uint16_t command, float param1 =0.0, float param2 =0.0);
        void publish_control(); /* 연산을 통해서 구해진 SETPOINT을 지금 */


        /*각 기능들을 구현한 맴버함수를 의미 */
        void arm(); /*arm을 키는 명령어임*/
        void disarm();
        void set_takeoff_setpoint(); /* 맨 처음 비행을 하는 경우에 이동을 할 지점을 의미*/
        void set_fw_velocity_setpoint(); /* 고정익 모드에 안전하게 진입을 하기 위한 지점을 의미*/
        void update_flight_phase();
        





};




}