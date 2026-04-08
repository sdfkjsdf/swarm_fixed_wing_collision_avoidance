#pragma once

#include <rclcpp/rclcpp.hpp>

/*px4 msg관련 내용 경로 설정 해주기*/
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>



/*멀티 스레드용 표준라이브러리 */
#include <thread>       // std::thread
#include <atomic>       // std::atomic<bool>
#include <chrono>       // std::chrono::milliseconds, steady_clock
#include <array>        // std::array
#include <cstdint>      // uint64_t
#include <cstddef>      // size_t

/*멀티스레드에서 lock-free을 위한 모듈*/
#include <collision_avoidance/StateType.hpp>



namespace safetyfilter
{


class CollisionAvoidanceControl: public rclcpp::Node

{

    public:
            CollisionAvoidanceControl();
            ~CollisionAvoidanceControl() override;   // ← 소멸자 추가

    private:
        /*퍼블리셔에 대한 맴버변수 정의 */
            rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
            rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr  position_setpoint_pub_;
            rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr  velocity_setpoint_pub_;
            rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr      vehicle_command_pub_;

        




        /*서브스크라이버에 대한 맴버변수 정의*/
            rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
            rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;


        /*TransferSameCoordinate 노드로 부터 발행되는 공통의 좌표계 기준으로 표현이 되는 state를 서브스크라이 하는 맴버 변수 정의*/
            std::vector<rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr> trans_odom_subs_; /*지금 trans_node에서 주는 값을 일단 받기 위한 것임*/


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

        /*서브스크라이버 및 프라이빗 변수를 생성 */
            px4_msgs::msg::VehicleStatus m_vehicle_status{}; /* px4로 부터 자신의 state을 서브스크라이브 하기 위한 맴버 변수임*/     
            px4_msgs::msg::VehicleLocalPosition m_local_position{}; /* px4 에 퍼블리시 하기 위한 맴버 변수임*/
            px4_msgs::msg::TrajectorySetpoint m_desired_setpoint{}; /* px4에 퍼블리시 하기 위한 맴버 변수임*/

        /*자체 프라이빗 변수를 생성*/
            std::array<StateType::State_for_Control, kMaxAgents> m_state_for_Control{};  /* 지금 Odomerty 로부터 subscribe 하는 것들 중에서 필요한 것만 받는 내용을 의미*/
            


        /*스레드용 프라이벗 변수를 생성*/
            std::thread       rt_thread_;          /*다른 스레드에서 지금 어느 class에서의 맴버 함수를 구현 할 것인지를 정함*/
            std::atomic<bool> rt_running_{false};  /* 지금 main노드가 꺼저있는 경우에 스래드가 할당이 되면 안됨으로 이것을 중지시키기 위한 것임*/

        /*스레드간 lock-free 을 위한 큐 기반 자료 전달을 위한 변수*/
            StateType::InputQueue m_input_queue{}; /* Total_state_for_Control m_Total_state_for_Control{}; 을 전달할 때 쓸 예정임*/
                // 아직 다른 output q_ueue는 정의를 하지 않은 상태임
            StateType::Check_update agent_updated_{}; /*전체 수신을 완료 했는 지를 확인하기 위한 용도임*/

        /*control에 필요한 state만을 집어 넣는 것임*/
        /*지금 여기서 어차피 5개 초과하는  agent은 다루지 않을 예정이다*/

            int m_total_agent_num{0};
            float m_propagation_time=0.0;

        

        /*TrajectorySetpoint의 position은 각 기체의 EKF 로컬 원점 기준*/


        /*ROS2 퍼블리시나 서브스크라이브 관련 맴버함수 정의*/
            void publish_offboard_control_mode(); /* OFFBOARD 명령이 실행되었을 때 어떤 유형의 SETPOINT을 줄 예정이다를 알리는 역활임*/
            void publish_vehicle_command(uint16_t command, float param1 =0.0, float param2 =0.0);
            void publish_control(); /* 연산을 통해서 구해진 SETPOINT을 지금 */


        /*멀티스레드에서 돌릴 맴버함수*/
            void rt_loop();

        /*각 기능들을 구현한 맴버함수를 의미 */
            void arm(); /*arm을 키는 명령어임*/
            void disarm();
            void set_takeoff_setpoint(); /* 맨 처음 비행을 하는 경우에 이동을 할 지점을 의미*/
            void set_fw_velocity_setpoint(); /* 고정익 모드에 안전하게 진입을 하기 위한 지점을 의미*/
            void update_flight_phase();
            


};




}