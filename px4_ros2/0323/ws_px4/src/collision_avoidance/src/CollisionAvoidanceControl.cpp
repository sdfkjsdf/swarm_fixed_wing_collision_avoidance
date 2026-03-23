#include <collision_avoidance/CollisionAvoidanceControl.hpp>
#include <iostream>


using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

 /*이런 식으로 지금 namespace을 사용해서 즉각 사용이 가능하도록 만듬
   실질적으로 지금  vehicle_command 에 쓰일 내용임   
   
   

   지금 터미널 1개가 임무컴퓨터 1대라고 가정을 하는 경우에는 
   
                                                                                                        
  # 아무 터미널 하나에서 1번만                                                                          
  cd ~/ws_px4 && colcon build --packages-select collision_avoidance
                                                                     
                                                                                                         
  # 터미널 1 (드론 1번 - 임무컴퓨터 1)                                                                  
  source ~/ws_px4/install/setup.bash                                                                    
  ros2 run collision_avoidance collision_avoidance_node --ros-args -p vehicle_ID:=1 -r __node:=collision_avoidance_1
  
  여기서 이런식으로 하는 이유가 지금 노드 이름이 다 같으면 지금 충돌이 일어남
                                                                                                        
  # 터미널 2 (드론 2번 - 임무컴퓨터 2)                                                                  
  source ~/ws_px4/install/setup.bash                                                                    
  ros2 run collision_avoidance collision_avoidance_node --ros-args -p vehicle_ID:=2 -r __node:=collision_avoidance_2                   
                                                                                                        
  # 터미널 3 (드론 3번 - 임무컴퓨터 3)                                                                  
  source ~/ws_px4/install/setup.bash                                                                    
  ros2 run collision_avoidance collision_avoidance_node --ros-args -p vehicle_ID:=3 -r __node:=collision_avoidance_3          
                                           
  

  # 터미널 4 (드론 4번 - 임무컴퓨터 4)                                                                  
  source ~/ws_px4/install/setup.bash                                                                    
  ros2 run collision_avoidance collision_avoidance_node --ros-args -p vehicle_ID:=4 -r __node:=collision_avoidance_4                  
                                                                                                        
  # 터미널 5 (드론 5번 - 임무컴퓨터 5)
  source ~/ws_px4/install/setup.bash                                                                    
  ros2 run collision_avoidance collision_avoidance_node --ros-args -p vehicle_ID:=5 -r __node:=collision_avoidance_5      
   
   

   
   
   */




namespace safetyfilter
{

CollisionAvoidanceControl::CollisionAvoidanceControl() : Node("Collision_avoidance_control")
{
    /* 파리미터 초기화 */
    this->declare_parameter<int>("vehicle_ID", 1);

    /*파라미터에 값을 할당*/

    this-> get_parameter("vehicle_ID",m_vehicle_ID);


    /*퍼블리셔 구체적으로 설정*/

        /*OFFboard 모드를 설정*/
        offboard_control_mode_pub_ =
            this->create_publisher<px4_msgs::msg::OffboardControlMode>(
                "/px4_" + std::to_string(m_vehicle_ID) + "/fmu/in/offboard_control_mode", 10);


        /*vehicle command 퍼블리셔 설정*/
        vehicle_command_pub_ =
            this->create_publisher<px4_msgs::msg::VehicleCommand>(
                "/px4_" + std::to_string(m_vehicle_ID) + "/fmu/in/vehicle_command", 10);
        
       position_setpoint_pub_=
            this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
                "/px4_" + std::to_string(m_vehicle_ID) + "/fmu/in/trajectory_setpoint", 10);

        velocity_setpoint_pub_=
            this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
                "/px4_" + std::to_string(m_vehicle_ID) + "/fmu/in/trajectory_setpoint", 10);

    
    

    /*서브스크라이버를 설정*/
     rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
     auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

     vehicle_status_sub_=
                this->create_subscription<px4_msgs::msg::VehicleStatus>(
                        "/px4_" + std::to_string(m_vehicle_ID) + "/fmu/out/vehicle_status_v2", qos,
                        [this](const px4_msgs::msg::VehicleStatus::UniquePtr msg)
                                {
                                    m_vehicle_status=*msg;


                                }
                        );

     vehicle_local_position_sub_=
                this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                        "/px4_" + std::to_string(m_vehicle_ID) + "/fmu/out/vehicle_local_position_v1", qos,
                        [this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg)
                                {
                                    m_local_position=*msg;
                                }
                        );


    /*callback 으로 계속 보낼 것을 진행*/

    auto timer_callback = [this]() -> void
    {
        offboard_counter_++;

        /*
         * 비행 순서:
         * 1) IDLE: 2초간 offboard setpoint 스트리밍 → arm + offboard 모드 진입
         * 2) TAKEOFF: 멀티콥터 모드로 고도 확보 (position 제어)
         * 3) TRANSITION: 고정익 전환 명령 전송
         * 4) FW_CRUISE: 고정익 velocity 제어로 순항
         */

         update_flight_phase();
        
        publish_control(); /*해당 코드는 지금 arm->offboard 명령어를 주겠다-> 무슨 유형의 setpoint을 주겠다를 다 설정한 다음에 지금 어떤 값을 주겠다라란 의미임*/
    };

    timer_ = create_wall_timer(10ms, timer_callback);
}

/*맴버 함수 구현시작*/

/* arm 명령어 구현 */
void CollisionAvoidanceControl::arm() {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

    RCLCPP_INFO(this->get_logger(), "Arm command send");
};

/* disarm 명령어 구현 */
void CollisionAvoidanceControl::disarm() {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}




void CollisionAvoidanceControl::publish_vehicle_command(uint16_t command, float param1, float param2)
/*
- 토픽: /px4_X/fmu/in/vehicle_command
  - "arm 해라", "offboard 켜라", "고정익 전환해라"
  - PX4 시스템의 상태를 바꾸는 것
  - 한 번만 보내면 됨
  - 지금 해당 양식으로 해야 무인기의 비행모드를 변환시킬 수가 있음

*/
{
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = m_vehicle_ID + 1; /* PX4 멀티 인스턴스에서 system_id는 MAVLink 규약상 1부터 시작하므로 토픽 인덱스 + 1 */   
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_pub_->publish(msg);
}


void CollisionAvoidanceControl::publish_offboard_control_mode()

/*offboard 명령어 구현 - 비행 단계에 따라 제어 모드 변경

해당 코드는 지금 offboard 명령어를 주고 나서 어떤 유형의 setpoint을 줄것인지를 알려주는 부분임

*/

{
    auto msg = px4_msgs::msg::OffboardControlMode();
    msg.timestamp = 0;
    msg.attitude = false;
    msg.body_rate = false;
    msg.thrust_and_torque = false;
    msg.direct_actuator = false;

    if (m_flight_phase == FlightPhase::FW_CRUISE) {
        /* 고정익: velocity 제어 */
        msg.position = false;
        msg.velocity = true;
        msg.acceleration = false;
    } else {
        /* 멀티콥터(이륙/전환): position 제어 */
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
    }

    offboard_control_mode_pub_->publish(msg);
}




/*publish_control()은 오프보드 모드 진입 후 제어값 전송을 매 10ms 마다 진행함 */
void CollisionAvoidanceControl::publish_control()
{
    publish_offboard_control_mode(); /*먼저 일단은 offboard에서 어떤 유형의 setpoint을 줄것인지 선언*/
    
    /*그다음에 각 비행 유형별로 실질적인 setpoint값을 선언*/
    if (m_flight_phase == FlightPhase::FW_CRUISE) {
        set_fw_velocity_setpoint(); /*실질적인 setpoint을 인가 */
        velocity_setpoint_pub_->publish(m_desired_setpoint);
    } else {
        set_takeoff_setpoint(); /*실질적인 setpoint을 인가 */
        position_setpoint_pub_->publish(m_desired_setpoint);
    }

   
}

/* 이륙용 position setpoint (멀티콥터 모드) */
void CollisionAvoidanceControl::set_takeoff_setpoint()
{
    m_desired_setpoint.position[0] = 0.0;
    m_desired_setpoint.position[1] = 0.0;
    m_desired_setpoint.position[2] = -50.0;

    // 미사용 필드는 NaN
    m_desired_setpoint.velocity[0] = std::nanf("");
    m_desired_setpoint.velocity[1] = std::nanf("");
    m_desired_setpoint.velocity[2] = std::nanf("");

    m_desired_setpoint.acceleration[0] = std::nanf("");
    m_desired_setpoint.acceleration[1] = std::nanf("");
    m_desired_setpoint.acceleration[2] = std::nanf("");

    m_desired_setpoint.yaw = 0.0f;
    m_desired_setpoint.yawspeed = std::nanf("");


}

/* 고정익 순항용 velocity setpoint */
void CollisionAvoidanceControl::set_fw_velocity_setpoint()
{
   

    // 미사용 필드는 반드시 NaN
    m_desired_setpoint.position[0] = std::nanf("");
    m_desired_setpoint.position[1] = std::nanf("");
    m_desired_setpoint.position[2] = std::nanf("");  // 이게 핵심

    m_desired_setpoint.velocity[0] = 15.0;
    m_desired_setpoint.velocity[1] = 0.0;
    m_desired_setpoint.velocity[2] = 0.0;

    m_desired_setpoint.acceleration[0] = std::nanf("");
    m_desired_setpoint.acceleration[1] = std::nanf("");
    m_desired_setpoint.acceleration[2] = std::nanf("");

    m_desired_setpoint.yaw = 0.0f;
    m_desired_setpoint.yawspeed = std::nanf("");

   
}

/* 지금 일단 offboard 명령어 부터 지금 고정익 천이구간까지 전부다 한번에 자동으로 처리를 하는 함수임*/
void CollisionAvoidanceControl::update_flight_phase()
{


    switch (m_flight_phase) {

        case FlightPhase::IDLE:
            /* 2초간 setpoint 스트리밍 후 arm + offboard 진입 */
            if (offboard_counter_ == 200) {
                arm();
                publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); /*이것은 지금 offboard 명령으로 하겠다라는 것을 알려주는 것임*/
                m_flight_phase = FlightPhase::TAKEOFF;
                RCLCPP_INFO(this->get_logger(), "TAKEOFF phase: climbing to altitude");
            }
            break;

        case FlightPhase::TAKEOFF:
            /* 현재 고도가 목표의 90% 이상 도달하면 고정익 전환 (NED 좌표계: z가 음수일수록 높음) */
            if (m_local_position.z < 0.95*m_desired_setpoint.position[2] && !m_transition_commanded) {  // 목표 100m 중 90m 이상 도달
                publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_VTOL_TRANSITION, 4.0); /* 해당 코드는 고정익 모드로 진입을 하곘다라는 명령을 줌*/
                m_transition_commanded = true;
                m_flight_phase = FlightPhase::TRANSITION;
                RCLCPP_INFO(this->get_logger(), "TRANSITION phase: switching to fixed-wing");
            }
            break;

        case FlightPhase::TRANSITION:
            /* vtol_vehicle_status로 전환 완료 확인 */
            if (m_vehicle_status.vehicle_type == 2) {  // 2 = FIXED_WING
                m_cruise_altitude = m_local_position.z;  // 전환 시점 고도 저장
                m_flight_phase = FlightPhase::FW_CRUISE;
                RCLCPP_INFO(this->get_logger(), "FW_CRUISE phase: fixed-wing cruise at z=%.1f", m_cruise_altitude);
            }
            break;

        case FlightPhase::FW_CRUISE:
            /* 고정익 순항 중 */
            break;
        }



}

















} // namespace safetyfilter