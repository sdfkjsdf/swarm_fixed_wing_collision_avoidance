#pragma once

#include <rclcpp/rclcpp.hpp>
#include <limits>
#include <thread>
#include <atomic>
#include <array>
#include <cmath>
#include <vector>
#include <memory>

/* px4-ros2-interface-lib */
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/fixedwing/lateral_longitudinal.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/control/vtol.hpp>
#include <px4_ros2/odometry/global_position.hpp>

/* px4 msgs */
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/wind.hpp>

/* 기존 모듈 유지 */
#include <collision_avoidance/StateType.hpp>
#include <collision_avoidance/FormationControl.hpp>


class VtolGuidanceMode : public px4_ros2::ModeBase
{
public:
    /* vehicle_id 는 생성자에서 받아야 함:
       - ModeBase 의 topic_namespace_prefix ("/px4_N") 결정
       - 멀티 인스턴스 PX4 와 통신하려면 필수 */
    VtolGuidanceMode(rclcpp::Node & node, int vehicle_id);
    ~VtolGuidanceMode() override;

    /* ────────────────────────────────────────────────────
       InternalState: 네가 직접 정의하는 내부 비행 단계
       비행 임무가 늘어날수록 여기에 추가하면 됨
       VTOL 전환은 콜백 API가 없어서 Mode 내부에서 폴링 처리
       ──────────────────────────────────────────────────── */
    enum class InternalState {
        TransitionToFw,     /* 멀티콥터 → 고정익 천이 (기존 TRANSITION) */
        FwCruise,           /* 고정익 순항 */
        FormationControl,   /* 편대비행 */
        /* 추가 단계가 필요하면 여기에 계속 추가
        Loitering,
        SearchPattern,
        */
        Done                /* 모든 임무 종료 → Executor 가 RTL 수행 */
    };

    /* 추가 파라미터 설정 (vehicle_id 는 생성자에서 받음) */
    void setTotalAgentNum(int num)   { m_total_agent_num = num; }
    void setPropagationTime(float t) { m_propagation_time = t; }

    /* ── 라이브러리가 오버라이드 요구하는 함수들 ── */
    void onActivate() override;
    void onDeactivate() override;

    /* 기존 타이머 콜백(10ms) 역할 대체 - 라이브러리가 자동 호출 */
    void updateSetpoint(float dt_s) override;

private:
    /* ── 내부 단계별 setpoint 송신 함수 (기존 set_fw_velocity_setpoint 등 대체) ── */
    void runTransitionToFw();
    void sendFwCruiseSetpoint();
    void sendFormationSetpoint();
    /* 추가 단계 함수도 여기에 선언
    void sendLoiteringSetpoint();
    */

    /* ── 바람 보상 계산 (지면속도 → airspeed 변환) ── */
    float computeRequiredAirspeed(float desired_ground_speed, float course);

    /* ── 멀티스레드에서 돌릴 맴버함수 (기존과 완전히 동일) ── */
    void rt_loop();

    /* 노드 핸들 (서브스크라이버 생성용 - 라이브러리가 ModeBase 에 보관함) */
    rclcpp::Node & _node;

    /* ── 라이브러리 setpoint 타입 ── */
    std::shared_ptr<px4_ros2::FwLateralLongitudinalSetpointType> _fw_setpoint;
    std::shared_ptr<px4_ros2::TrajectorySetpointType>            _mc_trajectory;
    std::shared_ptr<px4_ros2::VTOL>                              _vtol;
    std::shared_ptr<px4_ros2::OdometryGlobalPosition>            _global_position; /* 현재 AMSL 고도 조회 */

    /* ── 바람 서브스크라이버 (신규: 바람 보상용) ── */
    rclcpp::Subscription<px4_msgs::msg::Wind>::SharedPtr _wind_sub;

    /* TransferSameCoordinate 노드로 부터 발행되는 공통의 좌표계 기준으로 표현이 되는
       state를 서브스크라이 하는 맴버 변수 정의 (기존 trans_odom_subs_ 유지) */
    std::vector<rclcpp::Subscription<
        px4_msgs::msg::VehicleOdometry>::SharedPtr> _trans_odom_subs; /* 지금 trans_node에서 주는 값을 일단 받기 위한 것임 */

    /* ── 내부 단계 상태 변수 (신규) ── */
    InternalState _internal_state{InternalState::TransitionToFw};
    rclcpp::Time  _phase_start_time{};

    /* 토픽이름 제작을 위한 맴버 변수 정의 (기존 유지) */
    int   m_vehicle_ID{1};
    int   m_total_agent_num{0};
    float m_propagation_time{0.0f};

    /* ── 바람 추정값 (신규) ── */
    float _wind_n{0.f};
    float _wind_e{0.f};

    /* ── 목표값 (신규)
       _cruise_altitude_amsl: 천이 완료 시점의 현재 AMSL 고도를 캡처해서
                              그대로 순항 고도로 사용 (이륙 → 천이 → 그 고도 유지) */
    float _desired_course{0.f};
    float _cruise_altitude_amsl{NAN};
    float _desired_ground_speed{15.f};

    /* ────────────────────────────────────────────────────
       스레드용 프라이벗 변수 (기존과 완전히 동일)
       ──────────────────────────────────────────────────── */
    std::thread       rt_thread_;          /* 다른 스레드에서 지금 어느 class에서의 맴버 함수를 구현 할 것인지를 정함 */
    std::atomic<bool> rt_running_{false};  /* 지금 main노드가 꺼저있는 경우에 스래드가 할당이 되면 안됨으로 이것을 중지시키기 위한 것임 */

    /* ────────────────────────────────────────────────────
       스레드간 lock-free 을 위한 큐 기반 자료 전달을 위한 변수 (기존과 완전히 동일)
       ──────────────────────────────────────────────────── */
    StateType::InputQueue   m_input_queue{};   /* Total_state_for_Control m_Total_state_for_Control{}; 을 전달할 때 쓸 예정임 */
                                               /* 아직 다른 output queue는 정의를 하지 않은 상태임 */
    StateType::Check_update agent_updated_{};  /* 전체 수신을 완료 했는 지를 확인하기 위한 용도임 */

    /* 자체 프라이빗 변수: control에 필요한 state만을 집어 넣는 것임
       지금 여기서 어차피 5개 초과하는 agent은 다루지 않을 예정이다 */
    std::array<StateType::State_for_Control, kMaxAgents> m_state_for_Control{}; /* 지금 Odomerty 로부터 subscribe 하는 것들 중에서 필요한 것만 받는 내용을 의미 */

    /* ── 모듈을 장착 (기존과 완전히 동일) ── */
    FormationControl Moudle_FormationControl{};
};
