# px4-ros2-interface-lib 마이그레이션 가이드 (방법 A: InternalState 분기)

## 개요

기존 `CollisionAvoidanceControl` (rclcpp::Node 기반) 코드를  
`px4-ros2-interface-lib` 기반으로 재작성하는 가이드.

**설계 방침:**
- ModeBase 클래스 **하나**만 사용 (공식 권장 방식)
- 비행 단계는 `InternalState` enum으로 내부 분기
- 멀티스레드 / lock-free 큐 구조 **그대로 유지**

---

## 파일 구조

```
collision_avoidance/
├── include/
│   ├── VtolGuidanceMode.hpp      (신규 - 기존 CollisionAvoidanceControl.hpp 대체)
│   └── VtolGuidanceExecutor.hpp  (신규 - 비행 단계 순서 관리)
├── src/
│   ├── VtolGuidanceMode.cpp      (신규 - 기존 CollisionAvoidanceControl.cpp 대체)
│   ├── VtolGuidanceExecutor.cpp  (신규)
│   └── main.cpp                  (수정)
├── FormationControl.hpp           (유지)
├── StateType.hpp                  (유지)
└── TransferSameCoordinate.*       (유지)
```

---

## 핵심 설계 구조

```
VtolGuidanceExecutor (ModeExecutorBase)
│  비행 단계 순서만 관리
│  TakingOff → Transitioning → MyMode → RTL
│
│  scheduleMode(ownedMode().id()) 호출 시
│  setInternalState()로 내부 단계 알려줌
│
└── VtolGuidanceMode (ModeBase) ← owned_mode (하나뿐)
       │  실제 setpoint 계산 및 퍼블리시
       │
       ├── InternalState::FwCruise
       │     sendFwCruiseSetpoint()
       │
       └── InternalState::FormationControl
             sendFormationSetpoint()
             (추가 단계는 여기에 계속 추가)
```

---

## 핵심 변경사항 요약

| 항목 | 기존 | 변경 후 |
|------|------|---------|
| 클래스 상속 | `rclcpp::Node` | `px4_ros2::ModeBase` |
| 비행단계 관리 | `FlightPhase` enum + 타이머 | `ModeExecutorBase::runState()` |
| 내부 단계 분기 | `FlightPhase` switch | `InternalState` switch in `updateSetpoint()` |
| 타이머 콜백 | `create_wall_timer(10ms)` | `updateSetpoint()` 자동 호출 |
| Offboard 하트비트 | `publish_offboard_control_mode()` 직접 | 라이브러리 자동 처리 |
| arm/disarm | `publish_vehicle_command()` 직접 | `ModeExecutorBase` 내부 처리 |
| FW setpoint | `TrajectorySetpoint` (velocity=true 불완전) | `FwLateralLongitudinalSetpoint` |
| 멀티스레드 | `std::thread` + `atomic<bool>` | **그대로 유지** |
| lock-free 큐 | `StateType::InputQueue` | **그대로 유지** |
| 서브스크라이버 | `node.create_subscription()` | **그대로 유지** |

---

## 1. CMakeLists.txt 수정

```cmake
# 추가
find_package(px4_ros2_cpp REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(vtol_guidance_node
    src/main.cpp
    src/VtolGuidanceMode.cpp
    src/VtolGuidanceExecutor.cpp
)

ament_target_dependencies(vtol_guidance_node
    rclcpp
    px4_msgs
    px4_ros2_cpp
)

target_link_libraries(vtol_guidance_node
    Eigen3::Eigen
)

target_include_directories(vtol_guidance_node PUBLIC
    include
)
```

---

## 2. package.xml 수정

```xml
<depend>px4_ros2_cpp</depend>
```

---

## 3. VtolGuidanceMode.hpp

```cpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <limits>
#include <thread>
#include <atomic>
#include <array>
#include <cmath>

/* px4-ros2-interface-lib */
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/fw_lateral_longitudinal.hpp>
#include <px4_ros2/control/setpoint_types/multicopter_trajectory.hpp>

/* px4 msgs */
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/wind.hpp>

/* 기존 모듈 유지 */
#include <collision_avoidance/StateType.hpp>
#include <collision_avoidance/FormationControl.hpp>

class VtolGuidanceMode : public px4_ros2::ModeBase
{
public:
    explicit VtolGuidanceMode(rclcpp::Node & node);
    ~VtolGuidanceMode() override;

    /* ────────────────────────────────────────────────────
       InternalState: 네가 직접 정의하는 내부 비행 단계
       비행 임무가 늘어날수록 여기에 추가하면 됨
       ──────────────────────────────────────────────────── */
    enum class InternalState {
        FwCruise,           /* 고정익 순항 */
        FormationControl,   /* 편대비행 */
        /* 추가 단계가 필요하면 여기에 계속 추가
        Loitering,
        SearchPattern,
        */
    };

    /* Executor가 외부에서 내부 단계 변경할 때 사용 */
    void setInternalState(InternalState state);

    /* 파라미터 설정 (기존 생성자 파라미터 대체) */
    void setVehicleID(int id)        { m_vehicle_ID = id; }
    void setTotalAgentNum(int num)   { m_total_agent_num = num; }
    void setPropagationTime(float t) { m_propagation_time = t; }

    /* ── 라이브러리가 오버라이드 요구하는 함수들 ── */
    void onActivate() override;
    void onDeactivate() override;

    /* 기존 타이머 콜백(10ms) 역할 대체 - 라이브러리가 자동 호출 */
    void updateSetpoint(const rclcpp::Duration & dt) override;

private:
    /* ── 내부 단계별 setpoint 송신 함수 (기존 set_fw_velocity_setpoint 등 대체) ── */
    void sendFwCruiseSetpoint();
    void sendFormationSetpoint();
    /* 추가 단계 함수도 여기에 선언
    void sendLoiteringSetpoint();
    */

    /* ── 바람 보상 계산 (지면속도 → airspeed 변환) ── */
    float computeRequiredAirspeed(float desired_ground_speed, float course);

    /* ── 실시간 스레드 (기존과 완전히 동일) ── */
    void rt_loop();

    /* ── 라이브러리 setpoint 타입 ── */
    std::shared_ptr<px4_ros2::FwLateralLongitudinalSetpointType> _fw_setpoint;
    std::shared_ptr<px4_ros2::TrajectorySetpointType>            _mc_trajectory;

    /* ── 바람 서브스크라이버 (신규: 바람 보상용) ── */
    rclcpp::Subscription<px4_msgs::msg::Wind>::SharedPtr _wind_sub;

    /* TransferSameCoordinate 노드로 부터 발행되는 공통의 좌표계 기준으로 표현이 되는
       state를 서브스크라이 하는 맴버 변수 정의 (기존 trans_odom_subs_ 유지) */
    std::vector<rclcpp::Subscription<
        px4_msgs::msg::VehicleOdometry>::SharedPtr> _trans_odom_subs; /* 지금 trans_node에서 주는 값을 일단 받기 위한 것임 */

    /* ── 내부 단계 상태 변수 (신규) ── */
    InternalState _internal_state{InternalState::FwCruise};

    /* 토픽이름 제작을 위한 맴버 변수 정의 (기존 유지) */
    int   m_vehicle_ID{1};
    int   m_total_agent_num{0};
    float m_propagation_time{0.0f};

    /* ── 바람 추정값 (신규) ── */
    float _wind_n{0.f};
    float _wind_e{0.f};

    /* ── 목표값 (신규) ── */
    float _desired_course{0.f};
    float _desired_altitude{-50.f};
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
```

---

## 4. VtolGuidanceMode.cpp

```cpp
#include "VtolGuidanceMode.hpp"

using namespace px4_msgs::msg;

VtolGuidanceMode::VtolGuidanceMode(rclcpp::Node & node)
: ModeBase(node, Settings{"VTOL Guidance"})
{
    /* ── 라이브러리 setpoint 타입 등록 ── */
    _fw_setpoint = std::make_shared
        <px4_ros2::FwLateralLongitudinalSetpointType>(*this);

    _mc_trajectory = std::make_shared
        <px4_ros2::TrajectorySetpointType>(*this);

    /* 서브스크라이버를 설정 (기존과 동일한 QoS) */
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    /*
       해당 코드가 들어가는 이유는 다음과 같다
       : px4 같은 경우에는 best effort 으로 qos가 발행이 되는데 subscriber 측에서도
         지금 이것을 맞추어 주어야 구독이 가능
       : 따라서 qos의 값을 읽어 들어서 지금 아래의 subscriber 에 할당을 하기 위함이다
    */

    /* ── 바람 서브스크라이버 (신규: airspeed 보상용) ── */
    _wind_sub = node.create_subscription<px4_msgs::msg::Wind>(
        "/px4_" + std::to_string(m_vehicle_ID) + "/fmu/out/wind", qos,
        [this](const px4_msgs::msg::Wind::UniquePtr msg) {
            _wind_n = msg->windspeed_north;
            _wind_e = msg->windspeed_east;
        });

    /* 앞에서 받은 에이전트 수로 지금 반복문으로 서브스크라이버를 일단 생성을 해야 함
       그 다음에 서브스크라이버 이름 및 주소 지정이 가능 (기존과 동일) */
    _trans_odom_subs.resize(m_total_agent_num);
    for (int n = 0; n < m_total_agent_num; n++) {
        _trans_odom_subs[n] = node.create_subscription<VehicleOdometry>(
            "/common/px4_" + std::to_string(n) + "/trans_vehicle_odometry",
            qos,
            [this, n](const VehicleOdometry::UniquePtr msg) {

                /* 실질적으로 m_state_control에 값을 할당 */
                m_state_for_Control[n].check_vehicle_id  = n;
                m_state_for_Control[n].position          = msg->position;
                m_state_for_Control[n].velocity          = msg->velocity;
                m_state_for_Control[n].position_variance = msg->position_variance;
                m_state_for_Control[n].velocity_variance = msg->velocity_variance;
                m_state_for_Control[n].timestamp         = 0.0;

                /* 전체 다 수신이 된 경우 */
                agent_updated_[n] = true;

                bool all_updated = true;
                for (int i = 0; i < m_total_agent_num; i++) {
                    if (!agent_updated_[i]) { all_updated = false; break; }
                }

                /* 전체 서브스크라이브를 다 수신한 경우 */
                if (all_updated) {
                    StateType::Total_state_for_Control snapshot{};
                    snapshot.num_agents = m_total_agent_num;
                    snapshot.agents     = m_state_for_Control;
                    m_input_queue.try_push(snapshot);
                    agent_updated_.fill(false);  /* 플래그 초기화 */
                }
            });
    }

    /* 멀티 스레드 선언 (기존과 동일) */
    rt_running_.store(true); /* 멀티 스레드를 키는 명령어 */
    rt_thread_ = std::thread(&VtolGuidanceMode::rt_loop, this);
}

/* 소멸자 정의 (기존과 동일) */
VtolGuidanceMode::~VtolGuidanceMode()
{
    rt_running_.store(false);
    if (rt_thread_.joinable()) { rt_thread_.join(); }
}

/* ────────────────────────────────────────────────────────────
   setInternalState()
   Executor가 비행 단계 전환 시 호출
   기존 FlightPhase 변경 로직 대체
   ──────────────────────────────────────────────────────────── */
void VtolGuidanceMode::setInternalState(InternalState state)
{
    _internal_state = state;
    RCLCPP_INFO(node().get_logger(),
        "InternalState 변경: %d", static_cast<int>(state));
}

void VtolGuidanceMode::onActivate()
{
    RCLCPP_INFO(node().get_logger(), "VtolGuidanceMode 활성화");
}

void VtolGuidanceMode::onDeactivate()
{
    RCLCPP_INFO(node().get_logger(), "VtolGuidanceMode 비활성화");
}

/* ────────────────────────────────────────────────────────────
   updateSetpoint()
   기존 타이머 콜백(10ms) + publish_control() 역할 대체
   라이브러리가 주기적으로 자동 호출
   _internal_state 값 보고 어떤 setpoint 쏠지 결정
   ──────────────────────────────────────────────────────────── */
void VtolGuidanceMode::updateSetpoint(const rclcpp::Duration & dt)
{
    switch (_internal_state) {

        case InternalState::FwCruise:
            sendFwCruiseSetpoint();
            break;

        case InternalState::FormationControl:
            sendFormationSetpoint();
            break;

        /* 단계 추가 시 여기에 case 추가
        case InternalState::Loitering:
            sendLoiteringSetpoint();
            break;
        */
    }
}

/* ── 고정익 순항 setpoint (기존 set_fw_velocity_setpoint() 대체) ── */
void VtolGuidanceMode::sendFwCruiseSetpoint()
{
    const float NaN = std::numeric_limits<float>::quiet_NaN();

    /* 지면속도 → airspeed 변환 (바람 보상) */
    float required_airspeed =
        computeRequiredAirspeed(_desired_ground_speed, _desired_course);

    px4_ros2::FwLateralLongitudinalSetpoint sp{};

    /* Lateral: 방향 제어 */
    sp.lateral.course               = _desired_course;
    sp.lateral.airspeed_direction   = NaN;
    sp.lateral.lateral_acceleration = NaN;

    /* Longitudinal: 고도 + 속력 제어 */
    sp.longitudinal.altitude            = _desired_altitude;
    sp.longitudinal.height_rate         = NaN;
    sp.longitudinal.equivalent_airspeed = required_airspeed;
    sp.longitudinal.pitch_direct        = NaN;  /* TECS 바이패스 안 함 */
    sp.longitudinal.throttle_direct     = NaN;  /* TECS 바이패스 안 함 */

    _fw_setpoint->update(sp);
}

/* ── 편대비행 setpoint ── */
void VtolGuidanceMode::sendFormationSetpoint()
{
    /* TODO: FormationControl 모듈 결과로 setpoint 계산
     * rt_loop()에서 연산한 결과를 output_queue로 받아서 사용
     * 지금은 FwCruise와 동일하게 유지
     */
    sendFwCruiseSetpoint();
}

/* ── 바람 보상: 지면속도 → airspeed 계산 ── */
float VtolGuidanceMode::computeRequiredAirspeed(
    float desired_gs, float course)
{
    float gs_n = desired_gs * std::cos(course);
    float gs_e = desired_gs * std::sin(course);
    float as_n = gs_n - _wind_n;
    float as_e = gs_e - _wind_e;
    return std::sqrt(as_n * as_n + as_e * as_e);
}

/* ────────────────────────────────────────────────────────────
   멀티스레드에서 돌릴 맴버함수
   기존 CollisionAvoidanceControl::rt_loop()와 완전히 동일
   ──────────────────────────────────────────────────────────── */
void VtolGuidanceMode::rt_loop()
{
    while (rt_running_.load()) {
        std::optional<StateType::Total_state_for_Control> input_state =
            m_input_queue.try_pop();

        if (input_state.has_value())
        {
            /* TODO: collision avoidance / FormationControl 연산 구현 */
            /* auto result = Moudle_FormationControl.compute(input_state.value()); */
            /* output_queue에 결과 push */
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
```

---

## 5. VtolGuidanceExecutor.hpp

```cpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/vehicle_state/vtol.hpp>
#include "VtolGuidanceMode.hpp"

class VtolGuidanceExecutor : public px4_ros2::ModeExecutorBase
{
public:
    VtolGuidanceExecutor(
        rclcpp::Node & node,
        px4_ros2::ModeBase & owned_mode,
        VtolGuidanceMode & mode_ref);  /* InternalState 변경용 참조 */

    /* ────────────────────────────────────────────────────
       State: 비행 임무 순서 (유저 디파인)
       기존 FlightPhase enum과 동일 개념
       Executor 레벨에서 큰 흐름만 관리
       ──────────────────────────────────────────────────── */
    enum class State {
        Idle,
        TakingOff,          /* 기존 TAKEOFF */
        Transitioning,      /* 기존 TRANSITION */
        FwCruise,           /* 기존 FW_CRUISE */
        FormationControl,   /* 신규 */
        /* 추가 임무 단계는 여기에
        Loitering,
        */
        Returning,
        WaitDisarm
    };

    void onActivate() override;
    void onDeactivate(DeactivateReason) override;
    void runState(State state, px4_ros2::Result r);

private:
    rclcpp::Node     & _node;
    VtolGuidanceMode & _mode_ref;  /* setInternalState() 호출용 */
    std::shared_ptr<px4_ros2::VTOL> _vtol;
};
```

---

## 6. VtolGuidanceExecutor.cpp

```cpp
#include "VtolGuidanceExecutor.hpp"

VtolGuidanceExecutor::VtolGuidanceExecutor(
    rclcpp::Node & node,
    px4_ros2::ModeBase & owned_mode,
    VtolGuidanceMode & mode_ref)
: ModeExecutorBase(node, Settings{}, owned_mode)
, _node(node)
, _mode_ref(mode_ref)
{
    _vtol = std::make_shared<px4_ros2::VTOL>(*this);
}

void VtolGuidanceExecutor::onActivate()
{
    /* 활성화되면 바로 이륙 시작 */
    runState(State::TakingOff, px4_ros2::Result::Success);
}

void VtolGuidanceExecutor::onDeactivate(DeactivateReason) {}

/* ────────────────────────────────────────────────────────────
   runState()
   기존 update_flight_phase() 역할 대체
   비동기 콜백 체인으로 단계 전환
   ──────────────────────────────────────────────────────────── */
void VtolGuidanceExecutor::runState(State state, px4_ros2::Result r)
{
    /* 이전 단계 실패 시 중단 */
    if (r != px4_ros2::Result::Success) {
        RCLCPP_ERROR(_node.get_logger(),
            "State %d 실패", static_cast<int>(state));
        return;
    }

    switch (state) {

        /* ── 이륙 (기존 IDLE → TAKEOFF) ── */
        case State::TakingOff:
            RCLCPP_INFO(_node.get_logger(), "[1/5] 이륙 시작");
            takeoff([this](px4_ros2::Result r) {
                runState(State::Transitioning, r);
            });
            break;

        /* ── 고정익 전환 (기존 TRANSITION) ── */
        case State::Transitioning:
            RCLCPP_INFO(_node.get_logger(), "[2/5] 고정익 전환 시작");
            _vtol->toFixedwing([this](px4_ros2::Result r) {
                runState(State::FwCruise, r);
            });
            break;

        /* ── 고정익 순항 (기존 FW_CRUISE) ── */
        case State::FwCruise:
            RCLCPP_INFO(_node.get_logger(), "[3/5] 고정익 순항 시작");
            /* VtolGuidanceMode에 "지금 FwCruise 단계야" 알려줌 */
            _mode_ref.setInternalState(
                VtolGuidanceMode::InternalState::FwCruise);
            /* owned_mode(VtolGuidanceMode) 실행
               → updateSetpoint() 반복 호출 시작
               → FwCruise 케이스 실행됨
               → 완료되면 FormationControl로 이동 */
            scheduleMode(
                ownedMode().id(),
                [this](px4_ros2::Result r) {
                    runState(State::FormationControl, r);
                });
            break;

        /* ── 편대비행 ── */
        case State::FormationControl:
            RCLCPP_INFO(_node.get_logger(), "[4/5] 편대비행 시작");
            /* "지금 FormationControl 단계야" 알려줌 */
            _mode_ref.setInternalState(
                VtolGuidanceMode::InternalState::FormationControl);
            /* 같은 ownedMode 재실행
               → updateSetpoint()에서 FormationControl 케이스 실행됨 */
            scheduleMode(
                ownedMode().id(),
                [this](px4_ros2::Result r) {
                    runState(State::Returning, r);
                });
            break;

        /* 추가 단계 필요 시 여기에 case 추가
        case State::Loitering:
            _mode_ref.setInternalState(
                VtolGuidanceMode::InternalState::Loitering);
            scheduleMode(ownedMode().id(),
                [this](px4_ros2::Result r) {
                    runState(State::Returning, r);
                });
            break;
        */

        /* ── RTL ── */
        case State::Returning:
            RCLCPP_INFO(_node.get_logger(), "[5/5] 복귀 시작");
            rtl([this](px4_ros2::Result r) {
                runState(State::WaitDisarm, r);
            });
            break;

        /* ── Disarm 대기 ── */
        case State::WaitDisarm:
            waitUntilDisarmed([this](px4_ros2::Result r) {
                runState(State::Idle, r);
            });
            break;

        case State::Idle:
            break;
    }
}
```

---

## 7. main.cpp (수정)

```cpp
#include <rclcpp/rclcpp.hpp>
#include "VtolGuidanceMode.hpp"
#include "VtolGuidanceExecutor.hpp"

int main(int argc, char * argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    /* [1] Node 생성 */
    auto node = std::make_shared<rclcpp::Node>("vtol_guidance_node");

    /* [2] 파라미터 선언 및 할당 (기존과 동일) */
    node->declare_parameter<int>("vehicle_ID", 1);
    node->declare_parameter<int>("total_agent_num", 0);
    node->declare_parameter<float>("propagation_time", 0.0f);

    int   vehicle_ID       = node->get_parameter("vehicle_ID").as_int();
    int   total_agent_num  = node->get_parameter("total_agent_num").as_int();
    float propagation_time = node->get_parameter("propagation_time").as_double();

    /* [3] VtolGuidanceMode 생성 (ModeBase - 하나뿐) */
    auto my_mode = std::make_shared<VtolGuidanceMode>(*node);
    my_mode->setVehicleID(vehicle_ID);
    my_mode->setTotalAgentNum(total_agent_num);
    my_mode->setPropagationTime(propagation_time);

    /* [4] Executor 생성
           owned_mode = my_mode (하나뿐)
           mode_ref   = my_mode (setInternalState 호출용) */
    auto executor = std::make_shared<VtolGuidanceExecutor>(
        *node,
        *my_mode,   /* owned_mode */
        *my_mode    /* mode_ref (InternalState 변경용) */
    );

    /* [5] PX4에 등록 */
    if (!executor->doRegister()) {
        throw std::runtime_error("Mode registration failed");
    }

    /* [6] 실행 (기존과 동일) */
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

---

## 8. 비행 단계 추가 방법

비행 임무가 늘어나면 **3곳만 수정**하면 됨:

### Step 1 — VtolGuidanceMode.hpp에 InternalState 추가
```cpp
enum class InternalState {
    FwCruise,
    FormationControl,
    Loitering,          // ← 추가
};
```

### Step 2 — VtolGuidanceMode.cpp에 함수 추가
```cpp
// updateSetpoint() 안에
case InternalState::Loitering:
    sendLoiteringSetpoint();  // ← 추가
    break;

// 함수 구현 추가
void VtolGuidanceMode::sendLoiteringSetpoint() { ... }
```

### Step 3 — VtolGuidanceExecutor.hpp에 State 추가 + cpp에 case 추가
```cpp
// hpp
enum class State {
    ...
    FormationControl,
    Loitering,    // ← 추가
    Returning,
};

// cpp runState() 안에
case State::Loitering:
    _mode_ref.setInternalState(VtolGuidanceMode::InternalState::Loitering);
    scheduleMode(ownedMode().id(),
        [this](px4_ros2::Result r) { runState(State::Returning, r); });
    break;
```

---

## 9. 실행 방법 (기존과 동일)

```bash
# 빌드
cd ~/ros2_ws
colcon build --packages-select collision_avoidance
source install/setup.bash

# 실행 (기존과 동일한 파라미터 방식)
ros2 run collision_avoidance vtol_guidance_node \
  --ros-args \
  -p vehicle_ID:=1 \
  -p total_agent_num:=3 \
  -r __ns:=/px4_1
```

---

## 10. 삭제되는 기존 코드

| 기존 코드 | 삭제 이유 |
|---------|---------|
| `publish_offboard_control_mode()` | 라이브러리 자동 처리 |
| `arm()` / `disarm()` | `ModeExecutorBase::takeoff()` 내부 처리 |
| `publish_vehicle_command()` (전환 명령) | `_vtol->toFixedwing()` 대체 |
| `update_flight_phase()` | `runState()` 대체 |
| `offboard_counter_` | 라이브러리 자동 처리 |
| `FlightPhase` enum | `VtolGuidanceExecutor::State` 대체 |
| `timer_` / `create_wall_timer()` | `updateSetpoint()` 대체 |
| `offboard_control_mode_pub_` | 라이브러리 자동 처리 |
| `position_setpoint_pub_` | `_mc_trajectory->update()` 대체 |
| `velocity_setpoint_pub_` | `_fw_setpoint->update()` 대체 |
| `set_fw_velocity_setpoint()` | `sendFwCruiseSetpoint()` 대체 |
| `set_takeoff_setpoint()` | `ModeExecutorBase::takeoff()` 대체 |

## 11. 그대로 유지하는 것

| 유지 항목 | 이유 |
|---------|------|
| `rt_thread_` / `rt_running_` | 멀티스레드 구조 동일 |
| `rt_loop()` 내부 로직 | 동일 |
| `StateType::InputQueue` | lock-free 큐 동일 |
| `m_input_queue.try_push/pop()` | 동일 |
| `m_state_for_Control` | 동일 |
| `agent_updated_` | 동일 |
| `FormationControl` 모듈 | 동일 |
| `trans_odom_subs_` 서브스크라이버 | 선언 방식 동일 |
| `TransferSameCoordinate` 노드 | 변경 없음 |
| `StateType.hpp` | 변경 없음 |
| 파라미터 선언 방식 | 동일 |
| QoS 설정 방식 | 동일 |
