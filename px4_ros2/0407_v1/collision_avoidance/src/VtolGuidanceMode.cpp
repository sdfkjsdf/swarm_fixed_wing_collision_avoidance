#include <collision_avoidance/VtolGuidanceMode.hpp>

using namespace px4_msgs::msg;

VtolGuidanceMode::VtolGuidanceMode(rclcpp::Node & node, int vehicle_id)
: ModeBase(node, Settings{"VTOL Guidance"}, "/px4_" + std::to_string(vehicle_id) + "/")
, _node(node)
{
    m_vehicle_ID = vehicle_id;
    /* ── 라이브러리 setpoint / VTOL / GlobalPosition 핸들 등록 ── */
    _fw_setpoint     = std::make_shared<px4_ros2::FwLateralLongitudinalSetpointType>(*this);
    _mc_trajectory   = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    _vtol            = std::make_shared<px4_ros2::VTOL>(*this);
    _global_position = std::make_shared<px4_ros2::OdometryGlobalPosition>(*this);

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
    _wind_sub = _node.create_subscription<Wind>(
        "/px4_" + std::to_string(m_vehicle_ID) + "/fmu/out/wind", qos,
        [this](const Wind::UniquePtr msg) {
            _wind_n = msg->windspeed_north;
            _wind_e = msg->windspeed_east;
        });

    /* 앞에서 받은 에이전트 수로 지금 반복문으로 서브스크라이버를 일단 생성을 해야 함
       그 다음에 서브스크라이버 이름 및 주소 지정이 가능 (기존과 동일) */
    _trans_odom_subs.resize(m_total_agent_num);
    for (int n = 0; n < m_total_agent_num; n++) {
        _trans_odom_subs[n] = _node.create_subscription<VehicleOdometry>(
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

void VtolGuidanceMode::onActivate()
{
    RCLCPP_INFO(_node.get_logger(), "VtolGuidanceMode 활성화");

    /* 활성화 시 현재 VTOL 상태에 따라 시작 단계 결정 */
    const auto vtol_state = _vtol->getCurrentState();
    if (vtol_state == px4_ros2::VTOL::State::FixedWing) {
        _internal_state = InternalState::FwCruise;
    } else {
        _internal_state = InternalState::TransitionToFw;
    }
    _phase_start_time = _node.get_clock()->now();
}

void VtolGuidanceMode::onDeactivate()
{
    RCLCPP_INFO(_node.get_logger(), "VtolGuidanceMode 비활성화");
}

/* ────────────────────────────────────────────────────────────
   updateSetpoint()
   기존 타이머 콜백(10ms) + publish_control() 역할 대체
   라이브러리가 주기적으로 자동 호출
   _internal_state 값 보고 어떤 setpoint 쏠지 결정
   ──────────────────────────────────────────────────────────── */
void VtolGuidanceMode::updateSetpoint(float /*dt_s*/)
{
    switch (_internal_state) {

        case InternalState::TransitionToFw:
            runTransitionToFw();
            break;

        case InternalState::FwCruise:
            sendFwCruiseSetpoint();
            /* 30초 순항 후 편대비행으로 진입 (임시 조건) */
            if ((_node.get_clock()->now() - _phase_start_time).seconds() > 30.0) {
                _internal_state   = InternalState::FormationControl;
                _phase_start_time = _node.get_clock()->now();
                RCLCPP_INFO(_node.get_logger(), "InternalState → FormationControl");
            }
            break;

        case InternalState::FormationControl:
            sendFormationSetpoint();
            /* 60초 편대 후 종료 (임시 조건) */
            if ((_node.get_clock()->now() - _phase_start_time).seconds() > 60.0) {
                _internal_state = InternalState::Done;
                RCLCPP_INFO(_node.get_logger(), "InternalState → Done");
            }
            break;

        /* 단계 추가 시 여기에 case 추가
        case InternalState::Loitering:
            sendLoiteringSetpoint();
            break;
        */

        case InternalState::Done:
            /* Mode 완료 신호 → Executor 가 다음 상태(RTL) 로 진행 */
            completed(px4_ros2::Result::Success);
            break;
    }
}

/* ── 멀티콥터 → 고정익 천이 (기존 publish_vehicle_command(VTOL_TRANSITION,4) 대체) ── */
void VtolGuidanceMode::runTransitionToFw()
{
    _vtol->toFixedwing();

    const auto state = _vtol->getCurrentState();

    if (state == px4_ros2::VTOL::State::TransitionToFixedWing) {
        /* 천이 중: 라이브러리가 권장하는 조합 setpoint 인가
           - 멀티콥터: 천이 가속도 setpoint
           - 고정익: height_rate=0 (고도 유지) + 현재 코스 유지 */
        Eigen::Vector3f acceleration_sp = _vtol->computeAccelerationSetpointDuringTransition();
        Eigen::Vector3f velocity_sp{NAN, NAN, 0.f};

        _mc_trajectory->update(velocity_sp, acceleration_sp);
        _fw_setpoint->updateWithHeightRate(0.f, _desired_course);

    } else if (state == px4_ros2::VTOL::State::FixedWing) {
        /* 천이 완료 시점의 현재 AMSL 고도를 순항 고도로 캡처
           (이륙 → 천이 → 그 고도 그대로 유지하는 것이 가장 안전) */
        if (_global_position->positionValid()) {
            _cruise_altitude_amsl =
                static_cast<float>(_global_position->position().z());
            RCLCPP_INFO(_node.get_logger(),
                "고정익 전환 완료 → FwCruise (캡처된 고도 %.1f m AMSL)",
                _cruise_altitude_amsl);
        } else {
            RCLCPP_WARN(_node.get_logger(),
                "고정익 전환 완료했지만 global position invalid → height_rate=0 모드로 진입");
        }
        _internal_state   = InternalState::FwCruise;
        _phase_start_time = _node.get_clock()->now();
    }
}

/* ── 고정익 순항 setpoint (기존 set_fw_velocity_setpoint() 대체) ── */
void VtolGuidanceMode::sendFwCruiseSetpoint()
{
    /* 지면속도 → airspeed 변환 (바람 보상) */
    const float required_airspeed =
        computeRequiredAirspeed(_desired_ground_speed, _desired_course);

    /* 천이 시점에 캡처한 AMSL 고도가 없으면 height_rate=0 (현재 고도 유지) */
    if (!std::isfinite(_cruise_altitude_amsl)) {
        _fw_setpoint->updateWithHeightRate(0.f, _desired_course, required_airspeed);
        return;
    }

    /* 헬퍼 사용: 캡처한 AMSL 고도 + 코스 + 등가대기속도 */
    _fw_setpoint->updateWithAltitude(
        _cruise_altitude_amsl,
        _desired_course,
        required_airspeed);
}

/* ── 편대비행 setpoint ── */
void VtolGuidanceMode::sendFormationSetpoint()
{
    /* TODO: FormationControl 모듈 결과로 setpoint 계산
       rt_loop()에서 연산한 결과를 output_queue로 받아서 사용
       지금은 FwCruise와 동일하게 유지 */
    sendFwCruiseSetpoint();
}

/* ── 바람 보상: 지면속도 → airspeed 계산 ── */
float VtolGuidanceMode::computeRequiredAirspeed(float desired_gs, float course)
{
    const float gs_n = desired_gs * std::cos(course);
    const float gs_e = desired_gs * std::sin(course);
    const float as_n = gs_n - _wind_n;
    const float as_e = gs_e - _wind_e;
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
