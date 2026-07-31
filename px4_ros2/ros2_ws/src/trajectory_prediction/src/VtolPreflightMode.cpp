/* ════════════════════════════════════════════════════════════════════
   VtolPreflightMode.cpp — VtolPreflightMode.hpp 구현부
   ────────────────────────────────────────────────────────────────────
   하는 일:
     - 활성화 시 현재 VTOL 상태 확인 → TransitionToFw / FwCruise 분기
     - TransitionToFw: m_vtol->toFixedwing() 호출, 천이 가속도 setpoint 송신
     - FwCruise: cruise altitude 유지하며 5초 안정화 대기
     - 안정화 완료 → completed(Success) → Executor 가 다음 단계 (Replay) 로

   collision_avoidance 의 동명 클래스를 1대용으로 단순화:
     - m_total_agent_num / _vtol_status_subs[] / allAgentsInFw() 제거
     - m_agent_in_fw[] 배열 제거
     - "FW 상태 확인" 은 _vtol->getCurrentState() 단일 체크로 대체
   ════════════════════════════════════════════════════════════════════ */

#include <trajectory_prediction/VtolPreflightMode.hpp>

using namespace px4_msgs::msg;


VtolPreflightMode::VtolPreflightMode(rclcpp::Node & node, int vehicle_id)
: ModeBase(node, Settings{"VTOL Preflight (Replay)"}, "/px4_" + std::to_string(vehicle_id) + "/")
, _node(node)
{
    m_vehicle_id = vehicle_id;

    /* 라이브러리 setpoint / VTOL / GlobalPosition 핸들 */
    _fw_setpoint    = std::make_shared<px4_ros2::FwLateralLongitudinalSetpointType>(*this);
        /*
            _fw_setpoint    = std::make_shared<px4_ros2::FwLateralLongitudinalSetpointType>(*this); 의 정확한 의미는
            지금 px4_ros2::FwLateralLongitudinalSetpointType에 정의된 class 타입으로 지금 _fw_setpoint  이라는 새로운 객체를 만든다는 것임
        
        */
    _mc_trajectory  = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    _vtol           = std::make_shared<px4_ros2::VTOL>(*this);
    _global_position = std::make_shared<px4_ros2::OdometryGlobalPosition>(*this);

    /* PX4 best-effort QoS 매칭 */
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);


    

    /* 바람 구독 (airspeed 계산용) */
    _wind_sub = _node.create_subscription<Wind>(
        "/px4_" + std::to_string(m_vehicle_id) + "/fmu/out/wind", qos,
        [this](const Wind::UniquePtr msg) {
            m_wind_n = msg->windspeed_north;
            m_wind_e = msg->windspeed_east;
        });
}

void VtolPreflightMode::onActivate()
{
    RCLCPP_INFO(_node.get_logger(), "[Preflight] 활성화 (vehicle_id=%d)", m_vehicle_id);

    const auto vtol_state = _vtol->getCurrentState();
    if (vtol_state == px4_ros2::VTOL::State::FixedWing) {
        m_phase = Phase::FwCruise;
        if (_global_position->positionValid()) {
            m_cruise_altitude_amsl =
                static_cast<float>(_global_position->position().z());
        }
    } else {
        m_phase = Phase::TransitionToFw;
    }
    _phase_start_time = _node.get_clock()->now();
}

void VtolPreflightMode::onDeactivate()
{
    RCLCPP_INFO(_node.get_logger(), "[Preflight] 비활성화");
}

void VtolPreflightMode::updateSetpoint(float /*dt_s*/)
{
    switch (m_phase) {

        case Phase::TransitionToFw:
            runTransitionToFw();
            break;

        case Phase::FwCruise: {
            sendFwCruiseSetpoint();

            /* Replay 진입 조건: 천이 후 5초 안정화 (단일 기체이므로 동기화 게이트 없음) */
            const double elapsed =
                (_node.get_clock()->now() - _phase_start_time).seconds();

            const bool fw_ok =
                (_vtol->getCurrentState() == px4_ros2::VTOL::State::FixedWing);

            if (fw_ok && elapsed > kMinStabilizeSec) {
                RCLCPP_INFO(_node.get_logger(),
                    "[Preflight] 안정화 완료 → completed() (cruise alt=%.1f m)",
                    m_cruise_altitude_amsl);
                completed(px4_ros2::Result::Success);
            }
            break;
        }
    }
}

void VtolPreflightMode::runTransitionToFw()
{
    _vtol->toFixedwing();

    const auto state = _vtol->getCurrentState();

    if (state == px4_ros2::VTOL::State::TransitionToFixedWing) {
        Eigen::Vector3f acceleration_sp = _vtol->computeAccelerationSetpointDuringTransition();
        Eigen::Vector3f velocity_sp{NAN, NAN, 0.f};

        _mc_trajectory->update(velocity_sp, acceleration_sp);
        _fw_setpoint->updateWithHeightRate(0.f, m_desired_course);

    } else if (state == px4_ros2::VTOL::State::FixedWing) {
        if (_global_position->positionValid()) {
            m_cruise_altitude_amsl =
                static_cast<float>(_global_position->position().z());
            RCLCPP_INFO(_node.get_logger(),
                "[Preflight] 천이 완료 → FwCruise (alt=%.1f m AMSL)",
                m_cruise_altitude_amsl);
        } else {
            RCLCPP_WARN(_node.get_logger(),
                "[Preflight] 천이 완료, 그러나 global position invalid");
        }
        m_phase            = Phase::FwCruise;
        _phase_start_time = _node.get_clock()->now();
    }
}

void VtolPreflightMode::sendFwCruiseSetpoint()
{
    const float required_airspeed =
        computeRequiredAirspeed(m_desired_ground_speed, m_desired_course);

    if (!std::isfinite(m_cruise_altitude_amsl)) {
        _fw_setpoint->updateWithHeightRate(0.f, m_desired_course, required_airspeed);
        return;
    }

    _fw_setpoint->updateWithAltitude(
        m_cruise_altitude_amsl,
        m_desired_course,
        required_airspeed);
}

float VtolPreflightMode::computeRequiredAirspeed(float desired_gs, float course) const
{
    const float gs_n = desired_gs * std::cos(course);
    const float gs_e = desired_gs * std::sin(course);
    const float as_n = gs_n - m_wind_n;
    const float as_e = gs_e - m_wind_e;
    return std::sqrt(as_n * as_n + as_e * as_e);
}
