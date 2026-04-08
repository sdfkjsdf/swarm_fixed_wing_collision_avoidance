#include <collision_avoidance/VtolPreflightMode.hpp>

using namespace px4_msgs::msg;


VtolPreflightMode::VtolPreflightMode(rclcpp::Node & node, int vehicle_id, int total_agent_num)
: ModeBase(node, Settings{"VTOL Preflight"}, "/px4_" + std::to_string(vehicle_id) + "/")
, _node(node)
{
    m_vehicle_id      = vehicle_id;
    m_total_agent_num = total_agent_num;

    /* 라이브러리 setpoint / VTOL / GlobalPosition 핸들 */
    _fw_setpoint    = std::make_shared<px4_ros2::FwLateralLongitudinalSetpointType>(*this);
    _mc_trajectory  = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    _vtol           = std::make_shared<px4_ros2::VTOL>(*this);
    _global_position = std::make_shared<px4_ros2::OdometryGlobalPosition>(*this);

    /* PX4 best-effort QoS 매칭 */
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    /* 바람 구독 */
    _wind_sub = _node.create_subscription<Wind>(
        "/px4_" + std::to_string(m_vehicle_id) + "/fmu/out/wind", qos,
        [this](const Wind::UniquePtr msg) {
            m_wind_n = msg->windspeed_north;
            m_wind_e = msg->windspeed_east;
        });

    /* 모든 기체의 VTOL 상태 구독 (Formation 진입 동기화 게이트) */
    _vtol_status_subs.resize(m_total_agent_num);
    for (int n = 0; n < m_total_agent_num; n++) {
        _vtol_status_subs[n] = _node.create_subscription<VtolVehicleStatus>(
            "/px4_" + std::to_string(n) + "/fmu/out/vtol_vehicle_status", qos,
            [this, n](const VtolVehicleStatus::UniquePtr msg) {
                const bool in_fw =
                    (msg->vehicle_vtol_state == VtolVehicleStatus::VEHICLE_VTOL_STATE_FW);
                m_agent_in_fw[n].store(in_fw);
            });
    }
}

void VtolPreflightMode::onActivate()
{
    RCLCPP_INFO(_node.get_logger(), "[Preflight] 활성화");

    /* 활성화 시 현재 VTOL 상태에 따라 시작 단계 결정 */
    const auto vtol_state = _vtol->getCurrentState();
    if (vtol_state == px4_ros2::VTOL::State::FixedWing) {
        m_phase = Phase::FwCruise;
        /* 이미 FW 면 현재 고도를 즉시 캡처 */
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

        case Phase::FwCruise:
            sendFwCruiseSetpoint();
            {
                /* Formation 진입 조건:
                   1. 모든 기체가 FW
                   2. 천이 후 최소 5초 안정화 */
                const double elapsed =
                    (_node.get_clock()->now() - _phase_start_time).seconds();

                if (allAgentsInFw() && elapsed > kMinStabilizeSec) {
                    RCLCPP_INFO(_node.get_logger(),
                        "[Preflight] 안정화 완료 → completed() (cruise alt=%.1f m)",
                        m_cruise_altitude_amsl);
                    completed(px4_ros2::Result::Success);
                } else if (static_cast<int>(elapsed) % 5 == 0 && elapsed > 4.5) {
                    int waiting = 0;
                    for (int i = 0; i < m_total_agent_num; i++) {
                        if (!m_agent_in_fw[i].load()) waiting++;
                    }
                    if (waiting > 0) {
                        RCLCPP_INFO_THROTTLE(_node.get_logger(),
                            *_node.get_clock(), 5000,
                            "[Preflight] FW 진입 대기: %d/%d 미완료",
                            waiting, m_total_agent_num);
                    }
                }
            }
            break;
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

bool VtolPreflightMode::allAgentsInFw() const
{
    for (int i = 0; i < m_total_agent_num; i++) {
        if (!m_agent_in_fw[i].load()) {
            return false;
        }
    }
    return (m_total_agent_num > 0);
}

float VtolPreflightMode::computeRequiredAirspeed(float desired_gs, float course) const
{
    const float gs_n = desired_gs * std::cos(course);
    const float gs_e = desired_gs * std::sin(course);
    const float as_n = gs_n - m_wind_n;
    const float as_e = gs_e - m_wind_e;
    return std::sqrt(as_n * as_n + as_e * as_e);
}
