#pragma once

/* ────────────────────────────────────────────────────────────
   VtolPreflightMode (1대 단순화 버전)
   책임: 멀티콥터 → 고정익 천이 + 순항 안정화 (단일 비행기)
   - 천이 명령 송신 (m_vtol->toFixedwing())
   - FW 진입 시 cruise altitude 캡처
   - 설정된 안정화 시간 후 completed(Success) → Executor 가 다음 모드 schedule

   collision_avoidance 의 VtolPreflightMode 베이스로 다중 비행기 동기화
   (m_total_agent_num, _vtol_status_subs[], allAgentsInFw()) 제거.
   ──────────────────────────────────────────────────────────── */

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <cmath>

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/fixedwing/lateral_longitudinal.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/control/vtol.hpp>
#include <px4_ros2/odometry/global_position.hpp>

#include <px4_msgs/msg/wind.hpp>
#include <px4_msgs/msg/vehicle_air_data.hpp>

class VtolPreflightMode : public px4_ros2::ModeBase
{
public:
    VtolPreflightMode(rclcpp::Node & node, int vehicle_id,
                      double stabilize_s = 5.0);
    ~VtolPreflightMode() override = default;

    void onActivate() override;
    void onDeactivate() override;
    void updateSetpoint(float dt_s) override;

    /* Executor 가 읽어 Replay 모드에 인계 */
    float getCruiseAltitudeAmsl() const { return m_cruise_altitude_amsl; }
    float getDesiredCourse()      const { return m_desired_course; }
    float getDesiredGroundSpeed() const { return m_desired_ground_speed; }

private:
    enum class Phase { TransitionToFw, FwCruise };

    void runTransitionToFw();
    void sendFwCruiseSetpoint();

    /* ── ROS2 / PX4 핸들 ── */
    rclcpp::Node & _node;
    std::shared_ptr<px4_ros2::FwLateralLongitudinalSetpointType> _fw_setpoint;
    std::shared_ptr<px4_ros2::TrajectorySetpointType>            _mc_trajectory;
    std::shared_ptr<px4_ros2::VTOL>                              _vtol;
    std::shared_ptr<px4_ros2::OdometryGlobalPosition>            _global_position;
    rclcpp::Subscription<px4_msgs::msg::Wind>::SharedPtr         _wind_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleAirData>::SharedPtr
        _vehicle_air_data_sub;
    rclcpp::Time _phase_start_time{};

    /* ── 우리 데이터 ── */
    float m_wind_n{0.f};
    float m_wind_e{0.f};
    float m_air_density{1.225F};
    Phase m_phase{Phase::TransitionToFw};

    float m_cruise_altitude_amsl{NAN};
    float m_desired_course{0.f};
    float m_desired_ground_speed{15.f};

    int m_vehicle_id{0};
    double m_stabilize_s{5.0};
    bool m_completion_requested{false};
};
