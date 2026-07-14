#pragma once

/* ────────────────────────────────────────────────────────────
   VtolPreflightMode
   책임: 멀티콥터 → 고정익 천이 + 순항 안정화
   - 천이 명령 송신 (m_vtol->toFixedwing())
   - FW 진입 시 cruise altitude 캡처
   - 5초 안정화 + 모든 기체가 FW 일 때까지 대기
   - 대기 끝나면 completed(Success) → Executor 가 다음 모드 schedule

   설계 의도: "안정화된 코드" — 한번 만들면 거의 수정할 일 없음
              사용자 연구 대상은 FormationMode 임

   Naming convention (클래스 멤버 전용)
     _<name>  : framework 가 관리하는 핸들 (rclcpp / px4_ros2 객체)
     m_<name> : 우리가 소유한 데이터
   ※ 단일 스레드 (main thread 만) — 스레드 접미사 (_mt/_rt/...) 없음.
     자세한 규약은 FormationMode.hpp 헤더 주석 참고.
   ──────────────────────────────────────────────────────────── */

#include <rclcpp/rclcpp.hpp>
#include <array>
#include <atomic>
#include <memory>
#include <vector>
#include <cmath>

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/fixedwing/lateral_longitudinal.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/control/vtol.hpp>
#include <px4_ros2/odometry/global_position.hpp>

#include <px4_msgs/msg/wind.hpp>
#include <px4_msgs/msg/vtol_vehicle_status.hpp>

#include <collision_avoidance/StateType.hpp>   /* kMaxAgents */


class VtolPreflightMode : public px4_ros2::ModeBase
{
public:
    VtolPreflightMode(rclcpp::Node & node, int vehicle_id, int total_agent_num);
    ~VtolPreflightMode() override = default;

    void onActivate() override;
    void onDeactivate() override;
    void updateSetpoint(float dt_s) override;

    /* ── 외부(Executor 또는 FormationMode) 가 읽을 수 있는 값 ── */
    float getCruiseAltitudeAmsl() const { return m_cruise_altitude_amsl; }
    float getDesiredCourse()      const { return m_desired_course; }
    float getDesiredGroundSpeed() const { return m_desired_ground_speed; }

private:
    enum class Phase {
        TransitionToFw,   /* 천이 진행 중 */
        FwCruise          /* 천이 완료, 안정화 대기 */
    };

    void runTransitionToFw();
    void sendFwCruiseSetpoint();
    bool allAgentsInFw() const;
    float computeRequiredAirspeed(float desired_gs, float course) const;

    /* ── ROS2 / PX4 핸들 (외부 라이브러리 — _ prefix 그대로) ── */
    rclcpp::Node & _node;
    std::shared_ptr<px4_ros2::FwLateralLongitudinalSetpointType> _fw_setpoint;
    std::shared_ptr<px4_ros2::TrajectorySetpointType>            _mc_trajectory;
    std::shared_ptr<px4_ros2::VTOL>                              _vtol;
    std::shared_ptr<px4_ros2::OdometryGlobalPosition>            _global_position;
    rclcpp::Subscription<px4_msgs::msg::Wind>::SharedPtr         _wind_sub;
    std::vector<rclcpp::Subscription<
        px4_msgs::msg::VtolVehicleStatus>::SharedPtr>            _vtol_status_subs;
    rclcpp::Time _phase_start_time{};

    /* ── 우리 데이터 (m_ prefix) ── */
    float m_wind_n{0.f};
    float m_wind_e{0.f};
    std::array<std::atomic<bool>, kMaxAgents> m_agent_in_fw{};
    Phase m_phase{Phase::TransitionToFw};

    /* 캡처값 — 천이 완료 시점에 채워짐 */
    float m_cruise_altitude_amsl{NAN};
    float m_desired_course{0.f};
    float m_desired_ground_speed{15.f};

    /* 파라미터 */
    int m_vehicle_id{0};
    int m_total_agent_num{0};

    /* Formation 진입 전 최소 안정화 시간 */
    static constexpr double kMinStabilizeSec = 5.0;
};
