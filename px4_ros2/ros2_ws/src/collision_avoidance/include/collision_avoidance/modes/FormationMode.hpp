#pragma once

/* ═══════════════════════════════════════════════════════════════
   FormationMode
   책임: ★사용자 연구 대상★ — 순수한 편대비행 가이던스

   Producer-Consumer 패턴:
     rt_thread (producer):
       input_queue pop → Flocking 가속도 → 적분 → saturation
                      → (course, airspeed, height_rate) 변환
                      → output_queue.push()
     main thread updateSetpoint (consumer, 30Hz):
       output_queue.try_pop()
         - 성공: m_last_output_mt 갱신 + fw_setpoint->update()
         - 실패 + 이전 값 있음 (hold-last): 이전 값 재publish + 경고 로그
         - 실패 + 이전 값 없음 (초기 상태): cruise fallback

   ───────────────────────────────────────────────────────────────
   Naming convention (클래스 멤버 전용)

     _<name>          : framework 가 관리하는 핸들·플러그인
                        - rclcpp 객체 (Node, Subscription, Time)
                        - px4_ros2 객체 (FwSetpoint, VTOL, ModeBase 등)
                        - doRegister() 로 framework 에 등록된 우리 ModeBase 서브클래스
                          (예: VtolPreflightMode, FormationMode reference)
                        → 즉 우리 코드가 lifecycle 을 직접 관리하지 않는 모든 것

     m_<name>         : 우리가 완전히 소유한 데이터 (framework 와 무관, 단일 thread)
     m_<name>_mt      : 우리 데이터 — main thread 전용
     m_<name>_rt      : 우리 데이터 — rt_thread 전용
     m_<name>_mt2rt   : 우리 데이터 — main → rt 단방향 통신 (atomic / queue 필수)
     m_<name>_rt2mt   : 우리 데이터 — rt → main 단방향 통신 (atomic / queue 필수)

   지역변수는 접미사 없음 — 함수 스코프가 이미 스레드 컨텍스트를 결정함.
   ═══════════════════════════════════════════════════════════════ */

#include <rclcpp/rclcpp.hpp>
#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <thread>
#include <utility>
#include <vector>
#include <cmath>
#include <functional>

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/fixedwing/lateral_longitudinal.hpp>
#include <px4_ros2/vehicle_state/vtol_status.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_air_data.hpp>
#include <px4_msgs/msg/wind.hpp>

#include <collision_avoidance/common/GlobalTypes.hpp>
#include <collision_avoidance/guidance/FlockingGuidance.hpp>
#include <collision_avoidance/guidance/PointConvergenceGuidance.hpp>
#include <collision_avoidance/selection/ManeuverSelectionWorker.hpp>


class FormationMode : public px4_ros2::ModeBase
{
public:
    FormationMode(rclcpp::Node & node, int vehicle_id, int total_agent_num);
    ~FormationMode() override;

    void onActivate() override;
    void onDeactivate() override;
    void updateSetpoint(float dt_s) override;

    void setManeuverSelectionDecision(
        const collision_avoidance::selection::ManeuverSelectionDecision & decision)
    {
        m_maneuver_decision_mt = decision;
        m_has_maneuver_decision_mt = true;
    }

    void setCollisionAvoidanceShadowOnly(bool shadow_only)
    {
        m_collision_avoidance_shadow_only_mt = shadow_only;
    }

    void setManeuverActivationGateCallback(
        std::function<void(bool)> callback)
    {
        m_maneuver_activation_gate_callback_mt = std::move(callback);
    }

    void setNominalSetpointCallback(
        std::function<void(
            const collision_avoidance::selection::
                ManeuverSelectionNominalSetpointSnapshot &)> callback)
    {
        m_nominal_setpoint_callback_mt = std::move(callback);
    }

    /* Executor 가 Preflight 종료 시점에 캡처한 cruise altitude / 초기 코스를 주입.
       FormationMode 활성화 전에 호출되어야 함. */
    void setInitialCruiseState(float cruise_altitude_amsl,
                               float initial_course,
                               float initial_ground_speed)
    {
        m_cruise_altitude_amsl = cruise_altitude_amsl;
        m_initial_course       = initial_course;
        m_initial_ground_speed = initial_ground_speed;
    }

private:
    void rt_loop();

    /* ── ROS2 / PX4 핸들 (외부 라이브러리 — 관례상 _ prefix 그대로) ── */
    rclcpp::Node & _node;
    std::shared_ptr<px4_ros2::FwLateralLongitudinalSetpointType> _fw_setpoint;
    std::shared_ptr<px4_ros2::VtolStatus> _vtol_status;
    rclcpp::Subscription<px4_msgs::msg::Wind>::SharedPtr _wind_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleAirData>::SharedPtr
        _vehicle_air_data_sub;
    std::vector<rclcpp::Subscription<
        px4_msgs::msg::VehicleOdometry>::SharedPtr> _trans_odom_subs;

    /* ── 우리 데이터 ── */
    int m_vehicle_id{0};
    int m_total_agent_num{0};

    /* ── mt → rt: 바람 값 (콜백 쓰기, rt_loop 읽기) ── */
    std::atomic<float> m_wind_n_mt2rt{0.f};
    std::atomic<float> m_wind_e_mt2rt{0.f};
    float m_air_density_mt{1.225F};

    /* ── mt: 콜백이 채우는 누적 상태 ── */
    std::array<collision_avoidance::types::ControlState,
               collision_avoidance::types::kMaxAgents> m_state_for_control_mt{};
    collision_avoidance::types::AgentUpdateFlags m_agent_updated_mt{};

    /* ── mt → rt: swarm snapshot 채널 ── */
    collision_avoidance::types::ControlInputQueue m_input_queue_mt2rt{};

    /* ── immutable: Flocking 모듈 (포인터 자체는 생성 후 불변,
                    dereference 는 rt_loop 만) ── */
    std::unique_ptr<FlockingGuidance> m_flocking;
    std::unique_ptr<collision_avoidance::guidance::PointConvergenceGuidance>
        m_point_convergence;
    bool m_point_convergence_test_mode{false};

    /* ── mt: rt_thread 핸들 ── */
    std::thread m_rt_thread_mt;

    /* ── mt → rt: rt_thread 종료 플래그 (atomic) ── */
    std::atomic<bool> m_rt_running_mt2rt{false};

    /* ── rt → mt: 최종 setpoint 채널 ── */
    collision_avoidance::types::FwSetpointQueue m_output_queue_rt2mt{};

    /* ── mt: updateSetpoint 의 hold-last 버퍼 ── */
    collision_avoidance::types::FwSetpoint m_last_output_mt{};
    bool m_has_last_output_mt{false};

    /* ── rt: rt_loop 전용 정적 이웃 버퍼 (heap 할당 없음) ── */
    collision_avoidance::types::AgentStateArray m_others_buf_rt{};

    /* ── mt → rt: 활성화 시 V_desired 재초기화 신호 (atomic) ── */
    std::atomic<bool> m_reinit_vdesired_mt2rt{false};

    /* ── immutable after Executor 주입 (활성화 전에 한 번만 set) ── */
    float m_cruise_altitude_amsl{NAN};
    float m_initial_course{0.f};
    float m_initial_ground_speed{15.f};
    float m_minimum_level_eas{12.f};
    float m_gravity{9.80665f};

    /* ── mt: 고도 P-제어 ── */
    float m_ref_pos_d_mt{0.f};
    bool  m_ref_pos_d_valid_mt{false};
    float m_alt_hold_p_gain{0.5f};
    float m_alt_hold_hr_max{3.0f};

    /* ── mt: distributed maneuver-selection result and local command gate ── */
    collision_avoidance::selection::ManeuverSelectionDecision
        m_maneuver_decision_mt{};
    bool m_has_maneuver_decision_mt{false};
    bool m_collision_avoidance_shadow_only_mt{true};
    std::function<void(bool)> m_maneuver_activation_gate_callback_mt;
    std::function<void(
        const collision_avoidance::selection::
            ManeuverSelectionNominalSetpointSnapshot &)>
        m_nominal_setpoint_callback_mt;
    std::uint64_t m_latest_self_state_timestamp_us_mt{0};
};
