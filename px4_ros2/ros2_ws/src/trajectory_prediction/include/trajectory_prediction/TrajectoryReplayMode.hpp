#pragma once

/* ═══════════════════════════════════════════════════════════════
   TrajectoryReplayMode (FormationMode 자리)
   책임: SetpointSequencer lookup → fw_setpoint publish + Logger 기록

   Producer-Consumer 패턴 (FormationMode 와 동일):
     rt_thread (1ms 루프, producer):
       lookup(t) → FwSetpointOutput_rt2mt → m_output_queue_rt2mt.try_push()
     main thread updateSetpoint (consumer, 30Hz):
       try_pop() → ZOH (m_last_output_mt) → _fw_setpoint->update(sp)
                 → m_logger->pushAppliedSetpoint(...)
       (queue 비고 이전 값 없으면 cruise fallback — Preflight 와 동일 로직)

   t=0 기준점: onActivate() 호출 시점 (Preflight 인계 직후).
   FormationMode 와 다르게 height_rate 도 함께 publish (시퀀스가 직접 공급).
   ═══════════════════════════════════════════════════════════════ */

#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <cmath>

#include <px4_msgs/msg/vehicle_attitude.hpp>  

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/fixedwing/lateral_longitudinal.hpp>
#include <px4_ros2/vehicle_state/vtol_status.hpp>

#include <trajectory_prediction/StateType.hpp>
#include <trajectory_prediction/SetpointSequencer.hpp>
#include <trajectory_prediction/TrajectoryLogger.hpp>


class TrajectoryReplayMode : public px4_ros2::ModeBase
{
public:
    TrajectoryReplayMode(rclcpp::Node & node,
                         int vehicle_id,
                         std::unique_ptr<SetpointSequencer> sequencer,
                         std::shared_ptr<TrajectoryLogger> logger);
    ~TrajectoryReplayMode() override;

    void onActivate() override;
    void onDeactivate() override;
    void updateSetpoint(float dt_s) override;

    /* Executor 가 Preflight 종료 시점에 캡처한 cruise altitude / 코스 주입.
       Replay 활성화 전에 호출되어야 함. */
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

    /* ── ROS2 / PX4 핸들 ── */
    rclcpp::Node & _node;
    std::shared_ptr<px4_ros2::FwLateralLongitudinalSetpointType> _fw_setpoint;
    std::shared_ptr<px4_ros2::VtolStatus> _vtol_status;

    /* ── 우리 데이터 ── */
    int m_vehicle_id{0};

    std::unique_ptr<SetpointSequencer> m_sequencer;   /* rt_thread 단독 접근 */
    std::shared_ptr<TrajectoryLogger>  m_logger;      /* main thread 만 호출 */

    /* ── mt: rt_thread 핸들 ── */
    std::thread m_rt_thread_mt;

    /* ── mt → rt: 종료 / 활성화 / 재초기화 신호 (atomic) ── */
    std::atomic<bool> m_rt_running_mt2rt{false};
    std::atomic<bool> m_active_mt2rt{false};
    std::atomic<bool> m_reinit_mt2rt{false};

    /* ── rt → mt: 시퀀스 끝 도달 신호 (한 번만 사용) ──
       rt_loop 에서 t > sequencer.lastTEnd() 도달 시 true.
       updateSetpoint 에서 본 후 completed(Success) 호출하고 클리어. */
    std::atomic<bool> m_sequence_done_rt2mt{false};

    /* ── rt → mt: 최종 setpoint 채널 ── */
    StateType::OutputQueue_rt2mt m_output_queue_rt2mt{};

    /* ── mt: updateSetpoint 의 hold-last 버퍼 ── */
    StateType::FwSetpointOutput_rt2mt m_last_output_mt{};
    bool m_has_last_output_mt{false};

    /* ── mt: ★ 자극 시작 시점 baseline 재캡처 플래그 (2026-05-13).
       onActivate 시점에 캡처한 baseline_alt 는 Replay 진입 직후 alt 라
       V 가속 자연 climb 자료를 못 잡음. *제어입력 (sp_a) 변화 첫 시점* 에
       baseline 재캡처 → Beard-McLain 2차 지연 식의 h^c = *자극 시점 alt*. */
    bool m_baseline_re_captured_mt{false};

    /* ── rt: 시퀀스 시간 기준점 (rt_thread 단독) ── */
    std::chrono::steady_clock::time_point m_t0_rt{};

    /* ── immutable after Executor 주입 (활성화 전에 한 번만 set) ── */
    float m_cruise_altitude_amsl{NAN};
    float m_initial_course{0.f};
    float m_initial_ground_speed{15.f};
};
