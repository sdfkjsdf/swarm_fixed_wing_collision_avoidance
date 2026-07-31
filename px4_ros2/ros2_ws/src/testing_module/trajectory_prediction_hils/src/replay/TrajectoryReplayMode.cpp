/* ════════════════════════════════════════════════════════════════════
   TrajectoryReplayMode.cpp — TrajectoryReplayMode.hpp 구현부 (★사용자 모드★)
   ────────────────────────────────────────────────────────────────────
   하는 일 (Producer-Consumer 두 thread):

     [생성자]
       - _fw_setpoint, _vtol_status (px4_ros2 핸들) 생성
       - rt_thread 즉시 시작 (m_rt_running_mt2rt = true)
         단, m_active_mt2rt=false 라 idle (push 안 함)

     [onActivate]   ← Executor 가 Preflight 끝나고 호출
       - hold-last 버퍼 초기화
       - m_reinit_mt2rt = true → rt_thread 가 t0 재캡처 + cursor 0 리셋
       - m_active_mt2rt = true → rt_thread 가 push 시작
       - logger->enable() → CSV 기록 시작

     [onDeactivate]
       - m_active_mt2rt = false → rt_thread idle
       - logger->disable() → CSV 기록 중단

     [updateSetpoint]   ← px4_ros2 가 30Hz 로 호출 (main thread, consumer)
       (1) m_output_queue_rt2mt.try_pop()
       (2) 성공  → m_last_output_mt 갱신
           실패 + 이전 값 있음 → 이전 값 hold (ZOH) + WARN
           실패 + 이전 값 없음 → cruise fallback
       (3) is_fallback 이면 cruise altitude 로 publish
           아니면 sp.withLateralAcceleration().withEquivalentAirspeed()
                    .withHeightRate() → _fw_setpoint->update(sp)
           ※ FormationMode 와 다른 점: height_rate 도 같이 인가
              (시퀀스가 직접 공급하므로 alt_hold P-제어 불필요)
       (4) logger->pushAppliedSetpoint() → CSV 정합 정렬

     [rt_loop]   ← 별도 thread, 1ms 루프 (producer)
       (0) m_reinit_mt2rt 신호 처리: t0 재캡처 + cursor reset
       (1) 비활성 (m_active_mt2rt=false) 이면 idle sleep
       (2) t = steady_clock::now() - m_t0_rt
       (3) m_sequencer->lookup(t)
       (4) FwSetpointOutput_rt2mt 채워서 m_output_queue_rt2mt.try_push()
       (5) 1ms sleep

   t=0 기준점: onActivate() 호출 시점 (Preflight 인계 직후).
   ════════════════════════════════════════════════════════════════════ */

#include <trajectory_prediction_hils/replay/TrajectoryReplayMode.hpp>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;


TrajectoryReplayMode::TrajectoryReplayMode(rclcpp::Node & node,
                                           int vehicle_id,
                                           std::unique_ptr<SetpointSequencer> sequencer,
                                           std::shared_ptr<TrajectoryLogger> logger)
: ModeBase(node, Settings{"VTOL Trajectory Replay"},
           "/px4_" + std::to_string(vehicle_id) + "/")
, _node(node)
, m_vehicle_id(vehicle_id)
, m_sequencer(std::move(sequencer))
, m_logger(std::move(logger))
{
    _fw_setpoint = std::make_shared<px4_ros2::FwLateralLongitudinalSetpointType>(*this);
    _vtol_status = std::make_shared<px4_ros2::VtolStatus>(*this);

    /* rt_thread 는 노드 생성 시점부터 항상 돌도록 시작.
       비활성 시에는 m_active_mt2rt=false 로 idle (push 안 함). */
    m_rt_running_mt2rt.store(true);
    m_rt_thread_mt = std::thread(&TrajectoryReplayMode::rt_loop, this);
}

TrajectoryReplayMode::~TrajectoryReplayMode()
{
    m_rt_running_mt2rt.store(false);
    if (m_rt_thread_mt.joinable()) { m_rt_thread_mt.join(); }
}

void TrajectoryReplayMode::onActivate()
{
    RCLCPP_INFO(_node.get_logger(),
        "[Replay] 활성화 (vehicle=%d cruise_alt=%.1f m, t=0 시작)",
        m_vehicle_id, m_cruise_altitude_amsl);

    /* main thread hold-last 초기화 */
    m_has_last_output_mt = false;
    m_last_output_mt = StateType::FwSetpointOutput_rt2mt{};

    /* ★ baseline 재캡처 플래그 리셋 — 매 case 마다 자극 시작 시점에 다시 캡처 */
    m_baseline_re_captured_mt = false;

    /* 시퀀스 완료 flag 리셋 (이전 비행에서 set 됐을 수 있음) */
    m_sequence_done_rt2mt.store(false);

    /* ★ 본 작업 (2026-05-12 fix): baseline altitude 캡처.
       이전 버전은 m_cruise_altitude_amsl (AMSL 절대 고도 ~497m) 을 set 했으나,
       predict 람다의 x.h = -m.z 는 *home-relative NED 부호반전* (~7m 가량).
       두 origin 의 차이 (~490m) 가 b_h 항 (h_cmd − x.h) 에 곱해져 *4.5초 동안 발산*.
       Fix: 측정 m.z 를 *그 시점* 에 캡처해 home-relative 단위로 통일.
       이로써 predict 람다의 x.h 와 baseline 의 origin 일치 → b_h 항 안정.

       NaN guard: m.z 가 NaN 이면 baseline 미설정 → lookupHeightTarget 도 NaN →
       stepRK4 의 h_cmd NaN guard 가 x.h 로 fallback → b_h 항 0 → 1차 지연 모드. */
    if (m_logger) {
        const auto m = m_logger->getCurrentMeasurements();
        if (std::isfinite(m.z)) {
            const double baseline_h_local = -static_cast<double>(m.z);  /* NED z down → alt up */
            m_logger->setBaselineAlt(baseline_h_local);
            RCLCPP_INFO(_node.get_logger(),
                "[Replay] baseline_alt (home-rel) = %.2f m → predict h_cmd origin "
                "(cruise_amsl=%.1fm 은 RTL fallback 용으로만 보존)",
                baseline_h_local, m_cruise_altitude_amsl);
        } else {
            RCLCPP_WARN(_node.get_logger(),
                "[Replay] m.z NaN → baseline_alt 미설정 → predict h_cmd NaN guard 활성 (1차 지연 모드)");
        }
    }

    /* rt_thread 에 t0 재캡처 + cursor reset 신호 */
    m_reinit_mt2rt.store(true);
    m_active_mt2rt.store(true);

    /* CSV 기록 시작 */
    if (m_logger) m_logger->enable();
}

void TrajectoryReplayMode::onDeactivate()
{
    RCLCPP_INFO(_node.get_logger(), "[Replay] 비활성화");
    m_active_mt2rt.store(false);
    if (m_logger) m_logger->disable();
}

/* ──────────────────────────────────────────────────────────────
   updateSetpoint — output_queue 에서 pop 해서 PX4 에 인가.
   산수는 전혀 없음 (모두 rt_thread 가 수행). FormationMode.cpp:171-233 패턴.
   ────────────────────────────────────────────────────────────── */
void TrajectoryReplayMode::updateSetpoint(float /*dt_s*/)
{
    if (!_vtol_status->isFwMode()) {
        RCLCPP_ERROR_THROTTLE(_node.get_logger(), *_node.get_clock(), 1000,
            "[Replay] VTOL 이 고정익 모드가 아님! (state=%d) → TECS 미작동 위험",
            static_cast<int>(_vtol_status->last().vehicle_vtol_state));
    }

    /* (1) rt_thread 가 push 한 최신 setpoint pop */
    std::optional<StateType::FwSetpointOutput_rt2mt> maybe_out =
        m_output_queue_rt2mt.try_pop();

    if (maybe_out.has_value()) {
        m_last_output_mt     = maybe_out.value();
        m_has_last_output_mt = true;
    } else if (m_has_last_output_mt) {
        RCLCPP_WARN_THROTTLE(_node.get_logger(), *_node.get_clock(), 2000,
            "[Replay] output_queue 비어있음 → hold last (ZOH)");
    } else {
        /* 활성화 직후 rt_thread 가 아직 첫 결과 못 push → cruise fallback */
        if (std::isfinite(m_cruise_altitude_amsl)) {
            _fw_setpoint->updateWithAltitude(
                m_cruise_altitude_amsl, m_initial_course, m_initial_ground_speed);
        } else {
            _fw_setpoint->updateWithHeightRate(0.f, m_initial_course, m_initial_ground_speed);
        }
        /* logger 슬롯을 cruise hold 로 정확히 기록 — 라이프사이클 갭 메움.
           이 push 없으면 슬롯이 atomic 초기값 NaN 으로 남아, wall_timer (10Hz)
           predict 가 NaN 을 atan2 → 첫 CSV 행 trajectory 전체 NaN → chunk 0 발산.
           cruise hold 시점이므로 a_lat=0, h_dot=0, V=initial_ground_speed, fallback=true. */
        if (m_logger) {
            m_logger->pushAppliedSetpoint(m_initial_ground_speed, 0.f, 0.f, /*is_fallback=*/true);
        }
        return;
    }

    /* (2) is_fallback → cruise 인가 */
    if (m_last_output_mt.is_fallback) {
        if (std::isfinite(m_cruise_altitude_amsl)) {
            _fw_setpoint->updateWithAltitude(
                m_cruise_altitude_amsl, m_initial_course, m_initial_ground_speed);
        } else {
            _fw_setpoint->updateWithHeightRate(0.f, m_initial_course, m_initial_ground_speed);
        }
    } else {
        /* (3) 시퀀스 setpoint 직접 인가 — height_rate 도 같이.
               course 는 NAN → lateral_acceleration 이 단독 횡 제어 입력. */
        px4_ros2::FwLateralLongitudinalSetpoint sp;
        sp.withLateralAcceleration(m_last_output_mt.lateral_acceleration)
          .withEquivalentAirspeed(m_last_output_mt.airspeed)
          .withHeightRate(m_last_output_mt.height_rate);
        _fw_setpoint->update(sp);
    }

    /* (4) Logger 에 "이번 사이클 적용한 값" 기록 */
    if (m_logger) {
        m_logger->pushAppliedSetpoint(
            m_last_output_mt.airspeed,
            m_last_output_mt.height_rate,
            m_last_output_mt.lateral_acceleration,
            m_last_output_mt.is_fallback);
    }

    /* (4.5) ★ baseline_alt 재캡처 — sp_a (lateral_acceleration) 변화 첫 시점.
       Beard-McLain 2차 지연 식의 h^c = *제어입력 받는 그 순간의 alt* 로 갱신.
       이전: baseline = Replay 진입 시점 alt (V 가속 자연 climb 자료 못 잡음).
       새: baseline = 자극 시작 시점 alt (cruise V=20 도달 + 자극 동시 시점). */
    if (!m_baseline_re_captured_mt
        && !m_last_output_mt.is_fallback
        && std::abs(m_last_output_mt.lateral_acceleration) > 0.01f
        && m_logger)
    {
        const auto m_now = m_logger->getCurrentMeasurements();
        if (std::isfinite(m_now.z)) {
            const double new_baseline = -static_cast<double>(m_now.z);
            m_logger->setBaselineAlt(new_baseline);
            m_baseline_re_captured_mt = true;
            RCLCPP_INFO(_node.get_logger(),
                "[Replay] ★ 자극 시작 감지 (sp_a=%.3f) — baseline_alt 재캡처 = %.2fm "
                "(이전 onActivate 시점 자리에서 갱신, Beard-McLain h^c 정합)",
                m_last_output_mt.lateral_acceleration, new_baseline);
        }
    }

    /* 10초마다 상태 출력 */
    RCLCPP_INFO_THROTTLE(_node.get_logger(), *_node.get_clock(), 10000,
        "[Replay] vehicle=%d | sp_v=%.2f sp_h=%.2f sp_a=%.2f | fallback=%d",
        m_vehicle_id,
        m_last_output_mt.airspeed,
        m_last_output_mt.height_rate,
        m_last_output_mt.lateral_acceleration,
        m_last_output_mt.is_fallback ? 1 : 0);

    /* (5) rt_thread 가 set 한 시퀀스 완료 flag 확인.
           true 면 단 한 번 completed(Success) 호출 → Executor 가 다음 상태(RTL) 로.
           m_active_mt2rt 가 아직 true 일 때만 호출 (이미 비활성화된 상황 방지).
           ★ 본 작업: completed() 직후 *즉시* rclcpp::shutdown() 호출 — RTL/Disarm 대기
           안 함. CSV 는 TrajectoryLogger 의 destructor (fflush+fclose) 가 보장.
           run_all_cases.sh 가 다음 case 로 빠르게 전환할 수 있도록. */
    if (m_sequence_done_rt2mt.exchange(false) && m_active_mt2rt.load()) {
        RCLCPP_INFO(_node.get_logger(),
            "[Replay] 시퀀스 완료 (t > lastTEnd) → completed(Success) → 즉시 노드 종료 (RTL skip)");
        completed(px4_ros2::Result::Success);
        rclcpp::shutdown();
    }
}

/* ──────────────────────────────────────────────────────────────
   rt_loop — sequencer lookup 결과를 output_queue 로 push.
   FormationMode.cpp:239-328 단순화 (입력 snapshot 없음, sequencer 만 호출).
   ────────────────────────────────────────────────────────────── */
void TrajectoryReplayMode::rt_loop()
{
    bool first_push_done = false;
    bool sequence_done_signaled = false;  /* 단발 신호 가드 (한 활성화당 1회) */

    while (m_rt_running_mt2rt.load()) {

        /* (0) 활성화 직후 t0 재캡처 + cursor reset */
        if (m_reinit_mt2rt.exchange(false)) {
            m_t0_rt = std::chrono::steady_clock::now();
            if (m_sequencer) m_sequencer->resetCursor();
            first_push_done = false;
            sequence_done_signaled = false;
        }

        if (!m_active_mt2rt.load() || !m_sequencer) {
            std::this_thread::sleep_for(1ms);
            continue;
        }

        /* (1) t = now - t0 */
        const double t = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - m_t0_rt).count();

        /* main.cpp wall_timer schedule-aware predict 가 사용 */
        if (m_logger) m_logger->setReplayTime(t);

        /* (2) sequencer lookup */
        const auto sp = m_sequencer->lookup(t);

        /* (2b) 마지막 segment 의 t_end 초과 시 종료 신호 (단발).
               local sequence_done_signaled 가드로 매 1ms store 반복 방지.
               main thread 가 본 후 completed(Success) 호출 → Executor 가 RTL 진입.
               race-free: rt 만 store, mt 만 load+exchange. */
        if (!sequence_done_signaled && t > m_sequencer->lastTEnd()) {
            m_sequence_done_rt2mt.store(true);
            sequence_done_signaled = true;
        }

        /* (3) output queue 로 push */
        StateType::FwSetpointOutput_rt2mt out;
        out.airspeed             = sp.V;
        out.height_rate          = sp.h_dot;
        out.lateral_acceleration = sp.a_lat;
        out.is_fallback          = sp.is_fallback;
        m_output_queue_rt2mt.try_push(out);

        if (!first_push_done) {
            first_push_done = true;
            RCLCPP_INFO(_node.get_logger(),
                "[Replay] 첫 결과 push: t=%.3f V=%.2f h=%.2f a=%.2f fallback=%d",
                t, out.airspeed, out.height_rate, out.lateral_acceleration,
                out.is_fallback ? 1 : 0);
        }

        std::this_thread::sleep_for(1ms);
    }
}
