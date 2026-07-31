#pragma once
#include <cstddef>
#include <cmath>      /* NAN 매크로 (height_setpoint 의 default 값) */
#include <collision_avoidance/common/SpscQueue.hpp>

/* ★ 2026-05-13 마이그레이션: PredictState/Input/Params 정의는 본 파일에서 제거.
   collision_avoidance 의 collision_estimation library 가 단일 source of truth.
   본 파일의 *기존 사용처* 들은 sed 일괄 치환으로 namespace 만 변경됨
   (trajectory_prediction:: → collision_avoidance::estimation::).
   *튜닝 인프라 타입* (FwSetpointOutput_rt2mt) 는 trajectory_prediction:: 그대로 유지. */
#include <collision_avoidance/estimation/trajectory_prediction/PredictTypes.hpp>

/* ═══════════════════════════════════════════════════════════════
   trajectory_prediction 전용 StateType
   collision_avoidance/common/StateType.hpp 발췌 — 1대용이라 InputQueue / AgentState 불필요.
   rt → mt 채널 (Setpoint 출력) 만 유지.

   Naming convention (collision_avoidance 와 동일):
     _<name>          : framework 가 관리하는 핸들 (rclcpp / px4_ros2)
     m_<name>         : 우리 데이터 — 단일 thread
     m_<name>_mt      : main thread 전용
     m_<name>_rt      : rt_thread 전용
     m_<name>_mt2rt   : main → rt 단방향 (atomic / queue 필수)
     m_<name>_rt2mt   : rt → main 단방향 (atomic / queue 필수)
   ═══════════════════════════════════════════════════════════════ */

namespace StateType
{
    /* ──────────────────────────────────────────────────────────────
       rt → main 전달용 최종 fw setpoint
       rt_thread 가 SetpointSequencer lookup 결과를 채워 push.
       main thread (updateSetpoint) 가 pop 해서 _fw_setpoint->update() 인가.
       ────────────────────────────────────────────────────────────── */
    struct FwSetpointOutput_rt2mt {
        float airspeed             = 0.f;  /* [m/s] equivalent airspeed */
        float height_rate          = 0.f;  /* [m/s] ENU (NED 의 -v_d) */
        float height_setpoint      = NAN;  /* [m] ★ 신규 — *목표 고도* (NED z 의 부호반전 = altitude up).
                                                  collision_avoidance::FlockingGuidance 가 P 제어로 height_rate
                                                  도출하기 *전* 의 원본 정보. PD 적분 (PredictInput.h_cmd) 으로 사용.
                                                  NaN = 미설정 → trajectory_predict 가 자동으로 1차 지연 fallback. */
        float lateral_acceleration = 0.f;  /* [m/s^2] 횡방향 가속도 */
        bool  is_fallback          = true; /* true → main thread 가 cruise fallback */
    };

    /* ── rt → mt 채널 ── */
    using OutputQueue_rt2mt = SpscQueue<FwSetpointOutput_rt2mt, 8>;
}


/* ═══════════════════════════════════════════════════════════════
   ★ 2026-05-13 마이그레이션 완료:
   PredictState / PredictInput / PredictParams 정의는 본 파일에서 *제거*.
   collision_avoidance::estimation library (PredictTypes.hpp) 가
   *유일한 source of truth*. 본 파일 상단의 #include 가 그 헤더 가져옴.

   본 파일에 남은 것:
     - namespace StateType::FwSetpointOutput_rt2mt (튜닝 인프라 전용, rt→mt 채널)
     - namespace StateType::OutputQueue_rt2mt (SPSC queue alias)

   PredictState/Input/Params 가 필요한 곳은 *namespace 표기를*
   collision_avoidance::estimation:: 으로 사용 (sed 일괄 치환 완료).
   ═══════════════════════════════════════════════════════════════ */
