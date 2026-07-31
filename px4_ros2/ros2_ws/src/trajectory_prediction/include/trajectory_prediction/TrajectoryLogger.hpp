#pragma once

/* ────────────────────────────────────────────────────────────
   TrajectoryLogger
   책임:
     - PX4 로부터 vehicle_local_position, vehicle_attitude, airspeed_validated 구독
       (main thread executor 가 콜백 실행 — atomic snapshot 슬롯에 store)
     - 별도 ROS2 TimerBase 50Hz 로 CSV 한 줄 fwrite
       (publish jitter 와 분리 — 디스크 I/O 가 setpoint 발행 흔들지 않게)
     - TrajectoryReplayMode 가 매 publish 마다 pushAppliedSetpoint() 호출 →
       "보낸 값" 과 "측정값" 같은 행에 정렬
     - main.cpp 의 wall_timer 가 매 predict 후 publishPredictBundle(traj, m) 호출 →
       7-state 예측 (PredictState) × kPredictHorizon + 예측 시점 측정 snapshot (9 fields)
       을 같은 CSV 행에 펼쳐 기록.

   상태 / setpoint 는 atomic, PredictBundle (traj + m_at_pred) 는 mutex 보호.
   (predict array ≈ 2.5 KB + MeasuredSnapshot ≈ 36 byte. atomic 으로 감싸기엔 큼.
    한 mutex 한 struct 로 묶어서 *traj 와 m_at_pred 가 같은 predict 호출에서
    derive 됐다* 는 invariant 를 publish/consume 양쪽에서 보장.)

   CSV 컬럼 (md_file/TASK_trajectory_predictor.md 7-state + Option A m_*_atp 반영):
     t_us, sp_v, sp_h, sp_a, sp_fallback,
     x, y, z, vn, ve, vd, yaw, roll, true_airspeed,
     m_x_atp, m_y_atp, m_z_atp, m_vn_atp, m_ve_atp, m_vd_atp,
     m_yaw_atp, m_roll_atp, m_V_atp,
     p_pn_0, p_pe_0, p_h_0, p_V_0, p_psi_0, p_hdot_0, p_phi_0,
     p_pn_1, ..., p_phi_(kPredictHorizon-1)
   ──────────────────────────────────────────────────────────── */

#include <rclcpp/rclcpp.hpp>
#include <array>
#include <atomic>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/airspeed_validated.hpp>

#include <trajectory_prediction/StateType.hpp>

/* ★ 2026-05-20 — reconstruct (clamped cubic spline) 통합.
   publishPredictBundle 안에서 *4 시점 sample 추출 + spline 계수 산출 + 46점 평가*
   를 한 번에 수행하여 PredictBundle 의 spline 필드에 저장. 별도 spline CSV 에
   46 × 6 (NED pos+vel) = 276 컬럼 + t_us = 277 컬럼 으로 dump. */
#include <collision_avoidance/collision_estimation/reconstruct_trajectory/ReconstructTrajectory.hpp>


class TrajectoryLogger
{
public:
    /* ── horizon (predict 점 개수, endpoint-inclusive).
       ★ 2026-05-13 컨벤션 변경: predict_horizon_endpoint_s × predict_rate_hz + 1.
         "endpoint_s = 4.5초, rate = 10Hz" 이면 *0초, 0.1, ..., 4.5초* 의 46 점 (양 끝점 포함).
         이전 컨벤션 (+1 안 함, 45점 = 0~4.4초) 과의 차이는 마지막 점 0.1s.
         사용 정합성: yaml 의 predict_horizon_endpoint_s = 4.5 × 10Hz + 1 = 46. */
    static constexpr std::size_t kPredictHorizon = 46;   /* = 4.5s × 10Hz + 1 (endpoint-inclusive) */

    using PredictedTrajectory =
        std::array<collision_avoidance::collision_estimation::PredictState, kPredictHorizon>;

    /* 6-state 측정 + yaw + roll + airspeed 를 한 번에 atomic load 해서 반환.
       TODO(question) [향후 v2 작업 — 2026-05-11 사용자 확정으로 본 작업에서 변경 안 함]:
         명세 (claude_code_task_spec.md A-1) 는 double 권장. 기존 atomic 슬롯이 모두
         float 이라 일관성 위해 float 유지. 정밀도 요구 시 v2 에서 일괄 double 변경. */
    struct MeasuredSnapshot {
        float x, y, z;          /* NED [m] */
        float vn, ve, vd;       /* NED [m/s] */
        float yaw;              /* [rad] */
        float roll;             /* [rad] — ★ A-1 추가: 7-state φ 의 실측 초기값 */
        float true_airspeed;    /* [m/s] */
    };

    struct SetpointSnapshot {
        float V;
        float h_dot;
        float a_lat;
        bool  is_fallback;
    };

    /* topic_namespace_prefix: PX4 uXRCE 토픽 prefix.
       - "" (default)        : single-vehicle SITL — `/fmu/out/...` 직접 구독
       - "/px4_<id>"          : multi-vehicle SITL — `/px4_<id>/fmu/out/...` 구독
       호출자 (main.cpp) 가 파라미터로 결정. */
    TrajectoryLogger(rclcpp::Node & node,
                     int vehicle_id,
                     const std::string & csv_path_prefix,
                     double log_rate_hz = 50.0,
                     const std::string & topic_namespace_prefix = "");
    ~TrajectoryLogger();

    /* TrajectoryReplayMode 가 publish 직후 호출 — atomic store 만 */
    void pushAppliedSetpoint(float V, float h_dot, float a_lat, bool is_fallback);

    /* ── PredictBundle ──
       traj (예측 trajectory) + m_at_pred (예측 호출 *시점*의 측정 snapshot) 을
       하나의 struct 로 묶음. *같은 predict() 호출에서 나왔다* 는 invariant 를
       타입 + 단일 mutex 보호 + 단일 publish 함수로 강제.
       향후 prediction 이 RT thread 안으로 이전될 때 mutex → SPSC queue 로
       1:1 치환하면 됨 — struct / 호출부는 그대로. */
    /* ★ 2026-05-20 — spline 평가 결과 보관용 array.
       publishPredictBundle 안에서 reconstruct(t=0.0, 0.1, ..., 4.5) 호출 → kPredictHorizon 점
       의 PoseVel (pos+vel) 채움. onTick 이 spline CSV 에 dump 시 그대로 사용. */
    using SplineTrajectory =
        std::array<collision_avoidance::collision_estimation::PoseVel, kPredictHorizon>;

    struct PredictBundle {
        PredictedTrajectory traj;
        MeasuredSnapshot    m_at_pred;
        bool                valid;
        SplineTrajectory    spline;        /* ★ 2026-05-20: 46점 spline 평가 결과 */
    };

    /* main.cpp wall_timer (현재) / 미래 RT thread 가 predict 직후 호출.
       traj 와 m_at_pred 가 *반드시 같은 호출 안에서 derive 된 쌍* 이어야 함.
       (m_at_pred 가 predict 의 x0 를 만든 그 측정값과 동일 시각이라는 invariant). */
    void publishPredictBundle(const PredictedTrajectory & traj,
                              const MeasuredSnapshot &    m_at_pred);

    /* main.cpp wall_timer 가 predict 호출 직전에 가져갈 측정값 (한 번에) */
    MeasuredSnapshot getCurrentMeasurements() const;
    SetpointSnapshot getCurrentSetpointSnapshot() const;

    /* Replay rt_loop 가 매 cycle 마다 자신의 시퀀스 시간 (t = now - t0) 을 push.
       main.cpp wall_timer 가 schedule-aware predict 시 sequencer 미래 lookup 에 사용.
       NaN = Replay 비활성 (Preflight 단계 또는 종료 후). */
    void   setReplayTime(double t_relative);
    double getReplayTime() const;

    /* ★ 신규 — Replay 시작 시점의 cruise altitude 캡처 (PX4 의 alt_sp_internal 초기값 등가).
       TrajectoryReplayMode::onActivate 가 호출.
       main.cpp wall_timer 가 sequencer->lookupHeightTarget(t, baseline_alt) 에 사용. */
    void   setBaselineAlt(double alt);
    double getBaselineAlt() const;

    /* 로깅 시작/정지 — Replay 모드 onActivate/onDeactivate 에서 호출 권장 */
    void enable()  { m_enabled.store(true,  std::memory_order_relaxed); }
    void disable() { m_enabled.store(false, std::memory_order_relaxed); }

private:
    void onTick();
    static std::string makeFilename(const std::string & prefix);

    /* ★ 2026-05-20 — spline CSV 의 별도 파일명 생성.
       prefix 끝에 "_spline" 추가 후 동일 timestamp 사용 → predict CSV 와 *짝* 관계. */
    static std::string makeFilenameSpline(const std::string & prefix);

    rclcpp::Node & _node;
    int            m_vehicle_id;

    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _local_pos_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr      _attitude_sub;
    rclcpp::Subscription<px4_msgs::msg::AirspeedValidated>::SharedPtr    _airspeed_sub;
    rclcpp::TimerBase::SharedPtr _tick_timer;

    FILE * _fp{nullptr};
    std::string m_filename;

    /* ★ 2026-05-20 — spline CSV file handle + 파일명. predict CSV 와 동일 timestamp,
       suffix 만 "_spline". onTick 시 두 파일에 동일 t_us row 동시 write. */
    FILE *      _fp_spline{nullptr};
    std::string m_filename_spline;

    /* ★ 2026-05-20 — reconstruct 도구 (main thread executor 안에서만 호출).
       publishPredictBundle 안에서 calculate_clamp_cubic_spline + reconstruct(t) 호출.
       _mt suffix: 현재는 main thread executor 단일 컨텍스트. RT thread 통합 시 reconsider. */
    collision_avoidance::collision_estimation::ReconstructTrajectory m_reconstructor_mt;

    /* ── 마지막 측정 snapshot (콜백 단일 thread 에서 store, tick 에서 load) ── */
    std::atomic<float> m_x{NAN}, m_y{NAN}, m_z{NAN};
    std::atomic<float> m_vn{NAN}, m_ve{NAN}, m_vd{NAN};
    std::atomic<float> m_yaw{NAN};
    std::atomic<float> m_roll{NAN};   /* ★ A-1: vehicle_attitude quaternion → roll */
    std::atomic<float> m_true_airspeed{NAN};

    /* ── 마지막 적용 setpoint (Replay 가 store, tick 이 load) ── */
    std::atomic<float> m_sp_v{NAN}, m_sp_h{NAN}, m_sp_a{NAN};
    std::atomic<bool>  m_sp_fallback{true};

    /* ── Replay 시퀀스 상대 시간 (Replay rt_loop 가 store, wall_timer 가 load) ── */
    std::atomic<double> m_replay_t_relative{std::nan("")};

    /* ── Replay baseline altitude (Replay onActivate 가 store, wall_timer 가 load) ──
       시퀀스 시작 시점의 cruise alt. PX4 의 alt_sp_internal 초기값과 등가. */
    std::atomic<double> m_baseline_alt_mt2rt{std::nan("")};

    /* ── 마지막 예측 bundle (traj + 예측 시점 측정 snapshot, mutex 보호) ──
       _mt 접미사: 현재는 main thread (rclcpp executor) 단일 컨텍스트에서만 접근.
       추후 rt_thread 통합 시 SPSC queue 로 교체 + _rt2mt 로 rename 예정.
       두 데이터를 *한 mutex 한 struct* 로 묶어서 RT 이전 후에도 atomic publish
       invariant 가 자동 보존되도록 설계. */
    mutable std::mutex  m_predict_mutex_mt;
    PredictBundle       m_pred_bundle_mt{};

    std::atomic<bool>    m_enabled{false};
};
