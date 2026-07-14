#pragma once

/* ═══════════════════════════════════════════════════════════════
   PredictTypes.hpp — trajectory prediction 알고리즘 입출력 POD 정의

   2026-05-13 마이그레이션: 기존 trajectory_prediction/StateType.hpp:59-128 의
   *알고리즘 관련 타입* (PredictState, PredictInput, PredictParams) 만 추출.
   trajectory_prediction 의 *튜닝 인프라 타입* (FwSetpointOutput_rt2mt, OutputQueue_rt2mt
   등) 은 *trajectory_prediction 패키지에 그대로 잔류*.

   기준 문서: md_file/TASK_trajectory_predictor.md §2.1, §2.2, §4.3
   ───────────────────────────────────────────────────────────────

   설계 원칙:
     - 외부 의존 0  : Eigen / rclcpp / px4_msgs include 금지
     - POD 만      : sizeof, layout 명확, cache 친화
     - double      : 1차 지연 시정수 적분 정확도 확보
     - stateless   : 클래스 자체는 m_params 외 상태 없음 (§4.3)
   ═══════════════════════════════════════════════════════════════ */


namespace collision_avoidance::collision_estimation
{

/* §2.1 — 7-state, 56 byte = 1 cache line.
   ★ PATCH (2026-05-11): 마지막 변수 a_lat → phi (롤각 [rad]).
       Beard-McLain (9.19) 정형 모델로 전환. 외부 PredictInput 은 그대로
       a_lat_cmd 받음 (stepRK4 진입 시 atan2 로 phi_cmd 변환). */
struct PredictState {
    double p_n;     /* [m]    NED north                                */
    double p_e;     /* [m]    NED east                                 */
    double h;       /* [m]    altitude (up positive)                   */
    double V;       /* [m/s]  airspeed magnitude                       */
    double psi;     /* [rad]  heading, wrapped to [-pi, pi]            */
    double h_dot;   /* [m/s]  current climb rate (autopilot tracking)  */
    double phi;     /* [rad]  current roll angle (autopilot tracking)  */
};
static_assert(sizeof(PredictState) == 56,
              "PredictState must remain a single cache line (56B / 64B).");

/* §2.2 — 4-input (★ 본 작업: h_cmd 추가, Beard-McLain (9.19) PD 형태). */
struct PredictInput {
    double V_cmd;       /* [m/s]   airspeed setpoint                                  */
    double h_cmd;       /* [m]     altitude setpoint (목표 고도). NaN 이면 evaluateODE
                                     에서 P-term 0 으로 자동 fallback */
    double h_dot_cmd;   /* [m/s]   climb-rate setpoint                                */
    double a_lat_cmd;   /* [m/s²]  lateral acceleration setpoint                      */
};

/* §2.4 + §2.5 — 시정수 + 입력/상태 한계 + 수치 안전값.
   ★ 본 작업: 종 채널이 PD 형태로 확장 (Beard-McLain 9.19). b_h 추가. */
struct PredictParams {
    /* 1차 지연 시정수 [s]  (§2.4 표) */
    double tau_V       = 4.0;     /* FW_T_TAS_TC */
    double tau_hdot    = 2.0;     /* FW_T_ALT_TC (rate term inner loop) */
    double tau_phi     = 0.5;     /* FW_R_TC (default 0.5s) — roll loop 직접 매핑 */

    /* ★ 신규 — 종 채널 PD 의 altitude P 게인 [1/s].
       collision_avoidance::FlockingGuidance 의 alt_hold_p_gain 과 등가.
       0.0 으로 두면 기존 1차 지연 동작과 동일 (default 활성). */
    double b_h         = 0.0;

    /* ★ β_φ (roll-altitude coupling) 항 *제거* (2026-05-13 최종):
       옵션 A1 시험: d.h_dot += β_φ·(1−cos(φ)) 형태로 추가했었음.
       timing fix 적용 후 β_φ=0 vs 0.5 수치적 동일 (h end 차 0.01m) → 제거.
       *결정적 자리* = baseline_alt timing 동기화 (compare_single_input.py:97 의 +5 row).

       TODO(question): 6-DOF 확장 시 roll-altitude coupling 재도입 필요 여부.
         재도입 조건 (검증 기준):
           - 큰 roll (φ > 25°) 자리에서 h 예측 오차가 다시 증가 (h end > 0.5m)
           - 또는 R 시리즈 외 *회피 시나리오* (a_lat pulse) 에서 mismatch 발견
         재도입 시 참조:
           - 식 형태: TrajectoryPredict.cpp 이전 commit (β_φ·(1−cos φ) 추가/제거 자리)
           - 대안 형태: β_φ·sin²(φ) (옵션 A2), β_φ·tan²(φ) (옵션 A3) — 미시험
           - 또는 PX4 fw_att_control 의 turn coordination FF 직접 모사 (큰 작업) */

    /* 입력 saturation 한계 (§2.5) */
    double V_min       = 12.0;    /* FW_AIRSPD_MIN              */
    double V_max       = 25.0;    /* FW_AIRSPD_MAX              */
    double h_dot_max   = 5.0;     /* FW_T_CLMB_MAX (대칭 가정) */
    double a_lat_max   = 9.8;     /* g * tan(FW_R_LIM)          */

    /* 수치 안전 (§2.5) — sqrt(V^2 - h_dot^2) 가 0 으로 가는 것 방지 */
    double V_h_min     = 1.0;     /* [m/s] 수평 속도 floor      */
};


/* ════════════════════════════════════════════════════════════════════
   ★ 2026-05-13 추가: trajectory sample 추출 POD 와 인덱스 상수.

   목적: 45+1 점 trajectory 에서 *충돌 검사 핵심 시점* 4 점만 추출.
         TrajectoryPredict::extractKeySamples() 가 본 타입 반환.
         ReconstructTrajectory 도 같은 POD 사용 (중복 제거).
   ════════════════════════════════════════════════════════════════════ */

/* ── 3D 벡터 POD (위치/속도 공통) ──
   외부 의존 0 을 위해 Eigen::Vector3f 대신 자체 POD.
   호출자가 ROS2 geometry_msgs 등으로 변환 시 .x/.y/.z 직접 접근.

   ★ 2026-05-13 사용자 지적 — *float* 사용 이유:
     - PX4 측정값 (vehicle_local_position.vx, vy, vz 등) 이 *float32* → 일관
     - ROS2 publish 대역폭 *50% 절약* (Float32MultiArray, 18 × 4 = 72 byte)
     - 충돌 회피 정확도엔 *float 도 충분* (정밀도 1e-7 m ≈ 0.1µm, safety margin 5m 의 5e7 배)
     - PredictState (algorithm 내부) 는 *double 유지* — RK4 적분 정밀도 위함.
       float ← double narrowing 은 extractAt 에서 *명시적 static_cast*. */
struct Vec3 {
    float x;
    float y;
    float z;
};


/* ── 4 시점 sample 결과 POD ──
   - t=0:   ego 의 현재 위치 + 속도   (collision check 시작점)
   - t=1.5: 1.5s 후 위치              (중간 충돌 검사용)
   - t=3.0: 3.0s 후 위치              (중간 충돌 검사용)
   - t=4.5: 4.5s 후 위치 + 속도       (look-ahead 끝점, 방향성)

   *좌표계 = NED* :
     - pos.x = p_n (north)
     - pos.y = p_e (east)
     - pos.z = -h  (NED down, *altitude up 의 부호 반전*)
     - vel.x = V_h · cos(ψ) = vn  (NED north velocity)
     - vel.y = V_h · sin(ψ) = ve  (NED east velocity)
     - vel.z = -h_dot              (NED down velocity)
     여기서 V_h = √(V² - h_dot²)   (수평 속도)
     course-angle 패치 (2026-05-13) 후 ψ = atan2(ve, vn) 이고 V = |v_ground|
     이므로 *재구성된 (vn, ve, vd) 가 EKF 의 ground velocity 와 정확히 일치*. */
struct TrajectorySample {
    /* t = 0.0 s */
    Vec3 pos_t0;
    Vec3 vel_t0;

    /* t = 1.5 s */
    Vec3 pos_t15;

    /* t = 3.0 s */
    Vec3 pos_t30;

    /* t = 4.5 s */
    Vec3 pos_t45;
    Vec3 vel_t45;
};


/* ── 빌드 타임 sample 인덱스 (constexpr — 컴파일러가 최적화) ──
   본 4 인덱스는 0.1s 간격 trajectory 에서 t=0, 1.5, 3.0, 4.5s 위치에 해당.
   kPredictHorizon (=46, endpoint-inclusive) 와의 정합성은 TrajectoryLogger.hpp
   의 kPredictHorizon static_assert + 호출 시점의 N_STEPS template 매개변수 검증. */
inline constexpr std::size_t kSampleIdx_t0  = 0;    /* t = 0.0 s */
inline constexpr std::size_t kSampleIdx_t15 = 15;   /* t = 1.5 s */
inline constexpr std::size_t kSampleIdx_t30 = 30;   /* t = 3.0 s */
inline constexpr std::size_t kSampleIdx_t45 = 45;   /* t = 4.5 s (horizon endpoint) */

static_assert(kSampleIdx_t0  < kSampleIdx_t15 &&
              kSampleIdx_t15 < kSampleIdx_t30 &&
              kSampleIdx_t30 < kSampleIdx_t45,
              "Sample indices must be monotonically increasing.");
static_assert(kSampleIdx_t45 == 45,
              "kSampleIdx_t45 must be 45 for horizon=46 endpoint-inclusive (0-based last index).");

} /* namespace collision_avoidance::collision_estimation */
