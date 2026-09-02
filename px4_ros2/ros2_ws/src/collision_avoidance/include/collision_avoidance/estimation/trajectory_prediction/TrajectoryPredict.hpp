#pragma once

/* ════════════════════════════════════════════════════════════════════
   TrajectoryPredict.hpp — 7-state Beard-McLain (2012) 식 (9.19) 정형
                           reduced-order guidance model 의 RK4 예측기

   2026-05-13 마이그레이션: trajectory_prediction 패키지에서 본 위치로 이동.
     - 이전: trajectory_prediction/include/trajectory_prediction/TrajectoryPredict.hpp
     - 현재: collision_avoidance/include/collision_avoidance/collision_estimation/
              trajectory_prediction/TrajectoryPredict.hpp
     - namespace: trajectory_prediction → collision_avoidance::estimation

   기준 문서: md_file/TASK_trajectory_predictor.md (trajectory_prediction 패키지)
              md_file/PROPOSAL_trajectory_predictor_update.md
              md_file/PATCH_trajectory_predictor_phi_based.md (2026-05-11)
   ────────────────────────────────────────────────────────────────────
   Reference:
     Beard, R. W., & McLain, T. W. (2012).
     "Small Unmanned Aircraft: Theory and Practice", Princeton University Press.
     Equation (9.19) — reduced-order guidance model.

   Implementation note (interface vs internal model split):
     - External input (PredictInput) accepts a_lat_cmd [m/s²]
       → matches types::FwSetpoint::lateral_acceleration.
     - Internal state and ODE use phi (roll angle) as state variable
       and phi_cmd as control, following Beard-McLain (9.19) exactly.
     - Conversion phi_cmd = atan2(a_lat_cmd, g) is applied once per
       stepRK4() entry, not per ODE evaluation.
     - Altitude channel: PD form reduced to 1st-order lag (b_h = 0).

   PX4 parameter mapping:
     tau_V    <-> FW_T_TAS_TC
     tau_hdot <-> FW_T_ALT_TC
     tau_phi  <-> FW_R_TC (default 0.5s) ★ direct mapping ★

   Caveat: PX4 time constants are "error convergence" TCs.
     tau_phi maps most directly to the roll loop and can typically
     adopt FW_R_TC value without further identification.
   ────────────────────────────────────────────────────────────────────
   설계 원칙 (명세 §1, §4, §6):
     - rt_thread (1ms) 컨텍스트 안에서 직접 호출 가능 — stateless / heap-free / lock-free
     - 외부 의존 0   : Eigen / rclcpp / px4_msgs include 금지
     - POD 만        : PredictState / PredictInput (PredictTypes.hpp)
     - 결과 보관     : 호출자 멤버 std::array (본 클래스는 안 들고 있음, §5.1)
     - 분기 회피     : clamp 를 fmin/fmax 로 (§6.2 branch predictor miss 회피)
     - rt_thread 가 도는 동안 setParams 호출 금지 (initialization-time only, §2.4)
   ════════════════════════════════════════════════════════════════════ */

#include <array>
#include <algorithm>
#include <cstddef>
#include <cmath>

#include <collision_avoidance/estimation/trajectory_prediction/PredictTypes.hpp>


namespace collision_avoidance::estimation
{

class TrajectoryPredict
{
public:
    explicit TrajectoryPredict(const PredictParams & params);

    /* ── 단일 step RK4 ───────────────────────────────────────────────
       입력 u 는 zero-order hold 가정 (k1~k4 모두 같은 u 평가).
       psi 는 [-pi, pi] 로 wrap. saturation 재적용 후 반환.
       호출자가 매 step 마다 다른 u 를 쓰고 싶으면 직접 stepRK4 를 N 회 호출. */
    PredictState stepRK4(const PredictState & x,
                         const PredictInput & u,
                         double dt) const;

    /* ── 다중 step 예측 (zero-order hold) ────────────────────────────
       out_traj[0]   = x0  (입력 그대로)
       out_traj[k+1] = stepRK4(out_traj[k], u_zoh, dt)
       호출자가 std::array<PredictState, N_STEPS> 를 멤버로 보유 권장 (§5.1).
       함수 안에서 stack 큰 배열 만들지 않음 — out_traj 가 호출자 멤버. */
    template <std::size_t N_STEPS>
    void predict(const PredictState & x0,
                 const PredictInput & u_zoh,
                 double dt,
                 std::array<PredictState, N_STEPS> & out_traj) const
    {
        static_assert(N_STEPS >= 1, "N_STEPS must be >= 1");
        out_traj[0] = x0;
        for (std::size_t k = 0; k + 1 < N_STEPS; ++k) {
            out_traj[k + 1] = stepRK4(out_traj[k], u_zoh, dt);
        }
    }

    /* ── 파라미터 갱신 (initialization-time only) ────────────────────
       rt_thread 가 도는 도중에 호출하면 race. 명세 §2.4 / §4.3. */
    void setParams(const PredictParams & params);

    const PredictParams & getParams() const { return m_params; }

    /* ── extractKeySamples — 4 시점 sample 추출 (2026-05-13 추가) ─────
       이미 적분된 trajectory 에서 *충돌 검사 핵심 시점* 4 점만 추출:
         k=0  (t=0.0s) : 3D 위치 + 3D 속도
         k=15 (t=1.5s) : 3D 위치
         k=30 (t=3.0s) : 3D 위치
         k=45 (t=4.5s) : 3D 위치 + 3D 속도

       속도 재구성 (NED ground-relative frame):
         vel.x (vn) = sqrt(V² - h_dot²) * cos(psi)
         vel.y (ve) = sqrt(V² - h_dot²) * sin(psi)
         vel.z (vd) = -h_dot
       위치 (NED frame):
         pos.x = p_n, pos.y = p_e, pos.z = -h (altitude → NED down)

       설계:
         - stateless / RT-safe / heap 0  (TrajectoryPredict 의 기존 원칙 유지)
         - 외부 의존 0 (Eigen / rclcpp 사용 없음, std::cmath 만)
         - template 으로 N_STEPS 무관 — 컴파일러 인스턴스화
         - static_assert 로 N_STEPS > kSampleIdx_t45 보장

       호출 예:
         std::array<PredictState, 46> traj;
         predictor.predict(x0, u, 0.1, traj);            // 46 점 적분
         TrajectorySample s = predictor.extractKeySamples(traj);
         // s.pos_t0, s.vel_t0, s.pos_t15, ..., s.pos_t45, s.vel_t45 사용 */
    template <std::size_t N_STEPS>
    TrajectorySample extractKeySamples(
        const std::array<PredictState, N_STEPS> & trajectory) const
    {
        static_assert(N_STEPS > kSampleIdx_t45,
                      "N_STEPS must be greater than kSampleIdx_t45 (=45). "
                      "Use kPredictHorizon=46 (endpoint-inclusive) trajectory.");

        TrajectorySample s;
        extractAt(trajectory[kSampleIdx_t0],  s.pos_t0,  &s.vel_t0);
        extractAt(trajectory[kSampleIdx_t15], s.pos_t15, nullptr);
        extractAt(trajectory[kSampleIdx_t30], s.pos_t30, nullptr);
        extractAt(trajectory[kSampleIdx_t45], s.pos_t45, &s.vel_t45);
        return s;
    }

private:
    /* 내부 전용: 외부 a_lat_cmd 를 stepRK4 진입 시 atan2(a_lat_cmd, g) 로
       phi_cmd 로 변환한 결과. RK4 4 stages 에서 같은 값 재사용 (ZOH 가정).
       evaluateODE 가 받는 입력 타입. */
    struct InternalInput {
        double V_cmd;
        double h_cmd;         /* [m] — h_cmd 그대로 전달 (NaN guard 는 stepRK4 안에서) */
        double h_dot_cmd;
        double phi_cmd;       /* [rad] — atan2(a_lat_cmd, g) */
    };

    /* §2.3 — 7개 ODE 평가. saturation 은 호출자가 미리 적용해서 넣어야 함. */
    PredictState evaluateODE(const PredictState & x,
                             const InternalInput & u) const;

    /* §2.5 — 입력 saturation: V_cmd, h_dot_cmd, a_lat_cmd 클램프 */
    PredictInput applyInputSaturation(const PredictInput & u) const;

    /* §2.5 — 상태 안전성: |h_dot| < 0.99 V, V_h >= V_h_min, psi wrap,
                          phi clamp [-60°, +60°] (tan 발산 방지) */
    PredictState applyStateSafety(const PredictState & x) const;

    /* ── extractKeySamples 의 private helper ──
       단일 PredictState 에서 (pos, vel) 추출.
       out_vel == nullptr 이면 *위치만* 추출 (vel 계산 skip → cos/sin/sqrt 절약). */
    void extractAt(const PredictState & x,
                   Vec3 & out_pos,
                   Vec3 * out_vel) const;

    PredictParams m_params;   /* rt_thread 시작 후 변경 금지 */
};

} /* namespace collision_avoidance::estimation */
