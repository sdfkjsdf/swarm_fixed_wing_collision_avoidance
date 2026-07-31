/* ════════════════════════════════════════════════════════════════════
   TrajectoryPredict.cpp — TrajectoryPredict.hpp 구현부

   2026-05-13 마이그레이션: trajectory_prediction 패키지에서 본 위치로 이동.
     - 이전: trajectory_prediction/src/TrajectoryPredict.cpp
     - 현재: collision_avoidance/src/estimation/trajectory_prediction/TrajectoryPredict.cpp
     - namespace: trajectory_prediction → collision_avoidance::estimation

   기준 문서: md_file/TASK_trajectory_predictor.md  (§2.3, §2.5, §7)
              md_file/PATCH_trajectory_predictor_phi_based.md (2026-05-11)
   ────────────────────────────────────────────────────────────────────
   evaluateODE (§2.3, PATCH 적용 후 — Beard-McLain 정형):
     V_h         = sqrt(V^2 - h_dot^2)         (수평 속도, V_h_min 으로 floor)
     p_n_dot     = V_h * cos(psi)
     p_e_dot     = V_h * sin(psi)
     h_dot_pos   = h_dot                        (위치 h 의 미분 = 상태 h_dot)
     V_dot       = (V_cmd     - V    ) / tau_V
     psi_dot     = g * tan(phi) / V_h           ★ 조정선회 (정형)
     h_ddot      = (h_dot_cmd - h_dot) / tau_hdot
     phi_dot     = (phi_cmd   - phi  ) / tau_phi   ★ roll loop 1차 지연

   외부 입력 변환 (stepRK4 진입 시 1회):
     phi_cmd     = atan2(a_lat_cmd, g)          a_lat_cmd ∈ [-g·tan(45°), +g·tan(45°)]
                                                → phi_cmd ∈ [-45°, +45°]

   applyInputSaturation (§2.5, 변경 없음):
     V_cmd     <- clamp(V_cmd,    V_min,         V_max)
     h_dot_cmd <- clamp(h_dot_cmd, -h_dot_max,   +h_dot_max)
     a_lat_cmd <- clamp(a_lat_cmd, -a_lat_max,   +a_lat_max)
     fmin/fmax 만 사용 (분기 회피, §6.2).

   applyStateSafety (§2.5):
     V         <- max(V, V_h_min)               (수치 안전 floor)
     |h_dot|   <- clamp(h_dot, ±0.99·V)        (V_h 발산 방지)
     phi       <- clamp(phi, ±60°)              ★ tan 발산 방지 (FW_R_LIM 45° + 마진)
     psi       <- wrapPi(psi)

   stepRK4 (§7):
     u_sat = applyInputSaturation(u)
     x_sat = applyStateSafety(x)
     u_int = { V_cmd, h_dot_cmd, atan2(a_lat_cmd, g) }     ★ 1회 변환
     k1..k4 = f(...)
     x_next = applyStateSafety( x + dt/6·(k1+2k2+2k3+k4) )
   ════════════════════════════════════════════════════════════════════ */

#include <collision_avoidance/estimation/trajectory_prediction/TrajectoryPredict.hpp>

#include <cmath>


namespace collision_avoidance::estimation
{
namespace
{

/* 표준 중력 — PATCH §3.3 의 constexpr 9.80665 */
constexpr double k_g = 9.80665;

/* phi hard limit — PATCH §3.6, FW_R_LIM 45° 에 안전 마진 두고 ±60° */
constexpr double k_phi_hard_limit = 1.047197551;   /* 60° in rad */

/* psi 를 [-pi, pi] 로 wrap — 분기 없음 (fmod 2회) */
inline double wrapPi(double a)
{
    constexpr double kTwoPi = 2.0 * M_PI;
    a = std::fmod(a + M_PI,         kTwoPi);
    a = std::fmod(a + kTwoPi,       kTwoPi);   /* 음수 처리 — 분기 0 */
    return a - M_PI;
}

/* 분기 없는 clamp — fmin/fmax 만 사용 (§6.2 권고) */
inline double clampFM(double v, double lo, double hi)
{
    return std::fmin(std::fmax(v, lo), hi);
}

/* PredictState + scalar*PredictState — RK4 가중평균용 헬퍼.
   ★ PATCH: 마지막 멤버 a_lat → phi 명명 변경. 산술 동일. */
inline PredictState scaleAndAdd(const PredictState & x, double s, const PredictState & d)
{
    return PredictState{
        x.p_n   + s * d.p_n,
        x.p_e   + s * d.p_e,
        x.h     + s * d.h,
        x.V     + s * d.V,
        x.psi   + s * d.psi,
        x.h_dot + s * d.h_dot,
        x.phi   + s * d.phi,
    };
}

/* k1 + 2 k2 + 2 k3 + k4 의 가중합 — RK4 최종 합산 전용 */
inline PredictState rk4Combine(const PredictState & x,
                               double dt_over_6,
                               const PredictState & k1,
                               const PredictState & k2,
                               const PredictState & k3,
                               const PredictState & k4)
{
    return PredictState{
        x.p_n   + dt_over_6 * (k1.p_n   + 2.0 * k2.p_n   + 2.0 * k3.p_n   + k4.p_n  ),
        x.p_e   + dt_over_6 * (k1.p_e   + 2.0 * k2.p_e   + 2.0 * k3.p_e   + k4.p_e  ),
        x.h     + dt_over_6 * (k1.h     + 2.0 * k2.h     + 2.0 * k3.h     + k4.h    ),
        x.V     + dt_over_6 * (k1.V     + 2.0 * k2.V     + 2.0 * k3.V     + k4.V    ),
        x.psi   + dt_over_6 * (k1.psi   + 2.0 * k2.psi   + 2.0 * k3.psi   + k4.psi  ),
        x.h_dot + dt_over_6 * (k1.h_dot + 2.0 * k2.h_dot + 2.0 * k3.h_dot + k4.h_dot),
        x.phi   + dt_over_6 * (k1.phi   + 2.0 * k2.phi   + 2.0 * k3.phi   + k4.phi  ),
    };
}

} /* anonymous namespace */


TrajectoryPredict::TrajectoryPredict(const PredictParams & params)
: m_params(params)
{
}

void TrajectoryPredict::setParams(const PredictParams & params)
{
    m_params = params;
}


/* ─────────────────────────────────────────────────────────────────
   §2.5 — 입력 saturation (분기 없는 fmin/fmax)
   ★ PATCH: 변경 없음. a_lat_cmd 한계 그대로 (atan2 변환 후 자동으로
            |phi_cmd| ≤ FW_R_LIM 안에 들어옴).
   ───────────────────────────────────────────────────────────────── */
PredictInput TrajectoryPredict::applyInputSaturation(const PredictInput & u) const
{
    return PredictInput{
        clampFM(u.V_cmd,     m_params.V_min,        m_params.V_max),
        u.h_cmd,                                                   /* h_cmd: clamp 없음 (절대 고도 명령) */
        clampFM(u.h_dot_cmd, -m_params.h_dot_max,   m_params.h_dot_max),
        clampFM(u.a_lat_cmd, -m_params.a_lat_max,   m_params.a_lat_max),
    };
}


/* ─────────────────────────────────────────────────────────────────
   §2.5 — 상태 안전성:
   - V 음수/0 가드 (sqrt 발산)
   - |h_dot| < 0.99·V (V_h 발산 방지)
   - phi clamp ±60° (★ PATCH: tan 발산 방지)
   - psi wrap [-pi, pi]
   ───────────────────────────────────────────────────────────────── */
PredictState TrajectoryPredict::applyStateSafety(const PredictState & x) const
{
    PredictState y = x;
    /* 수치 안전 floor — V 가 0 또는 음수가 되어 sqrt/division 발산하는 것 방지.
       물리적 V_min (실속 한계) 검증은 호출자 또는 결과 후처리 책임. */
    y.V = std::fmax(y.V, m_params.V_h_min);
    /* |h_dot| < 0.99 * V — V_h 가 0 으로 가는 것 방지 */
    const double cap = 0.99 * y.V;
    y.h_dot = clampFM(y.h_dot, -cap, cap);
    /* phi clamp ±60° — tan(±π/2) 발산 방지 (FW_R_LIM 45° 에 마진 15°) */
    y.phi = clampFM(y.phi, -k_phi_hard_limit, k_phi_hard_limit);
    /* psi wrap [-pi, pi] */
    y.psi = wrapPi(y.psi);
    return y;
}


/* ─────────────────────────────────────────────────────────────────
   §2.3 — ODE 평가 (★ PATCH: phi 기반 Beard-McLain 정형)
   InternalInput 받음 — phi_cmd 는 stepRK4 에서 atan2 변환된 값.
   ───────────────────────────────────────────────────────────────── */
PredictState TrajectoryPredict::evaluateODE(const PredictState & x,
                                            const InternalInput & u) const
{
    /* 수평 속도 (NaN/0 가드) */
    const double V_sq    = x.V * x.V;
    const double hdot_sq = x.h_dot * x.h_dot;
    const double V_h_raw = std::sqrt(std::fmax(V_sq - hdot_sq, 0.0));
    const double V_h     = std::fmax(V_h_raw, m_params.V_h_min);

    const double cos_psi = std::cos(x.psi);
    const double sin_psi = std::sin(x.psi);

    PredictState d;
    d.p_n   = V_h * cos_psi;                                  /* 변경 없음 */
    d.p_e   = V_h * sin_psi;                                  /* 변경 없음 */
    d.h     = x.h_dot;                                        /* 변경 없음 */
    d.V     = (u.V_cmd     - x.V    ) / m_params.tau_V;       /* 변경 없음 */
    d.psi   = k_g * std::tan(x.phi) / V_h;                    /* ★ PATCH: phi 기반 조정선회 */

    /* ★ 종 채널 — Beard-McLain (9.19) PD 형태:
         d.h_dot = b_ḣ·(ḣ^c − ḣ) + b_h·(h^c − h)
                   └─────────┘    └──────┘
                  rate term      altitude P (Beard-McLain 9.19)
       h_cmd 가 NaN 이면 stepRK4 안에서 x.h 로 치환 → P-term 0 → 1차 지연 fallback. */
    d.h_dot = (u.h_dot_cmd - x.h_dot) / m_params.tau_hdot
            +  m_params.b_h * (u.h_cmd - x.h);

    d.phi   = (u.phi_cmd   - x.phi  ) / m_params.tau_phi;     /* ★ PATCH: roll loop 1차 지연 */
    return d;
}


/* ─────────────────────────────────────────────────────────────────
   §7 — RK4 1 step (zero-order hold input)
   ★ PATCH: stepRK4 진입 시 a_lat_cmd → phi_cmd atan2 변환 1회.
            그 InternalInput 으로 4 stages 모두 평가.
   ───────────────────────────────────────────────────────────────── */
PredictState TrajectoryPredict::stepRK4(const PredictState & x,
                                        const PredictInput & u,
                                        double dt) const
{
    /* 1) 입력/상태 saturation 한번 적용 (수치 안전) */
    const PredictInput  u_sat = applyInputSaturation(u);
    const PredictState  x_sat = applyStateSafety(x);

    /* 2) ★ PATCH: 외부 a_lat_cmd → 내부 phi_cmd 변환 (RK4 진입 시 1회).
                   atan2(a_sat, g) 가 자동으로 |phi_cmd| ≤ FW_R_LIM 보장
                   (a_sat 이 saturation 으로 a_lat_max = g·tan(FW_R_LIM) 안에 있음).

       ★ 본 작업: h_cmd NaN guard.
          PredictInput.h_cmd 가 NaN (예: collision_avoidance 가 미설정, 또는
          Replay 의 sequencer 가 baseline 모름) → InternalInput.h_cmd = x_sat.h 로 치환
          → evaluateODE 의 b_h·(h_cmd − h) = 0 → 1차 지연 fallback (기존 동작). */
    InternalInput u_int;
    u_int.V_cmd     = u_sat.V_cmd;
    u_int.h_cmd     = std::isfinite(u_sat.h_cmd) ? u_sat.h_cmd : x_sat.h;
    u_int.h_dot_cmd = u_sat.h_dot_cmd;
    u_int.phi_cmd   = std::atan2(u_sat.a_lat_cmd, k_g);

    /* 3) RK4 — k1, k2, k3, k4 (모두 같은 u_int 평가, ZOH) */
    const PredictState k1 = evaluateODE(x_sat,                                u_int);
    const PredictState k2 = evaluateODE(scaleAndAdd(x_sat, 0.5 * dt, k1),     u_int);
    const PredictState k3 = evaluateODE(scaleAndAdd(x_sat, 0.5 * dt, k2),     u_int);
    const PredictState k4 = evaluateODE(scaleAndAdd(x_sat,        dt, k3),    u_int);

    /* 4) 가중평균 */
    const PredictState x_next_raw = rk4Combine(x_sat, dt / 6.0, k1, k2, k3, k4);

    /* 5) 후처리: applyStateSafety 재적용 (수치오차로 phi/V_h_min 깨질 가능성 차단) */
    return applyStateSafety(x_next_raw);
}

/* ─────────────────────────────────────────────────────────────────
   extractAt — 단일 PredictState 에서 (pos, vel) NED frame 추출.
   (extractKeySamples 의 private helper, 2026-05-13 추가)

   좌표계 변환 (PredictState → NED Vec3):
     pos.x  =  p_n
     pos.y  =  p_e
     pos.z  = -h                                 (altitude up → NED down)
     vel.x  =  sqrt(V² - h_dot²) · cos(psi)      (NED vn)
     vel.y  =  sqrt(V² - h_dot²) · sin(psi)      (NED ve)
     vel.z  = -h_dot                              (NED vd)

   설계:
     - 분기 회피: 호출자가 *위치만 필요* 한 경우 out_vel=nullptr 로 cos/sin/sqrt 절약
     - V_h floor: 적용 안 함 — 외부 sample 추출은 *예측 결과 그대로 노출* 이 의도
                  (V_h_min 은 *적분 안정성* 만을 위한 내부 safety; 외부에는 raw 값 노출)
     - stateless / RT-safe / heap 0
   ───────────────────────────────────────────────────────────────── */
void TrajectoryPredict::extractAt(const PredictState & x,
                                   Vec3 & out_pos,
                                   Vec3 * out_vel) const
{
    /* 위치 — NED frame.
       ★ 2026-05-13: double → float narrowing. PredictState 내부는 double (RK4 정밀도),
       외부 Vec3 는 float (PX4 / ROS2 일관). 명시적 static_cast 로 narrowing 의도 표명. */
    out_pos.x = static_cast<float>( x.p_n);
    out_pos.y = static_cast<float>( x.p_e);
    out_pos.z = static_cast<float>(-x.h);

    /* 속도 — 호출자 필요 시만 계산 (cos/sin/sqrt 비용 절약) */
    if (out_vel) {
        const double V_sq    = x.V * x.V;
        const double hdot_sq = x.h_dot * x.h_dot;
        const double V_h     = std::sqrt(std::fmax(V_sq - hdot_sq, 0.0));
        out_vel->x = static_cast<float>( V_h * std::cos(x.psi));
        out_vel->y = static_cast<float>( V_h * std::sin(x.psi));
        out_vel->z = static_cast<float>(-x.h_dot);
    }
}

} /* namespace collision_avoidance::estimation */
