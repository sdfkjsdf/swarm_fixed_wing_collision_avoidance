#include <collision_avoidance/FlockingGuidance.hpp>
#include <algorithm>
#include <cmath>


namespace {

/* 각도를 [-π, π] 로 wrap. ψ 누적 발산 방지. */
inline float wrap_pi(float angle)
{
    while (angle >  static_cast<float>(M_PI)) angle -= 2.0f * static_cast<float>(M_PI);
    while (angle < -static_cast<float>(M_PI)) angle += 2.0f * static_cast<float>(M_PI);
    return angle;
}

constexpr float kGravity = 9.80665f;
constexpr float kDeg2Rad = static_cast<float>(M_PI) / 180.0f;

}  // namespace


FlockingGuidance::FlockingGuidance(const Parameters& params)
: _params(params) {}



Eigen::Vector2f FlockingGuidance::computeAcceleration(
    const StateType::AgentState_rt      & self,
    const StateType::AgentStateArray_rt & others,
    int                                   num_others) const
{
    Eigen::Vector2f a_ne = Eigen::Vector2f::Zero();

    if (num_others <= 0) {
        return a_ne;
    }

    const Eigen::Vector2f self_pos(self.pos_n, self.pos_e);
    const Eigen::Vector2f self_vel(self.vel_n, self.vel_e);

    for (int i = 0; i < num_others; i++) {
        const auto & other = others[i];
        const Eigen::Vector2f other_pos(other.pos_n, other.pos_e);
        const Eigen::Vector2f other_vel(other.vel_n, other.vel_e);

        const Eigen::Vector2f rel_pos = other_pos - self_pos;
        const Eigen::Vector2f rel_vel = other_vel - self_vel;

        const float norm_pos    = rel_pos.norm();
        const float norm_pos_sq = rel_pos.squaredNorm();

        if (norm_pos < 1e-3f) {
            continue;
        }

        const float dot_pos_vel = rel_pos.dot(rel_vel);

        /* (1) Alignment */
        const float alignment_gain =
            _params.lambda /
            std::pow(1.0f + norm_pos * norm_pos, _params.beta);
        const Eigen::Vector2f u1 = alignment_gain * rel_vel;

        /* (2) Cohesion */
        const float u2_scalar =
            (_params.k1 / (2.0f * norm_pos_sq)) * dot_pos_vel;
        const Eigen::Vector2f u2 = u2_scalar * rel_pos;

        /* (3) Separation / 거리 유지 */
        const float u3_scalar =
            (_params.k2 / (2.0f * norm_pos)) *
            (norm_pos - 2.0f * _params.desired_distance);
        const Eigen::Vector2f u3 = u3_scalar * rel_pos;

        a_ne += (u1 + u2 + u3);
    }

    const float divisor = (_params.neighbor_count > 0)
                              ? static_cast<float>(_params.neighbor_count)
                              : static_cast<float>(num_others);
    if (divisor > 0.f) {
        a_ne /= divisor;
    }

    return a_ne;
}


/* ══════════════════════════════════════════════════════════════════
   메인 엔트리 — Spherical state (v, ψ, γ) 기반 파이프라인.
   현재 여기서는 지금 속도를 그대로 적분한 다음에 saturation을 진행하는 방식으로




   ═══════════════ */
StateType::FwSetpointOutput_rt2mt FlockingGuidance::computeFwSetpoint(
    const StateType::AgentState_rt      & self,
    const StateType::AgentStateArray_rt & others,
    int                                   num_others,
    float wind_n, float wind_e)
{
    /* (1) Flocking 가속도 (NED, m/s^2) */
    const Eigen::Vector2f a_ne = computeAcceleration(self, others, num_others);
    const float aN = a_ne[0];
    const float aE = a_ne[1];
    constexpr float aD = 0.f;   /* 수직 가속 0 (현재 2D Flocking) */

    const Eigen::Vector3f acceleration (aN,aE,aD);


    /* (2) 현재 spherical state 캐시 (정합성을 위해 한 번에 읽음) */
    const float v   = self.speed;
    const float psi = self.psi;
    const float gam = self.gamma;

    
    const float cos_psi = std::cos(self.psi);
    const float sin_psi = std::sin(self.psi);
    const float cos_gam = std::cos(self.gamma);
    const float sin_gam = std::sin(self.gamma);

    /* (3) M^{-1} : (a_N, a_E, a_D) → (dv/dt, dψ/dt, dγ/dt)
           (수학적 유도는 헤더 주석 참고) */
           // ← 여기서 세미콜론으로 끝                                                                                                                                
                                              
    const float dv_dt = (aN * cos_psi * cos_gam)    // ← 별도 문장                                                                                                                                             
                    + (aE * sin_psi * cos_gam )                                                                                                                                                              
                    - (aD * sin_gam);                                                                                                                                                                        
                                     

    const float dpsi_dt = (-aN * sin_psi + aE * cos_psi) / (self.speed * cos_gam);

    const float dgam_dt = -(aN * cos_psi * sin_gam
                          + aE * sin_psi * sin_gam
                          + aD * cos_gam) / self.speed;

    /* (4) Rate 클램프
           dψ/dt: ω_max = g·tan(roll_max) / v   (coordinated turn)
           dγ/dt: 기체 pitch rate 한계
           dv/dt: 클램프 안 함 — TECS 가 throttle slew 처리 */
    // const float omega_max = kGravity * std::tan(_params.max_roll_deg * kDeg2Rad)
    //                         / std::max(v, _params.airspeed_min);
    // const float dpsi_clamped = std::clamp(dpsi_dt, -omega_max, omega_max);

    // const float dgam_max = _params.max_pitch_rate_deg_per_sec * kDeg2Rad;
    // const float dgam_clamped = std::clamp(dgam_dt, -dgam_max, dgam_max);

    /* (5) 적분 (Forward Euler, 고정 dt — ZOH) */
    const float dt = _params.integration_dt;
    const float gamma_max_rad = _params.max_pitch_deg * kDeg2Rad;

    /* (6) 상태 clamp / wrap (다음 iter 안전성 보장) */
    _v     = std::clamp(v+dv_dt*dt, _params.airspeed_min, _params.airspeed_max);
    _psi   = wrap_pi(psi + dpsi_dt * dt);        /* 클램프 없이 raw dpsi_dt */
    _gamma = std::clamp(gam + dgam_dt * dt, -gamma_max_rad, gamma_max_rad); /* 클램프 없이 raw dgam_dt */

    /* (7) 출력 변환 */

    /* 7a) course = ψ (직접 매핑) */
    const float course_sp = _psi;

    /* 7b) height_rate = +v · sin(γ)
           γ > 0 (climbing) → height_rate > 0 (ENU 위 양수) */
    const float new_sin_gam = std::sin(_gamma);
    const float new_cos_gam = std::cos(_gamma);
    float height_rate_sp = _v * new_sin_gam;
    /* 출력단 추가 안전 클램프 (벨트 + 멜빵) */
    height_rate_sp = std::clamp(height_rate_sp,
                                -_params.height_rate_max_abs,
                                 _params.height_rate_max_abs);

    /* 7c) airspeed = |v_ground - wind|  (3D 항공속도)
           v_ground = ( v·cos(ψ)·cos(γ),  v·sin(ψ)·cos(γ),  -v·sin(γ) )
           wind 은 수평만 (vertical wind = 0 가정) */
    const float new_cos_psi = std::cos(_psi);
    const float new_sin_psi = std::sin(_psi);
    const float vg_n = _v * new_cos_psi * new_cos_gam;
    const float vg_e = _v * new_sin_psi * new_cos_gam;
    const float vg_d = -_v * new_sin_gam;

    const float va_n = vg_n - wind_n;
    const float va_e = vg_e - wind_e;
    const float va_d = vg_d;   /* vertical wind = 0 */

    const float airspeed_raw = std::sqrt(va_n * va_n + va_e * va_e + va_d * va_d);
    const float airspeed_sp  =
        std::clamp(airspeed_raw, _params.airspeed_min, _params.airspeed_max);

    /* 7d) lateral acceleration = 가속도의 속도 수직 방향 성분
           a_lat = -aN·sin(ψ) + aE·cos(ψ)   (= v·cos(γ)·dψ/dt) */
    const float lat_accel = -aN * sin_psi + aE * cos_psi;

    /* (8) Output */
    StateType::FwSetpointOutput_rt2mt out;
    out.course               = course_sp;
    out.airspeed             = airspeed_sp;
    out.height_rate          = height_rate_sp;
    out.lateral_acceleration = lat_accel;
    out.is_fallback          = false;
    return out;
}
