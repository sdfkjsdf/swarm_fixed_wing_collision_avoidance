#include <collision_avoidance/guidance/FlockingGuidance.hpp>
#include <algorithm>
#include <cmath>


namespace {
constexpr float kDeg2Rad = static_cast<float>(M_PI) / 180.0f;

}  // namespace


FlockingGuidance::FlockingGuidance(
    const Parameters & params,
    float gravity_mps2)
: _params(params), m_gravity_mps2(gravity_mps2) {}



Eigen::Vector2f FlockingGuidance::computeAcceleration(
    const collision_avoidance::types::AgentState      & self,
    const collision_avoidance::types::AgentStateArray & others,
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


/* NE flocking acceleration to fixed-wing setpoints. */
collision_avoidance::types::FwSetpoint FlockingGuidance::computeFwSetpoint(
    const collision_avoidance::types::AgentState      & self,
    const collision_avoidance::types::AgentStateArray & others,
    int                                   num_others,
    float height_setpoint)
{
    /* (1) Flocking 가속도 (NE 평면, m/s^2) */
    const Eigen::Vector2f a_ne = computeAcceleration(self, others, num_others);
    const float aN = a_ne[0];
    const float aE = a_ne[1];

    /* (2) 현재 NE 평면 상태 */
    const float v_NE    = std::sqrt(self.vel_n * self.vel_n + self.vel_e * self.vel_e);
    const float cos_psi = std::cos(self.psi);
    const float sin_psi = std::sin(self.psi);

    /* (3) NE 평면 속도 방향 가속도 (γ 무관) */
    const float dv_dt_NE = aN * cos_psi + aE * sin_psi;

    /* (4) 적분 dt */
    const float dt = _params.integration_dt;

    /* (5) height_rate setpoint — alt_hold P 제어 + 비대칭 clamp
           단위: p_gain [1/s] × alt_error [m] = h_dot [m/s]
           부호: alt_error_ned = self.pos_d - height_setpoint
                 (+ = cur 이 ref 보다 아래 → 올라가야 → h_dot 양수)
           clamp: PX4 FW_T_CLMB_MAX / FW_T_SINK_MAX 와 일치 (비대칭) */
    float h_dot_p = 0.f;
    if (std::isfinite(height_setpoint)) {
        const float alt_error_ned = self.pos_d - height_setpoint;
        h_dot_p = _params.alt_hold_p_gain * alt_error_ned;
    }
    m_height_rate_setpoint_rt = std::clamp(
        h_dot_p,
        -_params.height_rate_max_sink,    /* 음수 한계 (sink) */
         _params.height_rate_max_climb);  /* 양수 한계 (climb) */

    /* (6) 3D ground-speed command magnitude, clamped to configured bounds. */
    const float v_NE_next = v_NE + dv_dt_NE * dt;
    const float ground_speed_next = std::sqrt(
        v_NE_next * v_NE_next
        + m_height_rate_setpoint_rt * m_height_rate_setpoint_rt);

    m_speed_setpoint_rt = std::clamp(ground_speed_next,
                                     _params.airspeed_min,
                                     _params.airspeed_max);

    /* (8) lateral acceleration — coordinated turn 한계 clamp
           a_lat = -aN·sin(ψ) + aE·cos(ψ) */
    const float lat_accel_max =
        std::tan(_params.max_roll_deg * kDeg2Rad) * m_gravity_mps2;
    const float lat_accel_raw = -aN * sin_psi + aE * cos_psi;
    m_lateral_acceleration_rt = std::clamp(lat_accel_raw, -lat_accel_max, lat_accel_max);

    /* (9) Output */
    collision_avoidance::types::FwSetpoint out;
    out.course               = self.psi;
    /* Legacy field name: this value is the ground-speed command. */
    out.airspeed             = m_speed_setpoint_rt;
    out.height_rate          = m_height_rate_setpoint_rt;
    out.height_setpoint      = height_setpoint;          /* ★ 신규 — 입력 그대로 전파.
                                                              trajectory_prediction 의 PD 적분에서 사용. */
    out.lateral_acceleration = m_lateral_acceleration_rt;
    out.is_fallback          = false;
    return out;
}
