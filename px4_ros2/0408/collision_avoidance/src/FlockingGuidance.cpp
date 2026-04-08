#include <collision_avoidance/FlockingGuidance.hpp>
#include <algorithm>
#include <cmath>


FlockingGuidance::FlockingGuidance(const Parameters& params)
: _params(params) {}


void FlockingGuidance::resetVdesired(float initial_course, float initial_ground_speed)
{
    _v_desired_n = initial_ground_speed * std::cos(initial_course);
    _v_desired_e = initial_ground_speed * std::sin(initial_course);
    _v_desired_d = 0.f;
}


Eigen::Vector2f FlockingGuidance::computeAcceleration(
    const StateType::AgentState_rt      & self,
    const StateType::AgentStateArray_rt & others,
    int                                num_others) const
{
    Eigen::Vector2f a_ne = Eigen::Vector2f::Zero();

    /* 이웃이 0 이면 0 가속도 */
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


/* ──────────────────────────────────────────────────────────────
   메인 엔트리 — snapshot + 바람 → FwSetpointOutput
   rt_thread 가 매 iteration 1회 호출.

   파이프라인:
     (1) Flocking 가속도 계산
     (2) 가속도 입력 saturation
     (3) V_desired 일차 적분 (ZOH)
     (4) V_desired 출력단 saturation
     (5) gs_h 체크 → 너무 느리면 fallback 마커
     (6) (course, height_rate, airspeed) 변환
     (7) airspeed clamp
   ────────────────────────────────────────────────────────────── */
StateType::FwSetpointOutput_rt2mt FlockingGuidance::computeFwSetpoint(
    const StateType::AgentState_rt      & self,
    const StateType::AgentStateArray_rt & others,
    int                                num_others,
    float wind_n, float wind_e)
{
    /* (1) Flocking 가속도 (NED, m/s^2) */
    const Eigen::Vector2f a_ne = computeAcceleration(self, others, num_others);

    /* (2) 가속도 입력 saturation */
    const float aN = std::clamp(a_ne.x(), -kAccelMaxAbs, kAccelMaxAbs);
    const float aE = std::clamp(a_ne.y(), -kAccelMaxAbs, kAccelMaxAbs);
    constexpr float aD = 0.f;   /* 수직 가속은 0 (수평 비행) */

    /* (3) 일차 적분 — ZOH, 고정 dt */
    _v_desired_n += aN * _params.integration_dt;
    _v_desired_e += aE * _params.integration_dt;
    _v_desired_d += aD * _params.integration_dt;

    /* (4) V_desired 출력단 saturation */
    const float gs_h = std::sqrt(_v_desired_n * _v_desired_n +
                                 _v_desired_e * _v_desired_e);
    if (gs_h > kVdesiredMaxNorm) {
        const float scale = kVdesiredMaxNorm / gs_h;
        _v_desired_n *= scale;
        _v_desired_e *= scale;
    }
    _v_desired_d = std::clamp(_v_desired_d, -kHeightRateMaxAbs, kHeightRateMaxAbs);

    /* (5) gs_h 너무 느리면 course 발산 → fallback 마커 */
    StateType::FwSetpointOutput_rt2mt out;
    const float gs_h_after = std::sqrt(_v_desired_n * _v_desired_n +
                                       _v_desired_e * _v_desired_e);
    if (gs_h_after < kMinGsForCourse) {
        out.is_fallback = true;
        return out;
    }

    /* (6) NED V_desired → (course, height_rate, airspeed) */
    out.course      = std::atan2(_v_desired_e, _v_desired_n);
    out.height_rate = -_v_desired_d;   /* NED → ENU 부호 반전 */

    const float as_n = _v_desired_n - wind_n;
    const float as_e = _v_desired_e - wind_e;
    float airspeed   = std::sqrt(as_n * as_n + as_e * as_e);

    /* (7) airspeed clamp — stall / overspeed 방지 */
    out.airspeed    = std::clamp(airspeed, kAirspeedMin, kAirspeedMax);
    out.is_fallback = false;

    return out;
}
