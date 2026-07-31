#pragma once

/*
   FlockingGuidance
   ----------------
   Olfati-Saber 스타일 flocking 가이던스 (NE 평면 2D) 의 "완전한 가이던스 솔루션".

   상태 공간: spherical (v, ψ, γ)
     v     : 속력 [m/s]
     ψ     : 코스각 [rad] = PX4 course 와 직접 매핑
     γ     : 비행 경로각 [rad] (위가 양)

     V_NED = ( v·cos(ψ)·cos(γ),  v·sin(ψ)·cos(γ),  -v·sin(γ) )

   파이프라인:
     1. Flocking → NED 가속도 (a_N, a_E, a_D)
     2. M^{-1} 적용 → (dv/dt, dψ/dt, dγ/dt)
     3. dψ/dt 클램프: ω_max = g·tan(roll_max)/v       (coordinated turn)
     4. dγ/dt 클램프: pitch rate 기체 한계
     5. dv/dt 클램프: 0.5 × STE_rate / TAS (ArduPilot TECS 방식, AirframeLimits)
     5. 적분: v, ψ, γ 갱신
     6. 상태 클램프: v ∈ [vmin,vmax], γ ∈ [-γmax,γmax], ψ wrap to [-π,π]
     7. 출력: course = ψ
              height_rate = +v·sin(γ)         (ENU 위가 양)
              airspeed    = |v_ground - wind|

   장점:
   - 상태 = setpoint (변환 불필요)
   - 모든 클램프가 물리적 의미 (좌표축 편향 없음)
   - atan2 불안정성·적분 드리프트 자연 해소
   - cos(γ)≥cos(γ_max)>0, v≥vmin 보장 → 분모 가드 불필요

   인터페이스:
   - 상태 포맷: StateType::AgentState_rt (rt_thread 전용)
   - 이웃 목록: std::array + count (정적 할당, heap 접근 0)
*/

#include <Eigen/Core>

#include <collision_avoidance/common/StateType.hpp>
#include <collision_avoidance/guidance/AirframeLimits.hpp>


class FlockingGuidance
{
public:
    struct Parameters
    {
        /* ── Flocking 알고리즘 튜닝 (flocking_params.yaml) ── */
        float lambda{1.0f};            /* alignment 게인 */
        float beta{0.5f};              /* alignment 거리 감쇠 지수 */
        float k1{0.5f};                /* cohesion 게인 (속도-위치 내적) */
        float k2{1.0f};                /* separation/cohesion 게인 (위치 오차) */
        float desired_distance{30.f};  /* 이웃과 유지하고 싶은 거리 [m] */
        int   neighbor_count{4};       /* 평균 분모용 (자기 제외 이웃 수) */
        float integration_dt{0.0333f}; /* 일차 적분 sample period [s] (ZOH) */

        /* ── Airframe spec / safety limit (airframe_spec.yaml) ── */
        float airspeed_min{10.0f};                /* stall 방지 [m/s] */
        float airspeed_max{25.0f};                /* overspeed 방지 [m/s] */
        float height_rate_max_climb{8.0f};        /* height_rate climb 한계 [m/s] — PX4 FW_T_CLMB_MAX */
        float height_rate_max_sink{2.7f};         /* height_rate sink  한계 [m/s] — PX4 FW_T_SINK_MAX */
        float max_roll_deg{50.0f};                /* FW_R_LIM — coordinated turn */
        float max_pitch_deg{30.0f};               /* γ 상한 [deg] */
        float max_pitch_rate_deg_per_sec{60.0f};  /* dγ/dt 상한 [deg/s] */

        /* ── alt_hold (height_rate 직접 산출) P 게인 ── */
        float alt_hold_p_gain{0.1f};              /* [1/s] — h_dot = +p_gain × (self.pos_d - height_setpoint) */
    };

    explicit FlockingGuidance(const Parameters& params,
                             const AirframeLimits& limits);

    /* ── 메인 엔트리 포인트 ──
       height_setpoint: formation 시작 시점에 캡처된 reference NED z (m).
                        FormationMode 의 trans_odom 콜백이 첫 유효 odometry 시 캡처.
                        아직 캡처 전이면 NaN 으로 전달됨 → 함수 안에서 isfinite 체크. */
    StateType::FwSetpointOutput_rt2mt computeFwSetpoint(
        const StateType::AgentState_rt      & self,
        const StateType::AgentStateArray_rt & others,
        int                                num_others,
        float wind_n, float wind_e,
        float height_setpoint);

    /* 런타임 파라미터 갱신 */
    void setParameters(const Parameters& params) { _params = params; }
    const Parameters & parameters() const { return _params; }

    void setAirframeLimits(const AirframeLimits& limits) { _airframe = limits; }
    const AirframeLimits & airframeLimits() const { return _airframe; }

    /* (선택) NE 평면 raw 가속도 — 디버깅/단위 테스트용. */
    Eigen::Vector2f computeAcceleration(
        const StateType::AgentState_rt      & self,
        const StateType::AgentStateArray_rt & others,
        int                                num_others) const;

private:
    Parameters    _params;
    AirframeLimits _airframe;

    /* ── setpoint 생성용 상태 (rt_thread 전용) ── */
    float m_speed_setpoint_rt{0.f};          /* 속력 setpoint [m/s] */
    float m_gamma_setpoint_rt{0.f};          /* 비행 경로각 setpoint [rad] */
    float m_lateral_acceleration_rt{0.f};    /* 횡방향 가속도 [m/s^2] */
    float m_airspeed_setpoint_rt{0.f};       /* 등가대기속도 setpoint [m/s] */
    float m_height_rate_setpoint_rt{0.f};    /* 고도변화율 setpoint [m/s] */
};
