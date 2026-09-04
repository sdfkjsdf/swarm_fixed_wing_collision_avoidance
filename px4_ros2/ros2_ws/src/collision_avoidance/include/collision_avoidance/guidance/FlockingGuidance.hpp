#pragma once

/*
   FlockingGuidance
   ----------------
   Olfati-Saber 스타일 Formation 가이던스의 수평 NE 구현.

   실제 파이프라인:
     1. 이웃 상태에서 NE 가속도 (a_N, a_E)를 계산한다.
     2. 현재 course 방향 성분을 ground-speed 명령에 한 step 적분한다.
     3. 수직 명령은 별도의 고도 P 제어로 계산한다.
     4. course 수직 성분을 lateral-acceleration 명령으로 변환한다.
     5. PX4 경계에서 ground-speed 명령을 EAS 명령으로 변환한다.

   이 모듈은 비행경로각 상태를 적분하거나 TECS 종방향 동역학을
   모사하지 않는다.

   인터페이스:
   - 상태 포맷: collision_avoidance::types::AgentState (rt_thread 전용)
   - 이웃 목록: std::array + count (정적 할당, heap 접근 0)
*/

#include <Eigen/Core>

#include <collision_avoidance/common/GlobalTypes.hpp>

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
        /* Legacy parameter names: ground-speed command bounds. The minimal
           adapter does not claim converted-EAS stall/overspeed enforcement. */
        float airspeed_min{10.0f};
        float airspeed_max{25.0f};
        float height_rate_max_climb{8.0f};        /* height_rate climb 한계 [m/s] — PX4 FW_T_CLMB_MAX */
        float height_rate_max_sink{2.7f};         /* height_rate sink  한계 [m/s] — PX4 FW_T_SINK_MAX */
        float max_roll_deg{50.0f};                /* FW_R_LIM — coordinated turn */

        /* ── alt_hold (height_rate 직접 산출) P 게인 ── */
        float alt_hold_p_gain{0.1f};              /* [1/s] — h_dot = +p_gain × (self.pos_d - height_setpoint) */
    };

    explicit FlockingGuidance(const Parameters & params, float gravity_mps2);

    /* ── 메인 엔트리 포인트 ──
       height_setpoint: formation 시작 시점에 캡처된 reference NED z (m).
                        FormationMode 의 trans_odom 콜백이 첫 유효 odometry 시 캡처.
                        아직 캡처 전이면 NaN 으로 전달됨 → 함수 안에서 isfinite 체크. */
    collision_avoidance::types::FwSetpoint computeFwSetpoint(
        const collision_avoidance::types::AgentState      & self,
        const collision_avoidance::types::AgentStateArray & others,
        int                                num_others,
        float height_setpoint);

    /* 런타임 파라미터 갱신 */
    void setParameters(const Parameters& params) { _params = params; }
    const Parameters & parameters() const { return _params; }

    /* (선택) NE 평면 raw 가속도 — 디버깅/단위 테스트용. */
    Eigen::Vector2f computeAcceleration(
        const collision_avoidance::types::AgentState      & self,
        const collision_avoidance::types::AgentStateArray & others,
        int                                num_others) const;

private:
    Parameters _params;
    float m_gravity_mps2{9.80665F};

    /* ── setpoint 생성용 상태 (rt_thread 전용) ── */
    float m_speed_setpoint_rt{0.f};          /* 속력 setpoint [m/s] */
    float m_lateral_acceleration_rt{0.f};    /* 횡방향 가속도 [m/s^2] */
    float m_height_rate_setpoint_rt{0.f};    /* 고도변화율 setpoint [m/s] */
};
