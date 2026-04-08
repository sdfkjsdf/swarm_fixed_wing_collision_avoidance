#pragma once

/*
   FlockingGuidance
   ----------------
   Olfati-Saber 스타일 flocking 가이던스 (NE 평면 2D) 의 "완전한 가이던스 솔루션".
   - alignment / cohesion / separation 가속도 계산
   - 가속도 입력 saturation
   - V_desired 일차 적분 (ZOH, 고정 dt)
   - V_desired 출력단 saturation
   - (course, height_rate, airspeed) 변환
   - fallback 마커 포함한 FwSetpointOutput_rt2mt 반환

   인터페이스 설계:
   - 에이전트 상태는 StateType::AgentState_rt 로 공용 (다른 가이던스 모듈과 공유)
   - 이웃 목록은 StateType::AgentStateArray_rt + count — 정적 할당, heap 접근 0
   - rt_thread 안전성 우선
*/

#include <Eigen/Core>

#include <collision_avoidance/StateType.hpp>   /* AgentState_rt, AgentStateArray_rt, FwSetpointOutput_rt2mt */


class FlockingGuidance
{
public:
    struct Parameters
    {
        float lambda{1.0f};            /* alignment 게인 */
        float beta{0.5f};              /* alignment 거리 감쇠 지수 */
        float k1{0.5f};                /* cohesion 게인 (속도-위치 내적) */
        float k2{1.0f};                /* separation/cohesion 게인 (위치 오차) */
        float desired_distance{30.f};  /* 이웃과 유지하고 싶은 거리 [m] */
        int   neighbor_count{4};       /* 평균 분모용 (자기 제외 이웃 수) */
        float integration_dt{0.0333f}; /* 일차 적분 sample period [s] (ZOH) */
    };

    explicit FlockingGuidance(const Parameters& params);

    /* ── 메인 엔트리 포인트 ──
       snapshot + 바람 → 내부 V_desired 적분 → FwSetpointOutput_rt2mt 반환.
       rt_thread 가 매 iteration 1회 호출.

       others     : 자기 자신 제외한 이웃 상태 배열 (정적 할당)
       num_others : 실제 유효한 이웃 수 (others[0..num_others-1] 만 읽음) */
    StateType::FwSetpointOutput_rt2mt computeFwSetpoint(
        const StateType::AgentState_rt      & self,
        const StateType::AgentStateArray_rt & others,
        int                                num_others,
        float wind_n, float wind_e);

    /* ── V_desired 재초기화 ──
       Formation 활성화 시점에 cruise 초기조건으로 리셋.
       rt_thread 안에서만 호출 (활성화 신호를 받고). */
    void resetVdesired(float initial_course, float initial_ground_speed);

    /* 런타임 파라미터 갱신 */
    void setParameters(const Parameters& params) { _params = params; }
    const Parameters & parameters() const { return _params; }

    /* (선택) NE 평면 raw 가속도 — 디버깅/단위 테스트용. */
    Eigen::Vector2f computeAcceleration(
        const StateType::AgentState_rt      & self,
        const StateType::AgentStateArray_rt & others,
        int                                num_others) const;

private:
    Parameters _params;

    /* 적분 상태 (rt_thread 전용) */
    float _v_desired_n{0.f};
    float _v_desired_e{0.f};
    float _v_desired_d{0.f};

    /* 안전 한계 */
    static constexpr float kAccelMaxAbs      = 5.0f;
    static constexpr float kVdesiredMaxNorm  = 25.0f;
    static constexpr float kHeightRateMaxAbs = 5.0f;
    static constexpr float kAirspeedMin      = 12.0f;
    static constexpr float kAirspeedMax      = 25.0f;
    static constexpr float kMinGsForCourse   = 1.0f;
};
