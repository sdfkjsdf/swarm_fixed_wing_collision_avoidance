#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <collision_avoidance/spsc_queue.hpp>
#include <Eigen/Dense>


   /*
    Naming convention (클래스 멤버 전용)

     _<name>          : framework 가 관리하는 핸들·플러그인
                        - rclcpp 객체 (Node, Subscription, Time)
                        - px4_ros2 객체 (FwSetpoint, VTOL, ModeBase 등)
                        - doRegister() 로 framework 에 등록된 우리 ModeBase 서브클래스
                          (예: VtolPreflightMode, FormationMode reference)
                        → 즉 우리 코드가 lifecycle 을 직접 관리하지 않는 모든 것

     m_<name>         : 우리가 완전히 소유한 데이터 (framework 와 무관, 단일 thread)
     m_<name>_mt      : 우리 데이터 — main thread 전용
     m_<name>_rt      : 우리 데이터 — rt_thread 전용
     m_<name>_mt2rt   : 우리 데이터 — main → rt 단방향 통신 (atomic / queue 필수)
     m_<name>_rt2mt   : 우리 데이터 — rt → main 단방향 통신 (atomic / queue 필수)

   지역변수는 접미사 없음 — 함수 스코프가 이미 스레드 컨텍스트를 결정함.
   
   
   */


static constexpr size_t kMaxAgents = 8;

namespace StateType
{

    /* ── mt 전용 구조체 ──
       subscription 콜백(main thread) 이 채우고, 같은 콜백 안에서만 읽힘.
       rt_thread 는 직접 접근하지 않음. */
    struct State_for_Control_mt
    {
        std::array<float, 3> position          = {0.0f, 0.0f, 0.0f};
        std::array<float, 3> velocity          = {0.0f, 0.0f, 0.0f};
        std::array<float, 3> position_variance = {0.0f, 0.0f, 0.0f};
        std::array<float, 3> velocity_variance = {0.0f, 0.0f, 0.0f};
        double timestamp        = 0.0;
        int    check_vehicle_id = 0;
        bool   update_state     = false;
    };


    /* ── mt → rt 전달용 snapshot ──
       mt 가 5대 전부 수신했을 때 한 번에 만들어서 InputQueue_mt2rt 에 push.
       rt_thread 가 pop 해서 가이던스 입력으로 사용. */
    struct Total_state_for_Control_mt2rt {
        std::array<State_for_Control_mt, kMaxAgents> agents{};
        double timestamp  = 0.0;
        int    num_agents = 0;
    };


    /* ── mt → rt 채널 ── */
    using InputQueue_mt2rt = SpscQueue<Total_state_for_Control_mt2rt, kMaxAgents>;

    /* ── mt 전용 (수신 완료 플래그 누적용) ── */
    using Check_update_mt = std::array<bool, kMaxAgents>;


    /* ──────────────────────────────────────────────────────────────
       가이던스 알고리즘용 에이전트 상태 (NE 평면 2D)
       rt_thread 가 snapshot 에서 추출해 FlockingGuidance 에 전달.
       정적 할당 (std::array, kMaxAgents 상한) — rt_thread 에서 heap 접근 0.
       ────────────────────────────────────────────────────────────── */
    struct AgentState_rt {
        float pos_n{0.f};
        float pos_e{0.f};
        float vel_n{0.f};
        float vel_e{0.f};
    };

    using AgentStateArray_rt = std::array<AgentState_rt, kMaxAgents>;


    /* ──────────────────────────────────────────────────────────────
       rt → main 전달용 최종 fw setpoint
       rt_thread 가 가이던스 파이프라인 통과 후 만들어서 OutputQueue_rt2mt 로 push.
       main thread (updateSetpoint) 가 pop 해서 _fw_setpoint->update() 에 인가.
       ────────────────────────────────────────────────────────────── */
    struct FwSetpointOutput_rt2mt {
        float course       = 0.f;  /* [rad] atan2(v_e, v_n) */
        float airspeed     = 0.f;  /* [m/s] clamp 적용 후 */
        float height_rate  = 0.f;  /* [m/s] ENU (NED 의 -v_d) */
        bool  is_fallback  = true; /* true = gs_h 너무 느림 → main thread 가 cruise 로 */
    };

    /* ── rt → mt 채널 ── */
    using OutputQueue_rt2mt = SpscQueue<FwSetpointOutput_rt2mt, 8>;


    /* ── rt 내부 전용 행렬 (현재는 미사용, 향후 예비) ── */
    using Total_State_Matrix_rt = Eigen::Matrix<float, 6, kMaxAgents>;
    using State_Vector_rt       = Eigen::Matrix<float, 6, 1>;

}/*namespace영역 종료*/
