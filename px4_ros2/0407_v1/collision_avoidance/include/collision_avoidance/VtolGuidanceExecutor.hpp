#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <collision_avoidance/VtolGuidanceMode.hpp>


class VtolGuidanceExecutor : public px4_ros2::ModeExecutorBase
{
public:
    explicit VtolGuidanceExecutor(VtolGuidanceMode & owned_mode);

    /* ────────────────────────────────────────────────────
       State: 비행 임무 순서 (유저 디파인)
       기존 FlightPhase enum과 동일 개념
       Executor 레벨에서 큰 흐름만 관리
       (VTOL 천이 / FW 순항 / 편대비행은 Mode 의 InternalState 가 담당)
       ──────────────────────────────────────────────────── */
    enum class State {
        Reset,
        WaitReadyToArm,     /* preflight check 통과 + arm 가능 상태 대기 */
        Arming,             /* arm 명령 송신 + armed 상태 진입 대기 */
        TakingOff,          /* 기존 TAKEOFF (이 시점엔 이미 armed) */
        MyMode,             /* VtolGuidanceMode 실행 (천이 → 순항 → 편대) */
        Returning,          /* RTL */
        WaitDisarm
    };

    void onActivate() override;
    void onDeactivate(DeactivateReason reason) override;
    void runState(State state, px4_ros2::Result previous_result);

private:
    rclcpp::Node & _node;
};
