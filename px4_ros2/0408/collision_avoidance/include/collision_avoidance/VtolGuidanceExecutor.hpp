#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/components/mode_executor.hpp>

#include <collision_avoidance/VtolPreflightMode.hpp>
#include <collision_avoidance/FormationMode.hpp>


class VtolGuidanceExecutor : public px4_ros2::ModeExecutorBase
{
public:
    /* owned_mode = Preflight (이륙 직후 처음 활성화)
       second_mode = Formation (Preflight 종료 후 schedule)
       두 모드 모두 별도로 doRegister() 되어야 함 (main.cpp 에서) */
    VtolGuidanceExecutor(VtolPreflightMode & preflight,
                         FormationMode & formation);

    enum class State {
        Reset,
        WaitReadyToArm,
        Arming,
        TakingOff,
        Preflight,    /* 천이 + 순항 안정화 */
        Formation,    /* ★사용자 연구 대상★ */
        Returning,
        WaitDisarm
    };

    void onActivate() override;
    void onDeactivate(DeactivateReason reason) override;
    void runState(State state, px4_ros2::Result previous_result);

private:
    /* framework 관리 핸들 — _ prefix
       (px4_ros2 가 lifecycle 관리: rclcpp::Node, doRegister() 된 ModeBase 서브클래스) */
    rclcpp::Node &      _node;
    VtolPreflightMode & _preflight;
    FormationMode &     _formation;
};
