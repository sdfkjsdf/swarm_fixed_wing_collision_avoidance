#pragma once

/* ════════════════════════════════════════════════════════════════════
   TrajectoryReplayExecutor.hpp — 임무 전체 상태머신
   ────────────────────────────────────────────────────────────────────
   책임: 노드 부팅 ~ 임무 종료까지 단계 전이 관리.

     [1] WaitReadyToArm  arm 가능 상태 대기
     [2] Arming          arm 명령 송신
     [3] TakingOff       이륙
     [4] Preflight       VtolPreflightMode 실행 (천이 + 안정화)
                         → 완료 시 cruise altitude 캡처값을 Replay 에 인계
     [5] Replay          ★TrajectoryReplayMode 실행 (사용자 시퀀스)★
     [6] Returning       RTL (Return To Launch)
     [7] WaitDisarm      disarm 대기 → 임무 종료

   각 단계는 px4_ros2 의 콜백 체인으로 연결됨. 한 단계 완료 시
   다음 단계 runState() 호출 → 비동기 실행.

   ActivateImmediately: 등록 직후 자동 활성화.
   ════════════════════════════════════════════════════════════════════ */

#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/components/mode_executor.hpp>

#include <trajectory_prediction_hils/modes/VtolPreflightMode.hpp>
#include <trajectory_prediction_hils/replay/TrajectoryReplayMode.hpp>


class TrajectoryReplayExecutor : public px4_ros2::ModeExecutorBase
{
public:
    /* owned_mode = Preflight (이륙 직후 처음 활성화)
       second_mode = Replay (Preflight 종료 후 schedule)
       두 모드 모두 별도로 doRegister() 되어야 함 (main.cpp 에서). */
    TrajectoryReplayExecutor(VtolPreflightMode & preflight,
                         TrajectoryReplayMode & replay);

    enum class State {
        Reset,
        WaitReadyToArm,
        Arming,
        TakingOff,
        Preflight,    /* 천이 + 순항 안정화 */
        Replay,       /* ★시퀀스 replay (사용자 모드)★ */
        Returning,
        WaitDisarm
    };

    void onActivate() override;
    void onDeactivate(DeactivateReason reason) override;
    void runState(State state, px4_ros2::Result previous_result);

private:
    rclcpp::Node &         _node;
    VtolPreflightMode &    _preflight;
    TrajectoryReplayMode & _replay;
};
