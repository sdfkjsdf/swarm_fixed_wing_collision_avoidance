/* ════════════════════════════════════════════════════════════════════
   TrajectoryReplayExecutor.cpp — TrajectoryReplayExecutor.hpp 구현부
   ────────────────────────────────────────────────────────────────────
   상태머신 7단계 전이 (자세한 흐름은 hpp 참고):
     WaitReadyToArm → Arming → TakingOff → Preflight
                    → Replay → Returning → WaitDisarm

   핵심 인계 로직 (Preflight → Replay):
     scheduleMode(_preflight.id(), [...]) 콜백에서
       _replay.setInitialCruiseState(
           _preflight.getCruiseAltitudeAmsl(),
           _preflight.getDesiredCourse(),
           _preflight.getDesiredGroundSpeed());
     → Replay 의 cruise fallback 이 Preflight 캡처값을 그대로 사용.

   각 단계는 비동기 콜백 체인. 콜백이 호출되어야 다음 단계로 진입.
   ════════════════════════════════════════════════════════════════════ */

#include <trajectory_prediction_hils/modes/TrajectoryReplayExecutor.hpp>


TrajectoryReplayExecutor::TrajectoryReplayExecutor(VtolPreflightMode & preflight,
                                           TrajectoryReplayMode & replay)
: ModeExecutorBase(
      px4_ros2::ModeExecutorBase::Settings{}.activate(
          px4_ros2::ModeExecutorBase::Settings::Activation::ActivateImmediately),
      preflight)
, _node(preflight.node())
, _preflight(preflight)
, _replay(replay)
{
}

void TrajectoryReplayExecutor::onActivate()
{
    RCLCPP_INFO(_node.get_logger(), "[Executor] onActivate()");
    runState(State::WaitReadyToArm, px4_ros2::Result::Success);
}

void TrajectoryReplayExecutor::onDeactivate(DeactivateReason reason)
{
    RCLCPP_WARN(_node.get_logger(),
        "[Executor] onDeactivate() reason=%d", static_cast<int>(reason));
}

void TrajectoryReplayExecutor::runState(State state, px4_ros2::Result previous_result)
{
    if (previous_result != px4_ros2::Result::Success) {
        RCLCPP_ERROR(_node.get_logger(),
            "State %d 이전 단계 실패: %s",
            static_cast<int>(state), resultToString(previous_result));
        return;
    }

    switch (state) {

        case State::Reset:
            break;

        case State::WaitReadyToArm:
            RCLCPP_INFO(_node.get_logger(), "[1/7] arm 가능 상태 대기");
            waitReadyToArm([this](px4_ros2::Result r) {
                runState(State::Arming, r);
            });
            break;

        case State::Arming:
            RCLCPP_INFO(_node.get_logger(), "[2/7] Arm 송신");
            arm([this](px4_ros2::Result r) {
                runState(State::TakingOff, r);
            });
            break;

        case State::TakingOff:
            RCLCPP_INFO(_node.get_logger(), "[3/7] 이륙 시작");
            takeoff([this](px4_ros2::Result r) {
                runState(State::Preflight, r);
            });
            break;

        case State::Preflight:
            RCLCPP_INFO(_node.get_logger(), "[4/7] Preflight (천이 + 안정화)");
            scheduleMode(_preflight.id(), [this](px4_ros2::Result r) {
                /* Preflight 가 끝나면 캡처한 cruise altitude 를 Replay 에 인계 */
                _replay.setInitialCruiseState(
                    _preflight.getCruiseAltitudeAmsl(),
                    _preflight.getDesiredCourse(),
                    _preflight.getDesiredGroundSpeed());
                RCLCPP_INFO(_node.get_logger(),
                    "[Executor] Preflight → Replay 인계 (alt=%.1f m)",
                    _preflight.getCruiseAltitudeAmsl());
                runState(State::Replay, r);
            });
            break;

        case State::Replay:
            RCLCPP_INFO(_node.get_logger(), "[5/7] Replay 실행 (★시퀀스 따라가기★)");
            scheduleMode(_replay.id(), [this](px4_ros2::Result r) {
                runState(State::Returning, r);
            });
            break;

        case State::Returning:
            RCLCPP_INFO(_node.get_logger(), "[6/7] RTL");
            rtl([this](px4_ros2::Result r) {
                runState(State::WaitDisarm, r);
            });
            break;

        case State::WaitDisarm:
            RCLCPP_INFO(_node.get_logger(), "[7/7] Disarm 대기");
            waitUntilDisarmed([this](px4_ros2::Result r) {
                RCLCPP_INFO(_node.get_logger(),
                    "임무 종료 (%s) → rclcpp::shutdown()", resultToString(r));
                /* rclcpp::shutdown() 은 thread-safe + idempotent.
                   main.cpp:241 의 두 번째 호출은 no-op. 콜백 종료 후 spin 정상 리턴
                   → RAII 로 ModeExecutor → Mode → Node 역순 소멸, rt_thread join 완료.
                   launch_1vtol_replay.sh 의 trap 이 gzserver/PX4/Agent 자동 정리. */
                rclcpp::shutdown();
            });
            break;
    }
}
