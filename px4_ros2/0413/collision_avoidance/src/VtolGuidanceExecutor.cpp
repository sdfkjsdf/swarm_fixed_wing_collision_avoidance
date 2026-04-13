#include <collision_avoidance/VtolGuidanceExecutor.hpp>


VtolGuidanceExecutor::VtolGuidanceExecutor(VtolPreflightMode & preflight,
                                           FormationMode & formation)
: ModeExecutorBase(
      px4_ros2::ModeExecutorBase::Settings{}.activate(
          px4_ros2::ModeExecutorBase::Settings::Activation::ActivateImmediately),
      preflight)
, _node(preflight.node())
, _preflight(preflight)
, _formation(formation)
{
    /* ActivateImmediately: 등록 직후 자동 활성화 */
}

void VtolGuidanceExecutor::onActivate()
{
    RCLCPP_INFO(_node.get_logger(), "[Executor] onActivate()");
    runState(State::WaitReadyToArm, px4_ros2::Result::Success);
}

void VtolGuidanceExecutor::onDeactivate(DeactivateReason reason)
{
    RCLCPP_WARN(_node.get_logger(),
        "[Executor] onDeactivate() reason=%d", static_cast<int>(reason));
}

void VtolGuidanceExecutor::runState(State state, px4_ros2::Result previous_result)
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
                /* Preflight 가 끝나면 캡처한 cruise altitude 를 Formation 에 인계 */
                _formation.setInitialCruiseState(
                    _preflight.getCruiseAltitudeAmsl(),
                    _preflight.getDesiredCourse(),
                    _preflight.getDesiredGroundSpeed());
                RCLCPP_INFO(_node.get_logger(),
                    "[Executor] Preflight → Formation 인계 (alt=%.1f m)",
                    _preflight.getCruiseAltitudeAmsl());
                runState(State::Formation, r);
            });
            break;

        case State::Formation:
            RCLCPP_INFO(_node.get_logger(), "[5/7] Formation 실행 (★연구 대상★)");
            scheduleMode(_formation.id(), [this](px4_ros2::Result r) {
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
                    "임무 종료 (%s)", resultToString(r));
            });
            break;
    }
}
