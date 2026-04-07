#include <collision_avoidance/VtolGuidanceExecutor.hpp>


VtolGuidanceExecutor::VtolGuidanceExecutor(VtolGuidanceMode & owned_mode)
: ModeExecutorBase(
      px4_ros2::ModeExecutorBase::Settings{}.activate(
          px4_ros2::ModeExecutorBase::Settings::Activation::ActivateImmediately),
      owned_mode)
, _node(owned_mode.node())
{
    /* ActivateImmediately: 등록 직후 자동 활성화 + 자동 arm
       (takeoff() 호출 시 ModeExecutorBase 가 내부적으로 arm 까지 처리)
       완전 자율 임무용 설정. 디폴트인 ActivateOnlyWhenArmed 를 쓰면
       외부에서 누가 arm + executor 선택을 해줘야만 onActivate 가 호출됨 */
}

void VtolGuidanceExecutor::onActivate()
{
    /* 활성화되면 먼저 preflight check 통과 대기 → 그 다음 이륙 */
    runState(State::WaitReadyToArm, px4_ros2::Result::Success);
}

void VtolGuidanceExecutor::onDeactivate(DeactivateReason /*reason*/) {}

/* ────────────────────────────────────────────────────────────
   runState()
   기존 update_flight_phase() 역할 대체
   비동기 콜백 체인으로 단계 전환
   ──────────────────────────────────────────────────────────── */
void VtolGuidanceExecutor::runState(State state, px4_ros2::Result previous_result)
{
    /* 이전 단계 실패 시 중단 */
    if (previous_result != px4_ros2::Result::Success) {
        RCLCPP_ERROR(_node.get_logger(),
            "State %d 이전 단계 실패: %s",
            static_cast<int>(state), resultToString(previous_result));
        return;
    }

    switch (state) {

        case State::Reset:
            break;

        /* ── Preflight check 통과 + arm 가능 상태 대기 ── */
        case State::WaitReadyToArm:
            RCLCPP_INFO(_node.get_logger(), "[0/5] arm 가능 상태 대기");
            waitReadyToArm([this](px4_ros2::Result r) { runState(State::Arming, r); });
            break;

        /* ── Arm (takeoff 는 arm 시키지 않으므로 직접 arm 해야 함) ── */
        case State::Arming:
            RCLCPP_INFO(_node.get_logger(), "[1/5] Arm 송신");
            arm([this](px4_ros2::Result r) { runState(State::TakingOff, r); });
            break;

        /* ── 이륙 (이 시점엔 이미 armed) ── */
        case State::TakingOff:
            RCLCPP_INFO(_node.get_logger(), "[2/5] 이륙 시작");
            takeoff([this](px4_ros2::Result r) { runState(State::MyMode, r); });
            break;

        /* ── VtolGuidanceMode 실행 (천이 → 순항 → 편대비행) ── */
        case State::MyMode:
            RCLCPP_INFO(_node.get_logger(), "[3/5] VtolGuidanceMode 실행");
            scheduleMode(
                ownedMode().id(),
                [this](px4_ros2::Result r) { runState(State::Returning, r); });
            break;

        /* ── RTL ── */
        case State::Returning:
            RCLCPP_INFO(_node.get_logger(), "[4/5] 복귀 시작");
            rtl([this](px4_ros2::Result r) { runState(State::WaitDisarm, r); });
            break;

        /* ── Disarm 대기 ── */
        case State::WaitDisarm:
            RCLCPP_INFO(_node.get_logger(), "[5/5] Disarm 대기");
            waitUntilDisarmed([this](px4_ros2::Result r) {
                RCLCPP_INFO(_node.get_logger(),
                    "임무 종료 (%s)", resultToString(r));
            });
            break;
    }
}
