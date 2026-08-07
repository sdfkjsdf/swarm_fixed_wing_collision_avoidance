#include <trajectory_prediction_hils/testing/PropagationTestExecutor.hpp>

PropagationTestExecutor::PropagationTestExecutor(
    VtolPreflightMode & transition_mode,
    PropagationTestMode & test_mode)
: ModeExecutorBase(
      px4_ros2::ModeExecutorBase::Settings{}.activate(
          px4_ros2::ModeExecutorBase::Settings::Activation::ActivateImmediately),
      transition_mode),
  m_node(transition_mode.node()),
  m_transition_mode(transition_mode),
  m_test_mode(test_mode)
{
}

void PropagationTestExecutor::onActivate()
{
    runState(State::WaitReadyToArm, px4_ros2::Result::Success);
}

void PropagationTestExecutor::onDeactivate(DeactivateReason reason)
{
    RCLCPP_WARN(
        m_node.get_logger(), "[propagation-test] executor deactivated: %d",
        static_cast<int>(reason));
}

void PropagationTestExecutor::runState(
    State state, px4_ros2::Result previous_result)
{
    if (previous_result != px4_ros2::Result::Success) {
        RCLCPP_ERROR(
            m_node.get_logger(), "[propagation-test] state %d failed",
            static_cast<int>(state));
        rclcpp::shutdown();
        return;
    }

    switch (state) {
        case State::WaitReadyToArm:
            waitReadyToArm([this](px4_ros2::Result result) {
                runState(State::Arming, result);
            });
            break;
        case State::Arming:
            arm([this](px4_ros2::Result result) {
                runState(State::TakingOff, result);
            });
            break;
        case State::TakingOff:
            takeoff([this](px4_ros2::Result result) {
                runState(State::Transition, result);
            });
            break;
        case State::Transition:
            scheduleMode(m_transition_mode.id(), [this](px4_ros2::Result result) {
                runState(State::Test, result);
            });
            break;
        case State::Test:
            scheduleMode(m_test_mode.id(), [this](px4_ros2::Result) {
                rclcpp::shutdown();
            });
            break;
    }
}
