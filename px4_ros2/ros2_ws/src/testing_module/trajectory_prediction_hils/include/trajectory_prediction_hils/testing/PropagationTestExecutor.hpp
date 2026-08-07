#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/components/mode_executor.hpp>

#include <trajectory_prediction_hils/modes/VtolPreflightMode.hpp>
#include <trajectory_prediction_hils/testing/PropagationTestMode.hpp>

class PropagationTestExecutor : public px4_ros2::ModeExecutorBase
{
public:
    PropagationTestExecutor(
        VtolPreflightMode & transition_mode,
        PropagationTestMode & test_mode);

    void onActivate() override;
    void onDeactivate(DeactivateReason reason) override;

private:
    enum class State { WaitReadyToArm, Arming, TakingOff, Transition, Test };
    void runState(State state, px4_ros2::Result previous_result);

    rclcpp::Node & m_node;
    VtolPreflightMode & m_transition_mode;
    PropagationTestMode & m_test_mode;
};
