#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/airspeed_validated.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/fixedwing/lateral_longitudinal.hpp>
#include <px4_ros2/vehicle_state/vtol_status.hpp>

class PropagationTestMode : public px4_ros2::ModeBase
{
public:
    struct CandidateInput
    {
        float equivalent_airspeed{20.0F};
        float lateral_acceleration{0.0F};
    };

    struct TrimGateParams
    {
        double timeout_s{45.0};
        double stable_hold_s{2.0};
        double airspeed_tolerance_mps{1.0};
        double vertical_speed_tolerance_mps{0.5};
        double roll_tolerance_rad{5.0 * 3.14159265358979323846 / 180.0};
        double max_sample_age_s{0.5};
    };

    PropagationTestMode(
        rclcpp::Node & node,
        int vehicle_id,
        std::string topic_namespace_prefix,
        CandidateInput candidate,
        TrimGateParams trim_gate,
        double cone_generation_duration_s,
        double tail_hold_duration_s);

    void onActivate() override;
    void onDeactivate() override;
    void updateSetpoint(float dt_s) override;

    bool coneGenerationActive() const;
    std::uint64_t appliedInputTimestamp() const;
    double baselineAltitude() const;
    const CandidateInput & candidate() const { return m_candidate; }

private:
    enum class Phase { Trim, Maneuver };

    bool trimConditionsSatisfied() const;
    void startManeuver();

    rclcpp::Node & m_node;
    std::shared_ptr<px4_ros2::FwLateralLongitudinalSetpointType> m_fw_setpoint;
    std::shared_ptr<px4_ros2::VtolStatus> m_vtol_status;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
        m_local_position_sub;
    rclcpp::Subscription<px4_msgs::msg::AirspeedValidated>::SharedPtr
        m_airspeed_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr
        m_attitude_sub;

    CandidateInput m_candidate;
    TrimGateParams m_trim_gate;
    double m_cone_generation_duration_s{5.0};
    double m_total_hold_duration_s{9.6};
    std::chrono::steady_clock::time_point m_activation_time{};
    std::chrono::steady_clock::time_point m_maneuver_start_time{};
    std::chrono::steady_clock::time_point m_trim_stable_since{};
    bool m_trim_stable_timer_active{false};
    bool m_completion_requested{false};
    Phase m_phase{Phase::Trim};
    std::atomic<bool> m_active{false};
    std::atomic<bool> m_input_applied{false};
    std::atomic<std::uint64_t> m_applied_input_timestamp{0};
    std::atomic<double> m_latest_local_z{0.0};
    std::atomic<double> m_latest_vertical_speed{NAN};
    std::atomic<double> m_latest_calibrated_airspeed{NAN};
    std::atomic<double> m_latest_roll{NAN};
    std::atomic<std::uint64_t> m_local_position_timestamp{0};
    std::atomic<std::uint64_t> m_airspeed_timestamp{0};
    std::atomic<std::uint64_t> m_attitude_timestamp{0};
    std::atomic<bool> m_has_local_z{false};
    double m_baseline_altitude{NAN};
};
