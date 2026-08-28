#include <trajectory_prediction_hils/testing/PropagationTestMode.hpp>

#include <algorithm>
#include <cmath>

#include <collision_avoidance/control/GroundToEasAdapter.hpp>

PropagationTestMode::PropagationTestMode(
    rclcpp::Node & node,
    int vehicle_id,
    std::string topic_namespace_prefix,
    CandidateInput candidate,
    TrimGateParams trim_gate,
    double cone_generation_duration_s,
    double tail_hold_duration_s,
    float minimum_level_eas,
    float gravity)
: ModeBase(
      node, Settings{"Propagation SILS Test"},
      "/px4_" + std::to_string(vehicle_id) + "/"),
  m_node(node),
  m_candidate(candidate),
  m_trim_gate(trim_gate),
  m_cone_generation_duration_s(std::max(0.1, cone_generation_duration_s)),
  m_total_hold_duration_s(
      std::max(0.1, cone_generation_duration_s)
      + std::max(4.5, tail_hold_duration_s)),
  m_minimum_level_eas(minimum_level_eas),
  m_gravity(gravity)
{
    m_fw_setpoint =
        std::make_shared<px4_ros2::FwLateralLongitudinalSetpointType>(*this);
    m_vtol_status = std::make_shared<px4_ros2::VtolStatus>(*this);
    m_local_position_sub =
        m_node.create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            topic_namespace_prefix + "/fmu/out/vehicle_local_position_v1",
            rclcpp::SensorDataQoS(),
            [this](px4_msgs::msg::VehicleLocalPosition::UniquePtr message) {
                m_local_position_timestamp.store(
                    message->timestamp, std::memory_order_relaxed);
                if (std::isfinite(message->vz)) {
                    m_latest_vertical_speed.store(
                        message->vz, std::memory_order_relaxed);
                }
                if (std::isfinite(message->vx)
                    && std::isfinite(message->vy)
                    && std::isfinite(message->vz)) {
                    m_latest_ground_speed.store(
                        std::sqrt(
                            static_cast<double>(message->vx) * message->vx
                            + static_cast<double>(message->vy) * message->vy
                            + static_cast<double>(message->vz) * message->vz),
                        std::memory_order_relaxed);
                    if (std::hypot(message->vx, message->vy) > 1.0e-3) {
                        m_latest_ground_course.store(
                            std::atan2(message->vy, message->vx),
                            std::memory_order_relaxed);
                    }
                }
                if (std::isfinite(message->z)) {
                    m_latest_local_z.store(message->z, std::memory_order_relaxed);
                    m_has_local_z.store(true, std::memory_order_release);
                }
            });
    m_airspeed_sub =
        m_node.create_subscription<px4_msgs::msg::AirspeedValidated>(
            topic_namespace_prefix + "/fmu/out/airspeed_validated_v1",
            rclcpp::SensorDataQoS(),
            [this](px4_msgs::msg::AirspeedValidated::UniquePtr message) {
                if (std::isfinite(message->calibrated_airspeed_m_s)) {
                    m_latest_calibrated_airspeed.store(
                        message->calibrated_airspeed_m_s,
                        std::memory_order_relaxed);
                }
            });
    m_attitude_sub =
        m_node.create_subscription<px4_msgs::msg::VehicleAttitude>(
            topic_namespace_prefix + "/fmu/out/vehicle_attitude",
            rclcpp::SensorDataQoS(),
            [this](px4_msgs::msg::VehicleAttitude::UniquePtr message) {
                m_attitude_timestamp.store(
                    message->timestamp, std::memory_order_relaxed);
                const double q_w = message->q[0];
                const double q_x = message->q[1];
                const double q_y = message->q[2];
                const double q_z = message->q[3];
                const double sin_roll = 2.0 * (q_w * q_x + q_y * q_z);
                const double cos_roll =
                    1.0 - 2.0 * (q_x * q_x + q_y * q_y);
                const double roll = std::atan2(sin_roll, cos_roll);
                if (std::isfinite(roll)) {
                    m_latest_roll.store(roll, std::memory_order_relaxed);
                }
            });
    m_wind_sub = m_node.create_subscription<px4_msgs::msg::Wind>(
        topic_namespace_prefix + "/fmu/out/wind",
        rclcpp::SensorDataQoS(),
        [this](px4_msgs::msg::Wind::UniquePtr message) {
            m_wind_north.store(
                message->windspeed_north, std::memory_order_relaxed);
            m_wind_east.store(
                message->windspeed_east, std::memory_order_relaxed);
        });
    m_vehicle_air_data_sub =
        m_node.create_subscription<px4_msgs::msg::VehicleAirData>(
            topic_namespace_prefix + "/fmu/out/vehicle_air_data",
            rclcpp::SensorDataQoS(),
            [this](px4_msgs::msg::VehicleAirData::UniquePtr message) {
                if (std::isfinite(message->rho) && message->rho > 0.0F) {
                    m_air_density.store(
                        message->rho, std::memory_order_relaxed);
                }
            });
}

void PropagationTestMode::onActivate()
{
    m_activation_time = std::chrono::steady_clock::now();
    m_phase = Phase::Trim;
    m_trim_stable_timer_active = false;
    m_completion_requested = false;
    m_input_applied.store(false, std::memory_order_release);
    m_applied_input_timestamp.store(0, std::memory_order_relaxed);
    m_baseline_altitude = NAN;
    m_active.store(true, std::memory_order_release);
    RCLCPP_INFO(
        m_node.get_logger(),
        "[propagation-test] trim started: V=%.2f, hdot=0, alat=0; "
        "gate=(ground dV<=%.2f, |vz|<=%.2f, |roll|<=%.1fdeg for %.1fs), "
        "timeout=%.1fs; maneuver alat=%.3f, cone=%.1fs tail=%.1fs",
        m_candidate.ground_speed,
        m_trim_gate.ground_speed_tolerance_mps,
        m_trim_gate.vertical_speed_tolerance_mps,
        m_trim_gate.roll_tolerance_rad * 180.0 / M_PI,
        m_trim_gate.stable_hold_s, m_trim_gate.timeout_s,
        m_candidate.lateral_acceleration, m_cone_generation_duration_s,
        m_total_hold_duration_s - m_cone_generation_duration_s);
}

void PropagationTestMode::onDeactivate()
{
    m_active.store(false, std::memory_order_release);
}

float PropagationTestMode::equivalentAirspeedCommand(
    float lateral_acceleration) const
{
    const float raw_eas =
        collision_avoidance::control::computeRequiredEquivalentAirspeed(
        m_candidate.ground_speed,
        static_cast<float>(
            m_latest_ground_course.load(std::memory_order_relaxed)),
        0.0F,
        m_wind_north.load(std::memory_order_relaxed),
        m_wind_east.load(std::memory_order_relaxed),
        0.0F,
        m_air_density.load(std::memory_order_relaxed));
    return collision_avoidance::control::applyTurnMinimumEquivalentAirspeed(
        raw_eas, m_minimum_level_eas, lateral_acceleration, m_gravity);
}

void PropagationTestMode::updateSetpoint(float)
{
    if (!m_vtol_status->isFwMode()) {
        RCLCPP_ERROR_THROTTLE(
            m_node.get_logger(), *m_node.get_clock(), 1000,
            "[propagation-test] FixedWing state lost");
        return;
    }

    const float applied_lateral_acceleration =
        (m_phase == Phase::Trim) ? 0.0F : m_candidate.lateral_acceleration;
    const float v_cmd_eas =
        equivalentAirspeedCommand(applied_lateral_acceleration);

    if (m_phase == Phase::Trim) {
        px4_ros2::FwLateralLongitudinalSetpoint trim_setpoint;
        trim_setpoint.withLateralAcceleration(0.0F)
            .withEquivalentAirspeed(v_cmd_eas)
            .withHeightRate(0.0F);
        m_fw_setpoint->update(trim_setpoint);

        const auto now = std::chrono::steady_clock::now();
        if (trimConditionsSatisfied()) {
            if (!m_trim_stable_timer_active) {
                m_trim_stable_since = now;
                m_trim_stable_timer_active = true;
            } else if (std::chrono::duration<double>(
                    now - m_trim_stable_since).count()
                    >= m_trim_gate.stable_hold_s) {
                startManeuver();
            }
        } else {
            m_trim_stable_timer_active = false;
        }

        RCLCPP_INFO_THROTTLE(
            m_node.get_logger(), *m_node.get_clock(), 2000,
            "[propagation-test] trim: GS=%.2f/%.2f CAS=%.2f EAScmd=%.2f "
            "vz=%.3f roll=%.2fdeg stable=%s",
            m_latest_ground_speed.load(std::memory_order_relaxed),
            m_candidate.ground_speed,
            m_latest_calibrated_airspeed.load(std::memory_order_relaxed),
            v_cmd_eas,
            m_latest_vertical_speed.load(std::memory_order_relaxed),
            m_latest_roll.load(std::memory_order_relaxed) * 180.0 / M_PI,
            m_trim_stable_timer_active ? "yes" : "no");

        const double trim_elapsed_s = std::chrono::duration<double>(
            now - m_activation_time).count();
        if (trim_elapsed_s >= m_trim_gate.timeout_s
            && !m_completion_requested) {
            m_completion_requested = true;
            RCLCPP_ERROR(
                m_node.get_logger(),
                "[propagation-test] trim gate timeout after %.1fs",
                trim_elapsed_s);
            completed(px4_ros2::Result::Timeout);
            rclcpp::shutdown();
        }
        return;
    }

    px4_ros2::FwLateralLongitudinalSetpoint maneuver_setpoint;
    maneuver_setpoint.withLateralAcceleration(m_candidate.lateral_acceleration)
        .withEquivalentAirspeed(v_cmd_eas)
        .withHeightRate(0.0F);
    m_fw_setpoint->update(maneuver_setpoint);

    if (!m_input_applied.exchange(true, std::memory_order_acq_rel)) {
        m_maneuver_start_time = std::chrono::steady_clock::now();
        const auto now_us = static_cast<std::uint64_t>(
            m_node.now().nanoseconds() / 1000);
        m_applied_input_timestamp.store(now_us, std::memory_order_release);
        RCLCPP_INFO(
            m_node.get_logger(),
            "[propagation-test] first lateral candidate applied at %lu us",
            static_cast<unsigned long>(now_us));
    }

    const double elapsed_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - m_maneuver_start_time).count();
    if (elapsed_s >= m_total_hold_duration_s && !m_completion_requested) {
        m_completion_requested = true;
        RCLCPP_INFO(
            m_node.get_logger(),
            "[propagation-test] hold complete (%.2fs); shutting down",
            elapsed_s);
        completed(px4_ros2::Result::Success);
        rclcpp::shutdown();
    }
}

bool PropagationTestMode::coneGenerationActive() const
{
    if (!m_active.load(std::memory_order_acquire)
        || !m_input_applied.load(std::memory_order_acquire)) {
        return false;
    }
    const double elapsed_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - m_maneuver_start_time).count();
    return elapsed_s <= m_cone_generation_duration_s;
}

std::uint64_t PropagationTestMode::appliedInputTimestamp() const
{
    return m_applied_input_timestamp.load(std::memory_order_acquire);
}

double PropagationTestMode::baselineAltitude() const
{
    return m_baseline_altitude;
}

bool PropagationTestMode::trimConditionsSatisfied() const
{
    const auto now_us = static_cast<std::uint64_t>(
        m_node.now().nanoseconds() / 1000);
    const auto fresh = [this, now_us](std::uint64_t timestamp) {
        if (timestamp == 0 || timestamp > now_us + 10000) {
            return false;
        }
        const std::uint64_t age_us = now_us > timestamp
            ? now_us - timestamp : 0;
        return static_cast<double>(age_us) * 1.0e-6
            <= m_trim_gate.max_sample_age_s;
    };
    const std::uint64_t local_timestamp =
        m_local_position_timestamp.load(std::memory_order_relaxed);
    const std::uint64_t attitude_timestamp =
        m_attitude_timestamp.load(std::memory_order_relaxed);
    if (!fresh(local_timestamp) || !fresh(attitude_timestamp)) {
        return false;
    }

    const double ground_speed =
        m_latest_ground_speed.load(std::memory_order_relaxed);
    const double vertical_speed =
        m_latest_vertical_speed.load(std::memory_order_relaxed);
    const double roll = m_latest_roll.load(std::memory_order_relaxed);
    return std::isfinite(ground_speed) && std::isfinite(vertical_speed)
        && std::isfinite(roll)
        && std::abs(ground_speed - m_candidate.ground_speed)
            <= m_trim_gate.ground_speed_tolerance_mps
        && std::abs(vertical_speed)
            <= m_trim_gate.vertical_speed_tolerance_mps
        && std::abs(roll) <= m_trim_gate.roll_tolerance_rad;
}

void PropagationTestMode::startManeuver()
{
    if (m_has_local_z.load(std::memory_order_acquire)) {
        m_baseline_altitude =
            -m_latest_local_z.load(std::memory_order_relaxed);
    }
    m_phase = Phase::Maneuver;
    m_trim_stable_timer_active = false;
    RCLCPP_INFO(
        m_node.get_logger(),
        "[propagation-test] trim gate passed: GS=%.2f CAS=%.2f vz=%.3f "
        "roll=%.2fdeg baseline_h=%.2fm; applying alat=%.3f",
        m_latest_ground_speed.load(std::memory_order_relaxed),
        m_latest_calibrated_airspeed.load(std::memory_order_relaxed),
        m_latest_vertical_speed.load(std::memory_order_relaxed),
        m_latest_roll.load(std::memory_order_relaxed) * 180.0 / M_PI,
        m_baseline_altitude, m_candidate.lateral_acceleration);
}
