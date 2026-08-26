#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <collision_avoidance/estimation/trajectory_prediction/TrajectoryPredict.hpp>
#include <collision_avoidance/estimation/trajectory_prediction/UncertaintyTypes.hpp>
#include <trajectory_prediction_hils/estimation/BeliefConePublisher.hpp>
#include <trajectory_prediction_hils/modes/VtolPreflightMode.hpp>
#include <trajectory_prediction_hils/msg/trajectory_prediction_debug.hpp>
#include <trajectory_prediction_hils/testing/PropagationTestExecutor.hpp>
#include <trajectory_prediction_hils/testing/PropagationTestMode.hpp>

using collision_avoidance::estimation::PredictInput;
using collision_avoidance::estimation::PredictParams;
using collision_avoidance::estimation::PredictionInputTrajectory;
using collision_avoidance::estimation::TrajectoryPredict;
using collision_avoidance::estimation::UncertaintyParams;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "trajectory_prediction_sils_test_node");

    node->declare_parameter<int>("vehicle_ID", 0);
    node->declare_parameter<std::string>("topic_namespace_prefix", "/px4_0");
    node->declare_parameter<std::string>("test.case_id", "ZOH_V20_STRAIGHT");
    node->declare_parameter<double>("test.ground_speed", 20.0);
    node->declare_parameter<double>("test.lateral_acceleration", 0.0);
    node->declare_parameter<double>("test.trim_timeout_s", 45.0);
    node->declare_parameter<double>("test.trim_hold_s", 2.0);
    node->declare_parameter<double>("test.trim_ground_speed_tolerance_mps", 1.0);
    node->declare_parameter<double>(
        "test.trim_vertical_speed_tolerance_mps", 0.5);
    node->declare_parameter<double>("test.trim_roll_tolerance_deg", 5.0);
    node->declare_parameter<double>("test.trim_max_sample_age_s", 0.5);
    node->declare_parameter<double>("test.cone_generation_duration_s", 5.0);
    node->declare_parameter<double>("test.tail_hold_duration_s", 4.6);

    node->declare_parameter<double>("predict_horizon_endpoint_s", 4.5);
    node->declare_parameter<double>("predict_rate_hz", 10.0);
    node->declare_parameter<double>("predict_call_hz", 10.0);
    node->declare_parameter<double>("airspeed_min", 10.0);
    node->declare_parameter<double>("airspeed_max", 25.0);
    node->declare_parameter<double>("height_rate_max_climb", 8.0);
    node->declare_parameter<double>("height_rate_min_sink", 2.7);
    node->declare_parameter<double>("max_roll_deg", 50.0);
    node->declare_parameter<double>("tc_tas", 5.0);
    node->declare_parameter<double>("tc_alt", 5.0);
    node->declare_parameter<double>("tc_roll", 0.5);
    node->declare_parameter<double>("b_h", 0.02);
    node->declare_parameter<double>("uncertainty.q_pn", 0.25);
    node->declare_parameter<double>("uncertainty.q_pe", 0.25);
    node->declare_parameter<double>("uncertainty.q_h", 0.25);
    node->declare_parameter<double>("uncertainty.q_v", 0.04);
    node->declare_parameter<double>("uncertainty.q_psi", 0.001);
    node->declare_parameter<double>("uncertainty.q_hdot", 0.04);
    node->declare_parameter<double>("uncertainty.q_phi", 0.001);
    node->declare_parameter<double>("uncertainty.q_scale", 0.8);
    node->declare_parameter<double>("uncertainty.estimator_output_delay_s", 0.13);

    const int vehicle_id = node->get_parameter("vehicle_ID").as_int();
    const std::string topic_namespace =
        node->get_parameter("topic_namespace_prefix").as_string();
    const std::string case_id = node->get_parameter("test.case_id").as_string();
    const double predict_rate = node->get_parameter("predict_rate_hz").as_double();
    const double predict_call = node->get_parameter("predict_call_hz").as_double();
    const double endpoint_s =
        node->get_parameter("predict_horizon_endpoint_s").as_double();
    const std::size_t point_count =
        static_cast<std::size_t>(std::round(endpoint_s * predict_rate)) + 1;
    if (point_count != collision_avoidance::estimation::kTrajectoryPointCount) {
        throw std::runtime_error(
            "trajectory point count must be 46 (0.0 through 4.5 s)");
    }

    PredictParams predict_params;
    predict_params.tau_V = node->get_parameter("tc_tas").as_double();
    predict_params.tau_hdot = node->get_parameter("tc_alt").as_double();
    predict_params.tau_phi = node->get_parameter("tc_roll").as_double();
    predict_params.b_h = node->get_parameter("b_h").as_double();
    predict_params.V_min = node->get_parameter("airspeed_min").as_double();
    predict_params.V_max = node->get_parameter("airspeed_max").as_double();
    predict_params.h_dot_max = std::min(
        node->get_parameter("height_rate_max_climb").as_double(),
        std::abs(node->get_parameter("height_rate_min_sink").as_double()));
    const double max_roll_rad =
        node->get_parameter("max_roll_deg").as_double() * M_PI / 180.0;
    predict_params.a_lat_max = 9.80665 * std::tan(max_roll_rad);
    predict_params.V_h_min = 1.0;
    auto predictor = std::make_shared<TrajectoryPredict>(predict_params);

    UncertaintyParams uncertainty_params;
    uncertainty_params.process_noise_diagonal = {
        node->get_parameter("uncertainty.q_pn").as_double(),
        node->get_parameter("uncertainty.q_pe").as_double(),
        node->get_parameter("uncertainty.q_h").as_double(),
        node->get_parameter("uncertainty.q_v").as_double(),
        node->get_parameter("uncertainty.q_psi").as_double(),
        node->get_parameter("uncertainty.q_hdot").as_double(),
        node->get_parameter("uncertainty.q_phi").as_double()};
    const double q_scale =
        node->get_parameter("uncertainty.q_scale").as_double();
    for (double & value : uncertainty_params.process_noise_diagonal) {
        value *= q_scale;
    }
    auto cone_publisher = std::make_shared<BeliefConePublisher>(
        *node, topic_namespace, predictor, uncertainty_params,
        node->get_parameter("uncertainty.estimator_output_delay_s").as_double());

    PropagationTestMode::CandidateInput candidate;
    candidate.ground_speed = static_cast<float>(
        node->get_parameter("test.ground_speed").as_double());
    candidate.lateral_acceleration = static_cast<float>(
        node->get_parameter("test.lateral_acceleration").as_double());
    PropagationTestMode::TrimGateParams trim_gate;
    trim_gate.timeout_s =
        node->get_parameter("test.trim_timeout_s").as_double();
    trim_gate.stable_hold_s =
        node->get_parameter("test.trim_hold_s").as_double();
    trim_gate.ground_speed_tolerance_mps = node->get_parameter(
        "test.trim_ground_speed_tolerance_mps").as_double();
    trim_gate.vertical_speed_tolerance_mps = node->get_parameter(
        "test.trim_vertical_speed_tolerance_mps").as_double();
    trim_gate.roll_tolerance_rad =
        node->get_parameter("test.trim_roll_tolerance_deg").as_double()
        * M_PI / 180.0;
    trim_gate.max_sample_age_s =
        node->get_parameter("test.trim_max_sample_age_s").as_double();
    auto transition_mode = std::make_shared<VtolPreflightMode>(
        *node, vehicle_id, 0.0);
    auto test_mode = std::make_shared<PropagationTestMode>(
        *node, vehicle_id, topic_namespace, candidate,
        trim_gate,
        node->get_parameter("test.cone_generation_duration_s").as_double(),
        node->get_parameter("test.tail_hold_duration_s").as_double());
    auto executor = std::make_shared<PropagationTestExecutor>(
        *transition_mode, *test_mode);

    auto debug_publisher =
        node->create_publisher<trajectory_prediction_hils::msg::TrajectoryPredictionDebug>(
            topic_namespace + "/testing/trajectory_prediction_debug",
            rclcpp::SensorDataQoS());
    const double dt = 1.0 / predict_rate;
    const auto timer_period = std::chrono::microseconds(
        static_cast<std::int64_t>(1.0e6 / predict_call));
    auto timer = node->create_wall_timer(
        timer_period,
        [node, case_id, test_mode, cone_publisher, debug_publisher, dt]() {
            if (!test_mode->coneGenerationActive()) {
                return;
            }
            const auto & candidate = test_mode->candidate();
            const PredictInput input{
                candidate.ground_speed,
                test_mode->baselineAltitude(),
                0.0,
                candidate.lateral_acceleration};
            PredictionInputTrajectory inputs;
            inputs.fill(input);
            BeliefConePublisher::PropagationSnapshot snapshot;
            if (!cone_publisher->publish(
                    inputs, dt, &snapshot,
                    test_mode->appliedInputTimestamp())) {
                return;
            }

            trajectory_prediction_hils::msg::TrajectoryPredictionDebug message;
            message.case_id = case_id;
            message.message_timestamp = static_cast<std::uint64_t>(
                node->now().nanoseconds() / 1000);
            message.cone_epoch_timestamp = snapshot.source_timestamp;
            message.source_timestamp = snapshot.source_timestamp;
            message.source_timestamp_sample = snapshot.source_timestamp_sample;
            message.applied_input_timestamp =
                test_mode->appliedInputTimestamp();
            message.source_delay_s = snapshot.source_delay_s;
            message.point_count = static_cast<std::uint8_t>(
                collision_avoidance::estimation::kTrajectoryPointCount);
            message.valid = true;
            for (std::size_t point = 0;
                 point < collision_avoidance::estimation::kTrajectoryPointCount;
                 ++point) {
                const auto & value = snapshot.cone[point];
                message.time_offset_s[point] = value.time_offset_s;
                const std::size_t base = point * 7;
                message.predicted_state[base] = value.mean.p_n;
                message.predicted_state[base + 1] = value.mean.p_e;
                message.predicted_state[base + 2] = value.mean.h;
                message.predicted_state[base + 3] = value.mean.V;
                message.predicted_state[base + 4] = value.mean.psi;
                message.predicted_state[base + 5] = value.mean.h_dot;
                message.predicted_state[base + 6] = value.mean.phi;
            }
            for (std::size_t interval = 0;
                 interval < collision_avoidance::estimation::kTrajectoryIntervalCount;
                 ++interval) {
                const std::size_t base = interval * 4;
                message.prediction_inputs[base] = input.V_cmd;
                message.prediction_inputs[base + 1] = input.h_cmd;
                message.prediction_inputs[base + 2] = input.h_dot_cmd;
                message.prediction_inputs[base + 3] = input.a_lat_cmd;
            }
            debug_publisher->publish(message);
        });

    RCLCPP_INFO(
        node->get_logger(),
        "[propagation-test] case=%s; FixedWing -> trim gate -> lateral maneuver; "
        "h_dot_cmd=0; endpoint=%.1fs rate=%.1fHz",
        case_id.c_str(), endpoint_s, predict_rate);
    if (!executor->doRegister() || !test_mode->doRegister()) {
        throw std::runtime_error("propagation SILS test mode registration failed");
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
