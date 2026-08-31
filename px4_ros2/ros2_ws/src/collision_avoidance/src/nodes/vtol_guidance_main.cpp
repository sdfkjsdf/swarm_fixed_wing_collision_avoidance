#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <collision_avoidance/modes/VtolPreflightMode.hpp>
#include <collision_avoidance/modes/FormationMode.hpp>
#include <collision_avoidance/modes/VtolGuidanceExecutor.hpp>
#include <collision_avoidance/communication/DistributedManeuverSelectionRuntime.hpp>


int main(int argc, char * argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    /* [1] Node 생성 */
    auto node = std::make_shared<rclcpp::Node>("vtol_guidance_node");

    /* [2] 파라미터 선언 및 할당 */
    node->declare_parameter<int>("vehicle_ID", 0);
    node->declare_parameter<int>("total_agent_num", 0);

    int vehicle_ID      = node->get_parameter("vehicle_ID").as_int();
    int total_agent_num = node->get_parameter("total_agent_num").as_int();

    node->declare_parameter<bool>("maneuver_selection_enabled", true);
    node->declare_parameter<bool>("collision_avoidance_shadow_only", true);
    node->declare_parameter<double>("maneuver_ground_speed_command", 20.0);
    node->declare_parameter<double>("desired_separation_distance", 10.0);
    node->declare_parameter<std::string>(
        "avoidance_execution_policy", "amac_ad_threshold");
    node->declare_parameter<double>("amac_activation_threshold_m", 0.0);
    node->declare_parameter<double>("aircraft_half_wingspan", 1.072);
    node->declare_parameter<bool>("positive_margin_filter_enabled", true);
    node->declare_parameter<double>("positive_margin_gamma", 0.02);
    node->declare_parameter<bool>(
        "maneuver_selection_exhaustive_test_mode", false);
    node->declare_parameter<bool>("v4_safe_control_enabled", true);
    node->declare_parameter<bool>("v4_shadow_only", true);
    node->declare_parameter<double>("airspeed_cruise", 15.0);
    node->declare_parameter<double>("max_yaw_rate_deg_per_s", 50.0);
    node->declare_parameter<double>("v4_margin_time_constant_s", 5.0);
    node->declare_parameter<double>("v4_candidate_guard_deg_per_s", 0.5);
    node->declare_parameter<double>("v4_constraint_tolerance_mps", 1.0e-5);
    node->declare_parameter<double>(
        "v4_interval_tolerance_radps", 1.0e-7);
    node->declare_parameter<double>("v4_speed_tolerance_mps", 1.0e-3);
    node->declare_parameter<double>("v4_direction_tolerance_m", 1.0e-4);
    node->declare_parameter<double>("v4_maximum_airspeed_age_s", 1.0);
    node->declare_parameter<double>("v4_maximum_nominal_age_s", 1.0);

    /* [3] 두 개의 Mode 인스턴스 생성
           - VtolPreflightMode: 천이 + 순항 안정화 (안정 코드)
           - FormationMode: 사용자 연구 대상 (Flocking 가이던스만) */
    auto preflight = std::make_shared<VtolPreflightMode>(*node, vehicle_ID, total_agent_num);
    auto formation = std::make_shared<FormationMode>(*node, vehicle_ID, total_agent_num);

    const bool shadow_only =
        node->get_parameter("collision_avoidance_shadow_only").as_bool();
    formation->setCollisionAvoidanceShadowOnly(shadow_only);

    /* [4] Executor 생성 (owned_mode = preflight, second_mode = formation) */
    auto executor = std::make_shared<VtolGuidanceExecutor>(*preflight, *formation);

    std::shared_ptr<
        collision_avoidance::communication::DistributedManeuverSelectionRuntime>
        maneuver_selection_runtime;
    if (node->get_parameter("maneuver_selection_enabled").as_bool()) {
        const auto seconds_to_microseconds = [](double seconds) {
            const double maximum_seconds = static_cast<double>(
                std::numeric_limits<std::uint64_t>::max()) * 1.0e-6;
            if (!std::isfinite(seconds) || seconds <= 0.0
                || seconds > maximum_seconds) {
                return std::uint64_t{0};
            }
            return static_cast<std::uint64_t>(
                std::llround(seconds * 1.0e6));
        };
        collision_avoidance::selection::ManeuverSelectionWorkerParams params;
        const std::string execution_policy = node->get_parameter(
            "avoidance_execution_policy").as_string();
        if (execution_policy == "amac_ad_threshold") {
            params.execution_policy = collision_avoidance::selection::
                ManeuverExecutionPolicy::AmacAdThreshold;
        } else if (execution_policy == "continuous_v4") {
            params.execution_policy = collision_avoidance::selection::
                ManeuverExecutionPolicy::ContinuousV4;
        } else {
            throw std::invalid_argument(
                "avoidance_execution_policy must be amac_ad_threshold or continuous_v4");
        }
        params.activation_params.activation_threshold_m = node->get_parameter(
            "amac_activation_threshold_m").as_double();
        params.ground_speed_command_mps =
            node->get_parameter("maneuver_ground_speed_command").as_double();
        params.gravity_mps2 = node->get_parameter("gravity").as_double();
        const double maximum_roll_radians =
            node->get_parameter("max_roll_deg").as_double()
            * std::acos(-1.0) / 180.0;
        params.predictor_params.a_lat_max =
            params.gravity_mps2 * std::tan(maximum_roll_radians);
        params.evaluator_params.desired_separation_distance_m =
            node->get_parameter("desired_separation_distance").as_double();
        params.evaluator_params.positive_margin_filter_enabled =
            node->get_parameter("positive_margin_filter_enabled").as_bool();
        params.evaluator_params.positive_margin_gamma =
            node->get_parameter("positive_margin_gamma").as_double();
        params.evaluator_params.positive_margin_reference_m =
            params.evaluator_params.desired_separation_distance_m;
        params.evaluator_params.maximum_lateral_acceleration_mps2 =
            params.predictor_params.a_lat_max;
        const double half_wingspan =
            node->get_parameter("aircraft_half_wingspan").as_double();
        params.evaluator_params.ownship_half_wingspan_m = half_wingspan;
        params.evaluator_params.threat_half_wingspan_m = half_wingspan;
        params.exhaustive_test_mode = node->get_parameter(
            "maneuver_selection_exhaustive_test_mode").as_bool();
        params.v4_safe_control_enabled = node->get_parameter(
            "v4_safe_control_enabled").as_bool();
        params.v4_shadow_only = node->get_parameter(
            "v4_shadow_only").as_bool();
        if (params.execution_policy == collision_avoidance::selection::
                ManeuverExecutionPolicy::ContinuousV4
            && (!params.v4_safe_control_enabled || params.v4_shadow_only
                || params.activation_params.activation_threshold_m != 0.0)) {
            throw std::invalid_argument(
                "continuous_v4 requires V4 cutover and amac_activation_threshold_m=0");
        }
        params.v4_trim_airspeed_mps = node->get_parameter(
            "airspeed_cruise").as_double();
        params.v4_maximum_airspeed_age_us = seconds_to_microseconds(
            node->get_parameter(
                "v4_maximum_airspeed_age_s").as_double());
        params.v4_maximum_nominal_age_us = seconds_to_microseconds(
            node->get_parameter(
                "v4_maximum_nominal_age_s").as_double());
        params.v4_safe_control_params.margin_reference_m =
            params.evaluator_params.desired_separation_distance_m;
        params.v4_safe_control_params.margin_time_constant_s =
            node->get_parameter("v4_margin_time_constant_s").as_double();
        params.v4_safe_control_params.control_period_s =
            static_cast<double>(params.trajectory_refresh_period_us) * 1.0e-6;
        params.v4_safe_control_params.gravity_mps2 = params.gravity_mps2;
        params.v4_safe_control_params.maximum_roll_rad = maximum_roll_radians;
        params.v4_safe_control_params.maximum_yaw_rate_radps =
            node->get_parameter("max_yaw_rate_deg_per_s").as_double()
            * std::acos(-1.0) / 180.0;
        params.v4_safe_control_params.tolerances.constraint_mps =
            node->get_parameter("v4_constraint_tolerance_mps").as_double();
        params.v4_safe_control_params.tolerances.interval_radps =
            node->get_parameter("v4_interval_tolerance_radps").as_double();
        params.v4_safe_control_params.tolerances.speed_mps =
            node->get_parameter("v4_speed_tolerance_mps").as_double();
        params.v4_safe_control_params.tolerances.direction_m =
            node->get_parameter("v4_direction_tolerance_m").as_double();
        params.v4_candidate_adapter_params.robustness_guard_radps =
            node->get_parameter("v4_candidate_guard_deg_per_s").as_double()
            * std::acos(-1.0) / 180.0;
        params.v4_candidate_adapter_params.duplicate_tolerance_radps =
            params.v4_safe_control_params.tolerances.interval_radps;
        params.v4_candidate_adapter_params.speed_tolerance_mps =
            params.v4_safe_control_params.tolerances.speed_mps;
        maneuver_selection_runtime = std::make_shared<
            collision_avoidance::communication::DistributedManeuverSelectionRuntime>(
            *node,
            vehicle_ID,
            total_agent_num,
            params,
            [formation](
                const collision_avoidance::selection::ManeuverSelectionDecision &
                    decision) {
                formation->setManeuverSelectionDecision(decision);
            });
        std::weak_ptr<
            collision_avoidance::communication::DistributedManeuverSelectionRuntime>
            weak_runtime = maneuver_selection_runtime;
        formation->setManeuverActivationGateCallback(
            [weak_runtime](bool enabled) {
                if (const auto runtime = weak_runtime.lock()) {
                    runtime->setActivationEnabled(enabled);
                }
            });
        if (params.v4_safe_control_enabled) {
            formation->setNominalSetpointCallback(
                [weak_runtime](
                    const collision_avoidance::selection::
                        ManeuverSelectionNominalSetpointSnapshot & snapshot) {
                    if (const auto runtime = weak_runtime.lock()) {
                        static_cast<void>(
                            runtime->pushNominalSetpoint(snapshot));
                    }
                });
        }
        RCLCPP_INFO(
            node->get_logger(),
            "[main] distributed maneuver selection: enabled=%d shadow_only=%d "
            "execution_policy=%s ad_threshold=%.3f exhaustive_test=%d "
            "v4_enabled=%d v4_shadow_only=%d",
            maneuver_selection_runtime->enabled() ? 1 : 0,
            shadow_only ? 1 : 0,
            execution_policy.c_str(),
            params.activation_params.activation_threshold_m,
            params.exhaustive_test_mode ? 1 : 0,
            params.v4_safe_control_enabled ? 1 : 0,
            params.v4_shadow_only ? 1 : 0);
    }

    /* [5] PX4에 등록 — Executor 와 Formation 둘 다 doRegister() 필요 */
    RCLCPP_INFO(node->get_logger(),
        "[main] doRegister() 시작 (vehicle_ID=%d)", vehicle_ID);

    if (!executor->doRegister()) {
        RCLCPP_ERROR(node->get_logger(),
            "[main] Executor doRegister() 실패");
        throw std::runtime_error("Executor registration failed");
    }
    if (!formation->doRegister()) {
        RCLCPP_ERROR(node->get_logger(),
            "[main] FormationMode doRegister() 실패");
        throw std::runtime_error("FormationMode registration failed");
    }

    RCLCPP_INFO(node->get_logger(),
        "[main] doRegister() 성공 → spin 시작");

    /* [6] 실행 */
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
