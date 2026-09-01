#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
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
    node->declare_parameter<bool>("amac_active_switching_enabled", false);
    node->declare_parameter<double>("amac_active_switch_cost_margin", 0.0);
    node->declare_parameter<double>(
        "amac_active_switch_minimum_ad_margin_m", 0.0);
    node->declare_parameter<double>("aircraft_half_wingspan", 1.072);
    // AMAC/Lockheed selection is AD-ranked. The positive-margin filter is a
    // separate CBF experiment and must be explicitly enabled by its profile.
    node->declare_parameter<bool>("positive_margin_filter_enabled", false);
    node->declare_parameter<double>("positive_margin_gamma", 0.02);
    node->declare_parameter<bool>(
        "maneuver_selection_exhaustive_test_mode", false);
    node->declare_parameter<bool>("v4_safe_control_enabled", true);
    node->declare_parameter<bool>("v4_shadow_only", true);
    node->declare_parameter<std::string>(
        "v4_control_architecture", "legacy_safe_control_set");
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
    node->declare_parameter<double>("mode_b_path_alpha_gain_per_s", 0.2);
    node->declare_parameter<double>("mode_b_terminal_alpha_gain_per_s", 0.5);
    node->declare_parameter<double>("mode_b_certification_tolerance_m", 1.0e-6);
    node->declare_parameter<double>("mode_b_maximum_intent_age_s", 1.0);
    node->declare_parameter<double>(
        "amac_relative_speed_epsilon_mps", 1.0e-6);
    node->declare_parameter<double>("amac_cpa_horizon_s", 4.5);
    node->declare_parameter<bool>("formation_discrimination_enabled", false);
    node->declare_parameter<std::string>(
        "formation_aggregation_policy", "per_threat_exemption_only");
    node->declare_parameter<std::string>(
        "formation_profile_name", "unconfigured");
    node->declare_parameter<std::string>(
        "formation_profile_kind", "uav_calibrated");
    node->declare_parameter<double>("formation_representative_wingspan_m", 0.0);
    node->declare_parameter<double>("formation_range0_wingspan_scale", 0.0);
    node->declare_parameter<double>("formation_uncertainty_margin_m", 0.0);
    node->declare_parameter<double>("formation_range1_offset_m", 0.0);
    node->declare_parameter<std::vector<double>>(
        "formation_closure_upper_entry_table", std::vector<double>{});
    node->declare_parameter<std::vector<double>>(
        "formation_closure_upper_exit_table", std::vector<double>{});
    node->declare_parameter<double>("formation_closure_lower_entry_mps", 0.0);
    node->declare_parameter<double>("formation_closure_lower_exit_mps", 0.0);
    node->declare_parameter<double>("formation_fdz_entry_limit_m", 0.0);
    node->declare_parameter<double>("formation_fdz_exit_limit_m", 0.0);
    node->declare_parameter<double>("formation_max_range_entry_m", 0.0);
    node->declare_parameter<double>("formation_max_range_exit_m", 0.0);
    node->declare_parameter<double>("formation_maximum_state_age_s", 0.0);
    node->declare_parameter<double>("formation_maximum_future_skew_s", 0.0);
    node->declare_parameter<double>("formation_maximum_timestamp_skew_s", 0.0);
    node->declare_parameter<std::string>(
        "formation_lookup_policy", "reject_outside_table");

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
        } else if (execution_policy == "horizon_gated_v4") {
            params.execution_policy = collision_avoidance::selection::
                ManeuverExecutionPolicy::HorizonGatedV4;
        } else {
            throw std::invalid_argument(
                "avoidance_execution_policy must be amac_ad_threshold, continuous_v4, or horizon_gated_v4");
        }
        params.ground_speed_command_mps =
            node->get_parameter("maneuver_ground_speed_command").as_double();
        params.active_switching_enabled = node->get_parameter(
            "amac_active_switching_enabled").as_bool();
        params.active_switch_cost_margin = node->get_parameter(
            "amac_active_switch_cost_margin").as_double();
        params.active_switch_minimum_ad_margin_m = node->get_parameter(
            "amac_active_switch_minimum_ad_margin_m").as_double();
        params.activation_params.relative_speed_epsilon_mps =
            node->get_parameter(
                "amac_relative_speed_epsilon_mps").as_double();
        params.activation_params.cpa_horizon_s = node->get_parameter(
            "amac_cpa_horizon_s").as_double();
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
        params.v4_horizon_trigger_m =
            params.evaluator_params.desired_separation_distance_m;
        if (params.execution_policy == collision_avoidance::selection::
                ManeuverExecutionPolicy::HorizonGatedV4) {
            // The horizon policy requires the robust cone barrier as its hard
            // candidate-admissibility layer. AD remains a secondary ranking.
            params.evaluator_params.robust_cone_filter_enabled = true;
        }
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
        const std::string v4_architecture = node->get_parameter(
            "v4_control_architecture").as_string();
        if (v4_architecture == "legacy_safe_control_set") {
            params.v4_control_architecture = collision_avoidance::selection::
                V4ControlArchitecture::LegacySafeControlSet;
        } else if (v4_architecture == "closed_form_backup_mode_b") {
            params.v4_control_architecture = collision_avoidance::selection::
                V4ControlArchitecture::ClosedFormBackupModeB;
        } else {
            throw std::invalid_argument(
                "v4_control_architecture must be legacy_safe_control_set or closed_form_backup_mode_b");
        }
        if ((params.execution_policy == collision_avoidance::selection::
                ManeuverExecutionPolicy::ContinuousV4
            || params.execution_policy == collision_avoidance::selection::
                ManeuverExecutionPolicy::HorizonGatedV4)
            && (!params.v4_safe_control_enabled || params.v4_shadow_only)) {
            throw std::invalid_argument(
                "V4 execution policies require V4 cutover");
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
        auto & mode_b = params.mode_b_interpolator_params;
        mode_b.certifier.model.gravity_mps2 = params.gravity_mps2;
        mode_b.certifier.model.maximum_roll_rad = maximum_roll_radians;
        mode_b.certifier.model.maximum_yaw_rate_radps =
            params.v4_safe_control_params.maximum_yaw_rate_radps;
        mode_b.certifier.model.speed_tolerance_mps =
            params.v4_safe_control_params.tolerances.speed_mps;
        mode_b.certifier.horizon_s = 4.5;
        mode_b.certifier.integration_step_s =
            collision_avoidance::estimation::kTrajectoryIntentStepSeconds;
        mode_b.certifier.reference_margin_m =
            params.evaluator_params.desired_separation_distance_m;
        mode_b.certifier.certification_tolerance_m = node->get_parameter(
            "mode_b_certification_tolerance_m").as_double();
        mode_b.path_alpha_gain_per_s = node->get_parameter(
            "mode_b_path_alpha_gain_per_s").as_double();
        mode_b.terminal_alpha_gain_per_s = node->get_parameter(
            "mode_b_terminal_alpha_gain_per_s").as_double();
        mode_b.tolerances.coefficient_mps =
            params.v4_safe_control_params.tolerances.constraint_mps;
        mode_b.tolerances.residual_mps =
            params.v4_safe_control_params.tolerances.constraint_mps;
        mode_b.tolerances.mu =
            params.v4_safe_control_params.tolerances.interval_radps;
        mode_b.tolerances.distance_m =
            mode_b.certifier.certification_tolerance_m;

        auto & intent_adapter = params.mode_b_intent_adapter_params;
        intent_adapter.predictor = params.predictor_params;
        intent_adapter.horizon_s = mode_b.certifier.horizon_s;
        intent_adapter.integration_step_s =
            mode_b.certifier.integration_step_s;
        intent_adapter.maximum_intent_age_us = seconds_to_microseconds(
            node->get_parameter("mode_b_maximum_intent_age_s").as_double());
        intent_adapter.time_tolerance_s =
            mode_b.certifier.threat_time_tolerance_s;

        params.formation_discrimination_enabled = node->get_parameter(
            "formation_discrimination_enabled").as_bool();
        params.formation_target_separation_m = node->get_parameter(
            "flocking_desired_distance").as_double();
        auto & formation_config = params.formation_boundary_config;
        formation_config.profile_name = node->get_parameter(
            "formation_profile_name").as_string();
        const std::string formation_profile_kind = node->get_parameter(
            "formation_profile_kind").as_string();
        if (formation_profile_kind == "source_reference") {
            formation_config.profile_kind = collision_avoidance::formation::
                FormationProfileKind::SourceReference;
        } else if (formation_profile_kind == "uav_calibrated") {
            formation_config.profile_kind = collision_avoidance::formation::
                FormationProfileKind::UavCalibrated;
        } else {
            throw std::invalid_argument(
                "formation_profile_kind must be source_reference or uav_calibrated");
        }
        const std::string aggregation_policy = node->get_parameter(
            "formation_aggregation_policy").as_string();
        if (aggregation_policy == "per_threat_exemption_only") {
            params.formation_aggregation_policy =
                collision_avoidance::formation::FormationAggregationPolicy::
                    PerThreatExemptionOnly;
        } else if (aggregation_policy == "all_relevant_threats_formation") {
            params.formation_aggregation_policy =
                collision_avoidance::formation::FormationAggregationPolicy::
                    AllRelevantThreatsFormation;
        } else if (aggregation_policy == "any_relevant_threat_formation") {
            params.formation_aggregation_policy =
                collision_avoidance::formation::FormationAggregationPolicy::
                    AnyRelevantThreatFormation;
        } else {
            throw std::invalid_argument(
                "invalid formation_aggregation_policy");
        }
        formation_config.representative_wingspan_m = node->get_parameter(
            "formation_representative_wingspan_m").as_double();
        formation_config.range0_wingspan_scale = node->get_parameter(
            "formation_range0_wingspan_scale").as_double();
        formation_config.uncertainty_margin_m = node->get_parameter(
            "formation_uncertainty_margin_m").as_double();
        formation_config.range1_offset_m = node->get_parameter(
            "formation_range1_offset_m").as_double();
        formation_config.closure_lower_entry_mps = node->get_parameter(
            "formation_closure_lower_entry_mps").as_double();
        formation_config.closure_lower_exit_mps = node->get_parameter(
            "formation_closure_lower_exit_mps").as_double();
        formation_config.fdz_entry_limit_m = node->get_parameter(
            "formation_fdz_entry_limit_m").as_double();
        formation_config.fdz_exit_limit_m = node->get_parameter(
            "formation_fdz_exit_limit_m").as_double();
        formation_config.max_range_entry_m = node->get_parameter(
            "formation_max_range_entry_m").as_double();
        formation_config.max_range_exit_m = node->get_parameter(
            "formation_max_range_exit_m").as_double();
        formation_config.maximum_state_age_s = node->get_parameter(
            "formation_maximum_state_age_s").as_double();
        formation_config.maximum_future_skew_s = node->get_parameter(
            "formation_maximum_future_skew_s").as_double();
        formation_config.maximum_timestamp_skew_s = node->get_parameter(
            "formation_maximum_timestamp_skew_s").as_double();
        if (params.formation_discrimination_enabled) {
            const auto parse_boundary_table = [](
                const std::vector<double> & flat_values) {
                if (flat_values.size() % 2U != 0U) {
                    throw std::invalid_argument(
                        "formation closure tables require [range, closure] pairs");
                }
                std::vector<
                    collision_avoidance::formation::FormationBoundaryNode>
                    table;
                table.reserve(flat_values.size() / 2U);
                for (std::size_t index = 0; index < flat_values.size();
                     index += 2U) {
                    table.push_back(
                        {flat_values[index], flat_values[index + 1U]});
                }
                return table;
            };
            formation_config.closure_upper_entry_table = parse_boundary_table(
                node->get_parameter(
                    "formation_closure_upper_entry_table").as_double_array());
            formation_config.closure_upper_exit_table = parse_boundary_table(
                node->get_parameter(
                    "formation_closure_upper_exit_table").as_double_array());
            const std::string formation_lookup_policy = node->get_parameter(
                "formation_lookup_policy").as_string();
            if (formation_lookup_policy == "reject_outside_table") {
                formation_config.lookup_policy =
                    collision_avoidance::formation::FormationLookupPolicy::
                        RejectOutsideTable;
            } else if (formation_lookup_policy == "clamp_to_endpoint") {
                formation_config.lookup_policy =
                    collision_avoidance::formation::FormationLookupPolicy::
                        ClampToEndpoint;
            } else {
                throw std::invalid_argument("invalid formation_lookup_policy");
            }
        }
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
            "execution_policy=%s exhaustive_test=%d active_switch=%d "
            "switch_cost_margin=%.6f switch_ad_margin=%.3f "
            "v4_enabled=%d v4_shadow_only=%d v4_architecture=%s",
            maneuver_selection_runtime->enabled() ? 1 : 0,
            shadow_only ? 1 : 0,
            execution_policy.c_str(),
            params.exhaustive_test_mode ? 1 : 0,
            params.active_switching_enabled ? 1 : 0,
            params.active_switch_cost_margin,
            params.active_switch_minimum_ad_margin_m,
            params.v4_safe_control_enabled ? 1 : 0,
            params.v4_shadow_only ? 1 : 0,
            v4_architecture.c_str());
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
