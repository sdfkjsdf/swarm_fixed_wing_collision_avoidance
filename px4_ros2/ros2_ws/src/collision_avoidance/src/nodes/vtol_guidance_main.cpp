#include <rclcpp/rclcpp.hpp>
#include <cmath>
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
    node->declare_parameter<double>("aircraft_half_wingspan", 1.072);
    node->declare_parameter<bool>(
        "maneuver_selection_exhaustive_test_mode", false);

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
        collision_avoidance::selection::ManeuverSelectionWorkerParams params;
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
        const double half_wingspan =
            node->get_parameter("aircraft_half_wingspan").as_double();
        params.evaluator_params.ownship_half_wingspan_m = half_wingspan;
        params.evaluator_params.threat_half_wingspan_m = half_wingspan;
        params.exhaustive_test_mode = node->get_parameter(
            "maneuver_selection_exhaustive_test_mode").as_bool();
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
        RCLCPP_INFO(
            node->get_logger(),
            "[main] distributed maneuver selection: enabled=%d shadow_only=%d "
            "exhaustive_test=%d",
            maneuver_selection_runtime->enabled() ? 1 : 0,
            shadow_only ? 1 : 0,
            params.exhaustive_test_mode ? 1 : 0);
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
