#include <rclcpp/rclcpp.hpp>
#include <collision_avoidance/VtolPreflightMode.hpp>
#include <collision_avoidance/FormationMode.hpp>
#include <collision_avoidance/VtolGuidanceExecutor.hpp>


int main(int argc, char * argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    /* [1] Node 생성 */
    auto node = std::make_shared<rclcpp::Node>("vtol_guidance_node");

    /* [2] 파라미터 선언 및 할당 */
    node->declare_parameter<int>("vehicle_ID", 1);
    node->declare_parameter<int>("total_agent_num", 0);

    int vehicle_ID      = node->get_parameter("vehicle_ID").as_int();
    int total_agent_num = node->get_parameter("total_agent_num").as_int();

    /* [3] 두 개의 Mode 인스턴스 생성
           - VtolPreflightMode: 천이 + 순항 안정화 (안정 코드)
           - FormationMode: 사용자 연구 대상 (Flocking 가이던스만) */
    auto preflight = std::make_shared<VtolPreflightMode>(*node, vehicle_ID, total_agent_num);
    auto formation = std::make_shared<FormationMode>(*node, vehicle_ID, total_agent_num);

    /* [4] Executor 생성 (owned_mode = preflight, second_mode = formation) */
    auto executor = std::make_shared<VtolGuidanceExecutor>(*preflight, *formation);

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
