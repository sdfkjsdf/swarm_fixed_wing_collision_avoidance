#include <rclcpp/rclcpp.hpp>
#include <collision_avoidance/VtolGuidanceMode.hpp>
#include <collision_avoidance/VtolGuidanceExecutor.hpp>


int main(int argc, char * argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    /* [1] Node 생성 */
    auto node = std::make_shared<rclcpp::Node>("vtol_guidance_node");

    /* [2] 파라미터 선언 및 할당 (기존과 동일) */
    node->declare_parameter<int>("vehicle_ID", 1);
    node->declare_parameter<int>("total_agent_num", 0);
    node->declare_parameter<float>("propagation_time", 0.0f);

    int   vehicle_ID       = node->get_parameter("vehicle_ID").as_int();
    int   total_agent_num  = node->get_parameter("total_agent_num").as_int();
    float propagation_time = static_cast<float>(
                                 node->get_parameter("propagation_time").as_double());

    /* [3] VtolGuidanceMode 생성 (ModeBase - 하나뿐)
           vehicle_ID 는 생성자에서 받아서 ModeBase 의 topic_namespace_prefix 에 사용 */
    auto my_mode = std::make_shared<VtolGuidanceMode>(*node, vehicle_ID);
    my_mode->setTotalAgentNum(total_agent_num);
    my_mode->setPropagationTime(propagation_time);

    /* [4] Executor 생성 (owned_mode = my_mode) */
    auto executor = std::make_shared<VtolGuidanceExecutor>(*my_mode);

    /* [5] PX4에 등록 */
    if (!executor->doRegister()) {
        throw std::runtime_error("Mode registration failed");
    }

    /* [6] 실행 (기존과 동일) */
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
