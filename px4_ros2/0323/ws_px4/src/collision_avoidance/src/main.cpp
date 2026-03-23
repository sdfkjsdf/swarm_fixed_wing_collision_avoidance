#include <collision_avoidance/CollisionAvoidanceControl.hpp>

using namespace safetyfilter;

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CollisionAvoidanceControl>());
    rclcpp::shutdown();
    return 0;
}
