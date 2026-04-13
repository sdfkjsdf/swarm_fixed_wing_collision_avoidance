#include <collision_avoidance/TransferSameCoordinate.hpp>

using namespace Transfer_coordinate;

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TransferSameCoordinate>());
    rclcpp::shutdown();
    return 0;
}
