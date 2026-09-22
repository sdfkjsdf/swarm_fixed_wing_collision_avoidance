// Offline startup check only: never linked into the guidance node.
// Run with RMW_IMPLEMENTATION=rmw_fastrtps_cpp and the production XML profile.
#include <collision_avoidance/msg/trajectory_intent_batch.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw_fastrtps_cpp/get_publisher.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <iostream>
#include <stdexcept>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<rclcpp::Node>("check_trajectory_dds_profile");
        auto qos = rclcpp::QoS(rclcpp::KeepLast(6)).reliable().durability_volatile();
        auto read_qos = [](const auto & publisher) {
            auto * writer = rmw_fastrtps_cpp::get_datawriter(
                rcl_publisher_get_rmw_handle(publisher->get_publisher_handle().get()));
            if (!writer) throw std::runtime_error("not a Fast DDS writer");
            return writer->get_qos();
        };
        // A real ROS publisher without a matching XML profile is the baseline.
        auto other = node->create_publisher<collision_avoidance::msg::TrajectoryIntentBatch>(
            "/common/dds_profile_control", qos);
        const auto baseline = read_qos(other);
        if (baseline.reliable_writer_qos().times.heartbeatPeriod.seconds != 3 ||
            baseline.reliable_writer_qos().times.heartbeatPeriod.nanosec != 0) {
            throw std::runtime_error("unrelated writer default was modified");
        }
        for (int id = 0; id < 5; ++id) {
            auto pub = node->create_publisher<collision_avoidance::msg::TrajectoryIntentBatch>(
                "/common/px4_" + std::to_string(id) + "/trajectory_intent", qos);
            auto actual = read_qos(pub);
            auto & heartbeat = actual.reliable_writer_qos().times.heartbeatPeriod;
            if (heartbeat.seconds != 0 || heartbeat.nanosec != 50'000'000) {
                throw std::runtime_error("trajectory heartbeat profile was not applied");
            }
            heartbeat = baseline.reliable_writer_qos().times.heartbeatPeriod;
            if (!(actual == baseline)) {
                throw std::runtime_error("profile changed QoS beyond heartbeat period");
            }
            std::cout << "vehicle=" << id << " heartbeat_ms=50 other_qos_unchanged=PASS\n";
        }
        std::cout << "unrelated_writer_heartbeat_ms=3000 PASS\n";
    } catch (const std::exception & e) {
        std::cerr << e.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
