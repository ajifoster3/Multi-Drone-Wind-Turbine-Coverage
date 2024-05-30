#include <rclcpp/rclcpp.hpp>
#include "TrackingNode.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    if (argc < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: ros2 run [package_name] [executable_name] [uasNumber] [centralised/decentralised]");
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::spin(std::make_shared<trackingNode::TrackingNode>(5));

    rclcpp::shutdown();
    return 0;
}

