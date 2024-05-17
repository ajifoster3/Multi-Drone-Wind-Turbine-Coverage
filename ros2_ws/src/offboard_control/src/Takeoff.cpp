#include "OffboardNode.h"
#include <rclcpp/rclcpp.hpp>
#include "CoverageModes.h"
#include "Takeoff.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    if (argc < 3) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: ros2 run [package_name] [executable_name] [uasNumber] [centralised/decentralised]");
        rclcpp::shutdown();
        return 1;
    }

    auto uasNumber = std::stoi(argv[1]); // Convert uasNumber to an integer
    auto mode = std::string(argv[2]);
    std::shared_ptr<OffboardNode> offboardNode;
    if(mode == "centralised")
    {
        offboardNode = std::make_shared<OffboardNode>("offboard_node", uasNumber, CoverageMode::Centralised);
    }
    else if(mode == "decentralised")
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Running in decentralised mode...");
        offboardNode = std::make_shared<OffboardNode>("offboard_node", uasNumber, CoverageMode::Decentralised);
    }
    offboardNode->spinNode();

    rclcpp::shutdown();
    return 0;
}
