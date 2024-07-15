#include "Takeoff.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;

    if (argc < 3)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: ros2 run [package_name] [executable_name] [uasNumber] [centralised/decentralised]");
        rclcpp::shutdown();
        return 1;
    }

    auto uasNumber = std::stoi(argv[1]);
    auto mode = std::string(argv[2]);
    std::shared_ptr<OffboardNode> offboardNode;
    if (mode == "centralised")
    {
        offboardNode = std::make_shared<OffboardNode>("offboard_node", uasNumber, CoverageMode::Centralised);
    }
    else if (mode == "decentralised")
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Running in decentralised mode...");
        offboardNode = std::make_shared<OffboardNode>("offboard_node", uasNumber, CoverageMode::Decentralised);
    }
    offboardNode->OffboardNodeSetup();
    executor.add_node(offboardNode);
    auto decentralisedCoverageNode = std::make_shared<DecentralisedCoverageNode>("decentralised_coverage_node", uasNumber);
    executor.add_node(decentralisedCoverageNode);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
