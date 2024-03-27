#include "OffboardNode.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	if (argc < 2) {
		RCLCPP_ERROR(
			rclcpp::get_logger(
				"rclcpp"), "Usage: ros2 run [package_name] [executable_name] [uasNumber]");
		rclcpp::shutdown();
		return 1;
	}

	int uasNumber = std::stoi(argv[1]); // Convert uasNumber to an integer
	auto offboardNode = std::make_shared<OffboardNode>("offboard_node", uasNumber);
	offboardNode->spinNode();

	rclcpp::shutdown();
	return 0;
}
