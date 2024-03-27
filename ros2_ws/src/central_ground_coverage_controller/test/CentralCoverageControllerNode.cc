#include <gtest/gtest.h>
#include "CentralCoverageControllerNode.h"

TEST(CentralCoverageControllerNodeTest, NodeStartingCorrectly)
{
    rclcpp::init(0, nullptr);
    
    auto central_coverage_controller_node = std::make_shared<CentralCoverageControllerNode>(
		"central_coverage_controller_node", 5);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(central_coverage_controller_node);
	
    ASSERT_NO_THROW(executor.spin(););

    std::this_thread::sleep_for(std::chrono::seconds(5));
    executor.cancel();

    rclcpp::shutdown();
}

TEST(CentralCoverageControllerNodeTest, globalPositionCbTest)
{
  
}

int main(int argc, char ** argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
