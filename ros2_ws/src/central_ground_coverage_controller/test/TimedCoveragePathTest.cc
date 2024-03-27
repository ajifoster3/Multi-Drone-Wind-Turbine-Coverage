#include <gtest/gtest.h>
#include "TimedCoveragePath.h"

TEST(TimedCoveragePathTest, setFirstZeroCoverageTimeViewpointTimeSetsFirstCorrectValue)
{
	CoveragePath path{0};

	TimedCoverageViewpoint viewpoint{CoverageViewpoint{Pose{Pose::Position{5,5,25},Pose::Orientation{0,0,0,0}}, 1}};
	path.addCoverageViewpoint(viewpoint);
	TimedCoveragePath timedPath{path};
	rosgraph_msgs::msg::Clock clock{};
	clock.clock.sec = 10;
	timedPath.setFirstZeroCoverageTimeViewpointTime(clock);
	ASSERT_EQ(true, !timedPath.getFirstZeroCoverageTimeViewpointTime().has_value());
}

TEST(TimedCoveragePathTest, getFirstZeroCoverageTimeViewpointTime)
{
	CoveragePath path{0};

	TimedCoverageViewpoint firstViewpoint{CoverageViewpoint{Pose{Pose::Position{5,5,10},Pose::Orientation{0,0,0,0}}, 1}};
	path.addCoverageViewpoint(firstViewpoint);
	TimedCoverageViewpoint secondViewpoint{CoverageViewpoint{Pose{Pose::Position{5,5,20},Pose::Orientation{0,0,0,0}}, 1}};
	path.addCoverageViewpoint(secondViewpoint);
	TimedCoveragePath timedPath{path};
	rosgraph_msgs::msg::Clock clock{};
	clock.clock.sec = 10;
	ASSERT_EQ(10, timedPath.getFirstZeroCoverageTimeViewpointTime().value().getPose().position.altitude);
	timedPath.setFirstZeroCoverageTimeViewpointTime(clock);
	ASSERT_EQ(20, timedPath.getFirstZeroCoverageTimeViewpointTime().value().getPose().position.altitude);
}

int main(int argc, char ** argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
