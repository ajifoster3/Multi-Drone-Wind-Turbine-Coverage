#include <gtest/gtest.h>
#include "GreedyIterativeCoveragePathPlanner.h"
#include "Pose.h"

TEST(GreedyIterativeCoveragePathPlannerTest, findClosestUnassignedViewpointIndexLongitudeTest)
{
    std::vector<Pose> initalPoses{Pose{Pose::Position{5.009371, 5.500150, 10}, Pose::Orientation{0,0,0,0}}};
    Pose coverageViewpointPoseClose{Pose{Pose::Position{5.009371, 5.500200, 10}, Pose::Orientation{0,0,0,0}}};
    Pose coverageViewpointPoseFar{Pose{Pose::Position{5.009371, 5.500000, 10}, Pose::Orientation{0,0,0,0}}};
    

    std::vector<CoverageViewpoint> coverageViewpoints{
        CoverageViewpoint{coverageViewpointPoseClose, false },
        CoverageViewpoint{coverageViewpointPoseFar, false }
        };
    
    GreedyIterativeCoveragePathPlanner planner(std::vector<int>{0}, initalPoses, coverageViewpoints);

    EXPECT_EQ(0, planner.getCoveragePaths()[0].getRobotId()) << "RobotId not assigned correct value.";
    EXPECT_EQ(5.500200, planner.getCoveragePaths()[0].getPath()[0].getPose().position.longitude) << "Incorrect robotic path: The more distant longitude pose was selected first.";
}

TEST(GreedyIterativeCoveragePathPlannerTest, findClosestUnassignedViewpointIndexLatitudeTest)
{
    std::vector<Pose> initalPoses{Pose{Pose::Position{5.0, 5.0, 10}, Pose::Orientation{0,0,0,0}}};
    Pose coverageViewpointPoseClose{Pose{Pose::Position{5.1, 5.0, 10}, Pose::Orientation{0,0,0,0}}};
    Pose coverageViewpointPoseFar{Pose{Pose::Position{4.8, 5.0, 10}, Pose::Orientation{0,0,0,0}}};
    

    std::vector<CoverageViewpoint> coverageViewpoints{
        CoverageViewpoint{coverageViewpointPoseClose, false },
        CoverageViewpoint{coverageViewpointPoseFar, false }
        };
    
    GreedyIterativeCoveragePathPlanner planner(std::vector<int>{0}, initalPoses, coverageViewpoints);

    EXPECT_EQ(0, planner.getCoveragePaths()[0].getRobotId()) << "RobotId not assigned correct value.";
    EXPECT_EQ(5.1, planner.getCoveragePaths()[0].getPath()[0].getPose().position.latitude) << "Incorrect robotic path: The more distant latitude pose was selected first.";
}

TEST(GreedyIterativeCoveragePathPlannerTest, findClosestUnassignedViewpointIndexAltitudeTest)
{
    std::vector<Pose> initalPoses{Pose{Pose::Position{5.0, 5.0, 10}, Pose::Orientation{0,0,0,0}}};
    Pose coverageViewpointPoseClose{Pose{Pose::Position{5.0, 5.0, -100}, Pose::Orientation{0,0,0,0}}};
    Pose coverageViewpointPoseFar{Pose{Pose::Position{5.0, 5.0, 30}, Pose::Orientation{0,0,0,0}}};
    

    std::vector<CoverageViewpoint> coverageViewpoints{
        CoverageViewpoint{coverageViewpointPoseClose, false },
        CoverageViewpoint{coverageViewpointPoseFar, false }
        };
    
    GreedyIterativeCoveragePathPlanner planner(std::vector<int>{0}, initalPoses, coverageViewpoints);

    EXPECT_EQ(0, planner.getCoveragePaths()[0].getRobotId()) << "RobotId not assigned correct value.";
    EXPECT_EQ(30, planner.getCoveragePaths()[0].getPath()[0].getPose().position.altitude) << "Incorrect robotic path: The more distant altitude pose was selected first.";
}

TEST(GreedyIterativeCoveragePathPlannerTest, findClosestUnassignedViewpointIndexAltitudeAndLatitudeTest)
{
    std::vector<Pose> initalPoses{Pose{Pose::Position{50.381017, -4.139476, 0}, Pose::Orientation{0,0,0,0}}};
    Pose coverageViewpointPoseClose{Pose{Pose::Position{50.382677, -4.139036, 0}, Pose::Orientation{0,0,0,0}}};//190m away
    Pose coverageViewpointPoseFar{Pose{Pose::Position{50.381017, -4.139476, 200}, Pose::Orientation{0,0,0,0}}};//200m up
    

    std::vector<CoverageViewpoint> coverageViewpoints{
        CoverageViewpoint{coverageViewpointPoseClose, false },
        CoverageViewpoint{coverageViewpointPoseFar, false }
        };
    
    GreedyIterativeCoveragePathPlanner planner(std::vector<int>{0}, initalPoses, coverageViewpoints);

    EXPECT_EQ(0, planner.getCoveragePaths()[0].getRobotId()) << "RobotId not assigned correct value.";
    EXPECT_EQ(0, planner.getCoveragePaths()[0].getPath()[0].getPose().position.altitude) << "Incorrect robotic path: The more distant altitude pose was selected first.";
}

TEST(GreedyIterativeCoveragePathPlannerTest, plannerSingleRobotThreeCoveragePointsTest)
{
    std::vector<Pose> initalPoses{Pose{Pose::Position{5.0, 5.0, 10}, Pose::Orientation{0,0,0,0}}};
    Pose coverageViewpointPoseClose{Pose{Pose::Position{5.0, 5.0, 20}, Pose::Orientation{0,0,0,0}}};
    Pose coverageViewpointPoseMiddle{Pose{Pose::Position{5.0, 5.0, 30}, Pose::Orientation{0,0,0,0}}};
    Pose coverageViewpointPoseFar{Pose{Pose::Position{5.0, 5.0, 40}, Pose::Orientation{0,0,0,0}}};

    std::vector<CoverageViewpoint> coverageViewpoints{
        CoverageViewpoint{coverageViewpointPoseClose, false },
        CoverageViewpoint{coverageViewpointPoseMiddle, false },
        CoverageViewpoint{coverageViewpointPoseFar, false }
        };
    
    GreedyIterativeCoveragePathPlanner planner(std::vector<int>{0}, initalPoses, coverageViewpoints);

    EXPECT_EQ(0, planner.getCoveragePaths()[0].getRobotId()) << "RobotId not assigned correct value.";
    EXPECT_EQ(20, planner.getCoveragePaths()[0].getPath()[0].getPose().position.altitude) << "Incorrect robotic path: The first Viewpoint assigned is incorrect.";
    EXPECT_EQ(30, planner.getCoveragePaths()[0].getPath()[1].getPose().position.altitude) << "Incorrect robotic path: The second Viewpoint assigned is incorrect.";
    EXPECT_EQ(40, planner.getCoveragePaths()[0].getPath()[2].getPose().position.altitude) << "Incorrect robotic path: The third Viewpoint assigned is incorrect.";
}

TEST(GreedyIterativeCoveragePathPlannerTest, plannerTwoRobotSixLinearCoveragePointsTest)
{
    //Both robots at same start pose
    std::vector<Pose> initalPoses{
        Pose{Pose::Position{5.0, 5.0, 10}, Pose::Orientation{0,0,0,0}},
        Pose{Pose::Position{5.0, 5.0, 10}, Pose::Orientation{0,0,0,0}}
        };
    //Equidistant Poses, expected behaviour of alternating allocation for each robot
    Pose coverageViewpointPose1{Pose{Pose::Position{5.0, 5.0, 20}, Pose::Orientation{0,0,0,0}}};
    Pose coverageViewpointPose2{Pose{Pose::Position{5.0, 5.0, 30}, Pose::Orientation{0,0,0,0}}};
    Pose coverageViewpointPose3{Pose{Pose::Position{5.0, 5.0, 40}, Pose::Orientation{0,0,0,0}}};
    Pose coverageViewpointPose4{Pose{Pose::Position{5.0, 5.0, 50}, Pose::Orientation{0,0,0,0}}};
    Pose coverageViewpointPose5{Pose{Pose::Position{5.0, 5.0, 60}, Pose::Orientation{0,0,0,0}}};
    Pose coverageViewpointPose6{Pose{Pose::Position{5.0, 5.0, 70}, Pose::Orientation{0,0,0,0}}};

    std::vector<CoverageViewpoint> coverageViewpoints{
        CoverageViewpoint{coverageViewpointPose1, false },
        CoverageViewpoint{coverageViewpointPose2, false },
        CoverageViewpoint{coverageViewpointPose3, false },
        CoverageViewpoint{coverageViewpointPose4, false },
        CoverageViewpoint{coverageViewpointPose5, false },
        CoverageViewpoint{coverageViewpointPose6, false }
        };
    
    GreedyIterativeCoveragePathPlanner planner(std::vector<int>{0,2}, initalPoses, coverageViewpoints);

    EXPECT_EQ(0, planner.getCoveragePaths()[0].getRobotId()) << "RobotId not assigned correct value.";
    EXPECT_EQ(20, planner.getCoveragePaths()[0].getPath()[0].getPose().position.altitude) << "Incorrect robotic path: The first Viewpoint assigned to the first robot is incorrect.";
    EXPECT_EQ(40, planner.getCoveragePaths()[0].getPath()[1].getPose().position.altitude) << "Incorrect robotic path: The second Viewpoint assigned to the first robot  is incorrect.";
    EXPECT_EQ(60, planner.getCoveragePaths()[0].getPath()[2].getPose().position.altitude) << "Incorrect robotic path: The third Viewpoint assigned to the first robot  is incorrect.";

    EXPECT_EQ(30, planner.getCoveragePaths()[1].getPath()[0].getPose().position.altitude) << "Incorrect robotic path: The first Viewpoint assigned to the second robot  is incorrect.";
    EXPECT_EQ(50, planner.getCoveragePaths()[1].getPath()[1].getPose().position.altitude) << "Incorrect robotic path: The second Viewpoint assigned to the second robot  is incorrect.";
    EXPECT_EQ(70, planner.getCoveragePaths()[1].getPath()[2].getPose().position.altitude) << "Incorrect robotic path: The third Viewpoint assigned to the second robot  is incorrect.";
}

