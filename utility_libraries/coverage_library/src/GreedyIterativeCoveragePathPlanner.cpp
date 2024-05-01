#include "GreedyIterativeCoveragePathPlanner.h"

GreedyIterativeCoveragePathPlanner::GreedyIterativeCoveragePathPlanner(
    const std::vector<int> &robotIDs,
    const std::vector<Pose> &initialPoses,
    std::vector<CoverageViewpoint> &viewpoints)
    : CoveragePathPlanner()
{
    this->robotIDs = robotIDs;
    this->initialRobotPoses = initialPoses;
    this->robotPoses = initialPoses;
    this->viewpoints = GoalCoverageViewpoints(viewpoints);
    this->coveragePaths = CoveragePaths(robotIDs);

    planCoveragePath();
}

void GreedyIterativeCoveragePathPlanner::planCoveragePath()
{
    while (viewpoints.hasUnassignedViewpoints())
    {
        for (size_t i = 0; i < robotIDs.size(); ++i)
        {
            AssignClosestViewpointToRobot(i);
        }
    }
    for(size_t i = 0; i < robotPoses.size(); i++)
    {
        coveragePaths.addCoverageViewpointForRobot(i, CoverageViewpoint(initialRobotPoses[i], true));
    }
}

void GreedyIterativeCoveragePathPlanner::AssignClosestViewpointToRobot(int robotId)
{
    auto& viewpoint = viewpoints.getClosestUnassignedViewpoint(robotPoses[robotId]);
    robotPoses[robotId] = viewpoint.getPose(); 
    coveragePaths.addCoverageViewpointForRobot(robotId, viewpoint);
    viewpoint.setAssigned(true); // setting assigned
}
