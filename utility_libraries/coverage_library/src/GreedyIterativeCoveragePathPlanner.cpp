#include "GreedyIterativeCoveragePathPlanner.h"

GreedyIterativeCoveragePathPlanner::GreedyIterativeCoveragePathPlanner(const std::vector<int> &robotIDs, const std::vector<Pose> &initialPoses, std::vector<CoverageViewpoint> &viewpoints)
    : robotIDs(robotIDs), robotPoses(initialPoses), viewpoints(viewpoints)
{
    // Initialize a CoveragePath for each robot
    for (int id : robotIDs)
    {
        coveragePaths.emplace_back(id);
    }
    planCoveragePath();
}

void GreedyIterativeCoveragePathPlanner::planCoveragePath()
{
    bool allAssigned = false;
    while (!allAssigned)
    {
        allAssigned = true;
        for (size_t i = 0; i < robotIDs.size(); ++i)
        {
            int closestIndex = findClosestUnassignedViewpointIndex(robotPoses[i]);
            if (closestIndex != -1)
            {
                viewpoints[closestIndex].setAssigned(true);
                viewpoints[closestIndex].setRobotIDAssigned(robotIDs[i]); // Set the robot ID for the assigned viewpoint

                robotPoses[i] = viewpoints[closestIndex].getPose(); // Update robot's pose

                // Add the assigned viewpoint to the corresponding CoveragePath
                coveragePaths[i].addCoverageViewpoint(viewpoints[closestIndex]);

                allAssigned = false;
            }
        }
    }
}

std::vector<CoveragePath> GreedyIterativeCoveragePathPlanner::getCoveragePaths() const
{
    return coveragePaths;
}

int GreedyIterativeCoveragePathPlanner::findClosestUnassignedViewpointIndex(const Pose &pose)
{
    double minDistance = std::numeric_limits<double>::max();
    int closestIndex = -1;
    for (size_t i = 0; i < viewpoints.size(); ++i)
    {
        if (!viewpoints[i].isAssigned())
        {
            double distance = std::sqrt(
                std::pow(HaversineDistance::calculateDistance(
                             pose.position.latitude, pose.position.longitude,
                             viewpoints[i].getPose().position.latitude, viewpoints[i].getPose().position.longitude), 2) +
                std::pow(pose.position.altitude - viewpoints[i].getPose().position.altitude, 2));
            if (distance < minDistance)
            {
                minDistance = distance;
                closestIndex = i;
            }
        }
    }
    return closestIndex;
}
