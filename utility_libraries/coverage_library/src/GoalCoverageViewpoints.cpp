#include "GoalCoverageViewpoints.h"

GoalCoverageViewpoints::GoalCoverageViewpoints(std::vector<CoverageViewpoint> goalViewpoints) 
: goalViewpoints(goalViewpoints){};

bool GoalCoverageViewpoints::hasUnassignedViewpoints()
{
    for (const auto viewpoint : goalViewpoints)
    {
        if (!viewpoint.isAssigned())
        {
            return true;
        }
    }
    return false;
};

int GoalCoverageViewpoints::findClosestUnassignedViewpointIndex(const Pose &pose)
{
    double minDistance = std::numeric_limits<double>::max();
    int closestIndex = -1;
    for (size_t i = 0; i < goalViewpoints.size(); ++i)
    {
        if (!goalViewpoints[i].isAssigned())
        {
            double distance = std::sqrt(
                std::pow(HaversineDistance::calculateDistance(
                             pose.position.latitude, pose.position.longitude,
                             goalViewpoints[i].getPose().position.latitude, goalViewpoints[i].getPose().position.longitude),
                         2) +
                std::pow(pose.position.altitude - goalViewpoints[i].getPose().position.altitude, 2));
            if (distance < minDistance)
            {
                minDistance = distance;
                closestIndex = i;
            }
        }
    }
    return closestIndex;
}

CoverageViewpoint& GoalCoverageViewpoints::getClosestUnassignedViewpoint(const Pose &pose)
{
    CoverageViewpoint* closestViewpoint;
    double minDistance = std::numeric_limits<double>::max();
    for (size_t i = 0; i < goalViewpoints.size(); ++i)
    {
        if (!goalViewpoints[i].isAssigned())
        {
            double distance = std::sqrt(
                std::pow(HaversineDistance::calculateDistance(
                             pose.position.latitude, pose.position.longitude,
                             goalViewpoints[i].getPose().position.latitude, goalViewpoints[i].getPose().position.longitude),
                         2) +
                std::pow(pose.position.altitude - goalViewpoints[i].getPose().position.altitude, 2));
            if (distance < minDistance)
            {
                closestViewpoint = &goalViewpoints[i];
                minDistance = distance;
            }
        }
    }
    return *closestViewpoint;
}
