#include "GoalCoverageViewpoints.h"

GoalCoverageViewpoints::GoalCoverageViewpoints(std::vector<CoverageViewpoint> goalViewpoints)
    : goalViewpoints_(goalViewpoints){};

bool GoalCoverageViewpoints::hasUnassignedViewpoints()
{
    for (const auto viewpoint : goalViewpoints_)
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
    for (size_t i = 0; i < goalViewpoints_.size(); ++i)
    {
        if (!goalViewpoints_[i].isAssigned())
        {
            double distance = std::sqrt(
                std::pow(HaversineDistance::calculateDistance(
                             pose.position.latitude, pose.position.longitude,
                             goalViewpoints_[i].getPose().position.latitude, goalViewpoints_[i].getPose().position.longitude),
                         2) +
                std::pow(pose.position.altitude - goalViewpoints_[i].getPose().position.altitude, 2));
            if (distance < minDistance)
            {
                minDistance = distance;
                closestIndex = i;
            }
        }
    }
    return closestIndex;
}

CoverageViewpoint &GoalCoverageViewpoints::getClosestUnassignedViewpoint(const Pose &pose)
{
    CoverageViewpoint *closestViewpoint;
    double minDistance = std::numeric_limits<double>::max();
    for (size_t i = 0; i < goalViewpoints_.size(); ++i)
    {
        if (!goalViewpoints_[i].isAssigned())
        {
            double distance = std::sqrt(
                std::pow(HaversineDistance::calculateDistance(
                             pose.position.latitude, pose.position.longitude,
                             goalViewpoints_[i].getPose().position.latitude, goalViewpoints_[i].getPose().position.longitude),
                         2) +
                std::pow(pose.position.altitude - goalViewpoints_[i].getPose().position.altitude, 2));
            if (distance < minDistance)
            {
                closestViewpoint = &goalViewpoints_[i];
                minDistance = distance;
            }
        }
    }
    return *closestViewpoint;
}

std::vector<Pose::Position> GoalCoverageViewpoints::getViewpointPositions()
{
    std::vector<Pose::Position> positions;
    positions.resize(goalViewpoints_.size());
    std::transform(
        goalViewpoints_.begin(),
        goalViewpoints_.end(),
        positions.begin(),
        [](const CoverageViewpoint &viewpoint) -> Pose::Position
        { return Pose::Position{
              viewpoint.getPose().position.latitude,
              viewpoint.getPose().position.longitude,
              viewpoint.getPose().position.altitude}; });
    return positions;
}

CoverageViewpoint GoalCoverageViewpoints::getViewpointAtIndex(int i)
{
    return goalViewpoints_[i];
}
