#ifndef GOAL_COVERAGE_VIEWPOINTS_H
#define GOAL_COVERAGE_VIEWPOINTS_H

#include <vector>
#include <algorithm>
#include "CoverageViewpoint.h"
#include "HaversineDistance.h"
#include "Pose.h"

class GoalCoverageViewpoints
{
public:
    GoalCoverageViewpoints() = default;
    GoalCoverageViewpoints(std::vector<CoverageViewpoint>);
    bool hasUnassignedViewpoints();
    int findClosestUnassignedViewpointIndex(const Pose &);
    CoverageViewpoint& getClosestUnassignedViewpoint(const Pose &pose);
    std::vector<Pose::Position> getViewpointPositions();
    CoverageViewpoint getViewpointAtIndex(int i);
private:
    std::vector<CoverageViewpoint> goalViewpoints_;
};

#endif
