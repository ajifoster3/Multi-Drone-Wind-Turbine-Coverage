#ifndef COVERAGE_PATH_PLANNER_H
#define COVERAGE_PATH_PLANNER_H

#include <vector>
#include "CoverageViewpoint.h"
#include "Path.h"
#include "GoalCoverageViewpoints.h"
#include "CoveragePaths.h"
#include "CoveragePathPlannerLogger.h"

class CoveragePathPlanner {
public:
    virtual ~CoveragePathPlanner() {}
    CoveragePaths getCoveragePaths() const;
    void logCoveragePath(std::vector<Path> paths);

protected:
    std::vector<int> robotIDs;
    std::vector<Pose> initialRobotPoses;
    std::vector<Pose> robotPoses;
    GoalCoverageViewpoints viewpoints;
    CoveragePaths coveragePaths;

    // Protected function that can be shared across derived classes
    virtual void planCoveragePath() = 0;
};

#endif // COVERAGE_PATH_PLANNER_H
