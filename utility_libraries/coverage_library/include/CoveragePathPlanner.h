#ifndef COVERAGE_PATH_PLANNER_H
#define COVERAGE_PATH_PLANNER_H

#include <vector>
#include "CoverageViewpoint.h"
#include "CoveragePath.h"

class CoveragePathPlanner {
public:
    virtual ~CoveragePathPlanner() {}
    virtual std::vector<CoveragePath> getCoveragePaths() const = 0;

protected:
    std::vector<int> robotIDs;
    std::vector<Pose> robotPoses;
    std::vector<CoverageViewpoint> viewpoints;
    std::vector<CoveragePath> coveragePaths;

    // Protected function that can be shared across derived classes
    virtual int findClosestUnassignedViewpointIndex(const Pose& pose) = 0;
    virtual void planCoveragePath() = 0;
};

#endif // COVERAGE_PATH_PLANNER_H
