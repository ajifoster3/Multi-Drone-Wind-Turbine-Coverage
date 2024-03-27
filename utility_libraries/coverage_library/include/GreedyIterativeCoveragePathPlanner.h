#ifndef GREEDY_ITERATIVE_COVERAGE_PATH_PLANNER_H
#define GREEDY_ITERATIVE_COVERAGE_PATH_PLANNER_H

#include <vector>
#include <limits>
#include <cmath>
#include "CoverageViewpoint.h"
#include "CoveragePath.h" // Include the CoveragePath header
#include "HaversineDistance.h"
#include "CoveragePathPlanner.h"

class GreedyIterativeCoveragePathPlanner : public CoveragePathPlanner {
public:
    GreedyIterativeCoveragePathPlanner(const std::vector<int>& robotIDs, 
    const std::vector<Pose>& initialPoses, 
    std::vector<CoverageViewpoint>& viewpoints);

    // New method to get the paths for each robot
    std::vector<CoveragePath> getCoveragePaths() const override;

private:
    void planCoveragePath() override;
    int findClosestUnassignedViewpointIndex(const Pose& pose) override;
};

#endif // GREEDY_ITERATIVE_COVERAGE_PATH_PLANNER_HPP
