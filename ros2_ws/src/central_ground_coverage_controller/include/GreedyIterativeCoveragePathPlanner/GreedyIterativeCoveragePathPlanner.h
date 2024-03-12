#ifndef GREEDY_ITERATIVE_COVERAGE_PATH_PLANNER_H
#define GREEDY_ITERATIVE_COVERAGE_PATH_PLANNER_H

#include <vector>
#include "CoverageViewpoint.h"
#include "CoveragePath.h" // Include the CoveragePath header
#include "HaversineDistance.h"

class GreedyIterativeCoveragePathPlanner {
public:
    GreedyIterativeCoveragePathPlanner(const std::vector<int>& robotIDs, const std::vector<Pose>& initialPoses, std::vector<CoverageViewpoint>& viewpoints);

    

    // New method to get the paths for each robot
    std::vector<CoveragePath> getCoveragePaths() const;

private:
    std::vector<int> robotIDs;
    std::vector<Pose> robotPoses;
    std::vector<CoverageViewpoint>& viewpoints;
    std::vector<CoveragePath> coveragePaths; // Store a CoveragePath for each robot

    void planCoveragePath();
    int findClosestUnassignedViewpointIndex(const Pose& pose);
};

#endif // GREEDY_ITERATIVE_COVERAGE_PATH_PLANNER_HPP
