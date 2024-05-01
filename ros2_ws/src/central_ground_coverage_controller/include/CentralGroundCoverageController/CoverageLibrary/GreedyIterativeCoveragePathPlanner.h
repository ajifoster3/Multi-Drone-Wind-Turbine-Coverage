#ifndef GREEDY_ITERATIVE_COVERAGE_PATH_PLANNER_H
#define GREEDY_ITERATIVE_COVERAGE_PATH_PLANNER_H

#include <vector>
#include "CoverageViewpoint.h"
#include "Pose.h"
#include "CoveragePathPlanner.h"
#include "CoveragePaths.h"

class GreedyIterativeCoveragePathPlanner : public CoveragePathPlanner {
public:
    GreedyIterativeCoveragePathPlanner(const std::vector<int> &robotIDs,
                                       const std::vector<Pose> &initialPoses,
                                       std::vector<CoverageViewpoint> &viewpoints);

private:
    void planCoveragePath() override;
    void AssignClosestViewpointToRobot(int i);
};

#endif // GREEDY_ITERATIVE_COVERAGE_PATH_PLANNER_HPP
