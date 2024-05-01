#ifndef LOG_DISTANCE_COST_MATRIX_PLANNER_H
#define LOG_DISTANCE_COST_MATRIX_PLANNER_H

#include <iostream>
#include <vector>
#include "CoverageViewpoint.h"
#include "Pose.h"
#include "CoveragePathPlanner.h"
#include "CoveragePaths.h"

class LogDistanceCostMatrixPlanner : public CoveragePathPlanner {
public:
    LogDistanceCostMatrixPlanner(const std::vector<int> &robotIDs,
                                       const std::vector<Pose> &initialPoses,
                                       std::vector<CoverageViewpoint> &viewpoints);

private:
    void planCoveragePath() override;
    void printCostMap(const std::vector<std::vector<int>> &costMap);
};

#endif // LOG_DISTANCE_COST_MATRIX_PLANNER_H
