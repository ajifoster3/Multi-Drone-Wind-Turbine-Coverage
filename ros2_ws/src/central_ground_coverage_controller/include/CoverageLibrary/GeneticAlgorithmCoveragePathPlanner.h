#ifndef GENETIC_ALGORITHM_COVERAGE_PATH_PLANNER_H
#define GENETIC_ALGORITHM_COVERAGE_PATH_PLANNER_H

#include <vector>
#include "ParthenoGeneticAlgorithm.h"
#include "ParthenoGeneticAlgorithmConfig.h"
#include "CoverageViewpoint.h"
#include "Pose.h"
#include "CoveragePathPlanner.h"
#include "CoveragePaths.h"

class GeneticAlgorithmCoveragePathPlanner : public CoveragePathPlanner {
public:
    GeneticAlgorithmCoveragePathPlanner(const std::vector<int> &robotIDs,
                                       const std::vector<Pose> &initialPoses,
                                       std::vector<CoverageViewpoint> &viewpoints);

private:
    void planCoveragePath() override;
    void AssignClosestViewpointToRobot(int i);
    std::vector<std::vector<int>> extractRoutes(const std::vector<int>& bestChromosome, int numberOfRobots);
};

#endif // GREEDY_ITERATIVE_COVERAGE_PATH_PLANNER_HPP
