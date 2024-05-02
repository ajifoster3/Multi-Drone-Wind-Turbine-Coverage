#ifndef COVERAGE_PATH_PLANNER_H
#define COVERAGE_PATH_PLANNER_H

#include <vector>
#include <json.hpp>
#include <fstream>
#include <iostream>
#include <typeinfo>
#include <regex>
#include <chrono>
#include <iomanip>
#include <filesystem>
#include "CoverageViewpoint.h"
#include "Path.h"
#include "GoalCoverageViewpoints.h"
#include "CoveragePaths.h"




class CoveragePathPlanner {
public:
    virtual ~CoveragePathPlanner() {}
    CoveragePaths getCoveragePaths() const;
    void logCoveragePath();

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
