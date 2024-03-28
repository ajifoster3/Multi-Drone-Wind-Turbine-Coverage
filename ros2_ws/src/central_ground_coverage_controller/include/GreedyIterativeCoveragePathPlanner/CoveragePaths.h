#ifndef COVERAGE_PATHS_H
#define COVERAGE_PATHS_H

#include "Path.h"
#include <optional>

class CoveragePaths
{
public:
    CoveragePaths() = default;
    CoveragePaths(std::vector<int>);
    
    Path getCoveragePathForRobot(int id); // maybe unnecessary?
    void addCoverageViewpointForRobot(int robotId, CoverageViewpoint);

private:
    std::vector<Path> paths;
    void initialiseCoveragePaths(std::vector<int>);
};

#endif
