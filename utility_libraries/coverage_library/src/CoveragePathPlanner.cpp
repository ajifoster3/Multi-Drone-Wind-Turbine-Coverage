#include "CoveragePathPlanner.h"


CoveragePaths CoveragePathPlanner::getCoveragePaths() const { return coveragePaths; }

void CoveragePathPlanner::logCoveragePath(std::vector<Path> paths){
    std::string className = typeid(*this).name();
    CoveragePathPlannerLogger::logCoveragePath(this->getCoveragePaths().getPaths(), className);
};
