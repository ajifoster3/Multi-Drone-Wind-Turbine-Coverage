#include "CoveragePaths.h"

CoveragePaths::CoveragePaths(std::vector<int> robotIds)
{
    initialiseCoveragePaths(robotIds);
};

Path CoveragePaths::getCoveragePathForRobot(int id)
{
    for(auto& path : paths)
    {
        if(path.getRobotId() == id)
        {
            return path;
        }
    }
    return Path();
};

void CoveragePaths::addCoverageViewpointForRobot(int id, CoverageViewpoint viewpoint)
{
    for(auto& path : paths)
    {
        if(path.getRobotId() == id)
        {
            path.addCoverageViewpoint(viewpoint);
        }
    }
};

void CoveragePaths::initialiseCoveragePaths(std::vector<int> robotIDs)
{
    for (int id : robotIDs)
    {
        paths.emplace_back(id);
    }
};
