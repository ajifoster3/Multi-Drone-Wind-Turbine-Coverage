#include "CoveragePath.h"

void CoveragePath::addCoverageViewpoint(const CoverageViewpoint &viewpoint)
{
    path.push_back(viewpoint);
}

const std::vector<CoverageViewpoint> &CoveragePath::getPath() const
{
    return path;
}

const int CoveragePath::getRobotId() const
{
    return robotID;
}

void CoveragePath::setRobotId(int robotID)
{
    this->robotID = robotID;
}
