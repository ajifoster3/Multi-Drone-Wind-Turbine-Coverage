#include "Path.h"

void Path::addCoverageViewpoint(const CoverageViewpoint &viewpoint)
{
    path.push_back(viewpoint);
}

const std::vector<CoverageViewpoint> &Path::getPath() const
{
    return path;
}

const int Path::getRobotId() const
{
    return robotID;
}

void Path::setRobotId(int robotID)
{
    this->robotID = robotID;
}
