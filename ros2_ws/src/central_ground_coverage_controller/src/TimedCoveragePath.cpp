
#include "TimedCoveragePath.h"

TimedCoveragePath::TimedCoveragePath(const CoveragePath &path)
{
    this->setRobotId(path.getRobotId());
    for (auto viewpoint : path.getPath())
    {
        TimedCoverageViewpoint timedViewpoint{viewpoint};
        this->path.push_back(timedViewpoint);
    }
}

std::optional<TimedCoverageViewpoint> TimedCoveragePath::getFirstZeroCoverageTimeViewpointTime()
{
    for (const TimedCoverageViewpoint &viewpoint : this->getPath())
    {
        if (!viewpoint.getIsCovered())
        {
            return viewpoint; // Returns a viewpoint if found
        }
    }
    return std::nullopt; // Return an empty optional if no viewpoint is found
}

void TimedCoveragePath::setFirstZeroCoverageTimeViewpointTime(rosgraph_msgs::msg::Clock clock)
{
    for (auto &viewpoint : path)
    {
        if (!viewpoint.getIsCovered())
        {
            viewpoint.setCoverageTime(clock);
            viewpoint.setIsCovered(true);
            break;
        }
    }
}

const std::vector<TimedCoverageViewpoint> &TimedCoveragePath::getPath() const
{
    return this->path;
}
