#ifndef TIMEDCOVERAGEPATHH
#define TIMEDCOVERAGEPATHH

#include <Path.h>
#include <optional>
#include "TimedCoverageViewpoint.h"

class TimedCoveragePath : public Path
{
public:
    TimedCoveragePath(const Path &path);

    std::optional<TimedCoverageViewpoint> getFirstZeroCoverageTimeViewpointTime();
    void setFirstZeroCoverageTimeViewpointTime(rosgraph_msgs::msg::Clock);
    const std::vector<TimedCoverageViewpoint> &getPath() const;


private:
    std::vector<TimedCoverageViewpoint> path;
};

#endif
