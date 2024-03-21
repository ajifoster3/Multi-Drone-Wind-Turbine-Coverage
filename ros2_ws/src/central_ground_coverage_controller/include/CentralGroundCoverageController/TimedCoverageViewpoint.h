#ifndef TIMEDCOVERAGEVIEWPOINTH
#define TIMEDCOVERAGEVIEWPOINTH

#include <rosgraph_msgs/msg/clock.hpp>
#include "CoverageViewpoint.h"

class TimedCoverageViewpoint : public CoverageViewpoint
{
public:
    TimedCoverageViewpoint(const CoverageViewpoint&);

    rosgraph_msgs::msg::Clock getCoverageTime() const;
    void setCoverageTime(rosgraph_msgs::msg::Clock);

    const bool getIsCovered() const;
    void setIsCovered(bool);
private:
    rosgraph_msgs::msg::Clock coverageTime;
    bool isCovered;
};

#endif
