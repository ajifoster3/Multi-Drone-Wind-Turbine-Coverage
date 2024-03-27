#include "TimedCoverageViewpoint.h"

TimedCoverageViewpoint::TimedCoverageViewpoint(const CoverageViewpoint & viewpoint)
{
	this->setAssigned(viewpoint.isAssigned());
	this->setPose(viewpoint.getPose());
}

rosgraph_msgs::msg::Clock TimedCoverageViewpoint::getCoverageTime() const
{
	return coverageTime;
}

void TimedCoverageViewpoint::setCoverageTime(rosgraph_msgs::msg::Clock time)
{
	this->coverageTime = time;
}

const bool TimedCoverageViewpoint::getIsCovered() const
{
	return this->isCovered;
}

void TimedCoverageViewpoint::setIsCovered(bool isCovered)
{
	this->isCovered = isCovered;
}
