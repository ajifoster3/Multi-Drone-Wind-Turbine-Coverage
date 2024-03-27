#ifndef COVERAGELOGGERH
#define COVERAGELOGGERH
#include "builtin_interfaces/msg/time.h"
#include "TimedCoveragePath.h"
#include <fstream>
#include <iostream>
#include <chrono>

class CoverageLogger
{
public:
static void setStartTime(builtin_interfaces::msg::Time);

static void setInitalPoses(std::vector < Pose >);

static void setViewpointCoverageTimes(std::vector < TimedCoveragePath >);

const static void logTimes();

private:
static builtin_interfaces::msg::Time startTime;
static std::vector < Pose > initalPoses;
static std::vector < TimedCoveragePath > viewpointCoverageTimes;
};

#endif
