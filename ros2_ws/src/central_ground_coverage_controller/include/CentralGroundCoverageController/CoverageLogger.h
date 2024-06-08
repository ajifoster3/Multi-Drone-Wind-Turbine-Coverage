#ifndef COVERAGELOGGERH
#define COVERAGELOGGERH

#include <rclcpp/rclcpp.hpp>
#include "builtin_interfaces/msg/time.h"
#include "TimedCoveragePath.h"
#include <fstream>
#include <iostream>
#include <chrono>
#include <filesystem>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <iomanip>

class CoverageLogger
{
public:
    static void setStartTime(builtin_interfaces::msg::Time);
    static void setInitalPoses(std::vector<Pose>);
    static void setViewpointCoverageTimes(std::vector<TimedCoveragePath>);
    static void setDronePositions(std::vector<std::vector<std::pair<rosgraph_msgs::msg::Clock, geographic_msgs::msg::GeoPose>>>);
    static void logCoverage(std::string_view);

private:
    static builtin_interfaces::msg::Time startTime;
    static std::vector<Pose> initalPoses;
    static std::vector<TimedCoveragePath> viewpointCoverageTimes;
    static std::vector<std::vector<std::pair<rosgraph_msgs::msg::Clock, geographic_msgs::msg::GeoPose>>> dronePositions_;
};

#endif // COVERAGELOGGERH
