#include "CoverageViewpoint.h"
#include "json.hpp"
#include <string>
#include <vector>
#include <fstream>
#include <iostream>

namespace CoverageViewpointLoader
{
    using json = nlohmann::json;
    
    std::vector<CoverageViewpoint> load(const std::string& filePath)
    {
        std::vector<CoverageViewpoint> viewpoints;
        std::ifstream file(filePath);
        if (!file.is_open())
        {
            std::cerr << "Failed to open file: " << filePath << std::endl;
            return viewpoints;
        }

        json j;
        file >> j;
        for (const auto &item : j)
        {
            auto &geoPose = item["GeoPose"];
            Pose pose;
            pose.position.latitude = geoPose["GeoPoint position"]["latitude"];
            pose.position.longitude = geoPose["GeoPoint position"]["longitude"];
            pose.position.altitude = geoPose["GeoPoint position"]["altitude"];
            pose.orientation.x = geoPose["geometry_msgs/Quaternion orientation"]["x"];
            pose.orientation.y = geoPose["geometry_msgs/Quaternion orientation"]["y"];
            pose.orientation.z = geoPose["geometry_msgs/Quaternion orientation"]["z"];
            pose.orientation.w = geoPose["geometry_msgs/Quaternion orientation"]["w"];
            // Assuming default coverageTime and assigned values as placeholders
            double coverageTime = 0; // Placeholder value
            bool assigned = false;   // Placeholder value
            viewpoints.emplace_back(pose, coverageTime, assigned);
        }

        return viewpoints;
    }
} // namespace CoverageViewpointLoader
