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
            pose.position.latitude = geoPose["position"]["latitude"];
            pose.position.longitude = geoPose["position"]["longitude"];
            pose.position.altitude = geoPose["position"]["altitude"];
            pose.orientation.x = geoPose["orientation"]["x"];
            pose.orientation.y = geoPose["orientation"]["y"];
            pose.orientation.z = geoPose["orientation"]["z"];
            pose.orientation.w = geoPose["orientation"]["w"];
            bool assigned = false;   // Placeholder value
            viewpoints.emplace_back(pose, assigned);
        }

        return viewpoints;
    }
} // namespace CoverageViewpointLoader
