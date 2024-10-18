#include "CoverageViewpointLoader.h"
#include <fstream>
#include <iostream>

namespace CoverageViewpointLoader
{
    std::vector<CoverageViewpoint> load(const std::string &filePath)
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
            std::chrono::seconds coverageTime = std::chrono::seconds(0); 
            bool assigned = false;                                       
            viewpoints.emplace_back(pose, assigned);
        }

        return viewpoints;
    }
} // namespace CoverageViewpointLoader
