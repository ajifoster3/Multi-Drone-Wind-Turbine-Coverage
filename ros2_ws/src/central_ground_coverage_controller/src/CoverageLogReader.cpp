#include "CoverageLogReader.h"


namespace CoverageLogReader
{
    CoveragePaths deLogCoveragePath(const std::string &filePath)
    {
        using json = nlohmann::json;


        // Read the JSON file
        std::ifstream file(filePath);
        if (!file.is_open())
        {
            std::cerr << "Failed to open file for reading." << std::endl;
            return CoveragePaths(); // Return an empty CoveragePaths object
        }

        json j;
        try {
            file >> j;
        } catch (const json::parse_error &e) {
            std::cerr << "JSON parse error: " << e.what() << std::endl;
            file.close();
            return CoveragePaths();
        }
        file.close();

        // Extract unique robot IDs from the JSON to initialize CoveragePaths
        std::set<int> robotIds;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Filling robot IDs");
        if (j.contains("paths") && j["paths"].is_array())
        {

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "If check");
            for (const auto &pathJson : j["paths"])
            {

                if (pathJson.contains("robot_id") && pathJson["robot_id"].is_number_integer())
                {

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "If robotId is int");
                    int robotId = pathJson["robot_id"];
                    robotIds.insert(robotId);
                }
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot IDs Added");

        CoveragePaths coveragePaths(std::vector<int>(robotIds.begin(), robotIds.end()));

        // Add CoverageViewpoints to the appropriate paths
        if (j.contains("paths") && j["paths"].is_array())
        {
            for (const auto &pathJson : j["paths"])
            {
                if (!pathJson.contains("robot_id") || !pathJson["robot_id"].is_number_integer())
                {
                    continue;
                }

                int robotId = pathJson["robot_id"];

                if (pathJson.contains("viewpoints") && pathJson["viewpoints"].is_array())
                {
                    for (const auto &vpJson : pathJson["viewpoints"])
                    {
                        if (!vpJson.contains("position") || !vpJson["position"].is_object() ||
                            !vpJson.contains("orientation") || !vpJson["orientation"].is_object())
                        {
                            continue;
                        }

                        Pose pose;
                        pose.position.latitude = vpJson["position"].value("latitude", 0.0);
                        pose.position.longitude = vpJson["position"].value("longitude", 0.0);
                        pose.position.altitude = vpJson["position"].value("altitude", 0.0);
                        pose.orientation.x = vpJson["orientation"].value("x", 0.0);
                        pose.orientation.y = vpJson["orientation"].value("y", 0.0);
                        pose.orientation.z = vpJson["orientation"].value("z", 0.0);
                        pose.orientation.w = vpJson["orientation"].value("w", 1.0);

                        bool assigned = vpJson.value("assigned", false);
                        CoverageViewpoint viewpoint(pose, assigned);

                        coveragePaths.addCoverageViewpointForRobot(robotId, viewpoint);
                    }
                }
            }
        }

        return coveragePaths;
    }
}
