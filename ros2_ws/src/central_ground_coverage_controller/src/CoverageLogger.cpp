#include "CoverageLogger.h"

namespace fs = std::filesystem;

builtin_interfaces::msg::Time CoverageLogger::startTime;
std::vector<Pose> CoverageLogger::initalPoses;
std::vector<TimedCoveragePath> CoverageLogger::viewpointCoverageTimes;
std::vector<std::vector<std::pair<rosgraph_msgs::msg::Clock, geographic_msgs::msg::GeoPose>>> CoverageLogger::dronePositions_;

void CoverageLogger::setStartTime(builtin_interfaces::msg::Time time)
{
    startTime = time;
}

void CoverageLogger::setInitalPoses(std::vector<Pose> poses)
{
    initalPoses = poses;
}

void CoverageLogger::setViewpointCoverageTimes(std::vector<TimedCoveragePath> paths)
{
    viewpointCoverageTimes = paths;
}

void CoverageLogger::setDronePositions(std::vector<std::vector<std::pair<rosgraph_msgs::msg::Clock, geographic_msgs::msg::GeoPose>>> dronePositions)
{
    dronePositions_ = dronePositions;
}

void CoverageLogger::logCoverage(std::string_view approach)
{
    // Find the next available folder name
    int run_number = 1;
    std::string folder_name;
    do {
        folder_name = "runlog/coverage_run_centralised_" + std::string(approach) + "_" + std::to_string(run_number);
        run_number++;
    } while (fs::exists(folder_name));

    // Create the directory
    fs::create_directory(folder_name);

    // Log coverage times
    for (const auto& path : viewpointCoverageTimes) {
        int robotId = path.getRobotId();

        // Create a stringstream to hold the file name
        std::stringstream ss;

        // Format the filename with the robot ID
        ss << folder_name << "/coverage_times_robot_" << robotId << ".csv";

        // Open the file with the unique name
        std::ofstream outFile(ss.str());

        if (!outFile.is_open()) {
            continue;
        }

        outFile << "Robot_ID,Latitude,Longitude,Altitude,Viewpoint Time\n";

        // Log initial position and start time for the robot
        const auto& initialPos = initalPoses[robotId];
        outFile << robotId << ","
                << std::setprecision(12) << initialPos.position.latitude << ","
                << initialPos.position.longitude << ","
                << initialPos.position.altitude << ","
                << "Start Time," << startTime.sec << "." << startTime.nanosec << "\n";

        const auto& viewpoints = path.getPath();
        for (const auto& viewpoint : viewpoints) {
            outFile << robotId << ","
                    << std::setprecision(12) << viewpoint.getPose().position.latitude << ","
                    << viewpoint.getPose().position.longitude << ","
                    << viewpoint.getPose().position.altitude << ","
                    << viewpoint.getCoverageTime().clock.sec << "." << viewpoint.getCoverageTime().clock.nanosec << "\n";
        }

        outFile.close();
    }

    // Log drone positions
    std::ofstream positionFile(folder_name + "/drone_positions.csv");
    if (positionFile.is_open())
    {
        positionFile << "Drone,Time,Latitude,Longitude,Altitude\n";
        for (size_t i = 0; i < dronePositions_.size(); ++i)
        {
            for (const auto &position : dronePositions_[i])
            {
                positionFile << std::setprecision(12) << i + 1 << ","
                             << position.first.clock.sec << "." << position.first.clock.nanosec << ","
                             << position.second.position.latitude << ","
                             << position.second.position.longitude << ","
                             << position.second.position.altitude << "\n";
            }
        }
        positionFile.close();
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("CoverageLogger"), "Unable to open drone positions file for logging");
    }
}
