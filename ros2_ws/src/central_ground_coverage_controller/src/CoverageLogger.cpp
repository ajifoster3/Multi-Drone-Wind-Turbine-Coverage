
#include "CoverageLogger.h"

namespace fs = std::filesystem;

builtin_interfaces::msg::Time CoverageLogger::startTime;
std::vector<Pose> CoverageLogger::initalPoses;
std::vector<TimedCoveragePath> CoverageLogger::viewpointCoverageTimes;

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

const void CoverageLogger::logTimes() {
    // Find the next available folder name
    int run_number = 1;
    std::string folder_name;
    do {
        folder_name = "runlog/centralised_coverage_run_" + std::to_string(run_number);
        run_number++;
    } while (fs::exists(folder_name));

    // Create the directory
    fs::create_directory(folder_name);

    // Iterate through each TimedCoveragePath and its TimedCoverageViewpoints
    for (const auto& path : viewpointCoverageTimes) {
        int robotId = path.getRobotId();

        // Create a stringstream to hold the file name
        std::stringstream ss;

        // Format the filename with the current date and time
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
}


