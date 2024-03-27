#include "CoverageLogger.h"

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

const void CoverageLogger::logTimes()
{
	// Get the current time
	auto now = std::chrono::system_clock::now();
	auto now_c = std::chrono::system_clock::to_time_t(now);

	// Convert it to tm struct
	std::tm now_tm = *std::localtime(&now_c);

	// Create a stringstream to hold the file name
	std::stringstream ss;

	// Format the filename with the current date and time
	ss << "runlog/coverage_times_";
	ss << (now_tm.tm_year + 1900) << "-"
	   << (now_tm.tm_mon + 1) << "-"
	   << now_tm.tm_mday << "_"
	   << now_tm.tm_hour << "-"
	   << now_tm.tm_min << "-"
	   << now_tm.tm_sec << ".csv";

	// Open the file with the unique name
	std::ofstream outFile(ss.str());

	if (!outFile.is_open()) {
		std::cerr << "Failed to open file for logging.\n";
		return;
	}
	for (int i = 0; i < initalPoses.size(); i++) {
		outFile << "Robot_" << i << ",";
	}
	outFile << "\n";
	for (auto pose : initalPoses) {
		outFile << pose.position.latitude << "_" << pose.position.longitude
		        << "_" << pose.position.altitude << ",";
	}
	outFile << "\n\n";
	outFile << "Start Time," << startTime.sec << "." << startTime.nanosec << "\n\n";


	outFile << "Robot_ID, Latitude, Longitude, Altitude, Viewpoint Time\n";

	// Iterate through each TimedCoveragePath and its TimedCoverageViewpoints
	for (const auto & path : viewpointCoverageTimes) {
		const auto & viewpoints = path.getPath();
		for (const auto & viewpoint : viewpoints) {
			outFile << path.getRobotId() << ",";
			outFile << viewpoint.getPose().position.latitude << ",";
			outFile << viewpoint.getPose().position.longitude << ",";
			outFile << viewpoint.getPose().position.altitude << ",";
			outFile << viewpoint.getCoverageTime().clock.sec << "." <<
			        viewpoint.getCoverageTime().clock.nanosec << "\n";
		}
	}

	outFile.close();
}
