#include "CoveragePathPlanner.h"


CoveragePaths CoveragePathPlanner::getCoveragePaths() const { return coveragePaths; }

void CoveragePathPlanner::logCoveragePath(){
    using json = nlohmann::json;
    namespace fs = std::filesystem;

    json j;
    auto paths = this->getCoveragePaths().getCoveragePathForRobot();
    for (const auto& path : paths) {
        json pathJson;
        pathJson["robot_id"] = path.getRobotId();
        for (const auto& viewpoint : path.getPath()) {
            json vpJson;
            vpJson["position"] = { {"latitude", viewpoint.getPose().position.latitude},
                                   {"longitude", viewpoint.getPose().position.longitude},
                                   {"altitude", viewpoint.getPose().position.altitude} };
            vpJson["orientation"] = { {"x", viewpoint.getPose().orientation.x},
                                      {"y", viewpoint.getPose().orientation.y},
                                      {"z", viewpoint.getPose().orientation.z},
                                      {"w", viewpoint.getPose().orientation.w} };
            vpJson["assigned"] = viewpoint.isAssigned();
            pathJson["viewpoints"].push_back(vpJson);
        }
        j["paths"].push_back(pathJson);
    }

    fs::path dirPath = fs::current_path() / "pathlog";
    fs::create_directories(dirPath); 

    // Generate filename using the class name
    std::string className = typeid(*this).name();
    std::regex specialChars(R"([-[\]{}()*+?.,\^$|#\s])");
    className = std::regex_replace(className, specialChars, "_");
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
    std::string timeStr = ss.str();
    std::string filename = className + "_" + timeStr + "_coverage_paths.json";

    fs::path filePath = dirPath / filename;
    
    std::ofstream file(filePath);
    if (file.is_open()) {
        file << j.dump(4);
        file.close();
        std::cout << "Coverage paths successfully logged to 'coverage_paths.json'." << std::endl;
    } else {
        std::cerr << "Failed to open file for writing." << std::endl;
    }
};
