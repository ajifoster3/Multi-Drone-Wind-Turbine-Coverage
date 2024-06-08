#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <GreedyIterativeCoveragePathPlanner.h>
#include <GeneticAlgorithmCoveragePathPlanner.h>
#include <CoverageViewpointLoader.h>
#include <Path.h>
#include <Pose.h>
#include <GeographicLib/Geoid.hpp>
#include "CentralCoverageControllerNode.h"
#include "TimedCoveragePath.h"
#include "LogDistanceCostMatrixPlanner.h"

std::unique_ptr<CoveragePathPlanner> createPlanner(const std::string_view approach, std::vector<int> &robotIds, std::vector<Pose> &poses, std::vector<CoverageViewpoint> &viewpoints)
{
    if (approach == "GreedyIterative")
    {
        return std::make_unique<GreedyIterativeCoveragePathPlanner>(robotIds, poses, viewpoints);
    }
    else if (approach == "GeneticAlgorithm")
    {
        return std::make_unique<GeneticAlgorithmCoveragePathPlanner>(robotIds, poses, viewpoints);
    }
    else if (approach == "LogDistance")
    {
        return std::make_unique<LogDistanceCostMatrixPlanner>(robotIds, poses, viewpoints);
    }
    else
    {
        throw std::invalid_argument("Unsupported coverage approach");
    }
}

int main(int argc, char **argv)
{

    GeographicLib::Geoid geoid("egm96-5");
    rclcpp::init(argc, argv);
    if (argc < 4)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: ros2 run [package_name] [executable_name] [team_size] [coverage_approach] [goal_pose_filename]");
        rclcpp::shutdown();
        return 1;
    }

    // Load arguments from the ros2 run call
    int team_size{std::stoi(argv[1])};
    std::string_view coverage_approach{argv[2]};
    std::string goal_pose_filename{argv[3]};

    // Setup the ros2 Node "central_coverage_controller_node"
    auto central_coverage_controller_node = std::make_shared<CentralCoverageControllerNode>("central_coverage_controller_node", team_size);

    // Create robotIds Vector with the robot IDs
    std::vector<int> robotIds(team_size);
    std::iota(robotIds.begin(), robotIds.end(), 0);

    // Load the inital poses of the robots from the companion computers via the central_control/uav_i/global_pose topic
    std::vector<Pose> poses{};

    // For each drone, get the initial position
    for (int i = 1; i <= team_size; i++)
    {
        geographic_msgs::msg::GeoPoseStamped geoMsg{};
        std::string uasNumber{std::to_string(i)};
        while (!rclcpp::wait_for_message(geoMsg, central_coverage_controller_node, "central_control/uas_" + uasNumber + "/global_pose", std::chrono::seconds(10)))
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "waiting for message from uas_%s", uasNumber.c_str());
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "recieved message from uas_%s", uasNumber.c_str());
        Pose pose{};
        pose.position.longitude = geoMsg.pose.position.longitude;
        pose.position.latitude = geoMsg.pose.position.latitude;
        double geoidHeight = geoid(
            pose.position.latitude,
            pose.position.longitude);
        pose.position.altitude = geoMsg.pose.position.altitude - geoidHeight;
        poses.push_back(pose);
    }

    CoverageLogger::setInitalPoses(poses);

    // Load the goal poses from the specified file
    // TODO: Make file name an argument
    std::vector<CoverageViewpoint> viewpoints{CoverageViewpointLoader::load(goal_pose_filename)};
    std::unique_ptr<CoveragePathPlanner> planner;
    // Compute robot path
    planner = createPlanner(coverage_approach, robotIds, poses, viewpoints);

    planner->logCoveragePath(planner->getCoveragePaths().getPaths());

    std::vector<TimedCoveragePath> coveragePaths;

    for (auto id : robotIds)
    {
        coveragePaths.push_back(TimedCoveragePath{planner->getCoveragePaths().getCoveragePathForRobot(id)});
    }

    // Give paths to the central_coverage_controller_node
    central_coverage_controller_node->setCoveragePaths(coveragePaths);

    //*****************************************************************************

    rosgraph_msgs::msg::Clock clockMsg{};
    while (!rclcpp::wait_for_message(clockMsg, central_coverage_controller_node, "clock", std::chrono::seconds(10)))
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "waiting for simulation time...");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "recieved simulation time: %d", clockMsg.clock.sec);
    CoverageLogger::setStartTime(clockMsg.clock);

    //*****************************************************************************

    rclcpp::spin(central_coverage_controller_node);
    CoverageLogger::logCoverage(coverage_approach);
    rclcpp::shutdown();
    return 0;
}
