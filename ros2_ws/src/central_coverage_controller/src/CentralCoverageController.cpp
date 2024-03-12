#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <GeographicLib/Geoid.hpp>
#include <GreedyIterativeCoveragePathPlanner.h>
#include <CoverageViewpointLoader.h>
#include <CoveragePath.h>
#include <Pose.h>
#include "CentralCoverageControllerNode.h"

/* -*- GograpticLib utils -*- */

std::shared_ptr<GeographicLib::Geoid> egm96_5;

template <class T>
inline double geoid_to_ellipsoid_height(T lla)
{
    if (egm96_5)
        return GeographicLib::Geoid::GEOIDTOELLIPSOID * (*egm96_5)(lla->latitude, lla->longitude);
    else
        return 0.0;
}

template <class T>
inline double ellipsoid_to_geoid_height(T lla)
{
    if (egm96_5)
        return GeographicLib::Geoid::ELLIPSOIDTOGEOID * (*egm96_5)(lla->latitude, lla->longitude);
    else
        return 0.0;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    if (argc < 2)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: ros2 run [package_name] [executable_name] [team_size]");
        rclcpp::shutdown();
        return 1;
    }

    // Load arguments from the ros2 run call
    int team_size{std::stoi(argv[1])};
    std::string goalPoseFileName{"/home/ajifoster3/Downloads/wind_turbine_relative_positions.json"};

    // Setup the ros2 Node "central_coverage_controller_node"
    auto central_coverage_controller_node = std::make_shared<CentralCoverageControllerNode>("central_coverage_controller_node", team_size);

    // Create robotIds Vector with the robot IDs
    std::vector<int> robotIds(team_size);
    std::iota(robotIds.begin(), robotIds.end(), 1);

    // Load the inital poses of the robots from the companion computers via the central_control/uav_i/global_pose topic
    std::vector<Pose> poses{};

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
        pose.position.altitude = geoMsg.pose.position.altitude;
        poses.push_back(pose);
    }

    // Load the goal poses from the specified file * TODO: Make file name an argument *
    std::vector<CoverageViewpoint> viewpoints{CoverageViewpointLoader::load(goalPoseFileName)};

    // Compute robot path
    GreedyIterativeCoveragePathPlanner planner{robotIds, poses, viewpoints};
    std::vector<CoveragePath> coveragePaths = planner.getCoveragePaths();

    // Give paths to the central_coverage_controller_node
    central_coverage_controller_node->setCoveragePaths(coveragePaths);

    rclcpp::spin(central_coverage_controller_node);

    rclcpp::shutdown();
    return 0;
}
