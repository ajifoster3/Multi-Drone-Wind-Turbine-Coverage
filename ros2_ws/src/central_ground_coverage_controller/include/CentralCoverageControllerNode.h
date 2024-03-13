#ifndef CENTRALCOVERAGECONTROLLERNODE_H
#define CENTRALCOVERAGECONTROLLERNODE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <GreedyIterativeCoveragePathPlanner.h>
#include <CoveragePath.h>
#include <HaversineDistance.h>
#include <memory>
#include <GeographicLib/Geoid.hpp>

using namespace std::chrono_literals;

class CentralCoverageControllerNode : public rclcpp::Node
{
public:
    CentralCoverageControllerNode(const std::string &name, int team_size);

    void setCoveragePaths(std::vector<CoveragePath> &);

private:
    int teamSize_{};
    std::vector<CoveragePath> coveragePaths_{};
    std::vector<rclcpp::Subscription<geographic_msgs::msg::GeoPoseStamped>::SharedPtr> centralGlobalPosSubs_;
    std::vector<rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr> centralGlobalGoalPosPubs_;
    std::vector<geographic_msgs::msg::GeoPoseStamped> goalGpsPositions_;
    std::vector<geographic_msgs::msg::GeoPoseStamped> currentGpsPositions_;
    rclcpp::TimerBase::SharedPtr timer_;

    void globalPositionCb(const geographic_msgs::msg::GeoPoseStamped::SharedPtr msg, size_t uasId);

    void timerCallback();

    void coveragePoseToGeoPose(geographic_msgs::msg::GeoPoseStamped &geopose, Pose &pose);

    void initializeSubscribers();

    void initializePublishers();

    void initializeClients(){};
};

#endif
