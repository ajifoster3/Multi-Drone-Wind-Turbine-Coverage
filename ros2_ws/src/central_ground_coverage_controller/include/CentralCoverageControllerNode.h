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
    GeographicLib::Geoid geoid;
    std::vector<CoveragePath> coveragePaths_{};
    std::vector<rclcpp::Subscription<geographic_msgs::msg::GeoPoseStamped>::SharedPtr> centralGlobalPosSubs_;
    std::vector<rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr> centralGlobalGoalPosPubs_;
    std::vector<geographic_msgs::msg::GeoPoseStamped> goalGpsPositions_;
    std::vector<geographic_msgs::msg::GeoPoseStamped> currentGpsPositions_;
    rclcpp::TimerBase::SharedPtr timer_;

    /**
     *  Sets currentGpsPositions_ with altitude accounting for geoid height.
     */
    void CentralCoverageControllerNode::setCurrentGpsPosition(const geographic_msgs::msg::GeoPoseStamped& geopose, int uas_id);

    /**
     *  Sets currentGpsPositions_ to the recieved GeoPoseStamped values recieved
     *  from the central_control/uas_{i}/global_pose topic.
     */
    void globalPositionCb(const geographic_msgs::msg::GeoPoseStamped::SharedPtr msg, size_t uasId);

    /**
     *  Publishes a goal geopose for each drone, based on the next uncovered point.
     */
    void timerCallback();

    /**
     *  Allocates the members of the provided geopose with those of the pose.
     */
    void coveragePoseToGeoPose(geographic_msgs::msg::GeoPoseStamped &geopose, Pose &pose);

    /**
     *  Creates a global_pose subscriber, for each drone.
     */
    void initializeSubscribers();

    /**
     *  Create a goal_pose publisher, for each drone.
     */
    void initializePublishers();
};

#endif
