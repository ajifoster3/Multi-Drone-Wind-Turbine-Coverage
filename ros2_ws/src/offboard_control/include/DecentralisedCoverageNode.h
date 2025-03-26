#ifndef DECENTRALISEDCOVERAGENODE_H
#define DECENTRALISEDCOVERAGENODE_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include "offboard_control_interfaces/msg/drone_allocation.hpp"
#include "offboard_control_interfaces/msg/drone_environmental_representation.hpp"
#include "offboard_control_interfaces/msg/drone_ping.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <GeographicLib/Geoid.hpp>
#include "Pose.h"
#include "CoverageViewpointLoader.h"
#include "HaversineDistance.h"

class DecentralisedCoverageNode : public rclcpp::Node
{
public:
    DecentralisedCoverageNode(const std::string &name, int uas_number);

private:
    int uasNumber_;
    std::unordered_set<int> timedOutDrones_;
    bool isCoverageStarted_{false};
    int viewpointAssigned_;
    double goalPoseTolerance_ = 0.4;
    offboard_control_interfaces::msg::DroneAllocation droneAllocation_;
    offboard_control_interfaces::msg::DroneEnvironmentalRepresentation droneEnvironmentalRepresentation_;
    std::vector<CoverageViewpoint> coverageViewpoints_;
    geographic_msgs::msg::GeoPoseStamped currentGps_;
    bool isGPSSet_ = false;
    geographic_msgs::msg::GeoPoseStamped initialGps_;
    GeographicLib::Geoid geoid_;
    geographic_msgs::msg::GeoPoseStamped geoposeGoalGps_{};
    std::unordered_map<int, rclcpp::Time> lastHeardFrom_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool isKilled_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr decentralisedCoverageSub_;
    rclcpp::Subscription<offboard_control_interfaces::msg::DroneAllocation>::SharedPtr droneAllocationSub_;
    rclcpp::Subscription<offboard_control_interfaces::msg::DroneEnvironmentalRepresentation>::SharedPtr droneEnvironmentalRepresentationSub_;
    rclcpp::Subscription<offboard_control_interfaces::msg::DronePing>::SharedPtr dronePingSub_;
    rclcpp::Subscription<geographic_msgs::msg::GeoPoseStamped>::SharedPtr centralGlobalPosSub_;
    rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr centralGoalPosPub_;
    rclcpp::Publisher<offboard_control_interfaces::msg::DroneAllocation>::SharedPtr droneAllocationPub_;
    rclcpp::Publisher<offboard_control_interfaces::msg::DroneEnvironmentalRepresentation>::SharedPtr droneEnvironmentalRepresentationPub_;
    rclcpp::Publisher<offboard_control_interfaces::msg::DronePing>::SharedPtr dronePingPub_;
    rclcpp::TimerBase::SharedPtr decentralisedPubTimer_;
    rclcpp::TimerBase::SharedPtr checkDroneTimer_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shutdownSub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr killSub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reviveSub_;




    void startDecentralisedCoverageCb(const std_msgs::msg::Bool::SharedPtr msg);
    void droneAllocationCb(const offboard_control_interfaces::msg::DroneAllocation::SharedPtr msg);
    void droneEnvironmentalRepresentationCb(const offboard_control_interfaces::msg::DroneEnvironmentalRepresentation::SharedPtr msg);
    void dronePingCb(const offboard_control_interfaces::msg::DronePing::SharedPtr msg);
    void globalPositionCb(const geographic_msgs::msg::GeoPoseStamped msg);
    void initializeSubscribers();
    void initializePublishers();
    void decentralisedCoverageTimerCallback();
    void publishDroneAllocation();
    void publishDroneEnvironmentalRepresentation();
    void publishDronePing();
    int getClosestViewpointIndex();
    void allocateUnassignedViewpoint();
    void checkCoveragePositionReached();
    void coveragePoseToGeoPose(geographic_msgs::msg::GeoPoseStamped &geopose, Pose &pose);
    void checkDroneTimeouts();
    void removeTimedOutDroneAllocations(int drone_id);
    void shutdownCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void killCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void reviveCallback(const std_msgs::msg::Bool::SharedPtr msg);
};
#endif
