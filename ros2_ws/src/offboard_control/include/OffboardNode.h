#ifndef OFFBOARDNODE_H
#define OFFBOARDNODE_H

#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <GeographicLib/Geoid.hpp>
#include "offboard_control_interfaces/msg/drone_allocation.hpp"
#include "offboard_control_interfaces/msg/drone_environmental_representation.hpp"
#include "offboard_control_interfaces/msg/drone_ping.hpp"
#include "CoverageModes.h"
#include "Pose.h"
#include "CoverageViewpointLoader.h"
#include "HaversineDistance.h"

#define FLIGHT_ALTITUDE -50.0f

class OffboardNode : public rclcpp::Node
{
public:
    OffboardNode(const std::string &name, int uas_number, CoverageMode);

    void OffboardNodeSetup();

private:
    CoverageMode coverageMode_;
    int uasNumber_;
    mavros_msgs::msg::State currentState_;
    bool isGpsSet_{false};
    bool isCoverageStarted_{false};
    int viewpointAssigned_;
    double goalPoseTolerance_ = 0.4;
    offboard_control_interfaces::msg::DroneAllocation droneAllocation_;
    offboard_control_interfaces::msg::DroneEnvironmentalRepresentation droneEnvironmentalRepresentation_;
    std::vector<CoverageViewpoint> coverageViewpoints_;
    sensor_msgs::msg::NavSatFix initialGps_;
    sensor_msgs::msg::NavSatFix currentGps_;
    sensor_msgs::msg::NavSatFix goalGps_;
    GeographicLib::Geoid geoid_;
    geographic_msgs::msg::GeoPoseStamped geoposeGoalGps_{};
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr stateSub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr globalPosSub_;
    rclcpp::Subscription<geographic_msgs::msg::GeoPoseStamped>::SharedPtr centralGoalPosSub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr decentralisedCoverageSub_;
    rclcpp::Subscription<offboard_control_interfaces::msg::DroneAllocation>::SharedPtr droneAllocationSub_;
    rclcpp::Subscription<offboard_control_interfaces::msg::DroneEnvironmentalRepresentation>::SharedPtr droneEnvironmentalRepresentationSub_;
    rclcpp::Subscription<offboard_control_interfaces::msg::DronePing>::SharedPtr dronePingSub_;
    rclcpp::TimerBase::SharedPtr pubTimer_;
    rclcpp::TimerBase::SharedPtr decentralisedPubTimer_;
    rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr globalPosPub_;
    rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr centralGlobalPosPub_;
    rclcpp::Publisher<offboard_control_interfaces::msg::DroneAllocation>::SharedPtr droneAllocationPub_;
    rclcpp::Publisher<offboard_control_interfaces::msg::DroneEnvironmentalRepresentation>::SharedPtr droneEnvironmentalRepresentationPub_;
    rclcpp::Publisher<offboard_control_interfaces::msg::DronePing>::SharedPtr dronePingPub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr armingClient_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr setModeClient_;

    void stateCb(const mavros_msgs::msg::State::SharedPtr msg) { currentState_ = *msg; }

    void globalPositionCb(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    void globalGoalPositionCb(const geographic_msgs::msg::GeoPoseStamped::SharedPtr msg);

    //void startDecentralisedCoverageCb(const std_msgs::msg::Bool::SharedPtr msg);

    //void droneAllocationCb(const offboard_control_interfaces::msg::DroneAllocation::SharedPtr msg);

    //void droneEnvironmentalRepresentationCb(const offboard_control_interfaces::msg::DroneEnvironmentalRepresentation::SharedPtr msg);

    void dronePingCb(const offboard_control_interfaces::msg::DronePing::SharedPtr msg);

    void initializeSubscribers();

    //void initializeDecentralisedSubscribers();

    void initializePublishers();

    //void initializeDecentralisedPublishers();

    void positionTimerCallback();

    //void decentralisedCoverageTimerCallback();

    void initializeClients();

    void waitForConnection();

    void setOffboardMode();

    void publishGeoPose();

    //void publishDroneAllocation();

    //void publishDroneEnvironmentalRepresentation();

    //void publishDronePing();

    void publishTargetPose();

    void armDrone();

    geographic_msgs::msg::GeoPoseStamped getCurrentGeoPose();

    //int getClosestViewpointIndex();

    //void allocateUnassignedViewpoint();
    
    void checkCoveragePositionReached();
    
    void coveragePoseToGeoPose(geographic_msgs::msg::GeoPoseStamped &geopose, Pose &pose);
};

#endif
