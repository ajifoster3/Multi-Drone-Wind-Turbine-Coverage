#ifndef DecentralisedCoverageTracker__TrackingNode_HPP_
#define DecentralisedCoverageTracker__TrackingNode_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <fstream>
#include <filesystem>
#include "HaversineDistance.h"
#include "offboard_control_interfaces/msg/drone_environmental_representation.hpp"
#include "offboard_control_interfaces/msg/drone_allocation.hpp"

namespace trackingNode
{
    class TrackingNode : public rclcpp::Node
    {
    public:
        TrackingNode(int teamsize);

    private:
        int teamsize_;
        rosgraph_msgs::msg::Clock currentTime;
        rosgraph_msgs::msg::Clock coverageStartTime;
        rosgraph_msgs::msg::Clock coverageEndTime;
        rclcpp::Subscription<offboard_control_interfaces::msg::DroneEnvironmentalRepresentation>::SharedPtr droneEnvironmentalRepresentationSub_;
        rclcpp::Subscription<offboard_control_interfaces::msg::DroneAllocation>::SharedPtr droneAllocationSub_;
        std::vector<rclcpp::Subscription<geographic_msgs::msg::GeoPoseStamped>::SharedPtr> dronePositionSub_;
        rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clockSub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr startDecentralisedCoverageSub_;
        std::vector<std::vector<std::pair<rosgraph_msgs::msg::Clock, geographic_msgs::msg::GeoPose>>> dronePositions_;
        std::vector<std::pair<rosgraph_msgs::msg::Clock, offboard_control_interfaces::msg::DroneAllocation>> droneAllocations_;
        std::vector<std::pair<rosgraph_msgs::msg::Clock, offboard_control_interfaces::msg::DroneEnvironmentalRepresentation>> droneEnvironmentalRepresentation_;

        void initialiseStartDecentralisedCoverageSubscriber();
        void initialiseCoverageSubscribers();
        void initialiseDronePositionSubscribers();
        void droneEnvironmentalRepresentationSubCB(const offboard_control_interfaces::msg::DroneEnvironmentalRepresentation::SharedPtr msg);
        void droneAllocationSubCB(const offboard_control_interfaces::msg::DroneAllocation::SharedPtr msg);
        void dronePositionSubCB(const geographic_msgs::msg::GeoPoseStamped::SharedPtr msg, int robotId);
        void startDecentralisedCoverageSubCB(const std_msgs::msg::Bool::SharedPtr msg);
        void clockSubCB(const rosgraph_msgs::msg::Clock::SharedPtr msg);
        void logCoverage();
    };
} // namespace TrackingNode
#endif // DecentralisedCoverageTracker__TrackingNode_HPP_
