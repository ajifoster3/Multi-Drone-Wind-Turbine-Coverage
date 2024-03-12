#include "CentralCoverageControllerNode.h"

CentralCoverageControllerNode::CentralCoverageControllerNode(const std::string &name, int team_size)
    : Node(name), teamSize_(team_size)
{
    initializeSubscribers();
    initializePublishers();
    initializeClients();
    timer_ = this->create_wall_timer(
        500ms, std::bind(&CentralCoverageControllerNode::timer_callback, this));
}

void CentralCoverageControllerNode::setCoveragePaths(std::vector<CoveragePath> &coveragePaths)
{
    this->coveragePaths_ = coveragePaths;
}

void CentralCoverageControllerNode::globalPositionCb(const geographic_msgs::msg::GeoPoseStamped::SharedPtr msg, size_t uas_id)
{
    // Ensure the vector is large enough to hold the GPS position for the given UAS ID
    if (uas_id < currentGpsPositions_.size())
    {
        currentGpsPositions_[uas_id] = *msg;
    }
}

void CentralCoverageControllerNode::timer_callback()
{
    for (int i = 0; i < teamSize_; ++i)
    {
        auto pub = centralGlobalGoalPosPubs_[i];
        std::optional<CoverageViewpoint> nextViewPoint = coveragePaths_[i].getFirstZeroCoverageTimeViewpoint();
        if (nextViewPoint)
        {
            CoverageViewpoint viewpoint = nextViewPoint.value();
            Pose pose = viewpoint.getPose();
            geographic_msgs::msg::GeoPoseStamped geopose;
            coveragePoseToGeoPose(geopose, pose);
            geopose.header.stamp = this->now();
            pub->publish(geopose);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing geopose");
        }
    }
}

void CentralCoverageControllerNode::coveragePoseToGeoPose(geographic_msgs::msg::GeoPoseStamped &geopose, Pose &pose)
{
    geopose.pose.position.altitude = pose.position.altitude;
    geopose.pose.position.latitude = pose.position.latitude;
    geopose.pose.position.longitude = pose.position.longitude;
    geopose.pose.orientation.x = pose.orientation.x;
    geopose.pose.orientation.y = pose.orientation.y;
    geopose.pose.orientation.z = pose.orientation.z;
    geopose.pose.orientation.w = pose.orientation.w;
}

void CentralCoverageControllerNode::initializeSubscribers()
{
    currentGpsPositions_.resize(teamSize_); // Resize the vector to hold GPS positions for all team members

    for (int i = 0; i < teamSize_; ++i)
    {
        std::string topic_name = "central_control/uas_" + std::to_string(i + 1) + "/global_pose";
        centralGlobalPosSubs_.push_back(
            this->create_subscription<geographic_msgs::msg::GeoPoseStamped>(
                topic_name, rclcpp::SensorDataQoS(),
                [this, i](const geographic_msgs::msg::GeoPoseStamped::SharedPtr msg)
                {
                    globalPositionCb(msg, i);
                }));
    }
}

void CentralCoverageControllerNode::initializePublishers()
{
    goalGpsPositions_.resize(teamSize_); // Resize the vector to hold GPS positions for all team members

    for (int i = 0; i < teamSize_; ++i)
    {
        std::string topic_name = "central_control/uas_" + std::to_string(i + 1) + "/goal_pose";
        centralGlobalGoalPosPubs_.push_back(
            this->create_publisher<geographic_msgs::msg::GeoPoseStamped>(
                topic_name, rclcpp::SensorDataQoS()));
    }
}
