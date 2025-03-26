#include "OffboardNode.h"

using namespace std::chrono_literals;

OffboardNode::OffboardNode(const std::string &name, int uas_number, CoverageMode mode)
    : Node("offboard_node_uas_" + std::to_string(uas_number)), coverageMode_(mode), uasNumber_(uas_number), geoid_("egm96-5")
{
    initializeSubscribers();
    initializePublishers();
    initializeClients();
}

void OffboardNode::OffboardNodeSetup()
{
    RCLCPP_INFO(this->get_logger(), "Connecting...");

    waitForConnection();

    RCLCPP_INFO(this->get_logger(), "Setting up offboard mode...");


    RCLCPP_INFO(this->get_logger(), "Publishing initial Pose...");
    // Publish target altitude to maintain OFFBOARD mode
    publishTargetPose();

}

void OffboardNode::globalPositionCb(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    if (!isGpsSet_)
    {
        initialGps_ = *msg;
        isGpsSet_ = true;
    }
    currentGps_ = *msg;
}

void OffboardNode::globalGoalPositionCb(const geographic_msgs::msg::GeoPoseStamped::SharedPtr msg)
{
    if (!isCoverageStarted_)
    {
        RCLCPP_INFO(this->get_logger(), "Switching to external control...");
        isCoverageStarted_ = true;
    }
    geoposeGoalGps_ = *msg;
}

void OffboardNode::dronePingCb(const offboard_control_interfaces::msg::DronePing::SharedPtr msg)
{
}

void OffboardNode::initializeSubscribers()
{
    stateSub_ = this->create_subscription<mavros_msgs::msg::State>(
        "mavros/uas_" + std::to_string(uasNumber_) + "/state", 10, std::bind(&OffboardNode::stateCb, this, std::placeholders::_1));
    globalPosSub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "mavros/uas_" + std::to_string(uasNumber_) + "/global_position/global", rclcpp::SensorDataQoS(), std::bind(&OffboardNode::globalPositionCb, this, std::placeholders::_1));
    centralGoalPosSub_ = this->create_subscription<geographic_msgs::msg::GeoPoseStamped>(
        "central_control/uas_" + std::to_string(uasNumber_) + "/goal_pose", rclcpp::SensorDataQoS(), std::bind(&OffboardNode::globalGoalPositionCb, this, std::placeholders::_1));
    //decentralisedCoverageSub_ = this->create_subscription<std_msgs::msg::Bool>(
    //    "decentralised_control/start_decentralised_coverage", rclcpp::SensorDataQoS(), std::bind(&OffboardNode::startDecentralisedCoverageCb, this, std::placeholders::_1));
}

void OffboardNode::initializePublishers()
{
    globalPosPub_ = this->create_publisher<geographic_msgs::msg::GeoPoseStamped>(
        "mavros/uas_" + std::to_string(uasNumber_) + "/setpoint_position/global", 10);
    centralGlobalPosPub_ = this->create_publisher<geographic_msgs::msg::GeoPoseStamped>(
        "central_control/uas_" + std::to_string(uasNumber_) + "/global_pose", 10);

    pubTimer_ = this->create_wall_timer(100ms, std::bind(&OffboardNode::positionTimerCallback, this));
}

void OffboardNode::positionTimerCallback()
{
    publishGeoPose();
    publishTargetPose();
}

void OffboardNode::initializeClients()
{
    armingClient_ = this->create_client<mavros_msgs::srv::CommandBool>(
        "mavros/uas_" + std::to_string(uasNumber_) + "/cmd/arming");
    setModeClient_ = this->create_client<mavros_msgs::srv::SetMode>(
        "mavros/uas_" + std::to_string(uasNumber_) + "/set_mode");
}

void OffboardNode::waitForConnection()
{
    rclcpp::Rate rate(1.0);
    auto self = shared_from_this();
    while (rclcpp::ok() && !currentState_.connected)
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for FCU connection...");
        rclcpp::spin_some(self);
        rate.sleep();
    }
    RCLCPP_INFO(this->get_logger(), "FCU connected.");
}

void OffboardNode::setOffboardMode()
{
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = "OFFBOARD";

    for (int i = 0; i < 3 && rclcpp::ok(); ++i)
    {
        auto future = setModeClient_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(shared_from_this(), future, std::chrono::seconds(5)) ==
                rclcpp::FutureReturnCode::SUCCESS &&
            future.get()->mode_sent)
        {
            RCLCPP_INFO(this->get_logger(), "OFFBOARD mode set.");
            return;
        }
        RCLCPP_ERROR(this->get_logger(), "Failed to set OFFBOARD mode. Retrying...");
    }
}

void OffboardNode::publishGeoPose()
{
    geographic_msgs::msg::GeoPoseStamped geoPose = getCurrentGeoPose();
    centralGlobalPosPub_->publish(geoPose);
}

void OffboardNode::publishTargetPose()
{
    if (!isCoverageStarted_)
    {
        geographic_msgs::msg::GeoPoseStamped geo_pose;
        geo_pose.pose.position.latitude = initialGps_.latitude;
        geo_pose.pose.position.longitude = initialGps_.longitude;
        geo_pose.pose.position.altitude = initialGps_.altitude + FLIGHT_ALTITUDE;
        geo_pose.header.stamp = this->now();
        globalPosPub_->publish(geo_pose);
    }
    else
    {
        globalPosPub_->publish(geoposeGoalGps_);
    }
}

void OffboardNode::armDrone()
{
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = true;

    auto future = armingClient_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(shared_from_this(), future, std::chrono::seconds(5)) ==
            rclcpp::FutureReturnCode::SUCCESS &&
        future.get()->success)
    {
        RCLCPP_INFO(this->get_logger(), "Drone armed.");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to arm drone.");
    }
}

geographic_msgs::msg::GeoPoseStamped OffboardNode::getCurrentGeoPose()
{
    geographic_msgs::msg::GeoPoseStamped geo_pose;
    geo_pose.pose.position.latitude = currentGps_.latitude;
    geo_pose.pose.position.longitude = currentGps_.longitude;
    geo_pose.pose.position.altitude = currentGps_.altitude;
    geo_pose.header.stamp = this->now();
    return geo_pose;
}

void OffboardNode::checkCoveragePositionReached()
{
    // Calculate the distance with the haversine distances and the altitudes adjusted for geoid_ height
    double geoidHeight = geoid_(
        coverageViewpoints_[viewpointAssigned_].getPose().position.latitude,
        coverageViewpoints_[viewpointAssigned_].getPose().position.longitude);
    double distance = HaversineDistance::calculateDistance(
        currentGps_.latitude,
        currentGps_.longitude,
        currentGps_.altitude,
        coverageViewpoints_[viewpointAssigned_].getPose().position.latitude,
        coverageViewpoints_[viewpointAssigned_].getPose().position.longitude,
        coverageViewpoints_[viewpointAssigned_].getPose().position.altitude + geoidHeight);

    if (distance < goalPoseTolerance_)
    {
        RCLCPP_INFO(this->get_logger(), "Coverage position %d reached by UAS %d.", viewpointAssigned_, uasNumber_);
        droneEnvironmentalRepresentation_.is_covered[viewpointAssigned_] = true;
        viewpointAssigned_ = -1;
    }
}

void OffboardNode::coveragePoseToGeoPose(geographic_msgs::msg::GeoPoseStamped &geopose, Pose &pose)
{
    geopose.pose.position.altitude = pose.position.altitude;
    geopose.pose.position.latitude = pose.position.latitude;
    geopose.pose.position.longitude = pose.position.longitude;
    geopose.pose.orientation.x = pose.orientation.x;
    geopose.pose.orientation.y = pose.orientation.y;
    geopose.pose.orientation.z = pose.orientation.z;
    geopose.pose.orientation.w = pose.orientation.w;
}
