#include "OffboardNode.h"

OffboardNode::OffboardNode(const std::string &name, int uas_number)
    : Node(name), uasNumber_(uas_number)
{
    initializeSubscribers();
    initializePublishers();
    initializeClients();
}

void OffboardNode::spinNode()
{
    rclcpp::Rate rate(20.0);

    // Wait for FCU connection
    waitForConnection();

    // Try to set OFFBOARD mode
    setOffboardMode();

    // Publish target altitude to maintain OFFBOARD mode
    publishTargetPose();

    // Arm the drone
    armDrone();

    // Main loop
    while (rclcpp::ok())
    {
        publishGeoPose();
        publishTargetPose();
        //rclcpp::spin_some(shared_from_this());
        rate.sleep();
    }
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
    if(!isCentralControl_)
    {
        RCLCPP_INFO(this->get_logger(), "Switching to central control...");
        isCentralControl_ = true;
    }
    geoposeGoalGps_ = *msg;
}

void OffboardNode::initializeSubscribers()
{
    stateSub_ = this->create_subscription<mavros_msgs::msg::State>(
        "mavros/uas_" + std::to_string(uasNumber_) + "/state", 10, std::bind(&OffboardNode::stateCb, this, std::placeholders::_1));
    globalPosSub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "mavros/uas_" + std::to_string(uasNumber_) + "/global_position/global", rclcpp::SensorDataQoS(), std::bind(&OffboardNode::globalPositionCb, this, std::placeholders::_1));
    centralGoalPosSub_ = this->create_subscription<geographic_msgs::msg::GeoPoseStamped>(
        "central_control/uas_" + std::to_string(uasNumber_) + "/goal_pose", rclcpp::SensorDataQoS(), std::bind(&OffboardNode::globalGoalPositionCb, this, std::placeholders::_1));
}

void OffboardNode::initializePublishers()
{
    globalPosPub_ = this->create_publisher<geographic_msgs::msg::GeoPoseStamped>(
        "mavros/uas_" + std::to_string(uasNumber_) + "/setpoint_position/global", 10);
    centralGlobalPosPub_ = this->create_publisher<geographic_msgs::msg::GeoPoseStamped>(
        "central_control/uas_" + std::to_string(uasNumber_) + "/global_pose", 10);
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
    while (rclcpp::ok() && !currentState_.connected)
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for FCU connection...");
        rclcpp::spin_some(shared_from_this());
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
    if (!isCentralControl_)
    {
        geographic_msgs::msg::GeoPoseStamped geo_pose;
        geo_pose.pose.position.latitude = currentGps_.latitude;
        geo_pose.pose.position.longitude = currentGps_.longitude;
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
