#include "OffboardNode.h"

OffboardNode::OffboardNode(const std::string &name, int uas_number, CoverageMode mode)
    : Node(name), coverageMode_(mode), uasNumber_(uas_number), geoid_("egm96-5")
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

    RCLCPP_INFO(this->get_logger(), "Setting up offboard mode...");

    // Try to set OFFBOARD mode
    setOffboardMode();

    RCLCPP_INFO(this->get_logger(), "Publishing initial Pose...");

    // Publish target altitude to maintain OFFBOARD mode
    publishTargetPose();

    // Arm the drone
    armDrone();

    // Main loop
    while (rclcpp::ok())
    {

        publishGeoPose();
        publishTargetPose();
        if (isCoverageStarted_)
        {
            if (viewpointAssigned_ == -1)
            {
                allocateUnassignedViewpoint();
            }
            else
            {
                checkCoveragePositionReached();
            }
            publishDroneAllocation();
            publishDroneEnvironmentalRepresentation();
        }
        rclcpp::spin_some(shared_from_this());
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
    if (!isCoverageStarted_)
    {
        RCLCPP_INFO(this->get_logger(), "Switching to central control...");
        isCoverageStarted_ = true;
    }
    geoposeGoalGps_ = *msg;
}

void OffboardNode::startDecentralisedCoverageCb(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!isCoverageStarted_)
    {
        RCLCPP_INFO(this->get_logger(), "Loading Viewpoints...");
        coverageViewpoints_ = CoverageViewpointLoader::load("/home/ajifoster3/Downloads/all_geoposes_wind_turbine.json");
        RCLCPP_INFO(this->get_logger(), "Starting decentralised coverage...");
        isCoverageStarted_ = true;
        droneAllocation_.allocations = std::vector(coverageViewpoints_.size(), -1);
        droneEnvironmentalRepresentation_.is_covered = std::vector(coverageViewpoints_.size(), false);
        initializeDecentralisedSubscribers();
    }
}

void OffboardNode::droneAllocationCb(const offboard_control_interfaces::msg::DroneAllocation::SharedPtr msg)
{
    for (size_t i = 0; i < msg->allocations.size(); i++)
    {
        if (msg->allocations[i] != droneAllocation_.allocations[i])
        {
            droneAllocation_.allocations[i] = msg->allocations[i] > droneAllocation_.allocations[i] ? msg->allocations[i] : droneAllocation_.allocations[i];
        }
    }
    if (droneAllocation_.allocations[viewpointAssigned_] != uasNumber_)
    {
        allocateUnassignedViewpoint();
    }
}

void OffboardNode::droneEnvironmentalRepresentationCb(const offboard_control_interfaces::msg::DroneEnvironmentalRepresentation::SharedPtr msg)
{
    for (std::vector<bool>::size_type i = 0; i < msg->is_covered.size(); i++)
    {
        if (i < droneEnvironmentalRepresentation_.is_covered.size() && msg->is_covered[i] && !droneEnvironmentalRepresentation_.is_covered[i])
        {
            RCLCPP_INFO(this->get_logger(), "Discovered new coverage information");
            droneEnvironmentalRepresentation_.is_covered[i] = true;
        }
    }
}

void OffboardNode::initializeSubscribers()
{
    stateSub_ = this->create_subscription<mavros_msgs::msg::State>(
        "mavros/uas_" + std::to_string(uasNumber_) + "/state", 10, std::bind(&OffboardNode::stateCb, this, std::placeholders::_1));
    globalPosSub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "mavros/uas_" + std::to_string(uasNumber_) + "/global_position/global", rclcpp::SensorDataQoS(), std::bind(&OffboardNode::globalPositionCb, this, std::placeholders::_1));
    centralGoalPosSub_ = this->create_subscription<geographic_msgs::msg::GeoPoseStamped>(
        "central_control/uas_" + std::to_string(uasNumber_) + "/goal_pose", rclcpp::SensorDataQoS(), std::bind(&OffboardNode::globalGoalPositionCb, this, std::placeholders::_1));
    decentralisedCoverageSub_ = this->create_subscription<std_msgs::msg::Bool>(
        "decentralised_control/start_decentralised_coverage", rclcpp::SensorDataQoS(), std::bind(&OffboardNode::startDecentralisedCoverageCb, this, std::placeholders::_1));
}
void OffboardNode::initializeDecentralisedSubscribers()
{
    droneAllocationSub_ = this->create_subscription<offboard_control_interfaces::msg::DroneAllocation>(
        "decentralised_control/drone_allocation", rclcpp::SensorDataQoS(), std::bind(&OffboardNode::droneAllocationCb, this, std::placeholders::_1));
    droneEnvironmentalRepresentationSub_ = this->create_subscription<offboard_control_interfaces::msg::DroneEnvironmentalRepresentation>(
        "decentralised_control/drone_environmental_representation", rclcpp::SensorDataQoS(), std::bind(&OffboardNode::droneEnvironmentalRepresentationCb, this, std::placeholders::_1));
}

void OffboardNode::initializePublishers()
{
    globalPosPub_ = this->create_publisher<geographic_msgs::msg::GeoPoseStamped>(
        "mavros/uas_" + std::to_string(uasNumber_) + "/setpoint_position/global", 10);
    centralGlobalPosPub_ = this->create_publisher<geographic_msgs::msg::GeoPoseStamped>(
        "central_control/uas_" + std::to_string(uasNumber_) + "/global_pose", 10);
    droneAllocationPub_ = this->create_publisher<offboard_control_interfaces::msg::DroneAllocation>(
        "decentralised_control/drone_allocation", 10);
    droneEnvironmentalRepresentationPub_ = this->create_publisher<offboard_control_interfaces::msg::DroneEnvironmentalRepresentation>(
        "decentralised_control/drone_environmental_representation", 10);
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

void OffboardNode::publishDroneAllocation()
{
    offboard_control_interfaces::msg::DroneAllocation droneAllocation = droneAllocation_;
    droneAllocationPub_->publish(droneAllocation);
}

void OffboardNode::publishDroneEnvironmentalRepresentation()
{
    offboard_control_interfaces::msg::DroneEnvironmentalRepresentation DroneEnvironmentalRepresentation = droneEnvironmentalRepresentation_;
    droneEnvironmentalRepresentationPub_->publish(DroneEnvironmentalRepresentation);
}

// void OffboardNode::publishDroneAllocation()
// {
//     offboard_control_interfaces::msg::DroneAllocation droneAllocation;
//     droneAllocation.allocations[0] = 1;
//     bool allocation = droneAllocation.allocations[0];
// }

void OffboardNode::publishTargetPose()
{
    if (!isCoverageStarted_)
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
    geo_pose.pose.position.latitude = initialGps_.latitude;
    geo_pose.pose.position.longitude = initialGps_.longitude;
    geo_pose.pose.position.altitude = initialGps_.altitude;
    geo_pose.header.stamp = this->now();
    return geo_pose;
}

int OffboardNode::getClosestViewpointIndex()
{
    int closestViewpointIndex = -1;
    double closestViewpointDistance = std::numeric_limits<double>::max();

    for (size_t i = 0; i < coverageViewpoints_.size(); ++i)
    {
        if (droneAllocation_.allocations[i] == -1 && droneEnvironmentalRepresentation_.is_covered[i] != true)
        {
            auto viewpoint = coverageViewpoints_[i];
            double distance = HaversineDistance::calculateDistance(
                currentGps_.latitude,
                currentGps_.longitude,
                viewpoint.getPose().position.latitude,
                viewpoint.getPose().position.longitude);

            if (distance < closestViewpointDistance)
            {
                closestViewpointDistance = distance;
                closestViewpointIndex = i;
            }
        }
    }

    return closestViewpointIndex;
}

void OffboardNode::allocateUnassignedViewpoint()
{
    viewpointAssigned_ = getClosestViewpointIndex();
    if(viewpointAssigned_ != -1){
        droneAllocation_.allocations[viewpointAssigned_] = uasNumber_;

    RCLCPP_INFO(this->get_logger(), "Assigning viewpoint %d to UAS %d.", viewpointAssigned_, uasNumber_);

    geographic_msgs::msg::GeoPoseStamped geopose;
    auto pose = coverageViewpoints_[viewpointAssigned_].getPose();
    coveragePoseToGeoPose(geopose, pose);
    geopose.header.stamp = this->now();
    geoposeGoalGps_ = geopose;
    }
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
