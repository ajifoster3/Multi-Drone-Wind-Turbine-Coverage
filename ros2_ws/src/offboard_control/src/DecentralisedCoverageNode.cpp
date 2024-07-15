#include "DecentralisedCoverageNode.h"

using namespace std::chrono_literals;

DecentralisedCoverageNode::DecentralisedCoverageNode(const std::string &name, int uas_number)
    : Node(name), uasNumber_(uas_number), geoid_("egm96-5")
{
    decentralisedCoverageSub_ = this->create_subscription<std_msgs::msg::Bool>(
        "decentralised_control/start_decentralised_coverage", rclcpp::SensorDataQoS(), std::bind(&DecentralisedCoverageNode::startDecentralisedCoverageCb, this, std::placeholders::_1));
    viewpointAssigned_ = -1;
}

void DecentralisedCoverageNode::startDecentralisedCoverageCb(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!isCoverageStarted_)
    {
        coverageViewpoints_ = CoverageViewpointLoader::load("/home/ajifoster3/Downloads/all_geoposes_wind_turbine.json");
        isCoverageStarted_ = true;
        droneAllocation_.allocations = std::vector<int>(coverageViewpoints_.size(), -1);
        droneEnvironmentalRepresentation_.is_covered = std::vector<bool>(coverageViewpoints_.size(), false);
        initializePublishers();
        initializeSubscribers();
        RCLCPP_INFO(this->get_logger(), "Drone %d setup", uasNumber_);
    }
}

void DecentralisedCoverageNode::droneAllocationCb(const offboard_control_interfaces::msg::DroneAllocation::SharedPtr msg)
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
        if(std::find(droneAllocation_.allocations.begin(), droneAllocation_.allocations.end(), -1) != droneAllocation_.allocations.end())
        {
            allocateUnassignedViewpoint();
        }
        else
        {
            centralGoalPosPub_->publish(initialGps_);
        }
    }
}

void DecentralisedCoverageNode::droneEnvironmentalRepresentationCb(const offboard_control_interfaces::msg::DroneEnvironmentalRepresentation::SharedPtr msg)
{
    for (std::vector<bool>::size_type i = 0; i < msg->is_covered.size(); i++)
    {
        if (i < droneEnvironmentalRepresentation_.is_covered.size() && msg->is_covered[i] && !droneEnvironmentalRepresentation_.is_covered[i])
        {
            droneEnvironmentalRepresentation_.is_covered[i] = true;
        }
    }
}

void DecentralisedCoverageNode::dronePingCb(const offboard_control_interfaces::msg::DronePing::SharedPtr msg)
{
}

void DecentralisedCoverageNode::globalPositionCb(const geographic_msgs::msg::GeoPoseStamped msg)
{
    if (!isGPSSet_)
    {
        isGPSSet_ = true;
        initialGps_ = msg;
        initialGps_.pose.position.altitude = initialGps_.pose.position.altitude - geoid_(initialGps_.pose.position.latitude, initialGps_.pose.position.longitude);
    }
    currentGps_ = msg;
}

void DecentralisedCoverageNode::initializeSubscribers()
{
    droneAllocationSub_ = this->create_subscription<offboard_control_interfaces::msg::DroneAllocation>(
        "decentralised_control/drone_allocation", rclcpp::SensorDataQoS(), std::bind(&DecentralisedCoverageNode::droneAllocationCb, this, std::placeholders::_1));
    droneEnvironmentalRepresentationSub_ = this->create_subscription<offboard_control_interfaces::msg::DroneEnvironmentalRepresentation>(
        "decentralised_control/drone_environmental_representation", rclcpp::SensorDataQoS(), std::bind(&DecentralisedCoverageNode::droneEnvironmentalRepresentationCb, this, std::placeholders::_1));
    dronePingSub_ = this->create_subscription<offboard_control_interfaces::msg::DronePing>(
        "decentralised_control/drone_ping", rclcpp::SensorDataQoS(), std::bind(&DecentralisedCoverageNode::dronePingCb, this, std::placeholders::_1));
    centralGlobalPosSub_ = this->create_subscription<geographic_msgs::msg::GeoPoseStamped>(
        "central_control/uas_" + std::to_string(uasNumber_) + "/global_pose", rclcpp::SensorDataQoS(), std::bind(&DecentralisedCoverageNode::globalPositionCb, this, std::placeholders::_1));
}

void DecentralisedCoverageNode::initializePublishers()
{
    droneAllocationPub_ = this->create_publisher<offboard_control_interfaces::msg::DroneAllocation>(
        "decentralised_control/drone_allocation", 10);
    droneEnvironmentalRepresentationPub_ = this->create_publisher<offboard_control_interfaces::msg::DroneEnvironmentalRepresentation>(
        "decentralised_control/drone_environmental_representation", 10);
    dronePingPub_ = this->create_publisher<offboard_control_interfaces::msg::DronePing>(
        "decentralised_control/drone_ping", 10);
    centralGoalPosPub_ = this->create_publisher<geographic_msgs::msg::GeoPoseStamped>(
        "central_control/uas_" + std::to_string(uasNumber_) + "/goal_pose", 10);

    decentralisedPubTimer_ = this->create_wall_timer(500ms, std::bind(&DecentralisedCoverageNode::decentralisedCoverageTimerCallback, this));
}

void DecentralisedCoverageNode::decentralisedCoverageTimerCallback()
{
    RCLCPP_INFO(this->get_logger(), "Drone %d decentralisedCoverageTimerCallback", uasNumber_);
    if (isGPSSet_)
    {
        if (viewpointAssigned_ == -1)
        {
            RCLCPP_INFO(this->get_logger(), "Drone %d allocateUnassignedViewpoint", uasNumber_);
            allocateUnassignedViewpoint();
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Drone %d viewPoint already assigned: %d", uasNumber_, viewpointAssigned_);
            geographic_msgs::msg::GeoPoseStamped geopose;
            auto pose = coverageViewpoints_[viewpointAssigned_].getPose();
            coveragePoseToGeoPose(geopose, pose);
            centralGoalPosPub_->publish(geopose);
            checkCoveragePositionReached();
        }
    }
    publishDroneAllocation();
    publishDroneEnvironmentalRepresentation();
}

void DecentralisedCoverageNode::publishDroneAllocation()
{
    RCLCPP_INFO(this->get_logger(), "Drone %d publishDroneAllocation", uasNumber_);
    offboard_control_interfaces::msg::DroneAllocation droneAllocation = droneAllocation_;
    droneAllocationPub_->publish(droneAllocation);
}

void DecentralisedCoverageNode::publishDroneEnvironmentalRepresentation()
{
    RCLCPP_INFO(this->get_logger(), "Drone %d publishDroneEnvironmentalRepresentation", uasNumber_);
    offboard_control_interfaces::msg::DroneEnvironmentalRepresentation DroneEnvironmentalRepresentation = droneEnvironmentalRepresentation_;
    droneEnvironmentalRepresentationPub_->publish(DroneEnvironmentalRepresentation);
}

void DecentralisedCoverageNode::publishDronePing()
{
    offboard_control_interfaces::msg::DronePing dronePing = offboard_control_interfaces::msg::DronePing();
    dronePing.drone_id = uasNumber_;
    dronePingPub_->publish(dronePing);
}

int DecentralisedCoverageNode::getClosestViewpointIndex()
{
    int closestViewpointIndex = -1;
    double closestViewpointDistance = std::numeric_limits<double>::max();

    for (size_t i = 0; i < coverageViewpoints_.size(); ++i)
    {
        if (droneAllocation_.allocations[i] == -1 && droneEnvironmentalRepresentation_.is_covered[i] != true)
        {
            auto viewpoint = coverageViewpoints_[i];
            double geoidHeight = geoid_(
                coverageViewpoints_[i].getPose().position.latitude,
                coverageViewpoints_[i].getPose().position.longitude);
            double distance = HaversineDistance::calculateDistance(
                currentGps_.pose.position.latitude,
                currentGps_.pose.position.longitude,
                currentGps_.pose.position.altitude,
                coverageViewpoints_[i].getPose().position.latitude,
                coverageViewpoints_[i].getPose().position.longitude,
                coverageViewpoints_[i].getPose().position.altitude + geoidHeight);

            if (distance < closestViewpointDistance)
            {
                closestViewpointDistance = distance;
                closestViewpointIndex = i;
            }
        }
    }

    return closestViewpointIndex;
}

void DecentralisedCoverageNode::allocateUnassignedViewpoint()
{
    viewpointAssigned_ = getClosestViewpointIndex();
    if (viewpointAssigned_ != -1)
    {
        droneAllocation_.allocations[viewpointAssigned_] = uasNumber_;

        RCLCPP_INFO(this->get_logger(), "Assigning viewpoint %d to UAS %d.", viewpointAssigned_, uasNumber_);

        geographic_msgs::msg::GeoPoseStamped geopose;
        auto pose = coverageViewpoints_[viewpointAssigned_].getPose();
        coveragePoseToGeoPose(geopose, pose);
        geopose.header.stamp = this->now();
        geoposeGoalGps_ = geopose;
    }
}

void DecentralisedCoverageNode::checkCoveragePositionReached()
{
    // Calculate the distance with the haversine distances and the altitudes adjusted for geoid_ height
    double geoidHeight = geoid_(
        coverageViewpoints_[viewpointAssigned_].getPose().position.latitude,
        coverageViewpoints_[viewpointAssigned_].getPose().position.longitude);
    double distance = HaversineDistance::calculateDistance(
        currentGps_.pose.position.latitude,
        currentGps_.pose.position.longitude,
        currentGps_.pose.position.altitude,
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

void DecentralisedCoverageNode::coveragePoseToGeoPose(geographic_msgs::msg::GeoPoseStamped &geopose, Pose &pose)
{
    geopose.pose.position.altitude = pose.position.altitude;
    geopose.pose.position.latitude = pose.position.latitude;
    geopose.pose.position.longitude = pose.position.longitude;
    geopose.pose.orientation.x = pose.orientation.x;
    geopose.pose.orientation.y = pose.orientation.y;
    geopose.pose.orientation.z = pose.orientation.z;
    geopose.pose.orientation.w = pose.orientation.w;
}
