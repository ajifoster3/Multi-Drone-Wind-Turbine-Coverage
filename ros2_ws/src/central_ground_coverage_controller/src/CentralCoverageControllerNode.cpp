#include "CentralCoverageControllerNode.h"

CentralCoverageControllerNode::CentralCoverageControllerNode(const std::string &name, int team_size)
    : Node(name), teamSize_(team_size), geoid_("egm96-5")
{
    dronePositions_.resize(team_size);
    initializeSubscribers();
    initializePublishers();
    populateCoverageSettings();

    timer_ = this->create_wall_timer(
        500ms, std::bind(&CentralCoverageControllerNode::timerCallback, this));
}

void CentralCoverageControllerNode::setCoveragePaths(std::vector<TimedCoveragePath> &coveragePaths)
{
    this->coveragePaths_ = coveragePaths;
}

void CentralCoverageControllerNode::setCurrentGpsPosition(const geographic_msgs::msg::GeoPoseStamped &geopose, int uas_id)
{
    if (uas_id < currentGpsPositions_.size())
    {
        currentGpsPositions_[uas_id] = geopose;
        double geoidHeight = geoid_(
            currentGpsPositions_[uas_id].pose.position.latitude,
            currentGpsPositions_[uas_id].pose.position.longitude);
        currentGpsPositions_[uas_id].pose.position.altitude -= geoidHeight;

        dronePositions_[uas_id].emplace_back(simulationTime_, geopose.pose);
    }
}

void CentralCoverageControllerNode::globalPositionCb(const geographic_msgs::msg::GeoPoseStamped::SharedPtr msg, size_t uas_id)
{
    setCurrentGpsPosition(*msg, uas_id);
}

void CentralCoverageControllerNode::simulationTimeCb(const rosgraph_msgs::msg::Clock::SharedPtr msg)
{
    simulationTime_ = *msg;
}

void CentralCoverageControllerNode::timerCallback()
{
    bool isCoverageComplete{true};
    for (int i = 0; i < teamSize_; ++i) // For each drone
    {
        // Get the publisher for the drone
        auto publisher = centralGlobalGoalPosPubs_[i];

        // Get the first unvisited viewpoint for the drone or not if coverage is complete
        std::optional<TimedCoverageViewpoint> goalViewPoint = coveragePaths_[i].getFirstZeroCoverageTimeViewpointTime();

        // If coverage isn't complete
        if (goalViewPoint)
        {
            isCoverageComplete = false;
            // Get the pose from the returned viewpoint
            Pose pose = goalViewPoint.value().getPose();

            // Calculate the distance with the haversine distances and the altitudes adjusted for geoid_ height
            double distance = HaversineDistance::calculateDistance(
                pose.position.latitude,
                pose.position.longitude,
                pose.position.altitude,
                currentGpsPositions_[i].pose.position.latitude,
                currentGpsPositions_[i].pose.position.longitude,
                currentGpsPositions_[i].pose.position.altitude);

            // If less than 40cm from the target, set the coverage time to the current time (complete coverage)
            if (distance < goalPoseTolerance_)
            {
                coveragePaths_[i].setFirstZeroCoverageTimeViewpointTime(simulationTime_);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Coverage Pose reached at %d", simulationTime_.clock.sec);
            }

            // build the coverage geopose and publish
            geographic_msgs::msg::GeoPoseStamped geopose;
            coveragePoseToGeoPose(geopose, pose);
            geopose.header.stamp = this->now();
            publisher->publish(geopose);
        }
    }
    if(isCoverageComplete)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Coverage Complete");
        CoverageLogger::setViewpointCoverageTimes(this->coveragePaths_);
        CoverageLogger::setDronePositions(this->dronePositions_);
        rclcpp::shutdown();
    }
}

/*
** TODO: This is suspicious for this class, think about where to move?
*/
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
    // Resize the vector to hold GPS positions for all team members
    currentGpsPositions_.resize(teamSize_);

    // For each drone create the global_pose subscriber
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

    simulationTimeSub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
        "clock", rclcpp::SensorDataQoS(), [this](const rosgraph_msgs::msg::Clock::SharedPtr msg)
        { simulationTimeCb(msg); });
}

void CentralCoverageControllerNode::initializePublishers()
{

    // Resize the vector to hold GPS goal positions for all team members
    goalGpsPositions_.resize(teamSize_);

    // For each drone create the goal_pose publisher
    for (int i = 0; i < teamSize_; ++i)
    {
        std::string topic_name = "central_control/uas_" + std::to_string(i + 1) + "/goal_pose";
        centralGlobalGoalPosPubs_.push_back(
            this->create_publisher<geographic_msgs::msg::GeoPoseStamped>(
                topic_name, rclcpp::SensorDataQoS()));
    }
}

void CentralCoverageControllerNode::populateCoverageSettings()
{
    toml::table tbl;
    try
    {
        tbl = toml::parse_file(config_header_path);
        goalPoseTolerance_ = tbl["coverage_settings"]["goal_pose_tolerance"].value_or<double>(0.0);
    }
    catch (const toml::parse_error &err)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Parsing failed");
    }
}
