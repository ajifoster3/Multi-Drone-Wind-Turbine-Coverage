#include "CentralCoverageControllerNode.h"

CentralCoverageControllerNode::CentralCoverageControllerNode(const std::string &name, int team_size)
    : Node(name), teamSize_(team_size), geoid("egm96-5")
{ 
    initializeSubscribers();
    initializePublishers();
    populateCoverageSettings();

    timer_ = this->create_wall_timer(
        500ms, std::bind(&CentralCoverageControllerNode::timerCallback, this));
}

void CentralCoverageControllerNode::setCoveragePaths(std::vector<CoveragePath> &coveragePaths)
{
    this->coveragePaths_ = coveragePaths;
}

void CentralCoverageControllerNode::setCurrentGpsPosition(const geographic_msgs::msg::GeoPoseStamped& geopose, int uas_id)
{
    if (uas_id < currentGpsPositions_.size())
    {
        currentGpsPositions_[uas_id] = geopose;
        double geoidHeight = geoid(
            currentGpsPositions_[uas_id].pose.position.latitude,
            currentGpsPositions_[uas_id].pose.position.longitude);
        currentGpsPositions_[uas_id].pose.position.altitude -= geoidHeight;
    }
}

void CentralCoverageControllerNode::globalPositionCb(const geographic_msgs::msg::GeoPoseStamped::SharedPtr msg, size_t uas_id)
{
    setCurrentGpsPosition(*msg, uas_id);
}

void CentralCoverageControllerNode::timerCallback()
{
    for (int i = 0; i < teamSize_; ++i) // For each drone
    {
        // Get the publisher for the drone
        auto publisher = centralGlobalGoalPosPubs_[i];

        // Get the first unvisited viewpoint for the drone or not if coverage is complete
        std::optional<CoverageViewpoint> goalViewPoint = coveragePaths_[i].getFirstZeroCoverageTimeViewpoint();

        // If coverage isn't complete
        if (goalViewPoint)
        {
            // Get the pose from the returned viewpoint
            Pose pose = goalViewPoint.value().getPose();

            // Calculate the distance with the haversine distances and the altitudes adjusted for geoid height
            double distance = HaversineDistance::calculateDistance(
                pose.position.latitude,
                pose.position.longitude,
                pose.position.altitude,
                currentGpsPositions_[i].pose.position.latitude,
                currentGpsPositions_[i].pose.position.longitude,
                currentGpsPositions_[i].pose.position.altitude);
            // If less than 40cm from the target, set the coverage time to the current time (complete coverage)
            // TODO: replace 0.4 with a variable "goalPoseTolerance", Better being moved to the coverage_library?
            if (distance < 0.4)
            {
                coveragePaths_[i].setFirstZeroCoverageTimeViewpointTime();
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Coverage Pose reached");
            }

            // build the coverage geopose and publish
            geographic_msgs::msg::GeoPoseStamped geopose;
            coveragePoseToGeoPose(geopose, pose);
            geopose.header.stamp = this->now();
            publisher->publish(geopose);
        }
        else
        {
            // TODO: Handle coverage completion
        }
    }
}

/*
** TODO: Move to Pose class
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
        goalPoseTolerance = tbl["coverage_settings"]["goal_pose_tolerance"].value_or<double>(0.0);
    }
    catch (const toml::parse_error& err)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Parsing failed");
    }
}
