#ifndef CENTRALCOVERAGECONTROLLERNODEH
#define CENTRALCOVERAGECONTROLLERNODEH

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <GreedyIterativeCoveragePathPlanner.h>
#include <HaversineDistance.h>
#include <memory>
#include <GeographicLib/Geoid.hpp>
#include <toml.hpp>
#include "ConfigHeaderPath.h"
#include "TimedCoveragePath.h"
#include "CoverageLogger.h"

using namespace std::chrono_literals;

class CentralCoverageControllerNode: public rclcpp::Node
{
public:
  CentralCoverageControllerNode(const std::string & name, int team_size);

  void setCoveragePaths(std::vector < TimedCoveragePath > &);

private:
  int teamSize_ {};
  GeographicLib::Geoid geoid_;
  std::vector < TimedCoveragePath > coveragePaths_ {};
  std::vector < rclcpp::Subscription < geographic_msgs::msg::GeoPoseStamped > ::SharedPtr >
  centralGlobalPosSubs_;
  rclcpp::Subscription < rosgraph_msgs::msg::Clock > ::SharedPtr simulationTimeSub_;
  std::vector < rclcpp::Publisher < geographic_msgs::msg::GeoPoseStamped > ::SharedPtr >
  centralGlobalGoalPosPubs_;
  std::vector < geographic_msgs::msg::GeoPoseStamped > goalGpsPositions_;
  std::vector < geographic_msgs::msg::GeoPoseStamped > currentGpsPositions_;
  rosgraph_msgs::msg::Clock simulationTime_;
  rclcpp::TimerBase::SharedPtr timer_;
  double goalPoseTolerance_;


  /**
   *  Sets currentGpsPositions_ with altitude accounting for geoid_ height.
   */
  void setCurrentGpsPosition(const geographic_msgs::msg::GeoPoseStamped & geopose, int uas_id);

  /**
   *  Sets currentGpsPositions_ to the recieved GeoPoseStamped values recieved
   *  from the central_control/uas_{i}/global_pose topic.
   */
  void globalPositionCb(const geographic_msgs::msg::GeoPoseStamped::SharedPtr msg, size_t uasId);

  /**
   *  Sets simulationTime_ to the recieved rosgraph_msgs::msg::Clock value recieved
   *  from the clock topic.
   */
  void simulationTimeCb(const rosgraph_msgs::msg::Clock::SharedPtr msg);

  /**
   *  Publishes a goal geopose for each drone, based on the next uncovered point.
   */
  void timerCallback();

  /**
   *  Allocates the members of the provided geopose with those of the pose.
   */
  void coveragePoseToGeoPose(geographic_msgs::msg::GeoPoseStamped & geopose, Pose & pose);

  /**
   *  Creates a global_pose subscriber, for each drone.
   */
  void initializeSubscribers();

  /**
   *  Create a goal_pose publisher, for each drone.
   */
  void initializePublishers();

  void populateCoverageSettings();
};

#endif
