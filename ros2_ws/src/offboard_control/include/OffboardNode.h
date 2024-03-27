#ifndef OFFBOARDNODE_H
#define OFFBOARDNODE_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/state.hpp>

#define FLIGHT_ALTITUDE -50.0f

class OffboardNode : public rclcpp::Node
{
public:
OffboardNode(const std::string & name, int uas_number);

void spinNode();

private:
int uasNumber_;
mavros_msgs::msg::State currentState_;
bool isGpsSet_ {false};
bool isCentralControl_ {false};
sensor_msgs::msg::NavSatFix initialGps_;
sensor_msgs::msg::NavSatFix currentGps_;
sensor_msgs::msg::NavSatFix goalGps_;
geographic_msgs::msg::GeoPoseStamped geoposeGoalGps_ {};
rclcpp::Subscription < mavros_msgs::msg::State > ::SharedPtr stateSub_;
rclcpp::Subscription < sensor_msgs::msg::NavSatFix > ::SharedPtr globalPosSub_;
rclcpp::Subscription < geographic_msgs::msg::GeoPoseStamped > ::SharedPtr centralGoalPosSub_;
rclcpp::Publisher < geographic_msgs::msg::GeoPoseStamped > ::SharedPtr globalPosPub_;
rclcpp::Publisher < geographic_msgs::msg::GeoPoseStamped > ::SharedPtr centralGlobalPosPub_;
rclcpp::Client < mavros_msgs::srv::CommandBool > ::SharedPtr armingClient_;
rclcpp::Client < mavros_msgs::srv::SetMode > ::SharedPtr setModeClient_;

void stateCb(const mavros_msgs::msg::State::SharedPtr msg)
{
	currentState_ = *msg;
}

void globalPositionCb(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

void globalGoalPositionCb(const geographic_msgs::msg::GeoPoseStamped::SharedPtr msg);

void initializeSubscribers();

void initializePublishers();

void initializeClients();

void waitForConnection();

void setOffboardMode();

void publishGeoPose();

void publishTargetPose();

void armDrone();

geographic_msgs::msg::GeoPoseStamped getCurrentGeoPose();
};

#endif
