#include "TrackingNode.hpp"

namespace fs = std::filesystem;

namespace trackingNode
{
    TrackingNode::TrackingNode(int teamsize) : Node("tracking_node"), teamsize_(teamsize)
    {
        initialiseStartDecentralisedCoverageSubscriber();
    };

    void TrackingNode::initialiseStartDecentralisedCoverageSubscriber()
    {
        using std::placeholders::_1;
        startDecentralisedCoverageSub_ = this->create_subscription<std_msgs::msg::Bool>(
            "decentralised_control/start_decentralised_coverage", 10, std::bind(&TrackingNode::startDecentralisedCoverageSubCB, this, _1));
        clockSub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
            "clock", 10, std::bind(&TrackingNode::clockSubCB, this, _1));
    }

    void TrackingNode::initialiseCoverageSubscribers()
    {
        using std::placeholders::_1;
        RCLCPP_INFO(this->get_logger(), "droneEnvironmentalRepresentationSub_ init");
        droneEnvironmentalRepresentationSub_ = this->create_subscription<offboard_control_interfaces::msg::DroneEnvironmentalRepresentation>(
            "decentralised_control/drone_environmental_representation", 10, std::bind(&TrackingNode::droneEnvironmentalRepresentationSubCB, this, _1));
        RCLCPP_INFO(this->get_logger(), "droneAllocationSub_ init");
        droneAllocationSub_ = this->create_subscription<offboard_control_interfaces::msg::DroneAllocation>(
            "decentralised_control/drone_allocation", 10, std::bind(&TrackingNode::droneAllocationSubCB, this, _1));
        RCLCPP_INFO(this->get_logger(), "initialiseDronePositionSubscribers init");
        initialiseDronePositionSubscribers();
        RCLCPP_INFO(this->get_logger(), "all init");
    }

    void TrackingNode::initialiseDronePositionSubscribers()
    {
        dronePositions_.reserve(teamsize_);
        for (int i = 0; i < teamsize_; i++)
        {

            using std::placeholders::_1;
            std::string topic_name = "/central_control/uas_" + std::to_string(i + 1) + "/global_pose";
            dronePositions_.emplace_back();
            dronePositionSub_.push_back(
                this->create_subscription<geographic_msgs::msg::GeoPoseStamped>(
                    topic_name, 10,
                    [this, i](const geographic_msgs::msg::GeoPoseStamped::SharedPtr msg)
                    {
                        dronePositionSubCB(msg, i);
                    }));
        }
    }

    void TrackingNode::droneEnvironmentalRepresentationSubCB(const offboard_control_interfaces::msg::DroneEnvironmentalRepresentation::SharedPtr msg)
    {
        droneEnvironmentalRepresentation_.push_back(std::pair<rosgraph_msgs::msg::Clock, offboard_control_interfaces::msg::DroneEnvironmentalRepresentation>(currentTime, *msg));
        logDroneEnvironmentalRepresentation(msg);
    }

    void TrackingNode::droneAllocationSubCB(const offboard_control_interfaces::msg::DroneAllocation::SharedPtr msg)
    {
        droneAllocations_.push_back(std::pair<rosgraph_msgs::msg::Clock, offboard_control_interfaces::msg::DroneAllocation>(currentTime, *msg));
        logDroneAllocation(msg);
    }

    void TrackingNode::dronePositionSubCB(const geographic_msgs::msg::GeoPoseStamped::SharedPtr msg, int robotId)
    {
        if (robotId >= 0 && robotId < teamsize_)
        {
            dronePositions_[robotId].emplace_back(currentTime, msg->pose);
            logDronePosition(robotId, msg);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Received position for invalid robotId: %d", robotId);
        }
    }

    void TrackingNode::startDecentralisedCoverageSubCB(const std_msgs::msg::Bool::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received message");
        initialiseCoverageSubscribers();
        startDecentralisedCoverageSub_.reset();
    }

    void TrackingNode::clockSubCB(const rosgraph_msgs::msg::Clock::SharedPtr msg)
    {
        currentTime = *msg;
    }

    void TrackingNode::logDronePosition(int robotId, const geographic_msgs::msg::GeoPoseStamped::SharedPtr msg)
    {
        std::ofstream positionFile("runlog/drone_positions.csv", std::ios::app);
        if (positionFile.is_open())
        {
            positionFile << std::setprecision(12) << robotId + 1 << ","
                         << currentTime.clock.sec << "." << currentTime.clock.nanosec << ","
                         << msg->pose.position.latitude << ","
                         << msg->pose.position.longitude << ","
                         << msg->pose.position.altitude << "\n";
            positionFile.close();
        }
    }

    void TrackingNode::logDroneAllocation(const offboard_control_interfaces::msg::DroneAllocation::SharedPtr msg)
    {
        std::ofstream allocationFile("runlog/drone_allocations.csv", std::ios::app);
        if (allocationFile.is_open())
        {
            allocationFile << currentTime.clock.sec << "." << currentTime.clock.nanosec << ",";
            for (const auto &alloc : msg->allocations)
            {
                allocationFile << alloc << " ";
            }
            allocationFile << "\n";
            allocationFile.close();
        }
    }

    void TrackingNode::logDroneEnvironmentalRepresentation(const offboard_control_interfaces::msg::DroneEnvironmentalRepresentation::SharedPtr msg)
    {
        std::ofstream envRepFile("runlog/drone_environmental_representations.csv", std::ios::app);
        if (envRepFile.is_open())
        {
            envRepFile << currentTime.clock.sec << "." << currentTime.clock.nanosec << ",";
            for (const auto &covered : msg->is_covered)
            {
                envRepFile << covered << " ";
            }
            envRepFile << "\n";
            envRepFile.close();
        }
    }


    void TrackingNode::logCoverage()
    {
        RCLCPP_INFO(this->get_logger(), "Coverage Finished");

        // Find next available folder name
        int run_number = 1;
        std::string folder_name;
        do
        {
            folder_name = "runlog/coverage_run_" + std::to_string(run_number);
            run_number++;
        } while (fs::exists(folder_name));

        // Create the directory
        fs::create_directory(folder_name);

        // Log drone positions in CSV format
        std::ofstream positionFile(folder_name + "/drone_positions.csv");
        if (positionFile.is_open())
        {
            positionFile << "Drone,Time,Latitude,Longitude,Altitude\n";
            for (size_t i = 0; i < dronePositions_.size(); ++i)
            {
                for (const auto &position : dronePositions_[i])
                {
                    positionFile << std::setprecision(12) << i + 1 << ","
                                 << position.first.clock.sec << "." << position.first.clock.nanosec << ","
                                 << position.second.position.latitude << ","
                                 << position.second.position.longitude << ","
                                 << position.second.position.altitude << "\n";
                }
            }
            positionFile.close();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to open drone positions file for logging");
        }

        // Log drone allocations in CSV format
        std::ofstream allocationFile(folder_name + "/drone_allocations.csv");
        if (allocationFile.is_open())
        {
            allocationFile << "Time,Allocations\n";
            for (const auto &allocation : droneAllocations_)
            {
                allocationFile << allocation.first.clock.sec << "." << allocation.first.clock.nanosec << ",";
                for (const auto &alloc : allocation.second.allocations)
                {
                    allocationFile << alloc << " ";
                }
                allocationFile << "\n";
            }
            allocationFile.close();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to open drone allocations file for logging");
        }

        // Log drone environmental representations in CSV format
        std::ofstream envRepFile(folder_name + "/drone_environmental_representations.csv");
        if (envRepFile.is_open())
        {
            envRepFile << "Time,is_covered\n";
            for (const auto &envRep : droneEnvironmentalRepresentation_)
            {
                envRepFile << envRep.first.clock.sec << "." << envRep.first.clock.nanosec << ",";
                for (const auto &covered : envRep.second.is_covered)
                {
                    envRepFile << covered << " ";
                }
                envRepFile << "\n";
            }
            envRepFile.close();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to open drone environmental representations file for logging");
        }

        droneEnvironmentalRepresentationSub_.reset();
        droneAllocationSub_.reset();
        for (auto &&i : dronePositionSub_)
        {
            i.reset();
        }
    }

} // namespace TrackingNode
