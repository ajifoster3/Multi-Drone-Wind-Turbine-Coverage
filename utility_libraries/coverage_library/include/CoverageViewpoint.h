#ifndef COVERAGE_VIEWPOINT_H
#define COVERAGE_VIEWPOINT_H

#include "Pose.h"
#include <chrono>

class CoverageViewpoint {
public:

CoverageViewpoint(const Pose& pose, std::chrono::seconds coverageTime, bool assigned)
: pose_(pose), coverageTime_(coverageTime), assigned_(assigned) {}

Pose getPose() const { return pose_; }
std::chrono::seconds getCoverageTime() const { return coverageTime_; }
bool isAssigned() const { return assigned_; }
int getRobotIDAssigned() const { return robotIDAssigned_; }

void setPose(const Pose& pose) { pose_ = pose; }
void setCoverageTime(std::chrono::seconds coverageTime) { coverageTime_ = coverageTime; }
void setAssigned(bool assigned) { assigned_ = assigned; }
void setRobotIDAssigned(int robotIDAssigned) { robotIDAssigned_ = robotIDAssigned; }


private:
    Pose pose_; // Custom Pose
    std::chrono::seconds coverageTime_; // Time in seconds
    bool assigned_; // Assigned flag
    int robotIDAssigned_;
};

#endif // COVERAGE_VIEWPOINT_HPP
