#ifndef COVERAGE_VIEWPOINT_H
#define COVERAGE_VIEWPOINT_H

#include "Pose.h"

class CoverageViewpoint {
public:

CoverageViewpoint(const Pose& pose, double coverageTime, bool assigned)
: pose_(pose), coverageTime_(coverageTime), assigned_(assigned) {}

Pose getPose() const { return pose_; }
double getCoverageTime() const { return coverageTime_; }
bool isAssigned() const { return assigned_; }
int getRobotIDAssigned() const { return robotIDAssigned_; }

void setPose(const Pose& pose) { pose_ = pose; }
void setCoverageTime(double coverageTime) { coverageTime_ = coverageTime; }
void setAssigned(bool assigned) { assigned_ = assigned; }
void setRobotIDAssigned(int robotIDAssigned) { robotIDAssigned_ = robotIDAssigned; }


private:
    Pose pose_; // Custom Pose
    double coverageTime_; // Time in seconds
    bool assigned_; // Assigned flag
    int robotIDAssigned_;
};

#endif // COVERAGE_VIEWPOINT_HPP
