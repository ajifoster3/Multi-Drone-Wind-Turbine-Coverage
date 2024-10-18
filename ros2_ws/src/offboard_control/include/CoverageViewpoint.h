#ifndef COVERAGE_VIEWPOINT_H
#define COVERAGE_VIEWPOINT_H

#include "Pose.h"
#include <chrono>

class CoverageViewpoint
{
public:
    CoverageViewpoint() {}
    CoverageViewpoint(const Pose &pose, bool assigned)
        : pose_(pose), assigned_(assigned) {}

    const Pose &getPose() const { return pose_; }
    bool isAssigned() const { return assigned_; }

    void setPose(const Pose &pose) { pose_ = pose; }
    void setAssigned(bool assigned) { assigned_ = assigned; }

private:
    Pose pose_;     // Custom Pose
    bool assigned_; // Assigned flag
};

#endif // COVERAGE_VIEWPOINT_HPP
