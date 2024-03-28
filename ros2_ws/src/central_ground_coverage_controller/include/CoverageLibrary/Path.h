#ifndef COVERAGEPATH_H
#define COVERAGEPATH_H

#include <vector>
#include <optional>
#include <chrono>
#include "CoverageViewpoint.h"

class Path {
public:
    Path() {};
    Path(int robotID) : robotID(robotID) {};

    void addCoverageViewpoint(const CoverageViewpoint& viewpoint);

    const std::vector<CoverageViewpoint>& getPath() const;
    const int getRobotId() const;
    void setRobotId(int);

private:
    int robotID;
    std::vector<CoverageViewpoint> path;
};

#endif // COVERAGEPATH_HPP
