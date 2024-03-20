#ifndef COVERAGEPATH_H
#define COVERAGEPATH_H

#include <vector>
#include <optional>
#include <chrono>
#include "CoverageViewpoint.h"

class CoveragePath {
public:
    CoveragePath() {};
    CoveragePath(int robotID) : robotID(robotID) {};

    void addCoverageViewpoint(const CoverageViewpoint& viewpoint);

    const std::vector<CoverageViewpoint>& getPath() const;
    const int getRobotId() const;

private:
    int robotID;
    std::vector<CoverageViewpoint> path;
};

#endif // COVERAGEPATH_HPP
