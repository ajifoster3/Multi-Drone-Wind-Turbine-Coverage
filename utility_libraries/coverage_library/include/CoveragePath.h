#ifndef COVERAGEPATH_H
#define COVERAGEPATH_H

#include <vector>
#include <optional>
#include "CoverageViewpoint.h"

class CoveragePath {
public:
    explicit CoveragePath(int vehicleID);

    void addCoverageViewpoint(const CoverageViewpoint& viewpoint);

    // Updated method name
    const std::vector<CoverageViewpoint>& getPath() const;

private:
    int vehicleID;
    std::vector<CoverageViewpoint> path;
    std::optional<CoverageViewpoint> getFirstZeroCoverageTimeViewpoint() const;
};

#endif // COVERAGEPATH_HPP
