#include "CoveragePath.h"

CoveragePath::CoveragePath(int vehicleID) : vehicleID(vehicleID) {}

void CoveragePath::addCoverageViewpoint(const CoverageViewpoint& viewpoint) {
    path.push_back(viewpoint);
}

// Updated method implementation
const std::vector<CoverageViewpoint>& CoveragePath::getPath() const {
    return path;
}

std::optional<CoverageViewpoint> CoveragePath::getFirstZeroCoverageTimeViewpoint() const {
    for (const auto& viewpoint : path) {
        if (viewpoint.getCoverageTime() == 0.0) {
            return viewpoint;
        }
    }
    return std::nullopt; // Return an empty optional if no match is found
}
