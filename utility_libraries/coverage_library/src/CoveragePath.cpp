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
        if (viewpoint.getCoverageTime() == std::chrono::seconds(0)) {
            return viewpoint;
        }
    }
    return std::nullopt; // Return an empty optional if no match is found
}

void CoveragePath::setFirstZeroCoverageTimeViewpointTime() {
    auto now = std::chrono::system_clock::now(); // Get the current time
    auto now_sec = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()); // Convert current time to duration since epoch in seconds

    for (auto& viewpoint : path) {
        if (viewpoint.getCoverageTime() == std::chrono::seconds(0)) {
            viewpoint.setCoverageTime(now_sec); // Set the coverage time to the current time in seconds
            break;
        }
    }
}
