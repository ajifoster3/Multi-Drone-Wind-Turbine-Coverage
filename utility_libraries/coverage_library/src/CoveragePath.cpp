#include "CoveragePath.h"

void CoveragePath::addCoverageViewpoint(const CoverageViewpoint& viewpoint) {
    path.push_back(viewpoint);
}

// Updated method implementation
const std::vector<CoverageViewpoint>& CoveragePath::getPath() const {
    return path;
}
