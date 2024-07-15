#ifndef COVERAGE_VIEWPOINT_LOADER_H
#define COVERAGE_VIEWPOINT_LOADER_H

#include "CoverageViewpoint.h"
#include "json.hpp"
#include <string>
#include <vector>
#include <fstream>
#include <iostream>

namespace CoverageViewpointLoader
{
    using json = nlohmann::json;

    std::vector<CoverageViewpoint> load(const std::string &filePath);
} // namespace CoverageViewpointLoader

#endif
