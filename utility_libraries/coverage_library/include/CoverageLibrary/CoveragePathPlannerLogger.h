#ifndef COVERAGEPATHPLANNERLOGGER_H
#define COVERAGEPATHPLANNERLOGGER_H

#include <json.hpp>
#include <fstream>
#include <iostream>
#include <typeinfo>
#include <regex>
#include <chrono>
#include <iomanip>
#include <filesystem>
#include <vector>
#include "Path.h"

namespace CoveragePathPlannerLogger
{
    void logCoveragePath(std::vector<Path> paths, std::string className);
}

#endif
