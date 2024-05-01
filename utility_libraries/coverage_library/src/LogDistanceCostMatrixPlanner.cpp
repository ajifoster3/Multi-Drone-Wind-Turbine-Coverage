#include "LogDistanceCostMatrixPlanner.h"
#include <iomanip>

LogDistanceCostMatrixPlanner::LogDistanceCostMatrixPlanner(
    const std::vector<int> &robotIDs,
    const std::vector<Pose> &initialPoses,
    std::vector<CoverageViewpoint> &viewpoints)
    : CoveragePathPlanner()
{
    this->robotIDs = robotIDs;
    this->initialRobotPoses = initialPoses;
    this->robotPoses = initialPoses;
    this->viewpoints = GoalCoverageViewpoints(viewpoints);
    this->coveragePaths = CoveragePaths(robotIDs);

    planCoveragePath();
}

void LogDistanceCostMatrixPlanner::planCoveragePath()
{
    std::vector<std::vector<int>> costMap;
    std::vector<Pose::Position> positions(initialRobotPoses.size() + viewpoints.getViewpointPositions().size());
    std::transform(initialRobotPoses.begin(), initialRobotPoses.end(), positions.begin(), [](Pose pose)
                   { return pose.position; });
    auto viewpointPositions = viewpoints.getViewpointPositions();
    std::copy(viewpointPositions.begin(), viewpointPositions.end(), positions.begin() + initialRobotPoses.size());

    costMap.resize(positions.size(), std::vector<int>(positions.size()));

    for (int i = 0; i < positions.size(); i++)
    {
        for (int j = 0; j < positions.size(); j++)
        {
            costMap[i][j] = HaversineDistance::calculateDistance(
                positions[i].latitude,
                positions[i].longitude,
                positions[i].altitude,
                positions[j].latitude,
                positions[j].longitude,
                positions[j].altitude);
        }
    }
    printCostMap(costMap);
}

void LogDistanceCostMatrixPlanner::printCostMap(const std::vector<std::vector<int>> &costMap)
{
    std::cout << "distance = [\n";
    for (size_t i = 0; i < costMap.size(); ++i)
    {
        const auto &row = costMap[i];
        std::cout << "    [";
        for (size_t j = 0; j < row.size(); ++j)
        {
            std::cout << row[j];
            if (j != row.size() - 1)
            {
                std::cout << ", ";
            }
        }
        std::cout << "]";
        if (i != costMap.size() - 1)
        {
            std::cout << ",";
        }
        std::cout << "\n";
    }
    std::cout << "];\n";
}
