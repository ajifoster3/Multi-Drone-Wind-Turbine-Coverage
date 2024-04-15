#include "DistanceFitnessFunction.h"

void DistanceFitnessFunction::calculateCostMap(std::vector<Position> &cities)
{
    for (int i = 0; i < cities.size(); i++)
    {
        for (int j = 0; j < cities.size(); j++)
        {
            costMap_[std::pair(i, j)] = HaversineDistance::calculateDistance(
                cities[i].latitude,
                cities[i].longitude,
                cities[i].altitude,
                cities[j].latitude,
                cities[j].longitude,
                cities[j].altitude);
        }
    }
}

const std::vector<double> DistanceFitnessFunction::getInversePathLengths(
    std::vector<std::vector<int>> &paths,
    std::vector<Position> &initialAgentPoses,
    std::vector<Position> &cities)
{
    std::vector<double> pathFitnesses;
    pathFitnesses.reserve(paths.size());

    for (size_t i = 0; i < paths.size(); i++)
    {
        auto pathLength = getInversePathLength(paths[i], initialAgentPoses[i], cities);
        pathFitnesses.emplace_back(pathLength);
    }

    return pathFitnesses;
}

double DistanceFitnessFunction::getInversePathLength(
    std::vector<int> &path,
    Position &initialAgentPose,
    std::vector<Position> &cities)
{
    double pathLength = 0;
    if (path.size() == 0)
    {
        return 0;
    };
    pathLength += HaversineDistance::calculateDistance(
        initialAgentPose.latitude,
        initialAgentPose.longitude,
        initialAgentPose.altitude,
        cities[path[0]].latitude,
        cities[path[0]].longitude,
        cities[path[0]].altitude);
    for (size_t i = 1; i < path.size(); i++)
    {
        pathLength += costMap_[std::pair(path[i - 1], path[i])];
    }

    return 1 / pathLength;
}

double DistanceFitnessFunction::calulateChromosomeFitness(
    Chromosome &chromosome,
    std::vector<Position> &initialAgentPoses,
    std::vector<Position> &cities)
{
    auto paths{getPaths(chromosome)};
    auto pathDistances{getInversePathLengths(paths, initialAgentPoses, cities)};

    return *std::min_element(pathDistances.begin(), pathDistances.end());
}

std::vector<std::vector<int>> DistanceFitnessFunction::getPaths(Chromosome &chromosome)
{
    std::vector<std::vector<int>> paths;
    paths.reserve(chromosome.getNumberOfAgents());
    int numberProcessedPositions{0};
    for (size_t i = 0; i < chromosome.getNumberOfAgents(); i++)
    {
        int agentPathLength{chromosome.getGenesAtIndex(chromosome.getNumberOfCities() + i)};

        paths.emplace_back(chromosome.getGenesBetweenIndices(numberProcessedPositions, numberProcessedPositions + agentPathLength));
        numberProcessedPositions += agentPathLength;
    }

    return paths;
}
