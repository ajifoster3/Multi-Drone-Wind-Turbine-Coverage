#include "DistanceFitnessFunction.h"

void DistanceFitnessFunction::calculateCostMap(std::vector<Position> &cities, std::vector<Position> &initialPositions)
{
    for (int i = 0; i < cities.size(); i++)
    {
        for (int j = 0; j < cities.size(); j++)
        {
            cityCostMap_[std::pair(i, j)] = HaversineDistance::calculateDistance(
                cities[i].latitude,
                cities[i].longitude,
                cities[i].altitude,
                cities[j].latitude,
                cities[j].longitude,
                cities[j].altitude);
        }
    }
    for (int i = 0; i < initialPositions.size(); i++)
    {
        for (int j = 0; j < cities.size(); j++)
        {
            initialPoseCostMap_[std::pair(i, j)] = HaversineDistance::calculateDistance(
                initialPositions[i].latitude,
                initialPositions[i].longitude,
                initialPositions[i].altitude,
                cities[j].latitude,
                cities[j].longitude,
                cities[j].altitude);
        }
    }
}

const std::vector<double> DistanceFitnessFunction::getInversePathLengths(
    std::vector<std::vector<int>> &paths,
    std::vector<Position> &cities)
{
    std::vector<double> pathFitnesses;
    pathFitnesses.reserve(paths.size());

    for (size_t i = 0; i < paths.size(); i++)
    {
        auto pathLength = getInversePathLength(paths[i], i, cities);
        pathFitnesses.emplace_back(pathLength);
    }

    return pathFitnesses;
}

double DistanceFitnessFunction::getInversePathLength(
    std::vector<int> &path,
    int agentID,
    std::vector<Position> &cities)
{
    double pathLength = 0;
    if (path.size() == 0)
    {
        return 0;
    };
    pathLength += initialPoseCostMap_[std::pair(agentID, path[0])];
    for (size_t i = 1; i < path.size(); i++)
    {
        pathLength += cityCostMap_[std::pair(path[i - 1], path[i])];
    }
    pathLength += initialPoseCostMap_[std::pair(agentID, path[path.size()-1])];
    return 1 / pathLength;
}

double DistanceFitnessFunction::calulateChromosomeFitness(
    Chromosome &chromosome,
    std::vector<Position> &cities)
{
    auto paths{getPaths(chromosome)};
    auto pathDistances{getInversePathLengths(paths, cities)};

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
