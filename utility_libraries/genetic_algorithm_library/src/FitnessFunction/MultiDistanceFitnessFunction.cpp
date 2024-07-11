#include "MultiDistanceFitnessFunction.h"
#include <iostream>

void MultiDistanceFitnessFunction::calculateCostMap(std::vector<Position> &cities, std::vector<Position> &initialPositions)
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

const std::vector<double> MultiDistanceFitnessFunction::getPathLengths(
    std::vector<std::vector<int>> &paths,
    std::vector<Position> &cities)
{
    std::vector<double> pathFitnesses;
    pathFitnesses.reserve(paths.size());

    for (size_t i = 0; i < paths.size(); i++)
    {
        auto pathLength = getPathLength(paths[i], i, cities);
        pathFitnesses.emplace_back(pathLength);
    }

    return pathFitnesses;
}

double MultiDistanceFitnessFunction::getPathLength(
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
    pathLength += initialPoseCostMap_[std::pair(agentID, path[path.size() - 1])];
    return pathLength;
}

std::map<Fitness, double> MultiDistanceFitnessFunction::calulateChromosomeFitness(
    Chromosome &chromosome,
    std::vector<Position> &cities)
{
    auto longestPathFitness = calulateChromosomeLongestPathFitness(chromosome, cities) ;
    auto TotalPathFitness = calulateChromosomeTotalPathFitness(chromosome, cities) ;
    return std::map<Fitness, double>{{Fitness::MAXPATHTOTALPATHWEIGHTEDSUM, (longestPathFitness*0.9) + (TotalPathFitness*0.1)}, {Fitness::MAXPATHLENGTH, longestPathFitness}, {Fitness::TOTALPATHDISTANCE, TotalPathFitness}};
}

double MultiDistanceFitnessFunction::calculateSubChromosomeFitness(std::vector<int> &subChromosome, int robotId, std::vector<Position> cities)
{
    return getPathLength(subChromosome, robotId, cities);
}

double MultiDistanceFitnessFunction::calulateChromosomeLongestPathFitness(Chromosome &chromosome, std::vector<Position> &cities)
{
    auto paths{getPaths(chromosome)};
    auto pathDistances{getPathLengths(paths, cities)};
    return *std::max_element(pathDistances.begin(), pathDistances.end());
}

double MultiDistanceFitnessFunction::calulateChromosomeTotalPathFitness(Chromosome &chromosome, std::vector<Position> &cities)
{
    auto paths{getPaths(chromosome)};
    auto pathDistances{getPathLengths(paths, cities)};
    auto sumOfPaths = std::accumulate(pathDistances.begin(), pathDistances.end(), 0.0);
    return sumOfPaths;
}

std::vector<std::vector<int>> MultiDistanceFitnessFunction::getPaths(Chromosome &chromosome)
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
