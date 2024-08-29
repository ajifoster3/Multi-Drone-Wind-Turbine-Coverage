#include "MultiDistanceFitnessFunction.h"
#include <iostream>

MultiDistanceFitnessFunction::MultiDistanceFitnessFunction(double alphaObjective, double alphaFitness) 
    : alphaObjective(alphaObjective), alphaFitness(alphaFitness) {
}

void MultiDistanceFitnessFunction::calculateCostMap(std::vector<Position> &cities, std::vector<Position> &initialPositions)
{
    /* for (int i = 0; i < cities.size(); i++)
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
            // Distance from initial position i to city j
            initialPoseCostMap_[std::make_pair(i, j)] = HaversineDistance::calculateDistance(
                initialPositions[i].latitude,
                initialPositions[i].longitude,
                initialPositions[i].altitude,
                cities[j].latitude,
                cities[j].longitude,
                cities[j].altitude);

            // Distance from city j to initial position i (not assumed symmetrical)
            initialPoseCostMap_[std::make_pair(j, i)] = HaversineDistance::calculateDistance(
                cities[j].latitude,
                cities[j].longitude,
                cities[j].altitude,
                initialPositions[i].latitude,
                initialPositions[i].longitude,
                initialPositions[i].altitude);
        }
    } */
    for (int i = 0; i < cities.size(); i++)
    {
        for (int j = 0; j < cities.size(); j++)
        {
            double verticalDistance = cities[j].altitude - cities[i].altitude;
            double horizontalDistance = HaversineDistance::calculateDistance(
                cities[i].latitude, cities[i].longitude, 0,  // altitude is set to 0 for horizontal distance
                cities[j].latitude, cities[j].longitude, 0);

            cityCostMap_[std::make_pair(i, j)] = calculateDroneDistance(
                verticalDistance, horizontalDistance);
        }
    }
    for (int i = 0; i < initialPositions.size(); i++)
    {
        for (int j = 0; j < cities.size(); j++)
        {
            // Distance from initial position i to city j
            double verticalDistanceToCity = cities[j].altitude - initialPositions[i].altitude;
            double horizontalDistanceToCity = HaversineDistance::calculateDistance(
                initialPositions[i].latitude, initialPositions[i].longitude, 0,
                cities[j].latitude, cities[j].longitude, 0);

            initialPoseCostMap_[std::make_pair(i, j)] = calculateDroneDistance(
                verticalDistanceToCity, horizontalDistanceToCity);

            // Distance from city j to initial position i (not assumed symmetrical)
            double verticalDistanceToInitial = initialPositions[i].altitude - cities[j].altitude;
            double horizontalDistanceToInitial = HaversineDistance::calculateDistance(
                cities[j].latitude, cities[j].longitude, 0,
                initialPositions[i].latitude, initialPositions[i].longitude, 0);

            initialPoseCostMap_[std::make_pair(j, i)] = calculateDroneDistance(
                verticalDistanceToInitial, horizontalDistanceToInitial);
        }
    }
}


double MultiDistanceFitnessFunction::calculateDroneDistance(double vertical_distance, double horizontal_distance, double ascending_speed, double descending_speed, double horizontal_speed) {
    double vertical_speed = (vertical_distance >= 0) ? ascending_speed : descending_speed;
    
    // Calculate the time required for each direction
    double time_vertical = std::abs(vertical_distance) / vertical_speed;
    double time_horizontal = horizontal_distance / horizontal_speed;
    
    // Determine the shorter time (when one direction is completed)
    double common_time = std::min(time_vertical, time_horizontal);
    
    // Calculate the distance covered during the common time
    double vertical_distance_common = vertical_speed * common_time;
    double horizontal_distance_common = horizontal_speed * common_time;
    
    // Calculate the distance for the first segment where both movements occur
    double segment1_distance = std::sqrt(vertical_distance_common * vertical_distance_common + horizontal_distance_common * horizontal_distance_common);
    
    // Calculate the remaining distance in the direction that wasn't completed
    double remaining_distance = 0.0;
    if (time_vertical > time_horizontal) {
        remaining_distance = vertical_speed * (time_vertical - time_horizontal); // remaining vertical distance
    } else if (time_horizontal > time_vertical) {
        remaining_distance = horizontal_speed * (time_horizontal - time_vertical); // remaining horizontal distance
    }
    
    // Total distance is the sum of the segment1 distance and the remaining distance
    return segment1_distance + remaining_distance;
}

double MultiDistanceFitnessFunction::getPathLength(
    std::vector<int> &path,
    int agentID,
    std::vector<Position> &cities)
{
    double pathLength = 0.0;
    if (path.size() == 0)
    {
        return 0;
    };
    auto p = initialPoseCostMap_[std::pair(agentID, path[0])];
    pathLength += initialPoseCostMap_[std::pair(agentID, path[0])];
    for (size_t i = 1; i < path.size(); i++)
    {
        auto p2 = cityCostMap_[std::pair(path[i - 1], path[i])];
        pathLength += cityCostMap_[std::pair(path[i - 1], path[i])];
    }
    pathLength += initialPoseCostMap_[std::pair(path[path.size() - 1], agentID)];
    return pathLength;
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


std::map<Fitness, double> MultiDistanceFitnessFunction::calulateChromosomeFitness(
    Chromosome &chromosome,
    std::vector<Position> &cities)
{
    auto longestPathFitness = calulateChromosomeLongestPathFitness(chromosome, cities) ;
    auto averagePathFitness = calulateChromosomeTotalPathFitness(chromosome, cities) ;
    auto map = std::map<Fitness, double>{
        {Fitness::MAXPATHTOTALPATHWEIGHTEDSUMOBJECTIVE, (longestPathFitness*alphaObjective) + (averagePathFitness*(1-alphaObjective))}, 
        {Fitness::MAXPATHTOTALPATHWEIGHTEDSUM, (longestPathFitness*alphaFitness) + (averagePathFitness*(1-alphaFitness))}, 
        {Fitness::MAXPATHLENGTH, longestPathFitness}, 
        {Fitness::TOTALPATHDISTANCE, averagePathFitness}};
    return map;
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
    auto averagePathLength = std::accumulate(pathDistances.begin(), pathDistances.end(), 0.0)/paths.size();
    return averagePathLength;
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
