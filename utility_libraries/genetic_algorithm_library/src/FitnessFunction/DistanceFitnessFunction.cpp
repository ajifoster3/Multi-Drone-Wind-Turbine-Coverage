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

const std::vector<double> DistanceFitnessFunction::getPathLengths(
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

double DistanceFitnessFunction::getPathLength(
    std::vector<int> &path,
    int agentID,
    std::vector<Position> &cities)
{
    double pathLength = 0;
    if (path.empty())
    {
        return 0;
    }

    // Calculate distance from initial position to the first city
    Position &initialPos = cities[agentID];  // Assuming initial positions are stored in cities with an offset
    Position &firstCity = cities[path[0]];
    double verticalDistance = firstCity.altitude - initialPos.altitude;
    double horizontalDistance = HaversineDistance::calculateDistance(
        initialPos.latitude,
        initialPos.longitude,
        0.0,
        firstCity.latitude,
        firstCity.longitude,
        0.0);

    pathLength += calculateDroneDistance(verticalDistance, horizontalDistance);

    // Calculate distances between each pair of cities in the path
    for (size_t i = 1; i < path.size(); i++)
    {
        Position &prevCity = cities[path[i - 1]];
        Position &currentCity = cities[path[i]];
        verticalDistance = currentCity.altitude - prevCity.altitude;
        horizontalDistance = HaversineDistance::calculateDistance(
            prevCity.latitude,
            prevCity.longitude,
            0.0,
            currentCity.latitude,
            currentCity.longitude,
            0.0);

        pathLength += calculateDroneDistance(verticalDistance, horizontalDistance);
    }

    // Calculate distance from the last city back to the initial position
    Position &lastCity = cities[path.back()];
    verticalDistance = lastCity.altitude - initialPos.altitude;
    horizontalDistance = HaversineDistance::calculateDistance(
        lastCity.latitude,
        lastCity.longitude,
        0.0,
        initialPos.latitude,
        initialPos.longitude,
        0.0);

    pathLength += calculateDroneDistance(verticalDistance, horizontalDistance);

    // Return the reciprocal of the total path length to match the original logic
    return pathLength;
}

double DistanceFitnessFunction::calculateDroneDistance(double vertical_distance, double horizontal_distance, double ascending_speed, double descending_speed, double horizontal_speed) {
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


std::map<Fitness, double> DistanceFitnessFunction::calulateChromosomeFitness(
    Chromosome &chromosome,
    std::vector<Position> &cities)
{
    auto paths{getPaths(chromosome)};
    auto pathDistances{getPathLengths(paths, cities)};

    return std::map<Fitness, double>{{Fitness::MAXPATHLENGTH, *std::max_element(pathDistances.begin(), pathDistances.end())}};
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
