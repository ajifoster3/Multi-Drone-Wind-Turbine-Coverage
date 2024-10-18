#include "MultiDistanceFitnessFunction.h"
#include <iostream>
#include <fstream>
#include <iomanip>

MultiDistanceFitnessFunction::MultiDistanceFitnessFunction(double alphaObjective, double alphaFitness)
    : alphaObjective(alphaObjective), alphaFitness(alphaFitness)
{
}

void MultiDistanceFitnessFunction::calculateCostMap(std::vector<Position> &cities, std::vector<Position> &initialPositions)
{
    // Open file for output
    std::ofstream outFile("cost_maps.txt");

    if (!outFile)
    {
        std::cerr << "Error: Could not open the file for writing.\n";
        return;
    }


    // Calculate cityCostMap_ (distance between cities)
    outFile << "City-City Cost Map:\n";
    outFile << std::fixed << std::setprecision(9);
    for (int i = 0; i < cities.size(); i++)
    {
        for (int j = 0; j < cities.size(); j++)
        {
            double verticalDistance = cities[j].altitude - cities[i].altitude;
            double horizontalDistance = HaversineDistance::calculateDistance(
                cities[i].latitude, cities[i].longitude, 0, // altitude is set to 0 for horizontal distance
                cities[j].latitude, cities[j].longitude, 0);

            cityCostMap_[std::make_pair(i, j)] = generate_idealized_trajectoryComponentHorz(
                horizontalDistance, verticalDistance);

            // Write city cost map to file
            outFile << "City " << i << " to City " << j << ": " << cityCostMap_[std::make_pair(i, j)] << "\n";
        }
    }

    // Calculate initialPoseCostMap_ (distance between initial positions and cities)
    outFile << "\nInitial Position-City Cost Map:\n";
    for (int i = 0; i < initialPositions.size(); i++)
    {
        for (int j = 0; j < cities.size(); j++)
        {
            // Distance from initial position i to city j
            double verticalDistanceToCity = cities[j].altitude - initialPositions[i].altitude;
            double horizontalDistanceToCity = HaversineDistance::calculateDistance(
                initialPositions[i].latitude, initialPositions[i].longitude, 0,
                cities[j].latitude, cities[j].longitude, 0);

            initialPoseToCityCostMap_[std::make_pair(i, j)] = generate_idealized_trajectoryComponentHorz(
                horizontalDistanceToCity, verticalDistanceToCity);

            // Write initial pose cost map to file
            outFile << "Initial Position " << i << " to City " << j << ": " << initialPoseToCityCostMap_[std::make_pair(i, j)] << "\n";

            // Distance from city j to initial position i
            double verticalDistanceToInitial = initialPositions[i].altitude - cities[j].altitude;
            double horizontalDistanceToInitial = HaversineDistance::calculateDistance(
                cities[j].latitude, cities[j].longitude, 0,
                initialPositions[i].latitude, initialPositions[i].longitude, 0);

            initialCityToPoseCostMap_[std::make_pair(j, i)] = generate_idealized_trajectoryComponentHorz(
                horizontalDistanceToInitial, verticalDistanceToInitial);

            // Write reversed distance to file
            outFile << "City " << j << " to Initial Position " << i << ": " << initialPoseToCityCostMap_[std::make_pair(j, i)] << "\n";
        }
    }

    // Close the file
    outFile.close();
}

double horiz_accel_func(double time, double velocity, double remaining_distance)
{
    return std::min(0.2 * std::pow(1.3 * remaining_distance, 4), 4.0);
}

// Vertical ascent acceleration function
double vert_accel_func(double time, double velocity, double remaining_distance)
{
    return std::min(0.2 * std::pow(1.3 * remaining_distance, 4), 8.0);
}

// Vertical descent acceleration function
double descend_accel_func(double time, double velocity, double remaining_distance)
{
    return std::min(0.2 * std::pow(1.3 * remaining_distance, 4), 4.0);
}

double MultiDistanceFitnessFunction::generate_idealized_trajectoryComponentHorz(double total_horizontal_distance, double total_vertical_distance,
                                                                                double initial_horiz_velocity, double initial_vert_velocity,
                                                                                double max_horiz_velocity, double max_vert_velocity, double max_descend_velocity, double time_step)
{

    std::vector<double> horiz_positions{};
    std::vector<double> vert_positions{};
    std::vector<double> times{};
    // Initialize positions and velocities
    double horiz_velocity = initial_horiz_velocity;
    double vert_velocity = initial_vert_velocity;
    double current_horizontal_distance = 0;
    double current_vertical_distance = 0;
    double total_time = 0;
    double total_3d_distance = 0; // To accumulate the total 3D distance

    // Store the previous positions for distance calculation
    double prev_horizontal_distance = 0;
    double prev_vertical_distance = 0;

    // Main loop to update the trajectory
    while (std::abs(total_horizontal_distance - current_horizontal_distance) > 0.01 ||
           std::abs(total_vertical_distance - current_vertical_distance) > 0.01)
    {
        // Store current positions and time for saving
        horiz_positions.push_back(current_horizontal_distance);
        vert_positions.push_back(current_vertical_distance);
        times.push_back(total_time);

        // Get remaining horizontal and vertical distances
        double remaining_horizontal_distance = total_horizontal_distance - current_horizontal_distance;
        double remaining_vertical_distance = total_vertical_distance - current_vertical_distance;

        // Get current accelerations from the provided functions
        double horiz_accel = (remaining_horizontal_distance > 0) ? horiz_accel_func(total_time, horiz_velocity, remaining_horizontal_distance) : 0;

        double vert_accel = 0;
        if (remaining_vertical_distance > 0)
        {
            vert_accel = vert_accel_func(total_time, vert_velocity, remaining_vertical_distance);
        }
        else if (remaining_vertical_distance < 0)
        {
            vert_accel = descend_accel_func(total_time, vert_velocity, remaining_vertical_distance);
        }

        // Update velocities using current accelerations, if there's still distance to cover
        if (std::abs(remaining_horizontal_distance) > 0.01)
        {
            horiz_velocity += horiz_accel * time_step;
            horiz_velocity = std::min(horiz_velocity, max_horiz_velocity);

            // Prevent overshooting horizontal distance
            double max_horiz_velocity_allowed = remaining_horizontal_distance / time_step;
            horiz_velocity = std::min(horiz_velocity, max_horiz_velocity_allowed);
        }
        else
        {
            horiz_velocity = 0.01; // Stop horizontal movement
        }

        if (std::abs(remaining_vertical_distance) > 0.01)
        {
            vert_velocity += vert_accel * time_step;
            if (remaining_vertical_distance > 0)
            {
                vert_velocity = std::min(vert_velocity, max_vert_velocity);
            }
            else
            {
                vert_velocity = std::min(vert_velocity, max_descend_velocity);
            }
        }
        else
        {
            vert_velocity = 0.01; // Stop vertical movement
        }

        // Update distances covered
        double horiz_distance_covered = horiz_velocity * time_step;
        double vert_distance_covered = vert_velocity * time_step;

        // Update the current position (horizontal)
        if (horiz_distance_covered >= remaining_horizontal_distance)
        {
            horiz_distance_covered = remaining_horizontal_distance;
            current_horizontal_distance = total_horizontal_distance;
        }
        else
        {
            current_horizontal_distance += horiz_distance_covered;
        }

        // Update the current position (vertical)
        if (std::abs(vert_distance_covered) >= std::abs(remaining_vertical_distance))
        {
            vert_distance_covered = remaining_vertical_distance;
            current_vertical_distance = total_vertical_distance;
        }
        else
        {
            current_vertical_distance += vert_distance_covered;
        }

        // Calculate the Euclidean distance traveled during this time step
        double distance_covered = std::sqrt(
            std::pow(current_horizontal_distance - prev_horizontal_distance, 2) +
            std::pow(current_vertical_distance - prev_vertical_distance, 2));

        // Accumulate the total 3D distance
        total_3d_distance += distance_covered;

        // Update previous positions
        prev_horizontal_distance = current_horizontal_distance;
        prev_vertical_distance = current_vertical_distance;

        // Update total time
        total_time += time_step;

        // Stop the loop when both distances are covered
        if (current_horizontal_distance >= total_horizontal_distance &&
            std::abs(current_vertical_distance) >= std::abs(total_vertical_distance))
        {
            break;
        }
    }

    // Return the total 3D distance accumulated
    return total_3d_distance;
}

double MultiDistanceFitnessFunction::calculateDroneDistance(double vertical_distance, double horizontal_distance, double ascending_speed, double descending_speed, double horizontal_speed)
{
    // Open file for output (append mode, so multiple calls to this function will add data to the same file)
    std::ofstream outFile("drone_distances.txt", std::ios::app);

    if (!outFile)
    {
        std::cerr << "Error: Could not open the file for writing.\n";
        return -1.0; // return an invalid distance in case of file error
    }

    // Calculate the vertical speed based on direction
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
    if (time_vertical > time_horizontal)
    {
        remaining_distance = vertical_speed * (time_vertical - time_horizontal); // remaining vertical distance
    }
    else if (time_horizontal > time_vertical)
    {
        remaining_distance = horizontal_speed * (time_horizontal - time_vertical); // remaining horizontal distance
    }

    // Total distance is the sum of the segment1 distance and the remaining distance
    double total_distance = segment1_distance + remaining_distance;

    // Write the calculation details to the file
    outFile << "Vertical Distance: " << vertical_distance << ", Horizontal Distance: " << horizontal_distance << "\n";
    outFile << "Ascending Speed: " << ascending_speed << ", Descending Speed: " << descending_speed << ", Horizontal Speed: " << horizontal_speed << "\n";
    outFile << "Total Drone Distance: " << total_distance << "\n\n";

    // Close the file
    outFile.close();

    return total_distance;
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
    auto p = initialPoseToCityCostMap_[std::pair(agentID, path[0])];
    pathLength += initialPoseToCityCostMap_[std::pair(agentID, path[0])];
    for (size_t i = 1; i < path.size(); i++)
    {
        auto p2 = cityCostMap_[std::pair(path[i - 1], path[i])];
        pathLength += cityCostMap_[std::pair(path[i - 1], path[i])];
    }
    pathLength += initialCityToPoseCostMap_[std::pair(path[path.size() - 1], agentID)];
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
    auto longestPathFitness = calulateChromosomeLongestPathFitness(chromosome, cities);
    auto averagePathFitness = calulateChromosomeTotalPathFitness(chromosome, cities);
    auto map = std::map<Fitness, double>{
        {Fitness::MAXPATHTOTALPATHWEIGHTEDSUMOBJECTIVE, (longestPathFitness * alphaObjective) + (averagePathFitness * (1 - alphaObjective))},
        {Fitness::MAXPATHTOTALPATHWEIGHTEDSUM, (longestPathFitness * alphaFitness) + (averagePathFitness * (1 - alphaFitness))},
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
    auto averagePathLength = std::accumulate(pathDistances.begin(), pathDistances.end(), 0.0) / paths.size();
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
