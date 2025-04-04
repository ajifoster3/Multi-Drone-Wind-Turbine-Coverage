#ifndef MULTIDISTANCEFITNESSFFUNCTION_H
#define MULTIDISTANCEFITNESSFFUNCTION_H

#include <vector>
#include <stddef.h>
#include <numeric>
#include <unordered_map>
#include <algorithm>
#include "FitnessFunction.h"
#include "Position.h"
#include "Chromosome.h"
#include "HaversineDistance.h"

class MultiDistanceFitnessFunction : public FitnessFunction
{

public:
    MultiDistanceFitnessFunction(
        double alphaObjective,
        double alphaFitness);

    std::map<Fitness, double> calulateChromosomeFitness(
        Chromosome &,
        std::vector<Position> &cities);

    double calculateSubChromosomeFitness(std::vector<int> &subChromosome, int robotId, std::vector<Position> cities);

    double calulateChromosomeLongestPathFitness(
        Chromosome &,
        std::vector<Position> &cities);

    double calulateChromosomeTotalPathFitness(
        Chromosome &,
        std::vector<Position> &cities);

    std::vector<std::vector<int>> getPaths(Chromosome &chromosome);

    double generate_idealized_trajectoryComponentHorz(
        double total_horizontal_distance, double total_vertical_distance, 
        double initial_horiz_velocity=0, double initial_vert_velocity=0,
        double max_horiz_velocity=11, double max_vert_velocity=3, 
        double max_descend_velocity=1.45, double time_step = 0.1);

    void calculateCostMap(std::vector<Position> &cities, std::vector<Position> &initialPositions);

private:
    // Define a custom hash function for std::pair<int, int>
    struct pair_hash
    {
        template <class T1, class T2>
        std::size_t operator()(const std::pair<T1, T2> &pair) const
        {
            auto hash1 = std::hash<T1>{}(pair.first);
            auto hash2 = std::hash<T2>{}(pair.second);
            return hash1 ^ hash2 + 0x9e3779b9 + (hash1 << 6) + (hash1 >> 2);
        }
    };

    std::unordered_map<std::pair<int, int>, double, pair_hash> initialPoseToCityCostMap_;
    std::unordered_map<std::pair<int, int>, double, pair_hash> initialCityToPoseCostMap_;
    std::unordered_map<std::pair<int, int>, double, pair_hash> cityCostMap_;
    std::unordered_map<std::pair<int, int>, double, pair_hash> initialPoseVerticalCostMap_;
    std::unordered_map<std::pair<int, int>, double, pair_hash> cityVerticalCostMap_;
    double alphaObjective;
    double alphaFitness;

    double calculateDroneDistance(double vertical_distance, double horizontal_distance, double ascending_speed = 3.0, double descending_speed = 1.55, double horizontal_speed = 11.0);

    double calculateDroneDistance2(double start_lat, double start_long, double start_alt, double final_lat, double final_long, double final_alt, double initial_horiz_velocity, double initial_vert_velocity, double max_horiz_velocity, double max_vert_velocity, double max_descend_velocity, double time_step);

    const std::vector<double> getPathLengths(
        std::vector<std::vector<int>> &paths,
        std::vector<Position> &cities);

    double getPathLength(
        std::vector<int> &path,
        int agentID,
        std::vector<Position> &cities);
};

#endif
