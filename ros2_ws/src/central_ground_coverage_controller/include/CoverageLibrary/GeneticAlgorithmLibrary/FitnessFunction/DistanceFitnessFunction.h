#ifndef DISTANCEFITNESSFUNCTION_H
#define DISTANCEFITNESSFUNCTION_H

#include <vector>
#include <stddef.h>
#include <numeric>
#include <unordered_map>
#include <algorithm>
#include "FitnessFunction.h"
#include "Position.h"
#include "Chromosome.h"
#include "HaversineDistance.h"

class DistanceFitnessFunction : public FitnessFunction
{

public:

    double calulateChromosomeFitness(
        Chromosome &,
        std::vector<Position> &initialAgentPoses,
        std::vector<Position> &cities);

    std::vector<std::vector<int>> getPaths(Chromosome &chromosome);

    void calculateCostMap(std::vector<Position> &cities);

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

    std::unordered_map<std::pair<int, int>, double, pair_hash> costMap_;

    const std::vector<double> getInversePathLengths(
        std::vector<std::vector<int>> &paths,
        std::vector<Position> &initialAgentPoses,
        std::vector<Position> &cities);

    double getInversePathLength(
        std::vector<int> &path,
        Position &initialAgentPoses,
        std::vector<Position> &cities);
};

#endif
