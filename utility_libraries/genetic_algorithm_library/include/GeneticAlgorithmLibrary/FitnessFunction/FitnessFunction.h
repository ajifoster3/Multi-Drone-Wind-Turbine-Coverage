#ifndef FITNESSFUNCTION_H
#define FITNESSFUNCTION_H

#include <vector>
#include "Chromosome.h"
#include "Position.h"
#include "Fitnesses.h"
#include <map>

enum FitnessFunctions
{
    DISTANCE_FITNESS_FUNCTION,
    MULTI_DISTANCE_FITNESS_FUNCTION
};

class FitnessFunction
{

public:
    virtual ~FitnessFunction() = default;
    virtual std::map<Fitness, double> calulateChromosomeFitness(
        Chromosome &,
        std::vector<Position> &cities) = 0;
    virtual double calculateSubChromosomeFitness(std::vector<int> &subChromosome, int robotId, std::vector<Position> cities) = 0;
    virtual void calculateCostMap(std::vector<Position> &cities, std::vector<Position> &initialPositions) = 0;
};

#endif
