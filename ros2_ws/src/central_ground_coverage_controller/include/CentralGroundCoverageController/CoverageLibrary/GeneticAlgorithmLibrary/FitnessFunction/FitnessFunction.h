#ifndef FITNESSFUNCTION_H
#define FITNESSFUNCTION_H

#include <vector>
#include "Chromosome.h"
#include "Position.h"

enum FitnessFunctions { DISTANCE_FITNESS_FUNCTION };

class FitnessFunction
{
    
public:
    virtual ~FitnessFunction() = default;
    virtual double calulateChromosomeFitness(
        Chromosome &, 
        std::vector<Position> &initialAgentPoses, 
        std::vector<Position> &cities) = 0;
    virtual void calculateCostMap(std::vector<Position> &cities) = 0;
};

#endif
