#ifndef FITNESSCALCULATOR_H
#define FITNESSCALCULATOR_H

#include <memory>
#include "FitnessFunction.h"
#include "Chromosome.h"
#include "Position.h"

class FitnessCalculator
{

public:
    FitnessCalculator(){};
    FitnessCalculator(std::shared_ptr<FitnessFunction> fitnessFunction)
        : fitnessFunction_(std::move(fitnessFunction)){};
    void populateCostMap(std::vector<Position> &cities);
    double calculateFitness(
        Chromosome &chromosome,
        std::vector<Position> &initialAgentPoses,
        std::vector<Position> &cities);

private:
    std::shared_ptr<FitnessFunction> fitnessFunction_;
};

#endif
