#ifndef FITNESSCALCULATOR_H
#define FITNESSCALCULATOR_H

#include <memory>
#include "FitnessFunction.h"
#include "Chromosome.h"
#include "Position.h"
#include "Fitnesses.h"
#include <map>


class FitnessCalculator
{

public:
    FitnessCalculator(){};
    FitnessCalculator(std::shared_ptr<FitnessFunction> fitnessFunction)
        : fitnessFunction_(std::move(fitnessFunction)){};
    void populateCostMap(std::vector<Position> &cities, std::vector<Position> &initialPositions);
    std::map<Fitness, double> calculateFitness(
        Chromosome &chromosome,
        std::vector<Position> &cities);
    double calculateSubvectorFitness(
        std::vector<int> &subChromosome,
        int robotNumber,
        std::vector<Position> &cities);

private:
    std::shared_ptr<FitnessFunction> fitnessFunction_;
};

#endif
