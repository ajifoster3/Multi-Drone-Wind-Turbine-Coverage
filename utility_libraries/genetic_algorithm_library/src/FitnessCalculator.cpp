#include "FitnessCalculator.h"

void FitnessCalculator::populateCostMap(std::vector<Position> &cities, std::vector<Position> &initialPositions)
{
    fitnessFunction_->calculateCostMap(cities, initialPositions);
}

double FitnessCalculator::calculateFitness(
    Chromosome &chromosome,
    std::vector<Position> &cities)
{
    return fitnessFunction_->calulateChromosomeFitness(chromosome, cities);
}
