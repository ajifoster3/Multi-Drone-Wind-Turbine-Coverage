#include "FitnessCalculator.h"

void FitnessCalculator::populateCostMap(std::vector<Position> &cities)
{
    fitnessFunction_->calculateCostMap(cities);
}

double FitnessCalculator::calculateFitness(
    Chromosome &chromosome,
    std::vector<Position> &initialAgentPoses,
    std::vector<Position> &cities)
{
    return fitnessFunction_->calulateChromosomeFitness(chromosome, initialAgentPoses, cities);
}
