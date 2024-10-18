#include "FitnessCalculator.h"

void FitnessCalculator::populateCostMap(std::vector<Position> &cities, std::vector<Position> &initialPositions)
{
    fitnessFunction_->calculateCostMap(cities, initialPositions);
}

std::map<Fitness, double> FitnessCalculator::calculateFitness(
    Chromosome &chromosome,
    std::vector<Position> &cities)
{
    return fitnessFunction_->calulateChromosomeFitness(chromosome, cities);
}

double FitnessCalculator::calculateSubvectorFitness(std::vector<int> &subChromosome, int robotNumber, std::vector<Position> &cities)
{
    return fitnessFunction_->calculateSubChromosomeFitness(subChromosome, robotNumber, cities);
}
