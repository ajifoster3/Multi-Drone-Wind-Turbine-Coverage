#include "Population.h"
#include "Fitnesses.h"
#include <limits>

Population::Population(std::vector<Chromosome> &populationList)
{
    populationList_ = populationList;
}

std::vector<Chromosome> Population::getPopulationList()
{
    return populationList_;
}

double Population::getPopulationFitness(FitnessCalculator &fitnessCalculator, std::vector<Position> &agentStartPositions, std::vector<Position> &cities, Fitness fitnessChoice)
{
    double minFitness = std::numeric_limits<double>::max();
    ;
    for (auto chromosome : this->getPopulationList())
    {
        auto fitness = fitnessCalculator.calculateFitness(chromosome, cities);
        if (minFitness > fitness[fitnessChoice])
        {
            minFitness = fitness[fitnessChoice];
        }
    }
    return minFitness;
}

std::vector<int> Population::getFittestChromosomeGenes(
    FitnessCalculator &fitnessCalculator,
    std::vector<Position> &agentStartPositions,
    std::vector<Position> &cities,
    Fitness fitnessChoice)
{
    std::vector<int> fittestChromosomeGenes;
    double maxFitness = 0;
    for (auto chromosome : this->getPopulationList())
    {
        auto fitness = fitnessCalculator.calculateFitness(chromosome, cities);
        if (maxFitness < fitness[fitnessChoice])
        {
            fittestChromosomeGenes = chromosome.getGenes();
            maxFitness = fitness[fitnessChoice];
        }
    }
    return fittestChromosomeGenes;
}
