#include "Population.h"

Population::Population(std::vector<Chromosome> &populationList)
{
    populationList_ = populationList;
}

std::vector<Chromosome> Population::getPopulationList()
{
    return populationList_;
}

double Population::getPopulationFitness(FitnessCalculator &fitnessCalculator, std::vector<Position> &agentStartPositions, std::vector<Position> &cities)
{
    double maxFitness;
    for (auto chromosome : this->getPopulationList())
    {
        auto fitness = fitnessCalculator.calculateFitness(chromosome, cities);
        if(maxFitness < fitness){maxFitness = fitness;}
    }
    return maxFitness;
}

std::vector<int> Population::getFittestChromosomeGenes(
    FitnessCalculator &fitnessCalculator,
    std::vector<Position> &agentStartPositions,
    std::vector<Position> &cities)
{
    std::vector<int> fittestChromosomeGenes;
    double maxFitness = 0;
    for (auto chromosome : this->getPopulationList())
    {
        auto fitness = fitnessCalculator.calculateFitness(chromosome, cities);
        if (maxFitness < fitness)
        {
            fittestChromosomeGenes = chromosome.getGenes();
            maxFitness = fitness;
        }
    }
    return fittestChromosomeGenes;
}
