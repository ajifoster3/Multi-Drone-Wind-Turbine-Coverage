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
    double sumOfFitness{0};
    for (auto chromosome : this->getPopulationList())
    {
        sumOfFitness += fitnessCalculator.calculateFitness(chromosome, agentStartPositions, cities);
    }
    return sumOfFitness;
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
        auto fitness = fitnessCalculator.calculateFitness(chromosome, agentStartPositions, cities);
        if (maxFitness < fitness)
        {
            fittestChromosomeGenes = chromosome.getGenes();
            maxFitness = fitness;
        }
    }
    return fittestChromosomeGenes;
}
