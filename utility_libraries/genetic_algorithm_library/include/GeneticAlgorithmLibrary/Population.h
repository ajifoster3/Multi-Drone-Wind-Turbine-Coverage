#ifndef POPULATION_H
#define POPULATION_H

#include <vector>
#include "Chromosome.h"
#include "FitnessCalculator.h"

class Population
{
public:
    Population(){};
    Population(std::vector<Chromosome> &);

    std::vector<Chromosome> getPopulationList();
    double getPopulationFitness(FitnessCalculator &fitnessCalculator, std::vector<Position> &agentStartPositions, std::vector<Position> &cities);
    std::vector<int> getFittestChromosomeGenes(FitnessCalculator &fitnessCalculator, std::vector<Position> &agentStartPositions, std::vector<Position> &cities);

private:
    std::vector<Chromosome> populationList_;
    double maxFitness_;
    double minFitness_;
};

#endif
