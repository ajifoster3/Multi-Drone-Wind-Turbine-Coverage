#ifndef POPULATION_H
#define POPULATION_H

#include <vector>
#include "Chromosome.h"
#include "FitnessCalculator.h"
#include <limits>
#include <algorithm>
#include <cmath>
#include <chrono>

class Population
{
public:
    Population(){};
    Population(std::vector<Chromosome> &);

    std::vector<Chromosome> getPopulationList();
    std::vector<int> getFittestChromosomeGenes(FitnessCalculator &fitnessCalculator, std::vector<Position> &agentStartPositions, std::vector<Position> &cities, Fitness fitnessChoice);
    double calculateHypervolume(const std::vector<std::pair<double, double>>& paretoFront, std::pair<double, double> nadir);
    std::map<Fitness, double> getPopulationFitness(FitnessCalculator &fitnessCalculator, std::vector<Position> &agentStartPositions, std::vector<Position> &cities, Fitness fitnessChoice, std::chrono::_V2::steady_clock::time_point startTime, std::pair<double, double> nadir);
    std::vector<std::vector<Chromosome>> FastNonDominatedSort(std::vector<Position> &agentStartPositions, std::vector<Position> &cities, FitnessCalculator &fitnessCalculator);
    std::vector<Chromosome> getParetoFront(FitnessCalculator &fitnessCalculator, std::vector<Position> &agentStartPositions, std::vector<Position> &cities);
    bool isParentMalformed(Chromosome chromosome, int teamSize);
    bool dominates(const Chromosome &a, const Chromosome &b);

private:
    std::vector<Chromosome> populationList_;
    double maxFitness_;
    double minFitness_;
    bool dominates(const std::map<Fitness, double> &fitness1, const std::map<Fitness, double> &fitness2, const std::vector<Fitness> &fitnessChoices);
};

#endif
