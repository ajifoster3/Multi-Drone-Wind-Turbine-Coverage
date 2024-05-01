#ifndef IPGAREPRODUCTIONMECHANISM_H
#define IPGAREPRODUCTIONMECHANISM_H

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>
#include <map>
#include "ReproductionMechanism.h"
#include "Population.h"

class ReproductionPopulation
{

public:
    ReproductionPopulation(Population);

private:
    Population population_;
    double maxFitness;
    double minFitness;
};

class ReproductionChromosome
{

public:
    ReproductionChromosome(Chromosome &chromosome, std::shared_ptr<FitnessCalculator> fitnessCalculator, std::vector<Position> initialAgentPoses, std::vector<Position> cities);
    double getFitness() const;
    Chromosome& getChromosome();

private:
    Chromosome chromosome_;
    double fitness_;
};

class IPGAReproductionMechanism : public ReproductionMechanism
{
public:
    IPGAReproductionMechanism(std::shared_ptr<FitnessCalculator>);

    Population Reproduce(
        Population &oldPopulation,
        std::vector<Position> &initialAgentPoses,
        std::vector<Position> &cities);
    void shuffleReproductionChromosomeList(std::vector<ReproductionChromosome> &chromosomeFitness);

private:
    const int sampleSize_ = 10;
    const double citiesPerSalesmanMutationProbability_{0.025};
    const double routeMutationProbability_{0.5};

    std::mt19937 gen_;
    void flipInsert(std::vector<int> &vec, int numberOfCities);
    void swapInsert(std::vector<int> &vec, int numberOfCities);
    void lSlideInsert(std::vector<int> &vec, int numberOfCities);
    void rSlideInsert(std::vector<int> &vec, int numberOfCities);
    void randomlyInsertSubvector(std::vector<int> &vec, int index1, int index2, int numberOfCities);
    void distributeCities(std::vector<int>& vec, int numberOfCities, int numberOfAgents);
};

#endif
